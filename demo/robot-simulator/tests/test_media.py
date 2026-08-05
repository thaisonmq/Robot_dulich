import asyncio
import json
import time
from fractions import Fraction
from pathlib import Path
from subprocess import CompletedProcess

import pytest
from livekit import rtc
from livekit.rtc._proto import room_pb2 as proto_room

from simulator.config import SimulatorConfig
from simulator.media import (
    AUDIO_DEVICE_BUFFER_MS,
    AUDIO_DEVICE_PERIOD_MS,
    AUDIO_PLAYBACK_BUFFER_FRAMES,
    VIDEO_JITTER_BUFFER_FRAMES,
    CameraCaptureProfile,
    EncodedPipelineProgress,
    EncodedVideoPlan,
    MediaPublisher,
    SourceVideoProbe,
    analyze_runtime_video_packets,
    bounded_video_dimensions,
    parse_ffprobe_fraction,
    parse_v4l2_capture_profile,
    redact_media_source,
    video_pacer_max_latency_ms,
    video_pipe_buffer_limit,
)


def runtime_packets(
    fps: Fraction,
    count: int = 64,
) -> list[dict[str, str]]:
    interval = float(1 / fps)
    return [
        {
            "pts_time": f"{index * interval:.9f}",
            "dts_time": f"{index * interval:.9f}",
            "duration_time": f"{interval:.9f}",
            "size": "10000",
            "flags": "K_" if index == 0 else "__",
        }
        for index in range(count)
    ]


@pytest.mark.asyncio
async def test_main_room_subscribes_only_to_user_audio(monkeypatch) -> None:
    class Publication:
        def __init__(self, kind) -> None:
            self.kind = kind
            self.subscriptions: list[bool] = []

        def set_subscribed(self, subscribed: bool) -> None:
            self.subscriptions.append(subscribed)

    class Participant:
        def __init__(self, identity: str, publications: list[Publication]) -> None:
            self.identity = identity
            self.track_publications = {
                str(index): publication
                for index, publication in enumerate(publications)
            }

    user_audio = Publication(rtc.TrackKind.KIND_AUDIO)
    camera_video = Publication(rtc.TrackKind.KIND_VIDEO)
    robot_audio = Publication(rtc.TrackKind.KIND_AUDIO)
    user = Participant("user:operator", [user_audio, camera_video])
    camera = Participant("robot:camera", [robot_audio])

    class Room:
        def __init__(self) -> None:
            self.events = {}
            self.remote_participants = {"user": user, "camera": camera}
            self.options = None

        def on(self, event: str):
            def register(callback):
                self.events[event] = callback
                return callback

            return register

        async def connect(self, _url: str, _token: str, options) -> None:
            self.options = options

        async def disconnect(self) -> None:
            return None

    room = Room()
    monkeypatch.setattr("simulator.media.rtc.Room", lambda: room)

    async def token_provider(_purpose: str) -> str:
        return "token"

    async def no_media() -> None:
        return None

    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="test",
            video_pipeline="raw",
        ),
        token_provider,
    )
    monkeypatch.setattr(publisher, "_publish_video", no_media)
    monkeypatch.setattr(publisher, "_publish_audio", no_media)

    await publisher.connect()

    assert room.options.auto_subscribe is False
    assert user_audio.subscriptions == [True]
    assert camera_video.subscriptions == []
    assert robot_audio.subscriptions == []

    later_audio = Publication(rtc.TrackKind.KIND_AUDIO)
    room.events["track_published"](later_audio, user)
    assert later_audio.subscriptions == [True]

    await publisher.disconnect()


def test_rtsp_defaults_to_bounded_udp_first_input() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )

    arguments = publisher._video_input_args()

    assert arguments[:6] == [
        "-fflags", "+genpts+discardcorrupt+nobuffer",
        "-flags", "low_delay",
        "-thread_queue_size", "8",
    ]
    assert arguments[arguments.index("-use_wallclock_as_timestamps") + 1] == "1"
    assert arguments[arguments.index("-rtsp_transport") + 1] == "udp"
    assert arguments[arguments.index("-buffer_size") + 1] == "262144"
    assert arguments[arguments.index("-max_delay") + 1] == "100000"
    assert arguments[arguments.index("-reorder_queue_size") + 1] == "32"
    assert "-stimeout" not in arguments
    assert "-rw_timeout" not in arguments
    assert arguments[-2:] == ["-i", "rtsp://camera.local/live"]


def test_rtsp_path_is_appended_when_source_only_contains_host() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local:554",
            simulator_rtsp_path="/cam/main?channel=1",
        )
    )

    arguments = publisher._video_input_args()

    assert arguments[-1] == "rtsp://camera.local:554/cam/main?channel=1"


def test_rtsp_tcp_transport_remains_configurable() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            rtsp_transport="tcp",
            rtsp_normalize=False,
        )
    )

    arguments = publisher._video_input_args()

    assert arguments[arguments.index("-rtsp_transport") + 1] == "tcp"
    assert arguments[arguments.index("-max_delay") + 1] == "0"
    assert arguments[arguments.index("-reorder_queue_size") + 1] == "0"


@pytest.mark.parametrize(
    ("video_bitrate", "pacer_bitrate"),
    [(2_500_000, "8000000"), (12_000_000, "12000000")],
)
def test_encoded_publisher_paces_large_frames_without_reencoding(
    video_bitrate: int,
    pacer_bitrate: str,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(video_bitrate=video_bitrate)
    )

    command = publisher._publisher_command("secret", ["h264parse"])

    assert command[command.index("--pacer-bitrate") + 1] == pacer_bitrate
    assert command[command.index("--pacer-max-latency-ms") + 1] == "30"
    assert command[command.index("--video-fps") + 1] == "25"
    assert command[-2:] == ["--", "h264parse"]


@pytest.mark.parametrize(
    ("fps", "latency_ms"),
    [(10, 35), (25, 30), (30, 25), (60, 12)],
)
def test_pacer_deadline_stays_below_the_next_frame(
    fps: int,
    latency_ms: int,
) -> None:
    assert video_pacer_max_latency_ms(fps) == latency_ms
    assert latency_ms < 1000 / fps


def test_direct_usb_camera_pacer_uses_capture_fps() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_camera_fps=60,
            video_fps=25,
        )
    )

    command = publisher._publisher_command("secret", ["h264parse"])

    assert command[command.index("--pacer-max-latency-ms") + 1] == "12"
    assert command[command.index("--video-fps") + 1] == "60"


def test_encoded_publisher_keeps_fractional_fps_fallback_exact() -> None:
    publisher = MediaPublisher(SimulatorConfig())
    plan = EncodedVideoPlan(
        "bridge",
        "h264",
        "copy",
        output_fps=Fraction(30000, 1001),
    )

    command = publisher._publisher_command("secret", ["h264parse"], plan)

    assert command[command.index("--video-fps") + 1] == "30000/1001"


def test_rtcp_keyframe_request_waits_before_reconnecting_source() -> None:
    patch = (
        Path(__file__).resolve().parents[1]
        / "gstreamer-publisher-duration.patch"
    ).read_text()

    assert "case *rtcp.PictureLossIndication:" in patch
    assert "case *rtcp.FullIntraRequest:" in patch
    assert "keyframeRequestPending.CompareAndSwap(false, true)" in patch
    assert "action=request-upstream" in patch
    assert "gst.EventTypeCustomUpstream" in patch
    assert "GstForceKeyUnit" in patch
    assert "keyframesPublished.Load() > keyframesAtRequest" in patch
    assert "video-keyframe-timeout type=%s action=reconnect" in patch
    assert "video-keyframe-request type=%s action=reconnect" not in patch


def test_usb_camera_uses_v4l2_device_settings() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video2",
            simulator_camera_format="mjpeg",
            simulator_camera_width=1280,
            simulator_camera_height=720,
            simulator_camera_fps=30,
        )
    )

    arguments = publisher._video_input_args()

    assert arguments == [
        "-fflags", "+discardcorrupt+nobuffer",
        "-flags", "low_delay",
        "-thread_queue_size", "1",
        "-use_wallclock_as_timestamps", "1",
        "-f", "v4l2",
        "-framerate", "30",
        "-video_size", "1280x720",
        "-input_format", "mjpeg",
        "-i", "/dev/video2",
    ]


def test_v4l2_profile_parser_uses_negotiated_fractional_rate() -> None:
    profile = parse_v4l2_capture_profile(
        """
Format Video Capture:
    Width/Height      : 1920/1080
    Pixel Format      : 'MJPG' (Motion-JPEG, compressed)
Streaming Parameters Video Capture:
    Frames per second: 29.970 (30000/1001)
""",
        device="/dev/video2",
        fallback_format="",
        fallback_width=1280,
        fallback_height=720,
        fallback_fps=Fraction(25, 1),
    )

    assert profile == CameraCaptureProfile(
        device="/dev/video2",
        input_format="mjpeg",
        width=1920,
        height=1080,
        fps=Fraction(30000, 1001),
    )


def test_camera_profile_reads_back_what_v4l2_actually_accepted(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []

    def run(command: list[str], **_kwargs) -> CompletedProcess[str]:
        commands.append(command)
        if "--list-formats-ext" in command:
            return CompletedProcess(
                command,
                0,
                "[0]: 'MJPG' (Motion-JPEG, compressed)\n",
                "",
            )
        if "--get-parm" in command:
            return CompletedProcess(
                command,
                0,
                """
Format Video Capture:
    Width/Height      : 1920/1080
    Pixel Format      : 'MJPG' (Motion-JPEG, compressed)
Streaming Parameters Video Capture:
    Frames per second: 30.000 (30/1)
""",
                "",
            )
        return CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(
        "simulator.media.shutil.which",
        lambda binary: f"/usr/bin/{binary}" if binary == "v4l2-ctl" else None,
    )
    monkeypatch.setattr("simulator.media.subprocess.run", run)
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video2",
            simulator_camera_fps=25,
        )
    )

    profile = publisher._camera_capture_profile_for_source()

    assert profile.fps == Fraction(30, 1)
    assert profile.input_format == "mjpeg"
    set_command = next(command for command in commands if "--set-parm" in command)
    assert set_command[set_command.index("--set-parm") + 1] == "25"
    arguments = publisher._video_input_args()
    assert arguments[arguments.index("-framerate") + 1] == "30"
    assert arguments[arguments.index("-input_format") + 1] == "mjpeg"


def test_camera_disables_dynamic_exposure_frame_rate_when_supported(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []

    def run(command: list[str], **_kwargs) -> CompletedProcess[str]:
        commands.append(command)
        if "--list-ctrls" in command:
            return CompletedProcess(
                command,
                0,
                " exposure_dynamic_framerate (bool): default=0 value=1\n",
                "",
            )
        return CompletedProcess(command, 0, "", "")

    monkeypatch.setattr("simulator.media.subprocess.run", run)

    MediaPublisher._stabilize_camera_frame_timing(
        "/usr/bin/v4l2-ctl", "/dev/video2"
    )

    assert any(
        "--set-ctrl=exposure_dynamic_framerate=0" in command
        for command in commands
    )


def test_bluetooth_microphone_uses_pipewire_pulse_source(monkeypatch) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_source", lambda source: None
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_source_type="device",
            simulator_audio_source=(
                "pulse:bluez_input.41_42_FF_68_01_59.headset-head-unit"
            ),
        )
    )

    assert publisher._audio_input_args() == [
        "-f",
        "pulse",
        "-sample_rate",
        "48000",
        "-channels",
        "1",
        "-fragment_size",
        "1920",
        "-i",
        "bluez_input.41_42_FF_68_01_59.headset-head-unit",
    ]


def test_bluetooth_microphone_uses_low_latency_native_capture(monkeypatch) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_source", lambda source: None
    )
    monkeypatch.setattr(
        "simulator.media.shutil.which", lambda binary: f"/usr/bin/{binary}"
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_source_type="device",
            simulator_audio_source="pulse:bluez_input.test.headset-head-unit",
        )
    )

    command = publisher._audio_capture_command(48_000, 1)

    assert command[0] == "pacat"
    assert "--record" in command
    assert "--device=bluez_input.test.headset-head-unit" in command
    assert "--latency-msec=20" in command
    assert "--process-time-msec=10" in command


def test_alsa_microphone_uses_bounded_native_buffer(monkeypatch) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_source", lambda source: None
    )
    monkeypatch.setattr(
        "simulator.media.shutil.which", lambda binary: f"/usr/bin/{binary}"
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_source_type="device",
            simulator_audio_source="plughw:CARD=Mic,DEV=0",
        )
    )

    command = publisher._audio_capture_command(48_000, 1)

    assert command[0] == "arecord"
    assert "--device=plughw:CARD=Mic,DEV=0" in command
    assert "--buffer-time=40000" in command
    assert "--period-time=10000" in command


def test_pulse_speaker_uses_selected_device_with_low_latency(monkeypatch) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_output", lambda output: None
    )
    monkeypatch.setattr(
        "simulator.media.audio_output_args",
        lambda output: ["-f", "pulse", output.removeprefix("pulse:")],
    )
    monkeypatch.setattr(
        "simulator.media.shutil.which", lambda binary: f"/usr/bin/{binary}"
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_output_type="device",
            simulator_audio_output="pulse:alsa_output.usb-speaker",
        )
    )

    command = publisher._audio_output_command()

    assert command[0] == "pacat"
    assert "--playback" in command
    assert "--device=alsa_output.usb-speaker" in command
    assert "--latency-msec=40" in command
    assert "--process-time-msec=10" in command


def test_alsa_speaker_uses_bounded_native_buffer(monkeypatch) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_output", lambda output: None
    )
    monkeypatch.setattr(
        "simulator.media.audio_output_args",
        lambda output: ["-f", "alsa", output],
    )
    monkeypatch.setattr(
        "simulator.media.shutil.which", lambda binary: f"/usr/bin/{binary}"
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_output_type="device",
            simulator_audio_output="plughw:CARD=Speaker,DEV=0",
        )
    )

    command = publisher._audio_output_command()

    assert command[0] == "aplay"
    assert "--device=plughw:CARD=Speaker,DEV=0" in command
    assert "--buffer-time=40000" in command
    assert "--period-time=10000" in command


def test_speaker_queue_discards_old_audio_before_it_adds_latency() -> None:
    frames: asyncio.Queue[bytes] = asyncio.Queue(
        maxsize=AUDIO_PLAYBACK_BUFFER_FRAMES
    )
    for index in range(AUDIO_PLAYBACK_BUFFER_FRAMES + 3):
        MediaPublisher._queue_latest_audio_frame(frames, bytes([index]))

    queued = [frames.get_nowait() for _ in range(frames.qsize())]

    assert len(queued) == AUDIO_PLAYBACK_BUFFER_FRAMES
    assert queued[0] == bytes([3])
    assert queued[-1] == bytes([AUDIO_PLAYBACK_BUFFER_FRAMES + 2])


@pytest.mark.asyncio
async def test_speaker_playback_writes_pcm_to_native_player(monkeypatch) -> None:
    writes: list[bytes] = []

    class Stdin:
        def write(self, data: bytes) -> None:
            writes.append(data)

        async def drain(self) -> None:
            return None

        def close(self) -> None:
            return None

    class Process:
        def __init__(self) -> None:
            self.stdin = Stdin()
            self.stdout = None
            self.stderr = None
            self.returncode = None

        def terminate(self) -> None:
            self.returncode = 0

        def kill(self) -> None:
            self.returncode = -9

        async def wait(self) -> int:
            return int(self.returncode or 0)

    async def create_process(*args, **kwargs):
        return Process()

    async def capture_output(_process) -> str:
        await asyncio.Future()
        return ""

    monkeypatch.setattr(asyncio, "create_subprocess_exec", create_process)
    monkeypatch.setattr(
        "simulator.media.prepare_audio_output", lambda output: None
    )
    monkeypatch.setattr(
        "simulator.media.audio_output_args",
        lambda output: ["-f", "alsa", output],
    )
    monkeypatch.setattr(
        MediaPublisher, "_capture_process_output", staticmethod(capture_output)
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_output_type="device",
            simulator_audio_output="plughw:CARD=Speaker,DEV=0",
        )
    )
    frames: asyncio.Queue[bytes] = asyncio.Queue(maxsize=2)
    task = asyncio.create_task(publisher._audio_playback_loop(frames))

    frames.put_nowait(b"pcm-frame")
    for _ in range(5):
        await asyncio.sleep(0)
        if writes:
            break

    assert writes == [b"pcm-frame"]

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


@pytest.mark.asyncio
async def test_video_jitter_buffer_discards_oldest_burst_frames() -> None:
    class Stream:
        def __init__(self) -> None:
            self.frames = iter([b"a", b"b", b"c", b"d", b"e"])

        async def readexactly(self, _frame_size: int) -> bytes:
            try:
                return next(self.frames)
            except StopIteration:
                await asyncio.Future()
                raise AssertionError("unreachable")

    publisher = MediaPublisher(SimulatorConfig())
    frames: asyncio.Queue[bytes | Exception] = asyncio.Queue(
        maxsize=VIDEO_JITTER_BUFFER_FRAMES
    )
    task = asyncio.create_task(
        publisher._read_video_frames(Stream(), 1, frames)  # type: ignore[arg-type]
    )
    for _ in range(6):
        await asyncio.sleep(0)

    assert [frames.get_nowait(), frames.get_nowait()] == [b"d", b"e"]

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


def test_latest_video_frame_discards_queued_latency() -> None:
    frames: asyncio.Queue[bytes | Exception] = asyncio.Queue()
    frames.put_nowait(b"newer")
    frames.put_nowait(b"newest")

    frame = MediaPublisher._latest_video_frame(b"old", frames)

    assert frame == b"newest"
    assert frames.empty()


def test_balanced_defaults_leave_headroom_for_control_traffic() -> None:
    config = SimulatorConfig(_env_file=None)

    assert config.video_width == 1280
    assert config.video_height == 720
    assert config.video_fps == 20
    assert config.video_bitrate == 2_500_000
    assert config.rtsp_normalize is False
    assert config.rtsp_auto_normalize is True


def test_video_pipe_can_buffer_multiple_full_hd_frames() -> None:
    frame_size = 1920 * 1080 * 3 // 2

    assert video_pipe_buffer_limit(frame_size) == frame_size * 4


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("25/1", Fraction(25, 1)),
        ("30000/1001", Fraction(30000, 1001)),
        ("0/0", None),
        ("N/A", None),
        (None, None),
    ],
)
def test_ffprobe_frame_rate_parser_is_bounded_and_exact(
    value: object,
    expected: Fraction | None,
) -> None:
    assert parse_ffprobe_fraction(value) == expected


@pytest.mark.parametrize(
    ("metadata_fps", "actual_fps"),
    [
        (Fraction(25, 1), Fraction(30, 1)),
        (Fraction(30, 1), Fraction(18, 1)),
    ],
)
def test_runtime_probe_rejects_metadata_that_disagrees_with_actual_cadence(
    metadata_fps: Fraction,
    actual_fps: Fraction,
) -> None:
    result = analyze_runtime_video_packets(
        runtime_packets(actual_fps), metadata_fps
    )

    assert result["measured_fps"] == actual_fps
    assert result["timing_reliable"] is False
    assert "metadata-fps-mismatch" in str(result["timing_reason"])


def test_runtime_probe_preserves_fractional_2997_fps() -> None:
    fps = Fraction(30000, 1001)

    result = analyze_runtime_video_packets(runtime_packets(fps, 90), fps)

    assert result["measured_fps"] == fps
    assert result["timing_reliable"] is True


@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (lambda packets: packets[8].update(pts_time=packets[7]["pts_time"]),
         "timestamp-repeated"),
        (lambda packets: packets[8].update(pts_time="0.010000000"),
         "timestamp-backwards"),
    ],
)
def test_runtime_probe_rejects_repeated_or_backward_pts(mutate, reason: str) -> None:
    packets = runtime_packets(Fraction(25, 1))
    mutate(packets)

    result = analyze_runtime_video_packets(packets, Fraction(25, 1))

    assert result["timing_reliable"] is False
    assert reason in str(result["timing_reason"])


def test_runtime_probe_detects_bursty_cadence() -> None:
    packets = runtime_packets(Fraction(25, 1))
    timestamps = [0.0]
    for index in range(1, len(packets)):
        timestamps.append(timestamps[-1] + (0.004 if index % 3 else 0.112))
    for packet, timestamp in zip(packets, timestamps, strict=True):
        packet["pts_time"] = f"{timestamp:.6f}"
        packet["dts_time"] = f"{timestamp:.6f}"

    result = analyze_runtime_video_packets(packets, Fraction(25, 1))

    assert result["cadence_bursty"] is True
    assert "cadence-bursty" in str(result["timing_reason"])


def test_runtime_probe_measures_dts_cadence_while_reporting_b_frame_pts_reorder() -> None:
    packets = runtime_packets(Fraction(25, 1))
    packets[3]["pts_time"], packets[4]["pts_time"] = (
        packets[4]["pts_time"], packets[3]["pts_time"]
    )

    result = analyze_runtime_video_packets(packets, Fraction(25, 1))

    assert result["measured_fps"] == Fraction(25, 1)
    assert result["backward_timestamps"] == 1


def test_video_dimensions_never_upscale_small_sources() -> None:
    assert bounded_video_dimensions(1280, 720, 1920, 1080) == (1280, 720)
    assert bounded_video_dimensions(3840, 2160, 1920, 1080) == (1920, 1080)
    assert bounded_video_dimensions(1280, 960, 1280, 720) == (960, 720)


def test_rtsp_probe_prefers_average_rate_over_codec_tick_rate(monkeypatch) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    payload = json.dumps({
      "streams": [{
        "codec_name": "h264",
        "profile": "Main",
        "width": 1920,
        "height": 1080,
        "pix_fmt": "yuv420p",
        "r_frame_rate": "50/1",
        "avg_frame_rate": "25/1",
        "bit_rate": "8000000",
        "has_b_frames": 0,
      }],
      "packets": runtime_packets(Fraction(25, 1)),
    })
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, payload, ""),
    )

    probe = publisher._probe_video_source()

    assert probe.fps == Fraction(25, 1)
    assert probe.timing_reliable is True
    assert probe.bitrate == 8_000_000
    assert probe.passthrough_safe is True


def test_rtsp_probe_marks_camera_128_style_timing_unreliable(monkeypatch) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    payload = json.dumps({
      "streams": [{
        "codec_name": "h264",
        "profile": "Main",
        "width": 1920,
        "height": 1080,
        "pix_fmt": "yuv420p",
        "r_frame_rate": "100/1",
        "avg_frame_rate": "0/0",
        "has_b_frames": 0,
      }],
      "packets": runtime_packets(Fraction(25, 1)),
    })
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, payload, ""),
    )

    probe = publisher._probe_video_source()

    assert probe.fps is None
    assert probe.timing_reliable is False
    assert "timing-out-of-range" in probe.timing_reason


def test_video_publish_options_prioritize_smooth_motion() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            video_fps=25,
            video_bitrate=8_000_000,
            video_encoder="vaapi",
        )
    )

    options = publisher._video_publish_options()

    assert (
        options.degradation_preference
        == rtc.DegradationPreference.MAINTAIN_FRAMERATE
    )
    assert (
        options.video_encoder
        == proto_room.VideoEncoderBackend.ENCODER_BACKEND_VAAPI
    )
    assert options.video_encoding.max_framerate == 25
    assert options.video_encoding.max_bitrate == 8_000_000


def test_unknown_video_encoder_is_rejected() -> None:
    publisher = MediaPublisher(SimulatorConfig(video_encoder="not-an-encoder"))

    with pytest.raises(ValueError, match="unsupported VIDEO_ENCODER"):
        publisher._video_publish_options()


def test_auto_pipeline_uses_encoded_video_for_real_sources_only() -> None:
    camera = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
        )
    )
    generated = MediaPublisher(SimulatorConfig(simulator_media_source_type="test"))

    assert camera._video_pipeline() == "encoded"
    assert generated._video_pipeline() == "raw"


def test_encoded_video_plan_is_reused_while_source_is_unchanged(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )
    expected = EncodedVideoPlan("direct", "h264", "copy")
    builds = 0

    def build() -> EncodedVideoPlan:
        nonlocal builds
        builds += 1
        return expected

    monkeypatch.setattr(publisher, "_build_encoded_video_plan", build)

    assert publisher._prepare_encoded_video() is expected
    assert publisher._prepare_encoded_video() is expected
    assert builds == 1

    publisher.config.simulator_media_source = "rtsp://camera.local/other"
    assert publisher._prepare_encoded_video() is expected
    assert builds == 2


def test_rtsp_h264_direct_pipeline_never_decodes_or_encodes() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            rtsp_transport="tcp",
        )
    )

    pipeline = publisher._direct_h264_pipeline()
    command = " ".join(pipeline)

    assert "rtspsrc" in pipeline
    assert "rtph264depay" in pipeline
    assert "h264parse" in pipeline
    assert pipeline[-3:] == [
        "h264parse",
        "disable-passthrough=true",
        "config-interval=1",
    ]
    assert "latency=80" in pipeline
    assert "buffer-mode=none" in pipeline
    assert "max-ts-offset=0" in pipeline
    assert "drop-on-latency=true" in pipeline
    assert "decode" not in command
    assert all(
        encoder not in command
        for encoder in ("x264enc", "mpph264enc", "h264_vaapi", "h264_nvenc")
    )


def test_rtsp_h264_passthrough_uses_direct_gstreamer_route(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            rtsp_normalize=False,
        )
    )
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec="h264",
            width=1920,
            height=1080,
            fps=Fraction(25, 1),
            pixel_format="yuv420p",
            profile="Main",
        ),
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()
    assert plan.mode == "direct"
    assert plan.encoder == "copy"
    assert plan.output_fps == Fraction(25, 1)
    assert "rtspsrc" in publisher._direct_h264_pipeline()


def test_rtsp_h264_normalization_remains_available_as_opt_in(monkeypatch) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            rtsp_normalize=True,
        )
    )
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec="h264",
            width=1920,
            height=1080,
            fps=Fraction(25, 1),
            pixel_format="yuv420p",
            profile="Main",
        ),
    )
    monkeypatch.setattr(
        publisher, "_select_video_encoder", lambda: ("libx264", "ffmpeg")
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()
    command = publisher._encoded_ffmpeg_command(plan)
    video_filter = command[command.index("-vf") + 1]

    assert plan.mode == "bridge"
    assert plan.encoder == "libx264"
    assert "fps=fps=20:round=near:eof_action=pass" in video_filter
    assert "setpts=N/(20*TB)" in video_filter
    assert "scale=1280:720" in video_filter


def test_clean_rtsp_h264_keeps_real_30fps_and_avoids_reencoding(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            video_fps=25,
            video_bitrate=6_000_000,
        )
    )
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec="h264",
            width=1920,
            height=1080,
            fps=Fraction(30, 1),
            bitrate=8_000_000,
            pixel_format="yuv420p",
            profile="High",
        ),
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()
    command = publisher._publisher_command("secret", ["h264parse"], plan)

    assert plan.encoder == "copy"
    assert plan.output_fps == Fraction(30, 1)
    assert command[command.index("--video-fps") + 1] == "30"
    assert command[command.index("--pacer-bitrate") + 1] == "10800000"


def test_auto_normalize_transcodes_only_unreliable_h264_timing(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            video_fps=25,
        )
    )
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec="h264",
            width=1920,
            height=1080,
            pixel_format="yuv420p",
            profile="Main",
            timing_reliable=False,
            timing_reason="timing-out-of-range",
        ),
    )
    monkeypatch.setattr(
        publisher, "_select_video_encoder", lambda: ("libx264", "ffmpeg")
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()
    command = publisher._encoded_ffmpeg_command(plan)

    assert plan.encoder == "libx264"
    assert plan.output_fps == Fraction(20, 1)
    video_filter = command[command.index("-vf") + 1]
    assert "setpts=(RTCTIME-RTCSTART)/(TB*1000000)" in video_filter
    assert "fps=fps=20:round=near:eof_action=pass" in video_filter
    assert "setpts=N/(20*TB)" in video_filter


@pytest.mark.parametrize("codec", ["hevc", "mjpeg", "vp8"])
def test_non_h264_rtsp_is_converted_to_browser_h264(
    monkeypatch,
    codec: str,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec=codec,
            width=1280,
            height=720,
            fps=Fraction(25, 1),
        ),
    )
    monkeypatch.setattr(
        publisher, "_select_video_encoder", lambda: ("libx264", "ffmpeg")
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()
    command = publisher._encoded_ffmpeg_command(plan)

    assert plan.encoder == "libx264"
    assert plan.output_width == 1280
    assert plan.output_height == 720
    assert "scale=1280:720:flags=bicubic" in command[
        command.index("-vf") + 1
    ]


def test_x264_bridge_keeps_quality_stable_without_frame_lookahead() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(video_bitrate=2_500_000, video_fps=20)
    )

    arguments = publisher._video_encoder_args("libx264")

    assert arguments[arguments.index("-preset") + 1] == "superfast"
    assert arguments[arguments.index("-bufsize") + 1] == "1250000"
    assert arguments[arguments.index("-qcomp") + 1] == "0.75"
    assert arguments[arguments.index("-x264-params") + 1] == (
        "scenecut=0:force-cfr=1"
    )
    assert arguments[arguments.index("-bf") + 1] == "0"
    assert "-rc-lookahead" not in arguments


def test_h264_over_profile_limits_is_transcoded_to_selected_profile(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            video_width=1280,
            video_height=720,
            video_bitrate=2_500_000,
        )
    )
    probe = SourceVideoProbe(
        codec="h264",
        width=3840,
        height=2160,
        fps=Fraction(30, 1),
        bitrate=12_000_000,
        pixel_format="yuv420p",
        profile="High",
    )

    reasons = publisher._video_transcode_reasons(probe)

    assert "profile-resolution" in reasons
    assert "profile-bitrate" in reasons
    assert bounded_video_dimensions(3840, 2160, 1280, 720) == (1280, 720)


def test_overloaded_transcode_profile_reduces_resolution_and_fps(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            video_fps=25,
        )
    )
    publisher._video_degrade_level = 1
    monkeypatch.setattr(
        publisher,
        "_probe_video_source",
        lambda: SourceVideoProbe(
            codec="hevc",
            width=1920,
            height=1080,
            fps=Fraction(25, 1),
            measured_fps=Fraction(25, 1),
        ),
    )
    monkeypatch.setattr(
        publisher, "_select_video_encoder", lambda: ("libx264", "ffmpeg")
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()

    assert (plan.output_width, plan.output_height) == (960, 540)
    assert plan.output_fps == Fraction(16, 1)


def test_h264_with_b_frames_is_normalized_instead_of_ignored() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )

    reasons = publisher._video_transcode_reasons(
        SourceVideoProbe(
            codec="h264",
            fps=Fraction(25, 1),
            measured_fps=Fraction(25, 1),
            has_b_frames=True,
        )
    )

    assert "b-frames-require-normalization" in reasons


def test_usb_h264_direct_pipeline_keeps_requested_capture_profile() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video2",
            simulator_camera_format="h264",
            simulator_camera_width=1280,
            simulator_camera_height=720,
            simulator_camera_fps=30,
        )
    )

    pipeline = publisher._direct_h264_pipeline()
    command = " ".join(pipeline)

    assert "v4l2src" in pipeline
    assert "device=\"/dev/video2\"" in pipeline
    assert "video/x-h264,width=1280,height=720,framerate=30/1" in pipeline
    assert "h264parse" in pipeline
    assert pipeline[-3:] == [
        "h264parse",
        "disable-passthrough=true",
        "config-interval=1",
    ]
    assert "decode" not in command
    assert "enc" not in command


@pytest.mark.parametrize(
    ("capture_fps", "video_fps", "expected_fps"),
    [
        (Fraction(30, 1), 25, Fraction(25, 1)),
        (Fraction(60, 1), 30, Fraction(30, 1)),
        (Fraction(15, 1), 25, Fraction(15, 1)),
        (Fraction(30000, 1001), 30, Fraction(30000, 1001)),
    ],
)
def test_camera_bridge_drops_surplus_frames_without_upsampling(
    monkeypatch,
    capture_fps: Fraction,
    video_fps: int,
    expected_fps: Fraction,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video2",
            simulator_camera_format="mjpeg",
            video_fps=video_fps,
        )
    )
    profile = CameraCaptureProfile(
        device="/dev/video2",
        input_format="mjpeg",
        width=1920,
        height=1080,
        fps=capture_fps,
    )
    monkeypatch.setattr(
        publisher, "_camera_capture_profile_for_source", lambda: profile
    )

    plan = EncodedVideoPlan("bridge", "mjpeg", "libx264")
    command = publisher._encoded_ffmpeg_command(plan)
    video_filter = command[command.index("-vf") + 1]

    expected_text = (
        str(expected_fps.numerator)
        if expected_fps.denominator == 1
        else f"{expected_fps.numerator}/{expected_fps.denominator}"
    )
    assert "setpts=PTS-STARTPTS" in video_filter
    assert f"fps=fps={expected_text}:round=down:eof_action=pass" in video_filter
    assert command[command.index("-framerate") + 1] == (
        str(capture_fps.numerator)
        if capture_fps.denominator == 1
        else f"{capture_fps.numerator}/{capture_fps.denominator}"
    )
    assert command[command.index("-fps_mode") + 1] == "passthrough"
    assert command[command.index("-g") + 1] == str(round(float(expected_fps)))
    assert video_filter.endswith("format=yuv420p")


def test_v4l2m2m_camera_bridge_uses_standard_nv12_input() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
            simulator_camera_format="mjpeg",
        )
    )
    plan = EncodedVideoPlan("bridge", "mjpeg", "h264_v4l2m2m")

    command = publisher._encoded_ffmpeg_command(plan)

    assert command[command.index("-vf") + 1].endswith("format=nv12")


@pytest.mark.parametrize(
    ("capture_fps", "video_fps", "expected_mode", "expected_encoder"),
    [
        (Fraction(30, 1), 30, "direct", "copy"),
        (Fraction(60, 1), 30, "bridge", "libx264"),
    ],
)
def test_h264_camera_only_uses_passthrough_when_no_frame_drop_is_needed(
    monkeypatch,
    capture_fps: Fraction,
    video_fps: int,
    expected_mode: str,
    expected_encoder: str,
) -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video2",
            simulator_camera_format="h264",
            video_fps=video_fps,
        )
    )
    profile = CameraCaptureProfile(
        device="/dev/video2",
        input_format="h264",
        width=1920,
        height=1080,
        fps=capture_fps,
    )
    monkeypatch.setattr(
        publisher, "_camera_capture_profile_for_source", lambda: profile
    )
    monkeypatch.setattr(publisher, "_probe_source_codec", lambda: "h264")
    monkeypatch.setattr(
        publisher, "_select_video_encoder", lambda: ("libx264", "ffmpeg")
    )
    monkeypatch.setattr("simulator.media.shutil.which", lambda _binary: "/usr/bin/tool")
    monkeypatch.setattr(
        "simulator.media.subprocess.run",
        lambda command, **_kwargs: CompletedProcess(command, 0, "", ""),
    )

    plan = publisher._build_encoded_video_plan()

    assert plan.mode == expected_mode
    assert plan.encoder == expected_encoder


def test_h264_file_bridge_copies_codec_without_raw_video_pipe() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="file",
            simulator_media_source="/media/camera.mp4",
        )
    )
    plan = EncodedVideoPlan("bridge", "h264", "copy")

    command = publisher._encoded_ffmpeg_command(plan)

    assert command[0] == "ffmpeg"
    assert command[command.index("-c:v") + 1] == "copy"
    assert "h264_mp4toannexb" in command
    assert "-vf" not in command
    assert "rawvideo" not in command
    assert "mpegts" not in command
    assert command[command.index("-f") + 1] == "h264"
    assert command[-1] == "pipe:1"


def test_rkmpp_bridge_uses_arm_hardware_encoder() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
            simulator_camera_format="mjpeg",
            video_encoder="rkmpp",
            video_ffmpeg_binary="/opt/ffmpeg-rk/bin/ffmpeg",
        )
    )
    plan = EncodedVideoPlan(
        "bridge",
        "mjpeg",
        "h264_rkmpp",
        "/opt/ffmpeg-rk/bin/ffmpeg",
    )

    command = publisher._encoded_ffmpeg_command(plan)

    assert command[0] == "/opt/ffmpeg-rk/bin/ffmpeg"
    assert command[command.index("-c:v") + 1] == "h264_rkmpp"
    assert "format=nv12" in command[command.index("-vf") + 1]
    assert command[-1] == "pipe:1"


def test_vaapi_bridge_uses_cqp_for_older_intel_drivers() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="camera",
            simulator_media_source="/dev/video0",
            simulator_camera_format="mjpeg",
            video_encoder="vaapi",
        )
    )
    plan = EncodedVideoPlan("bridge", "mjpeg", "h264_vaapi")
    publisher._vaapi_rate_control = "cqp"
    publisher._vaapi_low_power = True

    command = publisher._encoded_ffmpeg_command(plan)

    assert command[command.index("-c:v") + 1] == "h264_vaapi"
    assert command[command.index("-rc_mode") + 1] == "CQP"
    assert command[command.index("-qp") + 1] == "27"
    assert command[command.index("-async_depth") + 1] == "1"
    assert command[command.index("-low_power") + 1] == "1"
    assert command[command.index("-quality") + 1] == "7"
    assert "setpts=PTS-STARTPTS" in command[command.index("-vf") + 1]
    assert "fps=fps=25:round=down:eof_action=pass" in command[
        command.index("-vf") + 1
    ]
    assert "-b:v" not in command
    assert "-maxrate" not in command


def test_vaapi_cbr_bridge_also_keeps_one_frame_in_flight() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(video_encoder="vaapi", video_bitrate=4_000_000)
    )
    publisher._vaapi_rate_control = "cbr"

    arguments = publisher._video_encoder_args("h264_vaapi")

    assert arguments[arguments.index("-rc_mode") + 1] == "CBR"
    assert arguments[arguments.index("-async_depth") + 1] == "1"
    assert arguments[arguments.index("-bufsize") + 1] == "1000000"


def test_vaapi_regular_entrypoint_remains_available() -> None:
    publisher = MediaPublisher(SimulatorConfig(video_encoder="vaapi"))
    publisher._vaapi_rate_control = "cqp"
    publisher._vaapi_low_power = False

    arguments = publisher._video_encoder_args("h264_vaapi")

    assert "-low_power" not in arguments
    assert "-quality" not in arguments


def test_bridge_pipeline_uses_an_os_pipe_instead_of_tcp() -> None:
    pipeline = MediaPublisher(SimulatorConfig(video_fps=25))._bridge_h264_pipeline()

    assert pipeline[:2] == ["fdsrc", "fd=0"]
    assert "tcpclientsrc" not in pipeline
    assert "h264parse" in pipeline
    assert "leaky=downstream" not in pipeline
    assert "tsdemux" not in pipeline
    assert "max-size-buffers=4" in pipeline
    assert pipeline[-3:] == [
        "h264parse",
        "disable-passthrough=true",
        "config-interval=1",
    ]


def test_rtsp_auto_transport_tries_udp_with_bounded_tcp_fallback() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
            rtsp_transport="auto",
        )
    )

    pipeline = publisher._direct_h264_pipeline()

    assert "protocols=udp+tcp" in pipeline
    assert "timeout=1500000" in pipeline
    assert "tcp-timeout=3000000" in pipeline


def test_full_duplex_audio_pipeline_feeds_render_audio_to_aec_probe(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        "simulator.media.prepare_audio_source", lambda _source: None
    )
    monkeypatch.setattr(
        "simulator.media.prepare_audio_output", lambda _output: None
    )
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_audio_source_type="device",
            simulator_audio_source="plughw:CARD=Mic,DEV=0",
            simulator_audio_output_type="device",
            simulator_audio_output="plughw:CARD=Speaker,DEV=0",
        )
    )

    command = publisher._audio_duplex_command()
    pipeline = " ".join(command)

    assert command[:2] == ["gst-launch-1.0", "-q"]
    assert "webrtcechoprobe name=robot_echo_reference" in pipeline
    assert "webrtcdsp probe=robot_echo_reference" in pipeline
    assert pipeline.index("webrtcechoprobe") < pipeline.index("alsasink")
    assert "echo-cancel=true" in pipeline
    assert "noise-suppression=true" in pipeline
    assert "gain-control=true" in pipeline
    assert "high-pass-filter=true" in pipeline
    assert "rate=48000" in pipeline
    assert AUDIO_DEVICE_BUFFER_MS == 40
    assert AUDIO_DEVICE_PERIOD_MS == 10


def test_auto_encoder_uses_first_backend_that_passes_real_probe(
    monkeypatch,
) -> None:
    publisher = MediaPublisher(SimulatorConfig(video_encoder="auto"))
    attempted: list[str] = []

    def probe(_binary: str, encoder: str) -> tuple[bool, str]:
        attempted.append(encoder)
        return (encoder == "h264_vaapi", "unavailable")

    monkeypatch.setattr(publisher, "_probe_video_encoder", probe)

    assert publisher._select_video_encoder() == ("h264_vaapi", "ffmpeg")
    assert attempted == ["h264_rkmpp", "h264_vaapi"]


@pytest.mark.asyncio
async def test_stubborn_ffmpeg_process_is_killed() -> None:
    class Process:
        returncode: int | None = None
        terminated = False
        killed = False

        def terminate(self) -> None:
            self.terminated = True

        def kill(self) -> None:
            self.killed = True
            self.returncode = -9

        async def wait(self) -> int:
            if self.killed:
                return -9
            await asyncio.Future()
            return 0

    process = Process()
    publisher = MediaPublisher(SimulatorConfig())

    await publisher._stop_process(process)  # type: ignore[arg-type]

    assert process.terminated
    assert process.killed


def test_publisher_progress_updates_only_when_counters_advance() -> None:
    progress = EncodedPipelineProgress(started_at=time.monotonic())

    MediaPublisher._update_encoded_progress(
        progress, "video-progress received=12 published=11\n"
    )
    first_progress_at = progress.last_progress_at
    MediaPublisher._update_encoded_progress(
        progress, "video-progress received=12 published=11\n"
    )

    assert progress.received == 12
    assert progress.published == 11
    assert progress.last_progress_at == first_progress_at


@pytest.mark.asyncio
async def test_encoded_watchdog_detects_process_that_is_alive_but_stalled(
    monkeypatch,
) -> None:
    class Process:
        returncode = None

    monkeypatch.setattr(
        "simulator.media.ENCODED_VIDEO_STALL_TIMEOUT_SECONDS", 0.01
    )
    progress = EncodedPipelineProgress(
        started_at=time.monotonic(),
        received=4,
        published=4,
        last_received_at=time.monotonic() - 1,
        last_published_at=time.monotonic() - 1,
    )
    publisher = MediaPublisher(SimulatorConfig())

    with pytest.raises(RuntimeError, match="made no progress"):
        await asyncio.wait_for(
            publisher._watch_encoded_video_progress(
                progress,
                [Process()],  # type: ignore[list-item]
                EncodedVideoPlan("bridge", "h264", "copy"),
            ),
            timeout=1,
        )


@pytest.mark.asyncio
async def test_encoded_watchdog_restarts_when_encoder_cannot_keep_realtime_rate(
    monkeypatch,
) -> None:
    class Process:
        returncode = None

    monkeypatch.setattr(
        "simulator.media.ENCODED_VIDEO_RATE_WINDOW_SECONDS", 0.1
    )
    monkeypatch.setattr(
        "simulator.media.ENCODED_VIDEO_STALL_TIMEOUT_SECONDS", 1.0
    )
    now = time.monotonic()
    progress = EncodedPipelineProgress(
        started_at=now,
        received=1,
        published=1,
        last_received_at=now,
        last_published_at=now,
    )
    publisher = MediaPublisher(SimulatorConfig(video_fps=25))

    async def advance_one_frame() -> None:
        await asyncio.sleep(0.6)
        MediaPublisher._update_encoded_progress(
            progress, "video-progress received=2 published=2\n"
        )

    update_task = asyncio.create_task(advance_one_frame())
    with pytest.raises(RuntimeError, match="below realtime rate"):
        await asyncio.wait_for(
            publisher._watch_encoded_video_progress(
                progress,
                [Process()],  # type: ignore[list-item]
                EncodedVideoPlan(
                    "bridge",
                    "hevc",
                    "libx264",
                    output_fps=Fraction(25, 1),
                ),
            ),
            timeout=2,
        )
    await update_task


def test_rtsp_credentials_are_removed_from_media_diagnostics() -> None:
    source = "rtsp://admin:super-secret@camera.local/live"
    detail = f"failed to open {source} and rtsp://other:password@backup/live"

    redacted = redact_media_source(detail, source)

    assert "super-secret" not in redacted
    assert "password" not in redacted
    assert "<media-source>" in redacted
