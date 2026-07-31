import asyncio
from fractions import Fraction
from subprocess import CompletedProcess

import pytest
from livekit import rtc
from livekit.rtc._proto import room_pb2 as proto_room

from simulator.config import SimulatorConfig
from simulator.media import (
    AUDIO_PLAYBACK_BUFFER_FRAMES,
    CameraCaptureProfile,
    EncodedVideoPlan,
    MediaPublisher,
    VIDEO_JITTER_BUFFER_FRAMES,
    parse_v4l2_capture_profile,
    video_pacer_max_latency_ms,
    video_pipe_buffer_limit,
)


def test_rtsp_defaults_to_low_jitter_udp_input() -> None:
    publisher = MediaPublisher(
        SimulatorConfig(
            simulator_media_source_type="rtsp",
            simulator_media_source="rtsp://camera.local/live",
        )
    )

    arguments = publisher._video_input_args()

    assert arguments[:8] == [
        "-fflags", "+genpts+discardcorrupt+nobuffer",
        "-flags", "low_delay",
        "-thread_queue_size", "64",
        "-rtsp_transport", "udp",
    ]
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
    assert "--buffer-time=60000" in command
    assert "--period-time=20000" in command


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
    assert "--latency-msec=60" in command
    assert "--process-time-msec=20" in command


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
    assert "--buffer-time=60000" in command
    assert "--period-time=20000" in command


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


def test_full_hd_defaults_leave_headroom_for_rtsp_reencoding() -> None:
    config = SimulatorConfig()

    assert config.video_width == 1920
    assert config.video_height == 1080
    assert config.video_bitrate == 8_000_000


def test_video_pipe_can_buffer_multiple_full_hd_frames() -> None:
    frame_size = 1920 * 1080 * 3 // 2

    assert video_pipe_buffer_limit(frame_size) == frame_size * 4


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
    assert "drop-on-latency=false" in pipeline
    assert "decode" not in command
    assert all(
        encoder not in command
        for encoder in ("x264enc", "mpph264enc", "h264_vaapi", "h264_nvenc")
    )


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
    assert "mpegts" in command
    assert command[command.index("-flush_packets") + 1] == "1"
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
    assert command[command.index("-quality") + 1] == "8"
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
    assert "latency=0" in pipeline
    assert "max-size-buffers=1" in pipeline
    assert pipeline[-3:] == [
        "h264parse",
        "disable-passthrough=true",
        "config-interval=1",
    ]


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
