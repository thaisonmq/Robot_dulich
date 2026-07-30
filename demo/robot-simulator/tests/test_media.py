import asyncio

import pytest
from livekit import rtc
from livekit.rtc._proto import room_pb2 as proto_room

from simulator.config import SimulatorConfig
from simulator.media import (
    EncodedVideoPlan,
    MediaPublisher,
    VIDEO_JITTER_BUFFER_FRAMES,
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
        "-f", "v4l2",
        "-framerate", "30",
        "-video_size", "1280x720",
        "-input_format", "mjpeg",
        "-i", "/dev/video2",
    ]


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
        "-i",
        "bluez_input.41_42_FF_68_01_59.headset-head-unit",
    ]


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
    assert "setpts=N/(25*TB)" in command[command.index("-vf") + 1]
    assert "fps=" not in command[command.index("-vf") + 1]
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
