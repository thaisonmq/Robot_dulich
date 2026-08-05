import asyncio
import json
import logging
import math
import os
import re
import shutil
import statistics
import struct
import subprocess
import threading
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace
from fractions import Fraction
from itertools import pairwise
from pathlib import Path
from urllib.parse import urlparse

from livekit import rtc
from livekit.rtc._proto import room_pb2 as proto_room

from simulator.config import SimulatorConfig
from simulator.media_devices import (
    audio_output_args,
    discover_v4l2_modes,
    prepare_audio_output,
    prepare_audio_source,
    select_v4l2_mode,
)

logger = logging.getLogger("simulator.media")
VIDEO_FRAME_TIMEOUT_SECONDS = 10
VIDEO_JITTER_BUFFER_FRAMES = 2
VIDEO_PIPE_BUFFER_FRAMES = 4
VIDEO_RUNTIME_PROBE_SECONDS = 2.5
VIDEO_RUNTIME_PROBE_TIMEOUT_SECONDS = 7
VIDEO_RUNTIME_PROBE_MIN_PACKETS = 6
VIDEO_FPS_MISMATCH_RATIO = 0.12
VIDEO_CADENCE_JITTER_RATIO = 0.35
ENCODED_VIDEO_STARTUP_TIMEOUT_SECONDS = 8.0
ENCODED_VIDEO_STALL_TIMEOUT_SECONDS = 3.0
ENCODED_VIDEO_HEALTHY_RESET_SECONDS = 10.0
ENCODED_VIDEO_RATE_WINDOW_SECONDS = 5.0
ENCODED_VIDEO_MIN_RATE_RATIO = 0.65
ENCODED_VIDEO_MAX_DEGRADE_LEVEL = 2
VIDEO_PACER_BASE_BITRATE = 8_000_000
VIDEO_PACER_MAX_BITRATE = 20_000_000
VIDEO_PACER_FRAME_FRACTION_MS = 750
VIDEO_PACER_MIN_LATENCY_MS = 12
VIDEO_PACER_MAX_LATENCY_MS = 35
UNRELIABLE_RTSP_FPS = 20
AUDIO_PLAYBACK_BUFFER_FRAMES = 2
AUDIO_DEVICE_BUFFER_MS = 40
AUDIO_DEVICE_PERIOD_MS = 10
AUDIO_PULSE_CAPTURE_LATENCY_MS = 20
AUDIO_PULSE_CAPTURE_PROCESS_MS = 10
AUDIO_LIVEKIT_QUEUE_MS = 40
AUDIO_FRAME_MS = 10
AUDIO_FRAME_SAMPLES = 480
AUDIO_FRAME_BYTES = AUDIO_FRAME_SAMPLES * 2
AUDIO_CAPTURE_STALL_TIMEOUT_SECONDS = 2.0
AUDIO_PLAYBACK_WRITE_TIMEOUT_SECONDS = 0.15

_V4L2_FOURCC_TO_FFMPEG = {
    "AVC1": "h264",
    "BGR3": "bgr24",
    "H264": "h264",
    "JPEG": "mjpeg",
    "MJPG": "mjpeg",
    "NV12": "nv12",
    "RGB3": "rgb24",
    "UYVY": "uyvy422",
    "YU12": "yuv420p",
    "YUYV": "yuyv422",
    "YV12": "yuv420p",
}
_FFMPEG_TO_V4L2_FOURCC = {
    "avc1": "AVC1",
    "bgr24": "BGR3",
    "h264": "H264",
    "jpeg": "JPEG",
    "mjpeg": "MJPG",
    "mjpg": "MJPG",
    "nv12": "NV12",
    "rgb24": "RGB3",
    "uyvy422": "UYVY",
    "yuv420p": "YU12",
    "yuyv": "YUYV",
    "yuyv422": "YUYV",
}
_V4L2_PREFERRED_FOURCCS = ("H264", "AVC1", "MJPG", "JPEG")


@dataclass(frozen=True)
class EncodedVideoPlan:
    """A video route that never sends raw Full HD frames through Python."""

    mode: str
    source_codec: str
    encoder: str
    ffmpeg_binary: str = "ffmpeg"
    source_fps: Fraction | None = None
    output_fps: Fraction | None = None
    source_bitrate: int = 0
    output_width: int = 0
    output_height: int = 0
    source_timing_reliable: bool = True


@dataclass(frozen=True)
class SourceVideoProbe:
    """Media properties used to choose the lowest-latency safe route."""

    codec: str
    width: int = 0
    height: int = 0
    fps: Fraction | None = None
    bitrate: int = 0
    pixel_format: str = ""
    profile: str = ""
    has_b_frames: bool = False
    timing_reliable: bool = True
    timing_reason: str = ""
    measured_fps: Fraction | None = None
    packet_count: int = 0
    repeated_timestamps: int = 0
    backward_timestamps: int = 0
    median_frame_interval_ms: float = 0.0
    p95_frame_interval_ms: float = 0.0
    frame_interval_jitter_ms: float = 0.0
    measured_bitrate: int = 0
    largest_access_unit: int = 0
    largest_keyframe: int = 0
    cadence_bursty: bool = False
    passthrough_safe: bool = False

    @property
    def effective_fps(self) -> Fraction | None:
        return self.measured_fps or self.fps


@dataclass
class EncodedPipelineProgress:
    """Progress reported by the native publisher for edge-side watchdogs."""

    started_at: float
    received: int = 0
    published: int = 0
    last_received_at: float = 0.0
    last_published_at: float = 0.0

    @property
    def last_progress_at(self) -> float:
        return max(self.last_received_at, self.last_published_at)


@dataclass(frozen=True)
class CameraCaptureProfile:
    """The format V4L2 actually accepted, not merely the requested profile."""

    device: str
    input_format: str
    width: int
    height: int
    fps: Fraction

    @property
    def source_codec(self) -> str:
        normalized = self.input_format.lower().replace(".", "")
        if normalized in {"h264", "avc", "avc1"}:
            return "h264"
        if normalized in {"mjpeg", "mjpg", "jpeg"}:
            return "mjpeg"
        return "rawvideo"


def fps_text(fps: Fraction) -> str:
    if fps.denominator == 1:
        return str(fps.numerator)
    return f"{fps.numerator}/{fps.denominator}"


def parse_ffprobe_fraction(value: object) -> Fraction | None:
    text = str(value or "").strip()
    if not text or text in {"0", "0/0", "N/A"}:
        return None
    try:
        parsed = Fraction(text).limit_denominator(1001)
    except (ValueError, ZeroDivisionError):
        return None
    return parsed if parsed > 0 else None


def _positive_int(value: object) -> int:
    try:
        parsed = int(str(value or "0"))
    except ValueError:
        return 0
    return max(0, parsed)


def _timestamp_values(
    packets: list[dict[str, object]], field: str
) -> list[float]:
    values: list[float] = []
    for packet in packets:
        try:
            value = float(str(packet.get(field, "")))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def analyze_runtime_video_packets(
    packets: list[dict[str, object]],
    metadata_fps: Fraction | None,
) -> dict[str, object]:
    """Measure source cadence from demuxed access units, not FPS metadata.

    ffprobe emits one packet per compressed access unit for the selected video
    stream. DTS is used for cadence when available because B-frames can reorder
    PTS; PTS is still checked independently for duplicates and regressions.
    """
    pts = _timestamp_values(packets, "pts_time")
    dts = _timestamp_values(packets, "dts_time")
    repeated = sum(
        math.isclose(current, previous, abs_tol=1e-6)
        for previous, current in pairwise(pts)
    )
    backwards = sum(
        current < previous - 1e-6
        for previous, current in pairwise(pts)
    )

    cadence_timestamps = dts if len(dts) >= 2 else pts
    intervals = [
        current - previous
        for previous, current in pairwise(cadence_timestamps)
        if current > previous + 1e-6
    ]
    median_interval = statistics.median(intervals) if intervals else 0.0
    sorted_intervals = sorted(intervals)
    p95_interval = (
        sorted_intervals[math.ceil(len(sorted_intervals) * 0.95) - 1]
        if sorted_intervals
        else 0.0
    )
    jitter = statistics.pstdev(intervals) if len(intervals) >= 2 else 0.0
    measured_fps = None
    if intervals and sum(intervals) > 0:
        measured_fps = Fraction(len(intervals) / sum(intervals)).limit_denominator(
            1001
        )

    tiny_intervals = (
        sum(interval < median_interval * 0.35 for interval in intervals)
        if median_interval > 0
        else 0
    )
    cadence_bursty = bool(
        median_interval > 0
        and (
            p95_interval > median_interval * 2.5
            or jitter > median_interval * VIDEO_CADENCE_JITTER_RATIO
            or tiny_intervals > max(1, len(intervals) // 5)
        )
    )

    reasons: list[str] = []
    if len(packets) < VIDEO_RUNTIME_PROBE_MIN_PACKETS:
        reasons.append("runtime-sample-too-short")
    if len(intervals) < VIDEO_RUNTIME_PROBE_MIN_PACKETS - 1:
        reasons.append("timestamps-missing")
    if repeated:
        reasons.append("timestamp-repeated")
    if backwards:
        reasons.append("timestamp-backwards")
    if measured_fps is None or measured_fps > 60 or measured_fps < 1:
        reasons.append("measured-fps-invalid")
    elif metadata_fps is None:
        reasons.append("metadata-fps-missing")
    else:
        mismatch = abs(float(measured_fps - metadata_fps))
        if mismatch > max(2.0, float(metadata_fps) * VIDEO_FPS_MISMATCH_RATIO):
            reasons.append("metadata-fps-mismatch")
    if cadence_bursty:
        reasons.append("cadence-bursty")

    duration = sum(intervals)
    total_bytes = sum(_positive_int(packet.get("size")) for packet in packets)
    measured_bitrate = round(total_bytes * 8 / duration) if duration > 0 else 0
    largest_access_unit = max(
        (_positive_int(packet.get("size")) for packet in packets), default=0
    )
    largest_keyframe = max(
        (
            _positive_int(packet.get("size"))
            for packet in packets
            if "K" in str(packet.get("flags") or "")
        ),
        default=0,
    )
    return {
        "measured_fps": measured_fps,
        "packet_count": len(packets),
        "repeated_timestamps": repeated,
        "backward_timestamps": backwards,
        "median_frame_interval_ms": median_interval * 1000,
        "p95_frame_interval_ms": p95_interval * 1000,
        "frame_interval_jitter_ms": jitter * 1000,
        "measured_bitrate": measured_bitrate,
        "largest_access_unit": largest_access_unit,
        "largest_keyframe": largest_keyframe,
        "cadence_bursty": cadence_bursty,
        "timing_reliable": not reasons,
        "timing_reason": ",".join(reasons),
    }


def redact_media_source(value: object, source: str = "") -> str:
    """Remove RTSP credentials and exact configured URLs from diagnostics."""
    text = str(value)
    if source:
        text = text.replace(source, "<media-source>")
    return re.sub(
        r"(rtsps?://)([^/@\s]+)@",
        r"\1<redacted>@",
        text,
        flags=re.IGNORECASE,
    )


def bounded_video_dimensions(
    source_width: int,
    source_height: int,
    maximum_width: int,
    maximum_height: int,
) -> tuple[int, int]:
    """Fit inside the selected profile without wasting CPU on upscaling."""
    if source_width <= 0 or source_height <= 0:
        return maximum_width, maximum_height
    scale = min(
        1.0,
        maximum_width / source_width,
        maximum_height / source_height,
    )
    width = max(2, round(source_width * scale / 2) * 2)
    height = max(2, round(source_height * scale / 2) * 2)
    return width, height


def parse_v4l2_capture_profile(
    output: str,
    *,
    device: str,
    fallback_format: str,
    fallback_width: int,
    fallback_height: int,
    fallback_fps: Fraction,
) -> CameraCaptureProfile:
    """Parse ``v4l2-ctl --get-fmt-video --get-parm`` across driver variants."""
    format_match = re.search(r"Pixel Format\s*:\s*'([^']+)'", output)
    size_match = re.search(r"Width/Height\s*:\s*(\d+)\s*/\s*(\d+)", output)
    rational_fps = re.search(
        r"Frames per second\s*:\s*[\d.]+\s*\((\d+)\s*/\s*(\d+)\)",
        output,
        re.IGNORECASE,
    )
    decimal_fps = re.search(
        r"Frames per second\s*:\s*([\d.]+)", output, re.IGNORECASE
    )

    fourcc = format_match.group(1).upper() if format_match else ""
    input_format = _V4L2_FOURCC_TO_FFMPEG.get(
        fourcc,
        fourcc.lower() if fourcc else fallback_format,
    )
    width = int(size_match.group(1)) if size_match else fallback_width
    height = int(size_match.group(2)) if size_match else fallback_height
    fps = fallback_fps
    if rational_fps and int(rational_fps.group(2)) > 0:
        fps = Fraction(
            int(rational_fps.group(1)), int(rational_fps.group(2))
        )
    elif decimal_fps:
        try:
            parsed_fps = Fraction(decimal_fps.group(1)).limit_denominator(1001)
        except (ValueError, ZeroDivisionError):
            pass
        else:
            if parsed_fps > 0:
                fps = parsed_fps

    return CameraCaptureProfile(
        device=device,
        input_format=input_format,
        width=width,
        height=height,
        fps=fps,
    )


def video_pipe_buffer_limit(frame_size: int) -> int:
    """Keep asyncio from pausing FFmpeg dozens of times inside one raw frame."""
    return frame_size * VIDEO_PIPE_BUFFER_FRAMES


def video_pacer_max_latency_ms(fps: float | Fraction) -> int:
    """Finish pacing an encoded frame before the following frame is due.

    LiveKit's leaky-bucket pacer runs every 5 ms and raises its temporary
    bitrate when the queue would otherwise exceed this deadline. Keeping the
    deadline below one frame prevents a large RTSP IDR from delaying the delta
    frame behind it, while the pacer still protects the network from a single
    packet burst. This calculation is independent of CPU architecture.
    """
    frame_aware_latency = round(VIDEO_PACER_FRAME_FRACTION_MS / max(1, fps))
    return min(
        VIDEO_PACER_MAX_LATENCY_MS,
        max(VIDEO_PACER_MIN_LATENCY_MS, frame_aware_latency),
    )


def rtsp_input_args(source: str, transport: str) -> list[str]:
    """Return only RTSP options shared by all supported FFmpeg builds."""
    return [
        "-rtsp_transport", transport,
        "-i", source,
    ]


class MediaPublisher:
    """LiveKit adapter. Media failures never stop the safety/control loop."""

    def __init__(
        self,
        config: SimulatorConfig,
        token_provider: Callable[[str], Awaitable[str]] | None = None,
    ) -> None:
        self.config = config
        self.token_provider = token_provider
        self.room: rtc.Room | None = None
        self.tasks: list[asyncio.Task] = []
        self.connected = False
        self.audio_level = 0.0
        self._camera_profile_key: tuple[object, ...] | None = None
        self._camera_profile: CameraCaptureProfile | None = None
        self._encoder_cache: tuple[str, str] | None = None
        self._failed_video_encoders: set[str] = set()
        self._video_degrade_level = 0
        self._vaapi_rate_control = "auto"
        self._vaapi_low_power = False
        self._video_plan_lock = threading.Lock()
        self._prepared_video_key: tuple[object, ...] | None = None
        self._prepared_video_plan: EncodedVideoPlan | None = None
        self._rtsp_selected_transport = ""
        self._operator_audio_frames: asyncio.Queue[bytes] = asyncio.Queue(
            maxsize=AUDIO_PLAYBACK_BUFFER_FRAMES
        )
        self.aec_active = False
        self.audio_capture_healthy = False
        self.audio_playback_healthy = False

    async def connect(self) -> None:
        if not self.config.media_enabled:
            return
        if self.token_provider is None:
            raise RuntimeError("Center media token provider is not configured")
        await self.disconnect()
        self.room = rtc.Room()

        @self.room.on("track_subscribed")
        def on_track_subscribed(track, _publication, participant) -> None:
            if (
                track.kind == rtc.TrackKind.KIND_AUDIO
                and participant.identity.startswith("user:")
            ):
                logger.info("receiving user audio identity=%s", participant.identity)
                self.tasks.append(asyncio.create_task(self._consume_audio(track)))

        @self.room.on("track_published")
        def on_track_published(publication, participant) -> None:
            self._subscribe_user_audio(publication, participant)

        @self.room.on("disconnected")
        def on_disconnected(*_args) -> None:
            self.connected = False
            logger.warning("LiveKit disconnected; media publisher will reconnect")

        token = await self.token_provider("main")
        # The encoded camera publisher uses a second LiveKit identity. The main
        # robot room must not auto-subscribe to that video or the Pi downloads
        # its own 8 Mbps stream and competes with control traffic on Wi-Fi.
        await self.room.connect(
            self.config.livekit_url,
            token,
            rtc.RoomOptions(auto_subscribe=False),
        )
        for participant in self.room.remote_participants.values():
            for publication in participant.track_publications.values():
                self._subscribe_user_audio(publication, participant)
        pipeline = self._video_pipeline()
        if pipeline == "encoded":
            plan = await asyncio.to_thread(self._prepare_encoded_video)
            self.tasks.append(asyncio.create_task(self._encoded_video_loop(plan)))
        else:
            await self._publish_video()
        await self._publish_audio()
        self.connected = True
        logger.info(
            "LiveKit connected room=robot-%s video_pipeline=%s",
            self.config.robot_id,
            pipeline,
        )

    @staticmethod
    def _is_user_audio_publication(publication, participant) -> bool:
        return (
            publication.kind == rtc.TrackKind.KIND_AUDIO
            and participant.identity.startswith("user:")
        )

    def _subscribe_user_audio(self, publication, participant) -> None:
        if self._is_user_audio_publication(publication, participant):
            publication.set_subscribed(True)

    def _video_pipeline(self) -> str:
        pipeline = self.config.video_pipeline.strip().lower()
        if pipeline not in {"auto", "encoded", "raw"}:
            raise ValueError(f"unsupported VIDEO_PIPELINE: {self.config.video_pipeline}")
        if pipeline == "auto":
            return (
                "raw"
                if self.config.simulator_media_source_type == "test"
                else "encoded"
            )
        if pipeline == "encoded" and self.config.simulator_media_source_type == "test":
            # The generated test pattern has no device encoder to optimize.
            return "raw"
        return pipeline

    async def _publish_video(self) -> None:
        assert self.room
        width, height = self.config.video_width, self.config.video_height
        source = rtc.VideoSource(width, height)
        track = rtc.LocalVideoTrack.create_video_track("camera", source)
        options = self._video_publish_options()
        try:
            await self.room.local_participant.publish_track(track, options)
        except Exception:
            if (
                options.video_encoder
                == proto_room.VideoEncoderBackend.ENCODER_BACKEND_AUTO
            ):
                raise
            logger.warning(
                "hardware video encoder unavailable; falling back to auto backend=%s",
                self.config.video_encoder,
            )
            options.video_encoder = (
                proto_room.VideoEncoderBackend.ENCODER_BACKEND_AUTO
            )
            await self.room.local_participant.publish_track(track, options)
        self.tasks.append(asyncio.create_task(self._video_loop(source, width, height)))

    def _video_encoder_backend(self) -> int:
        backends = {
            "auto": proto_room.VideoEncoderBackend.ENCODER_BACKEND_AUTO,
            "software": proto_room.VideoEncoderBackend.ENCODER_BACKEND_SOFTWARE,
            "hardware": proto_room.VideoEncoderBackend.ENCODER_BACKEND_HARDWARE,
            "vaapi": proto_room.VideoEncoderBackend.ENCODER_BACKEND_VAAPI,
            "nvenc": proto_room.VideoEncoderBackend.ENCODER_BACKEND_NVENC,
            "videotoolbox": (
                proto_room.VideoEncoderBackend.ENCODER_BACKEND_VIDEOTOOLBOX
            ),
        }
        backend = backends.get(self.config.video_encoder.strip().lower())
        if backend is None:
            raise ValueError(
                f"unsupported VIDEO_ENCODER: {self.config.video_encoder}"
            )
        return backend

    def _video_publish_options(self) -> rtc.TrackPublishOptions:
        return rtc.TrackPublishOptions(
            source=rtc.TrackSource.SOURCE_CAMERA,
            video_codec=rtc.VideoCodec.H264,
            video_encoder=self._video_encoder_backend(),
            video_encoding=rtc.VideoEncoding(
                max_bitrate=self.config.video_bitrate,
                max_framerate=float(self.config.video_fps),
            ),
            simulcast=False,
            # A live robot view is motion-first. Keeping both 1080p and the
            # target bitrate rigid made WebRTC/OpenH264 drop whole frames when
            # motion increased. Allow a temporary resolution reduction so the
            # encoder preserves the frame cadence instead.
            degradation_preference=rtc.DegradationPreference.MAINTAIN_FRAMERATE,
        )

    def _prepare_encoded_video(self) -> EncodedVideoPlan:
        key = self._video_plan_key()
        with self._video_plan_lock:
            if (
                self._prepared_video_key == key
                and self._prepared_video_plan is not None
            ):
                return self._prepared_video_plan
            if self._prepared_video_key != key:
                self._rtsp_selected_transport = ""
            plan = self._build_encoded_video_plan()
            self._prepared_video_key = key
            self._prepared_video_plan = plan
            return plan

    def warm_video_source(self) -> None:
        """Inspect the source before an operator asks for the first frame."""
        if self.config.media_enabled and self._video_pipeline() == "encoded":
            self._prepare_encoded_video()

    def _video_plan_key(self) -> tuple[object, ...]:
        source = self.config.simulator_media_source
        if self.config.simulator_media_source_type == "rtsp":
            source = self._resolved_rtsp_source()
        elif (
            self.config.simulator_media_source_type == "camera"
            and not source
        ):
            source = self.config.simulator_camera_device
        return (
            self.config.simulator_media_source_type,
            source,
            self.config.simulator_camera_format,
            self.config.simulator_camera_width,
            self.config.simulator_camera_height,
            self.config.simulator_camera_fps,
            self.config.rtsp_transport,
            self.config.rtsp_normalize,
            self.config.rtsp_auto_normalize,
            self.config.video_width,
            self.config.video_height,
            self.config.video_fps,
            self.config.video_bitrate,
            self.config.video_encoder,
            self.config.video_passthrough,
            self.config.video_ffmpeg_binary,
        )

    def _build_encoded_video_plan(self) -> EncodedVideoPlan:
        if shutil.which("gstreamer-publisher") is None:
            raise RuntimeError(
                "encoded video pipeline requires gstreamer-publisher in the image"
            )
        if shutil.which("gst-inspect-1.0") is None:
            raise RuntimeError("encoded video pipeline requires GStreamer")
        parser = subprocess.run(
            ["gst-inspect-1.0", "h264parse"],
            capture_output=True,
            check=False,
            text=True,
            timeout=5,
        )
        if parser.returncode != 0:
            raise RuntimeError("GStreamer h264parse plugin is unavailable")

        probe = self._probe_video_source()
        reasons = self._video_transcode_reasons(probe)
        output_width, output_height = bounded_video_dimensions(
            probe.width,
            probe.height,
            self.config.video_width,
            self.config.video_height,
        )
        support_level = "A"
        if reasons:
            support_level = "B"
            # Decode/encode is best effort on Pi 5. Start from a bounded 720p20
            # profile even when the selected display profile is 1080p; a clean
            # H.264 passthrough may still keep its native 1080p resolution.
            output_width, output_height = bounded_video_dimensions(
                output_width,
                output_height,
                1280,
                720,
            )
        if reasons and (
            probe.width > 1920
            or probe.height > 1080
            or (probe.codec != "h264" and probe.bitrate > 12_000_000)
        ):
            support_level = "C"
            logger.error(
                "video profile support=C reason=transcode-over-pi5-budget "
                "codec=%s source=%dx%d bitrate=%d recommendation=h264-substream-720p",
                probe.codec,
                probe.width,
                probe.height,
                probe.bitrate,
            )
        if self._video_degrade_level:
            scale = 0.75 ** self._video_degrade_level
            output_width = max(
                min(640, output_width), round(output_width * scale / 2) * 2
            )
            output_height = max(
                min(360, output_height), round(output_height * scale / 2) * 2
            )
        target_fps = Fraction(self.config.video_fps, 1)
        if reasons:
            target_fps = min(target_fps, Fraction(20, 1))
        if not reasons:
            # USB and RTSP H.264 go straight through one GStreamer pipeline.
            # Files still need FFmpeg to demux arbitrary containers, but are
            # emitted as Annex-B H.264 without the old MPEG-TS mux/demux loop.
            mode = (
                "direct"
                if self.config.simulator_media_source_type in {"camera", "rtsp"}
                else "bridge"
            )
            passthrough_fps = probe.effective_fps or target_fps
            logger.info(
                "video route support=A mode=%s codec=h264 copy=true fps=%s "
                "size=%dx%d bitrate=%d",
                mode,
                fps_text(passthrough_fps),
                probe.width,
                probe.height,
                probe.bitrate,
            )
            return EncodedVideoPlan(
                mode,
                probe.codec,
                "copy",
                source_fps=passthrough_fps,
                output_fps=passthrough_fps,
                source_bitrate=probe.bitrate,
                output_width=probe.width,
                output_height=probe.height,
                source_timing_reliable=probe.timing_reliable,
            )

        if probe.effective_fps is not None:
            output_fps = min(probe.effective_fps, target_fps)
        elif (
            self.config.simulator_media_source_type == "rtsp"
            and not probe.timing_reliable
        ):
            # Leave headroom below common 25 FPS RTSP sources. A camera with
            # corrupt timestamps cannot safely be paced faster than the frames
            # it really delivers; doing so makes the browser jitter buffer grow
            # until it starts dropping frames and live control looks delayed.
            output_fps = min(target_fps, Fraction(UNRELIABLE_RTSP_FPS, 1))
        else:
            output_fps = target_fps
        if self._video_degrade_level:
            output_fps = max(
                Fraction(10, 1),
                Fraction(
                    round(float(output_fps) * (0.8 ** self._video_degrade_level)),
                    1,
                ),
            )
            logger.warning(
                "video profile reduced to preserve realtime level=%d output=%dx%d@%s",
                self._video_degrade_level,
                output_width,
                output_height,
                fps_text(output_fps),
            )
        logger.info(
            "video route support=%s transcode=true codec=%s reason=%s output=%dx%d@%s",
            support_level,
            probe.codec,
            ",".join(reasons),
            output_width,
            output_height,
            fps_text(output_fps),
        )

        encoder, binary = self._select_video_encoder()
        if encoder == "h264_vaapi":
            logger.info(
                "video optimization selected mode=bridge source_codec=%s "
                "encoder=%s rate_control=%s low_power=%s",
                probe.codec,
                encoder,
                self._vaapi_rate_control,
                self._vaapi_low_power,
            )
        else:
            logger.info(
                "video optimization selected mode=bridge source_codec=%s encoder=%s",
                probe.codec,
                encoder,
            )
        return EncodedVideoPlan(
            "bridge",
            probe.codec,
            encoder,
            binary,
            source_fps=probe.effective_fps,
            output_fps=output_fps,
            source_bitrate=probe.bitrate,
            output_width=output_width,
            output_height=output_height,
            source_timing_reliable=probe.timing_reliable,
        )

    def _video_transcode_reasons(self, probe: SourceVideoProbe) -> list[str]:
        reasons: list[str] = []
        if probe.codec != "h264":
            reasons.append(f"codec-{probe.codec}")
        if not self.config.video_passthrough:
            reasons.append("passthrough-disabled")
        if self.config.simulator_media_source_type == "rtsp":
            if self.config.rtsp_normalize:
                reasons.append("normalize-forced")
            elif not probe.timing_reliable:
                reasons.append(probe.timing_reason or "timing-unreliable")
        if probe.has_b_frames:
            reasons.append("b-frames-require-normalization")
        if probe.width > self.config.video_width or probe.height > self.config.video_height:
            reasons.append("profile-resolution")
        if probe.effective_fps is not None and probe.effective_fps > 30:
            reasons.append("profile-fps")
        maximum_passthrough_bitrate = max(
            self.config.video_bitrate * 3 // 2,
            self.config.video_bitrate + 1_000_000,
        )
        if probe.bitrate > maximum_passthrough_bitrate:
            reasons.append("profile-bitrate")
        compatible_pixel_formats = {"", "nv12", "yuv420p", "yuvj420p"}
        if probe.pixel_format not in compatible_pixel_formats:
            reasons.append("pixel-format")
        compatible_profiles = {"", "baseline", "constrained baseline", "main", "high"}
        if probe.profile.casefold() not in compatible_profiles:
            reasons.append("h264-profile")
        return reasons

    def _probe_source_codec(self) -> str:
        """Compatibility helper retained for diagnostics and older callers."""
        return self._probe_video_source().codec

    def _probe_video_source(self) -> SourceVideoProbe:
        kind = self.config.simulator_media_source_type
        if kind == "camera":
            profile = self._camera_capture_profile_for_source()
            probe = SourceVideoProbe(
                codec=profile.source_codec,
                width=profile.width,
                height=profile.height,
                fps=profile.fps,
            )
            return replace(
                probe,
                passthrough_safe=not self._video_transcode_reasons(probe),
            )

        if kind not in {"rtsp", "file"}:
            return SourceVideoProbe(codec="rawvideo")
        if shutil.which("ffprobe") is None:
            raise RuntimeError("ffprobe is required to inspect the video source")
        if kind == "rtsp":
            configured = self.config.rtsp_transport.strip().lower()
            transports = (
                [self._rtsp_selected_transport]
                if self._rtsp_selected_transport
                else (["udp", "tcp"] if configured == "auto" else [configured])
            )
        else:
            transports = [""]
        runtime_args = (
            [
                "-read_intervals",
                f"%+{VIDEO_RUNTIME_PROBE_SECONDS}",
                "-show_packets",
            ]
            if kind == "rtsp"
            else []
        )
        result: subprocess.CompletedProcess[str] | None = None
        for transport in transports:
            input_args = (
                [
                    "-rtsp_transport",
                    transport,
                    "-i",
                    self._resolved_rtsp_source(),
                ]
                if kind == "rtsp"
                else ["-i", self.config.simulator_media_source]
            )
            try:
                attempt = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    *input_args,
                    "-select_streams",
                    "v:0",
                    *runtime_args,
                    "-show_entries",
                    (
                        "stream=codec_name,width,height,pix_fmt,profile,"
                        "avg_frame_rate,r_frame_rate,bit_rate,has_b_frames:"
                        "packet=pts_time,dts_time,duration_time,size,flags"
                    ),
                    "-of",
                    "json",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=VIDEO_RUNTIME_PROBE_TIMEOUT_SECONDS,
                )
            except subprocess.TimeoutExpired:
                attempt = None
            if attempt is not None:
                result = attempt
                if attempt.returncode == 0:
                    if kind == "rtsp":
                        self._rtsp_selected_transport = transport
                        logger.info("RTSP transport selected transport=%s", transport)
                    break
            if kind == "rtsp" and transport == "udp" and len(transports) > 1:
                logger.warning("RTSP UDP probe failed; falling back transport=tcp")
        if result is None:
            raise RuntimeError("video source runtime probe timed out")
        try:
            payload = json.loads(result.stdout or "{}")
            stream = payload.get("streams", [])[0]
        except (IndexError, KeyError, TypeError, json.JSONDecodeError):
            stream = None
        if result.returncode != 0 or not isinstance(stream, dict):
            detail = redact_media_source(
                result.stderr.strip(), self.config.simulator_media_source
            )
            raise RuntimeError(
                f"cannot detect video source codec: {detail[-300:] or 'no video stream'}"
            )
        codec = str(stream.get("codec_name") or "").strip().lower()
        if not codec:
            raise RuntimeError("cannot detect video source codec: no video codec")

        average_fps = parse_ffprobe_fraction(stream.get("avg_frame_rate"))
        advertised_fps = parse_ffprobe_fraction(stream.get("r_frame_rate"))
        fps = average_fps or advertised_fps
        timing_reliable = True
        timing_reason = ""
        runtime: dict[str, object] = {}
        if kind == "rtsp":
            raw_packets = payload.get("packets", [])
            packets = (
                [item for item in raw_packets if isinstance(item, dict)]
                if isinstance(raw_packets, list)
                else []
            )
            runtime = analyze_runtime_video_packets(packets, fps)
            timing_reliable = bool(runtime["timing_reliable"])
            timing_reason = str(runtime["timing_reason"])
        if kind == "rtsp" and (fps is None or fps > 60):
            timing_reliable = False
            metadata_reason = (
                "timing-missing" if fps is None else "timing-out-of-range"
            )
            timing_reason = ",".join(
                reason for reason in (metadata_reason, timing_reason) if reason
            )
            fps = None

        probe = SourceVideoProbe(
            codec=codec,
            width=_positive_int(stream.get("width")),
            height=_positive_int(stream.get("height")),
            fps=fps,
            bitrate=(
                _positive_int(stream.get("bit_rate"))
                or int(runtime.get("measured_bitrate", 0))
            ),
            pixel_format=str(stream.get("pix_fmt") or "").strip().lower(),
            profile=str(stream.get("profile") or "").strip(),
            has_b_frames=_positive_int(stream.get("has_b_frames")) > 0,
            timing_reliable=timing_reliable,
            timing_reason=timing_reason,
            measured_fps=runtime.get("measured_fps"),  # type: ignore[arg-type]
            packet_count=int(runtime.get("packet_count", 0)),
            repeated_timestamps=int(runtime.get("repeated_timestamps", 0)),
            backward_timestamps=int(runtime.get("backward_timestamps", 0)),
            median_frame_interval_ms=float(
                runtime.get("median_frame_interval_ms", 0.0)
            ),
            p95_frame_interval_ms=float(
                runtime.get("p95_frame_interval_ms", 0.0)
            ),
            frame_interval_jitter_ms=float(
                runtime.get("frame_interval_jitter_ms", 0.0)
            ),
            measured_bitrate=int(runtime.get("measured_bitrate", 0)),
            largest_access_unit=int(runtime.get("largest_access_unit", 0)),
            largest_keyframe=int(runtime.get("largest_keyframe", 0)),
            cadence_bursty=bool(runtime.get("cadence_bursty", False)),
        )
        probe = replace(
            probe,
            passthrough_safe=not self._video_transcode_reasons(probe),
        )
        logger.info(
            "video source probe kind=%s codec=%s size=%dx%d metadata_fps=%s "
            "measured_fps=%s packets=%d interval_median_ms=%.2f "
            "interval_p95_ms=%.2f jitter_ms=%.2f bitrate=%d largest_au=%d "
            "largest_keyframe=%d pix_fmt=%s profile=%s b_frames=%s timing=%s",
            kind,
            probe.codec,
            probe.width,
            probe.height,
            fps_text(probe.fps) if probe.fps else "unknown",
            fps_text(probe.measured_fps) if probe.measured_fps else "unknown",
            probe.packet_count,
            probe.median_frame_interval_ms,
            probe.p95_frame_interval_ms,
            probe.frame_interval_jitter_ms,
            probe.bitrate,
            probe.largest_access_unit,
            probe.largest_keyframe,
            probe.pixel_format or "unknown",
            probe.profile or "unknown",
            probe.has_b_frames,
            "ok" if probe.timing_reliable else probe.timing_reason,
        )
        return probe

    def _camera_device(self) -> str:
        return (
            self.config.simulator_media_source
            or self.config.simulator_camera_device
        )

    def _camera_capture_profile_for_source(self) -> CameraCaptureProfile:
        device = self._camera_device()
        key = (
            device,
            self.config.simulator_camera_format,
            self.config.simulator_camera_width,
            self.config.simulator_camera_height,
            self.config.simulator_camera_fps,
            self.config.video_width,
            self.config.video_height,
            self.config.video_fps,
        )
        if self._camera_profile_key == key and self._camera_profile is not None:
            return self._camera_profile

        requested_format = self.config.simulator_camera_format.strip().lower()
        requested_fourcc = _FFMPEG_TO_V4L2_FOURCC.get(requested_format, "")
        if not requested_fourcc and len(requested_format) == 4:
            requested_fourcc = requested_format.upper()
        v4l2_binary = shutil.which("v4l2-ctl")
        target_width = self.config.simulator_camera_width or self.config.video_width
        target_height = self.config.simulator_camera_height or self.config.video_height
        target_fps = self.config.simulator_camera_fps or self.config.video_fps
        selected_mode = None
        if v4l2_binary:
            selected_mode = select_v4l2_mode(
                discover_v4l2_modes(device),
                target_width,
                target_height,
                target_fps,
                requested_format,
            )
        if selected_mode:
            requested_fourcc = str(selected_mode["fourcc"])
            target_width = int(selected_mode["width"])
            target_height = int(selected_mode["height"])
            target_fps = float(selected_mode["fps"])
        elif v4l2_binary and not requested_format:
            requested_fourcc = self._preferred_camera_fourcc(v4l2_binary, device)
        fallback_format = (
            _V4L2_FOURCC_TO_FFMPEG.get(requested_fourcc, requested_format)
            if requested_fourcc
            else requested_format
        )
        fallback = CameraCaptureProfile(
            device=device,
            input_format=fallback_format,
            width=target_width,
            height=target_height,
            fps=Fraction(str(target_fps)).limit_denominator(1001),
        )
        profile = fallback
        if v4l2_binary:
            profile = self._negotiate_camera_capture_profile(
                v4l2_binary,
                fallback,
                requested_fourcc,
            )

        self._camera_profile_key = key
        self._camera_profile = profile
        logger.info(
            "camera capture negotiated device=%s requested=%dx%d@%d format=%s "
            "actual=%dx%d@%s format=%s",
            device,
            target_width,
            target_height,
            round(target_fps),
            requested_format or "auto",
            profile.width,
            profile.height,
            fps_text(profile.fps),
            profile.input_format or "auto",
        )
        return profile

    @staticmethod
    def _preferred_camera_fourcc(v4l2_binary: str, device: str) -> str:
        try:
            result = subprocess.run(
                [v4l2_binary, "--device", device, "--list-formats-ext"],
                capture_output=True,
                check=False,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        if result.returncode != 0:
            return ""
        available = {
            match.upper()
            for match in re.findall(r"\[\d+\]:\s*'([^']+)'", result.stdout)
        }
        return next(
            (fourcc for fourcc in _V4L2_PREFERRED_FOURCCS if fourcc in available),
            "",
        )

    def _negotiate_camera_capture_profile(
        self,
        v4l2_binary: str,
        fallback: CameraCaptureProfile,
        requested_fourcc: str,
    ) -> CameraCaptureProfile:
        self._stabilize_camera_frame_timing(v4l2_binary, fallback.device)
        format_fields = [
            f"width={fallback.width}",
            f"height={fallback.height}",
        ]
        if requested_fourcc:
            format_fields.append(f"pixelformat={requested_fourcc}")
        try:
            set_result = subprocess.run(
                [
                    v4l2_binary,
                    "--device",
                    fallback.device,
                    "--set-fmt-video",
                    ",".join(format_fields),
                    "--set-parm",
                    fps_text(fallback.fps),
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=4,
            )
            query_result = subprocess.run(
                [
                    v4l2_binary,
                    "--device",
                    fallback.device,
                    "--get-fmt-video",
                    "--get-parm",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            logger.warning(
                "camera capture negotiation unavailable device=%s error=%s",
                fallback.device,
                exc,
            )
            return fallback

        output = "\n".join(
            part
            for part in (
                set_result.stdout,
                set_result.stderr,
                query_result.stdout,
                query_result.stderr,
            )
            if part
        )
        profile = parse_v4l2_capture_profile(
            output,
            device=fallback.device,
            fallback_format=fallback.input_format,
            fallback_width=fallback.width,
            fallback_height=fallback.height,
            fallback_fps=fallback.fps,
        )
        if query_result.returncode != 0:
            logger.warning(
                "camera capture query returned exit=%d device=%s; using parsed/fallback profile",
                query_result.returncode,
                fallback.device,
            )
        return profile

    @staticmethod
    def _stabilize_camera_frame_timing(v4l2_binary: str, device: str) -> None:
        """Prevent auto exposure from silently lowering a USB camera's FPS."""
        try:
            controls = subprocess.run(
                [v4l2_binary, "--device", device, "--list-ctrls"],
                capture_output=True,
                check=False,
                text=True,
                timeout=3,
            )
            if controls.returncode != 0 or not re.search(
                r"^\s*exposure_dynamic_framerate\b",
                controls.stdout,
                re.MULTILINE,
            ):
                return
            result = subprocess.run(
                [
                    v4l2_binary,
                    "--device",
                    device,
                    "--set-ctrl=exposure_dynamic_framerate=0",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=3,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            logger.debug(
                "camera frame-timing control unavailable device=%s error=%s",
                device,
                exc,
            )
            return
        if result.returncode == 0:
            logger.info(
                "camera frame timing stabilized device=%s "
                "exposure_dynamic_framerate=0",
                device,
            )
        else:
            logger.debug(
                "camera frame-timing control rejected device=%s error=%s",
                device,
                result.stderr.strip(),
            )

    def _select_video_encoder(self) -> tuple[str, str]:
        if self._encoder_cache is not None:
            return self._encoder_cache
        requested = self.config.video_encoder.strip().lower()
        explicit = {
            "software": ["libx264"],
            "vaapi": ["h264_vaapi"],
            "nvenc": ["h264_nvenc"],
            "rkmpp": ["h264_rkmpp"],
            "v4l2m2m": ["h264_v4l2m2m"],
            "hardware": [
                "h264_rkmpp",
                "h264_vaapi",
                "h264_nvenc",
                "h264_v4l2m2m",
            ],
            "auto": [
                "h264_rkmpp",
                "h264_vaapi",
                "h264_nvenc",
                "h264_v4l2m2m",
                "libx264",
            ],
        }
        candidates = explicit.get(requested)
        if candidates is None:
            raise ValueError(f"unsupported VIDEO_ENCODER: {self.config.video_encoder}")
        if requested not in {"auto", "software"} and "libx264" not in candidates:
            candidates = [*candidates, "libx264"]
        failures: list[str] = []
        for encoder in candidates:
            if encoder in self._failed_video_encoders:
                failures.append(f"{encoder}: failed during this media session")
                continue
            binary = "ffmpeg"
            if encoder == "h264_rkmpp":
                binary = self.config.video_ffmpeg_binary
                bundled_rkmpp = Path("/opt/ffmpeg-rk/bin/ffmpeg")
                if binary == "ffmpeg" and bundled_rkmpp.is_file():
                    binary = str(bundled_rkmpp)
            ok, detail = self._probe_video_encoder(binary, encoder)
            if ok:
                self._encoder_cache = (encoder, binary)
                return self._encoder_cache
            failures.append(f"{encoder}: {detail}")
        raise RuntimeError(
            "no usable video encoder found; " + "; ".join(failures)
        )

    def _probe_video_encoder(self, binary: str, encoder: str) -> tuple[bool, str]:
        executable = shutil.which(binary)
        if executable is None and Path(binary).is_file():
            executable = binary
        if executable is None:
            return False, f"{binary} not found"

        command = [
            executable,
            "-hide_banner",
            "-nostdin",
            "-loglevel",
            "error",
        ]
        filters: list[str] = []
        if encoder == "h264_vaapi":
            render_devices = sorted(Path("/dev/dri").glob("renderD*"))
            if not render_devices:
                return False, "/dev/dri/renderD* not found"
            command.extend(["-vaapi_device", str(render_devices[0])])
            filters = ["-vf", "format=nv12,hwupload"]
        elif encoder == "h264_rkmpp":
            filters = ["-vf", "format=nv12"]
        command.extend(
            [
                "-f",
                "lavfi",
                "-i",
                (
                    f"color=size={self.config.video_width}x"
                    f"{self.config.video_height}:rate={self.config.video_fps}"
                ),
                "-frames:v",
                "2",
                *filters,
                "-c:v",
                encoder,
            ]
        )
        vaapi_options: list[tuple[str | None, bool | None]]
        if encoder == "h264_vaapi":
            # Prefer the low-power Intel encode entrypoint, then fall back to
            # the regular entrypoint for drivers which do not expose it.
            vaapi_options = [
                ("cbr", True),
                ("cqp", True),
                ("cbr", False),
                ("cqp", False),
            ]
        else:
            vaapi_options = [(None, None)]
        last_detail = ""
        for vaapi_mode, vaapi_low_power in vaapi_options:
            probe_command = [
                *command,
                *self._video_encoder_args(
                    encoder,
                    vaapi_mode,
                    vaapi_low_power=vaapi_low_power,
                ),
                "-f",
                "null",
                "-",
            ]
            try:
                result = subprocess.run(
                    probe_command,
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=8,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                last_detail = str(exc)
                continue
            if result.returncode == 0:
                if vaapi_mode is not None:
                    self._vaapi_rate_control = vaapi_mode
                    self._vaapi_low_power = bool(vaapi_low_power)
                return True, ""
            lines = [
                line.strip()
                for line in result.stderr.splitlines()
                if line.strip()
            ]
            last_detail = (
                lines[-1] if lines else f"exit {result.returncode}"
            )[-240:]
        return False, last_detail or "encoder probe failed"

    @staticmethod
    def _gst_quote(value: str) -> str:
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'

    def _direct_h264_pipeline(self) -> list[str]:
        kind = self.config.simulator_media_source_type
        if kind == "rtsp":
            configured_transport = self.config.rtsp_transport.strip().lower()
            protocols = (
                "udp+tcp" if configured_transport == "auto" else configured_transport
            )
            return [
                "rtspsrc",
                f"location={self._gst_quote(self._resolved_rtsp_source())}",
                f"protocols={protocols}",
                # Keep a small RTSP reorder buffer. Dropping an encoded H.264
                # delta frame corrupts the rest of its GOP, which appears as
                # scratches until the next keyframe.
                "latency=80",
                # Do not slave the local pipeline clock to unstable sender
                # reports. The publisher assigns a fixed RTP frame duration.
                "buffer-mode=none",
                "max-ts-offset=0",
                "drop-on-latency=true",
                # In auto mode rtspsrc starts with UDP and switches to TCP if
                # no UDP packets arrive. Both timeouts are bounded so a live
                # socket that stops producing frames reaches the watchdog.
                "timeout=1500000",
                "tcp-timeout=3000000",
                "!",
                "rtph264depay",
                "request-keyframe=true",
                "wait-for-keyframe=true",
                "!",
                "h264parse",
                "disable-passthrough=true",
                "config-interval=1",
            ]
        if kind == "camera":
            profile = self._camera_capture_profile_for_source()
            return [
                "v4l2src",
                f"device={self._gst_quote(profile.device)}",
                "do-timestamp=true",
                "!",
                (
                    "video/x-h264,"
                    f"width={profile.width},"
                    f"height={profile.height},"
                    f"framerate={profile.fps.numerator}/{profile.fps.denominator}"
                ),
                "!",
                "h264parse",
                "disable-passthrough=true",
                "config-interval=1",
            ]
        raise ValueError(f"direct H.264 is unsupported for source type: {kind}")

    def _bridge_h264_pipeline(
        self, plan: EncodedVideoPlan | None = None
    ) -> list[str]:
        # Decouple the two native processes without another container/demuxer.
        # The queue is bounded; if it stops draining the watchdog restarts both
        # processes instead of allowing an encoded GOP backlog to grow.
        queue_frames = max(
            2,
            min(6, math.ceil(float(self._publisher_video_fps(plan)) * 0.16)),
        )
        return [
            "fdsrc",
            "fd=0",
            "!",
            "queue",
            # Back-pressure is intentional: dropping an H.264 delta frame
            # corrupts the remainder of its GOP. The upstream RTSP queue and
            # publisher watchdog reconnect instead of growing local latency.
            f"max-size-buffers={queue_frames}",
            "max-size-bytes=0",
            "max-size-time=0",
            "!",
            "h264parse",
            "disable-passthrough=true",
            "config-interval=1",
        ]

    def _publisher_command(
        self,
        token: str,
        pipeline: list[str],
        plan: EncodedVideoPlan | None = None,
    ) -> list[str]:
        fps = self._publisher_video_fps(plan)
        pacer_bitrate = self._publisher_pacer_bitrate(plan)
        return [
            "gstreamer-publisher",
            "--url",
            self.config.livekit_url,
            "--token",
            token,
            "--pacer-bitrate",
            str(pacer_bitrate),
            "--pacer-max-latency-ms",
            str(video_pacer_max_latency_ms(fps)),
            "--video-fps",
            fps_text(fps),
            "--",
            *pipeline,
        ]

    def _publisher_video_fps(
        self, plan: EncodedVideoPlan | None = None
    ) -> Fraction:
        if plan is not None and plan.output_fps is not None:
            return plan.output_fps
        # Direct H.264 keeps the negotiated capture cadence. Transcoded camera
        # paths cap it at VIDEO_FPS and drop surplus raw frames before encode.
        if self.config.simulator_media_source_type == "camera":
            capture_fps = self._camera_capture_profile_for_source().fps
            if plan is None or plan.mode == "direct":
                return capture_fps
            return min(capture_fps, Fraction(self.config.video_fps, 1))
        return Fraction(self.config.video_fps, 1)

    def _publisher_pacer_bitrate(
        self, plan: EncodedVideoPlan | None = None
    ) -> int:
        bitrate = self.config.video_bitrate
        if plan is not None and plan.encoder == "copy":
            if plan.source_bitrate > 0:
                # Camera IDR frames arrive in a short burst. Pacing 35% above
                # the average clears them before the next frame without
                # changing the number of bytes sent over the network.
                bitrate = max(bitrate, math.ceil(plan.source_bitrate * 1.35))
            else:
                # RTSP commonly omits bit_rate. Reserve enough pacing headroom
                # for a normal 1080p H.264 main stream; actual bandwidth remains
                # the camera's encoded bitrate.
                bitrate = max(bitrate, self.config.video_bitrate * 2)
        return min(
            VIDEO_PACER_MAX_BITRATE,
            max(VIDEO_PACER_BASE_BITRATE, bitrate),
        )

    def _video_encoder_args(
        self,
        encoder: str,
        vaapi_mode: str | None = None,
        *,
        vaapi_low_power: bool | None = None,
        output_fps: Fraction | None = None,
    ) -> list[str]:
        selected_fps = output_fps or Fraction(self.config.video_fps, 1)
        # Hardware encoders keep a short 250 ms rate-control window. This is a
        # bitrate model, not a frame queue; actual media latency is bounded by
        # async_depth, B-frames and the downstream latest-wins queues.
        vbv_buffer = max(250_000, self.config.video_bitrate // 4)
        common = [
            "-g",
            str(max(1, round(float(selected_fps)))),
            "-bf",
            "0",
        ]
        if encoder == "h264_vaapi":
            selected_mode = vaapi_mode or self._vaapi_rate_control
            selected_low_power = (
                self._vaapi_low_power
                if vaapi_low_power is None
                else vaapi_low_power
            )
            low_latency = (
                [
                    "-low_power",
                    "1",
                    # VAAPI defines larger values as faster. The webcam path is
                    # latency-sensitive and CQP already controls visual quality.
                    "-quality",
                    "7",
                ]
                if selected_low_power
                else []
            )
            if selected_mode == "cqp":
                # Older Intel iHD/i965 generations can expose h264_vaapi while
                # supporting CQP only.
                return [
                    "-rc_mode",
                    "CQP",
                    "-qp",
                    # CQP is the only mode available on some older Intel
                    # generations. A slightly higher QP keeps webcam motion
                    # bursts from overwhelming the WebRTC sender.
                    "27",
                    # VA-API defaults to two frames in flight. One frame keeps
                    # the hardware path enabled without adding another frame
                    # of live-view delay.
                    "-async_depth",
                    "1",
                    *low_latency,
                    *common,
                ]
            # Prefer an actual bitrate ceiling on drivers that support it.
            return [
                "-rc_mode",
                "CBR",
                "-b:v",
                str(self.config.video_bitrate),
                "-maxrate",
                str(self.config.video_bitrate),
                "-bufsize",
                str(vbv_buffer),
                "-async_depth",
                "1",
                *low_latency,
                *common,
            ]
        rate_control = [
            "-b:v",
            str(self.config.video_bitrate),
            "-maxrate",
            str(self.config.video_bitrate),
            "-bufsize",
            str(vbv_buffer),
        ]
        if encoder == "libx264":
            # A 250 ms VBV makes each one-second IDR consume most of the
            # available budget and visibly starves the following P-frames.
            # A 500 ms model evens out QP without adding frame lookahead or an
            # encoder queue; zerolatency and bf=0 still emit each frame at once.
            software_vbv_buffer = max(500_000, self.config.video_bitrate // 2)
            return [
                "-preset",
                "superfast",
                "-tune",
                "zerolatency",
                "-b:v",
                str(self.config.video_bitrate),
                "-maxrate",
                str(self.config.video_bitrate),
                "-bufsize",
                str(software_vbv_buffer),
                "-qcomp",
                "0.75",
                "-x264-params",
                "scenecut=0:force-cfr=1",
                *common,
            ]
        return [*rate_control, *common]

    def _encoded_ffmpeg_command(self, plan: EncodedVideoPlan) -> list[str]:
        binary = plan.ffmpeg_binary if plan.encoder != "copy" else "ffmpeg"
        command = [
            binary,
            "-hide_banner",
            "-nostdin",
            "-loglevel",
            "warning",
        ]
        render_device = ""
        if plan.encoder == "h264_vaapi":
            render_devices = sorted(Path("/dev/dri").glob("renderD*"))
            if not render_devices:
                raise RuntimeError("/dev/dri/renderD* disappeared")
            render_device = str(render_devices[0])
            command.extend(["-vaapi_device", render_device])
        command.extend(
            [
                *self._video_input_args(preserve_timestamps=plan.encoder == "copy"),
                "-map",
                "0:v:0",
                "-an",
            ]
        )

        if plan.encoder == "copy":
            command.extend(
                ["-c:v", "copy", "-bsf:v", "h264_mp4toannexb"]
            )
        else:
            output_fps = self._publisher_video_fps(plan)
            if self.config.simulator_media_source_type == "camera":
                cadence_filter = (
                    "setpts=PTS-STARTPTS,"
                    f"fps=fps={fps_text(output_fps)}:round=down:eof_action=pass"
                )
            elif self.config.simulator_media_source_type == "rtsp":
                if not plan.source_timing_reliable:
                    # Some ONVIF cameras advertise values such as 100 FPS while
                    # actually producing about 20-25 FPS. Rebase those frames
                    # from their arrival clock before fps normalization; using
                    # the corrupt camera PTS creates long bursts and freezes.
                    cadence_filter = (
                        "setpts=(RTCTIME-RTCSTART)/(TB*1000000),"
                        f"fps=fps={fps_text(output_fps)}:"
                        "round=near:eof_action=pass,"
                        f"setpts=N/({fps_text(output_fps)}*TB)"
                    )
                else:
                    # Rebuild bursty arrival timestamps at the configured output
                    # cadence. The second setpts gives the publisher a clean
                    # monotonic RTP clock after duplicate/drop normalization.
                    cadence_filter = (
                        "setpts=PTS-STARTPTS,"
                        f"fps=fps={fps_text(output_fps)}:"
                        "round=near:eof_action=pass,"
                        f"setpts=N/({fps_text(output_fps)}*TB)"
                    )
            else:
                cadence_filter = (
                    "setpts=PTS-STARTPTS,"
                    f"fps=fps={self.config.video_fps}:round=near:eof_action=pass,"
                    f"setpts=N/({self.config.video_fps}*TB)"
                )
            output_width = plan.output_width or self.config.video_width
            output_height = plan.output_height or self.config.video_height
            filters = (
                f"{cadence_filter},"
                f"scale={output_width}:{output_height}:flags=bicubic"
            )
            # USB MJPEG cameras commonly decode to YUV 4:2:2. WebRTC H.264
            # decoders are most reliable on 4:2:0, and leaving x264 on the
            # High 4:2:2 profile can force software decode in the browser and
            # make live-view latency vary by client. Normalize every encoder
            # to a standard 4:2:0 input before publishing.
            if plan.encoder in {
                "h264_vaapi",
                "h264_rkmpp",
                "h264_v4l2m2m",
            }:
                filters += ",format=nv12"
            else:
                filters += ",format=yuv420p"
            if plan.encoder == "h264_vaapi":
                filters += ",hwupload"
            command.extend(["-vf", filters, "-c:v", plan.encoder])
            command.extend(
                self._video_encoder_args(plan.encoder, output_fps=output_fps)
            )
            # The fps filter already owns cadence. Do not let the output layer
            # duplicate frames and recreate a stale queue after the encoder.
            command.extend(["-fps_mode", "passthrough"])
        command.extend(["-f", "h264", "pipe:1"])
        return command

    async def _encoded_video_loop(self, plan: EncodedVideoPlan) -> None:
        delay = 1.0
        refresh_plan = False
        while True:
            processes: list[asyncio.subprocess.Process] = []
            output_tasks: list[asyncio.Task[str]] = []
            process_names: list[str] = []
            token = ""
            progress = EncodedPipelineProgress(time.monotonic())
            child_exited = ""
            keyframe_reconnect = False
            try:
                if refresh_plan:
                    with self._video_plan_lock:
                        self._prepared_video_key = None
                        self._prepared_video_plan = None
                    plan = await asyncio.to_thread(self._prepare_encoded_video)
                    refresh_plan = False
                assert self.token_provider is not None
                token = await self.token_provider("video")
                if plan.mode == "direct":
                    command = self._publisher_command(
                        token, self._direct_h264_pipeline(), plan
                    )
                    process, output_task = await self._start_media_process(
                        command, progress
                    )
                    processes.append(process)
                    output_tasks.append(output_task)
                    process_names.append("publisher")
                else:
                    (
                        processes,
                        output_tasks,
                        process_names,
                    ) = await self._start_bridge_media_processes(
                        token,
                        plan,
                        progress,
                    )

                logger.info(
                    "encoded video publisher started mode=%s encoder=%s "
                    "fps=%s pacer_bitrate=%d pacer_latency_ms=%d",
                    plan.mode,
                    plan.encoder,
                    fps_text(self._publisher_video_fps(plan)),
                    self._publisher_pacer_bitrate(plan),
                    video_pacer_max_latency_ms(self._publisher_video_fps(plan)),
                )
                waiters = [asyncio.create_task(process.wait()) for process in processes]
                watchdog = asyncio.create_task(
                    self._watch_encoded_video_progress(progress, processes, plan)
                )
                done, pending = await asyncio.wait(
                    [*waiters, watchdog], return_when=asyncio.FIRST_COMPLETED
                )
                for waiter in pending:
                    waiter.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
                if watchdog in done:
                    await watchdog
                for index, waiter in enumerate(waiters):
                    if waiter in done:
                        child_exited = process_names[index]
                        break
                exit_codes = [process.returncode for process in processes]
                for process in reversed(processes):
                    await self._stop_process(process)
                details = await asyncio.gather(
                    *output_tasks, return_exceptions=True
                )
                labeled_details = []
                for name, item in zip(process_names, details, strict=True):
                    if isinstance(item, str) and item:
                        labeled_details.append(f"{name}: {item[-1600:]}")
                detail = redact_media_source(
                    " | ".join(labeled_details).replace(token, "<redacted>"),
                    self.config.simulator_media_source,
                )
                keyframe_reconnect = "video-keyframe-timeout" in detail
                raise RuntimeError(
                    f"encoded video child exited codes={exit_codes} "
                    f"detail={detail or 'no detail'}"
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if (
                    "below realtime rate" in str(exc)
                    and plan.encoder != "copy"
                    and self._video_degrade_level
                    < ENCODED_VIDEO_MAX_DEGRADE_LEVEL
                ):
                    self._video_degrade_level += 1
                healthy_run = (
                    progress.published > 0
                    and time.monotonic() - progress.started_at
                    >= ENCODED_VIDEO_HEALTHY_RESET_SECONDS
                )
                if healthy_run:
                    delay = 1.0
                if (
                    child_exited == "ffmpeg"
                    and plan.encoder not in {"copy", "libx264"}
                ):
                    self._failed_video_encoders.add(plan.encoder)
                    self._encoder_cache = None
                    logger.warning(
                        "hardware video encoder failed; selecting fallback encoder=%s",
                        plan.encoder,
                    )
                if keyframe_reconnect:
                    # PLI/FIR recovery intentionally reconnects the existing
                    # source. It is not a source/encoder failure, so do not
                    # increase backoff or spend another probe interval here.
                    delay = 1.0
                    refresh_plan = False
                else:
                    refresh_plan = True
                safe_error = redact_media_source(
                    str(exc).replace(token, "<redacted>"),
                    self.config.simulator_media_source,
                )
                logger.warning(
                    "encoded video unavailable; retrying in %.1fs error=%s",
                    delay,
                    safe_error,
                )
            finally:
                for process in reversed(processes):
                    await self._stop_process(process)
                for task in output_tasks:
                    if not task.done():
                        task.cancel()
                if output_tasks:
                    await asyncio.gather(*output_tasks, return_exceptions=True)
            await asyncio.sleep(delay)
            delay = 1.0 if keyframe_reconnect else min(15.0, delay * 2)

    async def _watch_encoded_video_progress(
        self,
        progress: EncodedPipelineProgress,
        processes: list[asyncio.subprocess.Process],
        plan: EncodedVideoPlan,
    ) -> None:
        last_stats_at = progress.started_at
        previous_received = 0
        previous_published = 0
        rate_started_at = 0.0
        rate_started_published = 0
        while True:
            await asyncio.sleep(0.5)
            if any(process.returncode is not None for process in processes):
                return
            now = time.monotonic()
            if progress.last_progress_at == 0:
                if now - progress.started_at > ENCODED_VIDEO_STARTUP_TIMEOUT_SECONDS:
                    raise RuntimeError(
                        "encoded video watchdog: no access unit received during startup"
                    )
                continue
            if rate_started_at == 0:
                rate_started_at = now
                rate_started_published = progress.published
            elif now - rate_started_at >= ENCODED_VIDEO_RATE_WINDOW_SECONDS:
                rate_interval = now - rate_started_at
                published_fps = (
                    progress.published - rate_started_published
                ) / rate_interval
                expected_fps = float(self._publisher_video_fps(plan))
                if (
                    progress.published > rate_started_published
                    and published_fps
                    < expected_fps * ENCODED_VIDEO_MIN_RATE_RATIO
                ):
                    logger.warning(
                        "encoded video watchdog restart reason=below-realtime-rate "
                        "route=%s encoder=%s published_fps=%.1f target_fps=%.1f",
                        plan.mode,
                        plan.encoder,
                        published_fps,
                        expected_fps,
                    )
                    raise RuntimeError(
                        "encoded video watchdog: output below realtime rate"
                    )
                rate_started_at = now
                rate_started_published = progress.published
            if now - progress.last_progress_at > ENCODED_VIDEO_STALL_TIMEOUT_SECONDS:
                logger.warning(
                    "encoded video watchdog restart reason=no-progress route=%s "
                    "encoder=%s received=%d published=%d last_frame_age_ms=%d",
                    plan.mode,
                    plan.encoder,
                    progress.received,
                    progress.published,
                    round((now - progress.last_progress_at) * 1000),
                )
                raise RuntimeError("encoded video watchdog: pipeline made no progress")
            if now - last_stats_at >= 15:
                interval = max(0.001, now - last_stats_at)
                logger.info(
                    "encoded video stats route=%s encoder=%s input_fps=%.1f "
                    "published_fps=%.1f backlog_frames=%d backlog_ms=%d "
                    "last_frame_age_ms=%d",
                    plan.mode,
                    plan.encoder,
                    (progress.received - previous_received) / interval,
                    (progress.published - previous_published) / interval,
                    max(0, progress.received - progress.published),
                    round(
                        max(0, progress.received - progress.published)
                        * 1000
                        / max(1.0, float(self._publisher_video_fps(plan)))
                    ),
                    round((now - progress.last_progress_at) * 1000),
                )
                last_stats_at = now
                previous_received = progress.received
                previous_published = progress.published

    async def _start_bridge_media_processes(
        self,
        token: str,
        plan: EncodedVideoPlan,
        progress: EncodedPipelineProgress,
    ) -> tuple[
        list[asyncio.subprocess.Process],
        list[asyncio.Task[str]],
        list[str],
    ]:
        read_fd, write_fd = os.pipe()
        publisher: asyncio.subprocess.Process | None = None
        try:
            # An OS pipe removes the race between FFmpeg's TCP listener and
            # GStreamer's TCP client and keeps the compressed H.264 stream
            # entirely local without another buffering layer.
            publisher = await asyncio.create_subprocess_exec(
                *self._publisher_command(
                    token,
                    self._bridge_h264_pipeline(plan),
                    plan,
                ),
                stdin=read_fd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            os.close(read_fd)
            read_fd = -1
            ffmpeg = await asyncio.create_subprocess_exec(
                *self._encoded_ffmpeg_command(plan),
                stdout=write_fd,
                stderr=asyncio.subprocess.PIPE,
            )
            os.close(write_fd)
            write_fd = -1
            return (
                [ffmpeg, publisher],
                [
                    asyncio.create_task(
                        self._capture_stream_output(ffmpeg.stderr)
                    ),
                    asyncio.create_task(
                        self._capture_stream_output(publisher.stdout, progress)
                    ),
                ],
                ["ffmpeg", "publisher"],
            )
        except Exception:
            await self._stop_process(publisher)
            raise
        finally:
            if read_fd >= 0:
                os.close(read_fd)
            if write_fd >= 0:
                os.close(write_fd)

    @staticmethod
    async def _start_media_process(
        command: list[str],
        progress: EncodedPipelineProgress,
    ) -> tuple[asyncio.subprocess.Process, asyncio.Task[str]]:
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        return process, asyncio.create_task(
            MediaPublisher._capture_process_output(process, progress)
        )

    @staticmethod
    async def _capture_process_output(
        process: asyncio.subprocess.Process,
        progress: EncodedPipelineProgress | None = None,
    ) -> str:
        return await MediaPublisher._capture_stream_output(process.stdout, progress)

    @staticmethod
    async def _capture_stream_output(
        stream: asyncio.StreamReader | None,
        progress: EncodedPipelineProgress | None = None,
    ) -> str:
        if stream is None:
            return ""
        tail = ""
        while True:
            chunk = await stream.readline()
            if not chunk:
                return tail.strip()
            line = chunk.decode(errors="replace")
            if progress is not None:
                MediaPublisher._update_encoded_progress(progress, line)
            tail = (tail + line)[-4000:]

    @staticmethod
    def _update_encoded_progress(
        progress: EncodedPipelineProgress, line: str
    ) -> None:
        match = re.search(
            r"video-progress\s+received=(\d+)\s+published=(\d+)", line
        )
        if not match:
            return
        received, published = (int(value) for value in match.groups())
        now = time.monotonic()
        if received > progress.received:
            progress.received = received
            progress.last_received_at = now
        if published > progress.published:
            progress.published = published
            progress.last_published_at = now

    async def _video_loop(self, source: rtc.VideoSource, width: int, height: int) -> None:
        if self.config.simulator_media_source_type != "test":
            await self._ffmpeg_video_loop(source, width, height)
            return
        phase = 0
        while True:
            row = bytearray()
            for x in range(width):
                value = int(28 + 18 * math.sin((x + phase) / 35))
                row.extend((value, 45, 68, 255))
            data = bytes(row) * height
            source.capture_frame(
                rtc.VideoFrame(width, height, rtc.VideoBufferType.RGBA, data),
                timestamp_us=time.monotonic_ns() // 1_000,
            )
            phase = (phase + 4) % 1000
            await asyncio.sleep(0.1)

    def _resolved_rtsp_source(self) -> str:
        media_source = self.config.simulator_media_source
        if not media_source:
            raise ValueError("SIMULATOR_MEDIA_SOURCE is required for RTSP input")
        parsed = urlparse(media_source)
        if self.config.simulator_rtsp_path and parsed.path in ("", "/"):
            media_source = (
                media_source.rstrip("/")
                + "/"
                + self.config.simulator_rtsp_path.lstrip("/")
            )
        return media_source

    def _video_input_args(self, *, preserve_timestamps: bool = False) -> list[str]:
        kind = self.config.simulator_media_source_type
        media_source = self.config.simulator_media_source
        if kind == "file":
            if not media_source:
                raise ValueError("SIMULATOR_MEDIA_SOURCE is required for file input")
            return ["-stream_loop", "-1", "-re", "-i", media_source]
        if kind == "rtsp":
            media_source = self._resolved_rtsp_source()
            configured_transport = self.config.rtsp_transport.strip().lower()
            transport = self._rtsp_selected_transport or (
                "udp" if configured_transport == "auto" else configured_transport
            )
            # TCP already guarantees packet order, so a large RTP reorder queue
            # only makes stale video accumulate when decoding briefly falls
            # behind. Keep a small socket/input queue and ask FFmpeg to emit
            # decoded frames as soon as possible.
            low_latency = [
                "-fflags", (
                    "+discardcorrupt+nobuffer"
                    if preserve_timestamps
                    else "+genpts+discardcorrupt+nobuffer"
                ),
                "-flags", "low_delay",
                # Cap input backlog below half a second for common 25/30 fps
                # cameras. Preserving a larger queue only preserves stale video.
                "-thread_queue_size", "8",
                "-rtsp_transport", transport,
                "-buffer_size", "262144",
            ]
            if not preserve_timestamps:
                low_latency.extend(["-use_wallclock_as_timestamps", "1"])
            if transport == "tcp":
                low_latency.extend([
                    "-max_delay", "0",
                    "-reorder_queue_size", "0",
                ])
            else:
                # UDP still needs a short reorder window to tolerate normal
                # network jitter, but cap it well below the previous 500 ms.
                low_latency.extend([
                    "-max_delay", "100000",
                    "-reorder_queue_size", "32",
                ])
            return [
                *low_latency,
                "-i", media_source,
            ]
        if kind == "camera":
            profile = self._camera_capture_profile_for_source()
            arguments = [
                # Use wall-clock capture timestamps and keep only one packet in
                # the demux queue. The output fps filter then collapses surplus
                # frames before encode instead of stretching their timestamps.
                "-fflags", "+discardcorrupt+nobuffer",
                "-flags", "low_delay",
                "-thread_queue_size", "1",
                "-use_wallclock_as_timestamps", "1",
                "-f", "v4l2",
                "-framerate", fps_text(profile.fps),
                "-video_size",
                f"{profile.width}x{profile.height}",
            ]
            camera_format = profile.input_format
            if camera_format:
                arguments.extend(["-input_format", camera_format])
            return [*arguments, "-i", profile.device]
        raise ValueError(f"unsupported media source type: {kind}")

    async def _ffmpeg_video_loop(
        self, source: rtc.VideoSource, width: int, height: int
    ) -> None:
        frame_size = width * height * 3 // 2
        frame_interval = 1.0 / self.config.video_fps
        delay = 1.0
        while True:
            process = None
            reader_task: asyncio.Task | None = None
            try:
                command = [
                    "ffmpeg", "-hide_banner", "-loglevel", "error",
                    *self._video_input_args(),
                    "-an",
                    "-vf",
                    (
                        f"fps={self.config.video_fps},"
                        f"scale={width}:{height}:force_original_aspect_ratio=increase,"
                        f"crop={width}:{height}"
                    ),
                    "-pix_fmt", "yuv420p", "-f", "rawvideo", "pipe:1",
                ]
                process = await asyncio.create_subprocess_exec(
                    *command,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    # asyncio's 64 KiB default is tiny compared with a 3.1 MiB
                    # Full HD I420 frame. It repeatedly pauses FFmpeg mid-frame,
                    # turning a steady RTSP stream into visible 40/80 ms bursts.
                    limit=video_pipe_buffer_limit(frame_size),
                )
                assert process.stdout
                frames: asyncio.Queue[bytes | Exception] = asyncio.Queue(
                    maxsize=VIDEO_JITTER_BUFFER_FRAMES
                )
                reader_task = asyncio.create_task(
                    self._read_video_frames(process.stdout, frame_size, frames)
                )

                # FFmpeg can emit decoded RTSP frames in short bursts even when the
                # source timestamps are a steady 25/30 fps. Keep only two frames,
                # prefill one interval, then always select the freshest decoded
                # frame so the queue cannot accumulate live-stream latency.
                frame = await self._next_video_frame(frames)
                await asyncio.sleep(frame_interval)
                frame = self._latest_video_frame(frame, frames)
                next_capture_at = asyncio.get_running_loop().time()
                while True:
                    now = asyncio.get_running_loop().time()
                    if now < next_capture_at:
                        await asyncio.sleep(next_capture_at - now)
                    elif now - next_capture_at > frame_interval:
                        # Do not try to catch up with a burst after the event loop
                        # or decoder was delayed; that would recreate the stutter.
                        next_capture_at = now
                    frame = self._latest_video_frame(frame, frames)
                    source.capture_frame(
                        rtc.VideoFrame(width, height, rtc.VideoBufferType.I420, frame),
                        timestamp_us=time.monotonic_ns() // 1_000,
                    )
                    next_capture_at += frame_interval
                    frame = await self._next_video_frame(frames)
                    delay = 1.0
            except asyncio.TimeoutError:
                await self._stop_process(process)
                detail = await self._process_error(process)
                logger.warning(
                    "video source timed out; reconnecting kind=%s detail=%s",
                    self.config.simulator_media_source_type,
                    detail or f"no complete frame for {VIDEO_FRAME_TIMEOUT_SECONDS}s",
                )
            except asyncio.IncompleteReadError:
                detail = await self._process_error(process)
                logger.warning(
                    "video source ended; reconnecting kind=%s detail=%s",
                    self.config.simulator_media_source_type,
                    detail or "no frames",
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("video source error; retrying error=%s", exc)
            finally:
                # Keep draining stdout until FFmpeg exits. Cancelling the reader
                # first can leave asyncio's pipe transport paused on a full
                # buffer, causing process.wait() to hang even after SIGKILL.
                await self._stop_process(process)
                if reader_task:
                    reader_task.cancel()
                    try:
                        await reader_task
                    except asyncio.CancelledError:
                        pass
            await asyncio.sleep(delay)
            delay = min(15.0, delay * 2)

    async def _read_video_frames(
        self,
        stream: asyncio.StreamReader,
        frame_size: int,
        frames: asyncio.Queue[bytes | Exception],
    ) -> None:
        try:
            while True:
                frame = await stream.readexactly(frame_size)
                if frames.full():
                    frames.get_nowait()
                frames.put_nowait(frame)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if frames.full():
                frames.get_nowait()
            frames.put_nowait(exc)

    async def _next_video_frame(
        self, frames: asyncio.Queue[bytes | Exception]
    ) -> bytes:
        item = await asyncio.wait_for(
            frames.get(), timeout=VIDEO_FRAME_TIMEOUT_SECONDS
        )
        if isinstance(item, Exception):
            raise item
        return item

    @staticmethod
    def _latest_video_frame(
        current: bytes, frames: asyncio.Queue[bytes | Exception]
    ) -> bytes:
        """Drain a short burst and return the newest frame without waiting."""
        while True:
            try:
                item = frames.get_nowait()
            except asyncio.QueueEmpty:
                return current
            if isinstance(item, Exception):
                raise item
            current = item

    async def _process_error(self, process: asyncio.subprocess.Process | None) -> str:
        if not process or not process.stderr:
            return ""
        try:
            raw = await asyncio.wait_for(process.stderr.read(), timeout=2)
        except Exception:
            return ""
        detail = raw.decode(errors="replace").strip().replace(
            self.config.simulator_media_source, "<media-source>"
        )
        return detail[-500:]

    async def _stop_process(
        self, process: asyncio.subprocess.Process | None
    ) -> None:
        if not process or process.returncode is not None:
            return
        try:
            process.terminate()
        except ProcessLookupError:
            return
        try:
            await asyncio.wait_for(process.wait(), timeout=1.5)
            return
        except asyncio.TimeoutError:
            logger.warning("media child process did not terminate; killing it")
        try:
            process.kill()
        except ProcessLookupError:
            return
        try:
            await asyncio.wait_for(process.wait(), timeout=1.5)
        except asyncio.TimeoutError:
            logger.error("media child process did not exit after kill")

    async def _publish_audio(self) -> None:
        assert self.room
        sample_rate, channels = 48_000, 1
        # The SDK default is a one-second internal queue. That is suitable for
        # synthesized speech, not telepresence: a short CPU/network stall would
        # otherwise be replayed long after it is useful.
        source = rtc.AudioSource(
            sample_rate,
            channels,
            queue_size_ms=AUDIO_LIVEKIT_QUEUE_MS,
        )
        track = rtc.LocalAudioTrack.create_audio_track("robot-microphone", source)
        await self.room.local_participant.publish_track(
            track, rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE)
        )
        self.tasks.append(asyncio.create_task(self._audio_loop(source, sample_rate, channels)))

    async def _audio_loop(self, source: rtc.AudioSource, sample_rate: int, channels: int) -> None:
        samples = AUDIO_FRAME_SAMPLES
        if self._uses_full_duplex_aec():
            await self._full_duplex_audio_loop(
                source, sample_rate, channels, samples
            )
            return
        if (
            self.config.simulator_audio_source_type != "silent"
            and self.config.simulator_audio_source
        ):
            await self._ffmpeg_audio_loop(source, sample_rate, channels, samples)
            return
        silence = struct.pack("<" + "h" * samples, *([0] * samples))
        while True:
            frame = rtc.AudioFrame(silence, sample_rate, channels, samples)
            await source.capture_frame(frame)
            await asyncio.sleep(samples / sample_rate)

    def _uses_full_duplex_aec(self) -> bool:
        return (
            self.config.simulator_audio_source_type == "device"
            and bool(self.config.simulator_audio_source)
            and self.config.simulator_audio_output_type == "device"
            and bool(self.config.simulator_audio_output)
        )

    def _audio_duplex_command(self) -> list[str]:
        """Build one top-level pipeline for capture, render, and AEC reference."""
        microphone = self.config.simulator_audio_source
        speaker = self.config.simulator_audio_output
        capture_error = prepare_audio_source(microphone)
        playback_error = prepare_audio_output(speaker)
        if capture_error:
            raise ValueError(capture_error)
        if playback_error:
            raise ValueError(playback_error)

        source_element = "pulsesrc" if microphone.startswith("pulse:") else "alsasrc"
        source_device = microphone.removeprefix("pulse:")
        output_args = audio_output_args(speaker)
        output_format = output_args[1]
        output_device = output_args[-1]
        sink_element = "pulsesink" if output_format == "pulse" else "alsasink"
        caps = "audio/x-raw,format=S16LE,layout=interleaved,rate=48000,channels=1"
        buffer_us = AUDIO_DEVICE_BUFFER_MS * 1000
        period_us = AUDIO_DEVICE_PERIOD_MS * 1000
        return [
            "gst-launch-1.0",
            "-q",
            # Render branch. The bounded leaky raw-audio queue discards old PCM
            # before the echo probe, so the reverse stream represents audio
            # that is actually about to reach the loudspeaker.
            "fdsrc",
            "fd=0",
            "do-timestamp=true",
            f"blocksize={AUDIO_FRAME_BYTES}",
            "!",
            "rawaudioparse",
            "use-sink-caps=false",
            "format=pcm",
            "pcm-format=s16le",
            "sample-rate=48000",
            "num-channels=1",
            "!",
            caps,
            "!",
            "queue",
            "max-size-buffers=2",
            "max-size-bytes=0",
            "max-size-time=0",
            "leaky=downstream",
            "!",
            "webrtcechoprobe",
            "name=robot_echo_reference",
            "!",
            sink_element,
            f"device={self._gst_quote(output_device)}",
            f"buffer-time={buffer_us}",
            f"latency-time={period_us}",
            "sync=true",
            # Capture branch. DSP receives the render reference from the probe
            # above inside this same top-level GstPipeline.
            source_element,
            f"device={self._gst_quote(source_device)}",
            f"buffer-time={buffer_us}",
            f"latency-time={period_us}",
            "do-timestamp=true",
            "!",
            caps,
            "!",
            "queue",
            "max-size-buffers=2",
            "max-size-bytes=0",
            "max-size-time=0",
            "leaky=downstream",
            "!",
            "webrtcdsp",
            "probe=robot_echo_reference",
            "echo-cancel=true",
            "noise-suppression=true",
            "gain-control=true",
            "high-pass-filter=true",
            "delay-agnostic=true",
            "!",
            caps,
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]

    @staticmethod
    def _require_aec_plugins() -> None:
        if shutil.which("gst-launch-1.0") is None:
            raise RuntimeError("AEC unavailable: gst-launch-1.0 is missing")
        for element in ("webrtcechoprobe", "webrtcdsp"):
            try:
                result = subprocess.run(
                    ["gst-inspect-1.0", element],
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=4,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError(f"AEC unavailable: cannot inspect {element}") from exc
            if result.returncode != 0:
                raise RuntimeError(f"AEC unavailable: GStreamer element {element} is missing")

    async def _write_duplex_playback(
        self, stream: asyncio.StreamWriter
    ) -> None:
        silence = bytes(AUDIO_FRAME_BYTES)
        pending = bytearray()
        loop = asyncio.get_running_loop()
        next_write_at = loop.time()
        while True:
            # Drain queued far-end frames, but cap retained PCM to the most
            # recent 20 ms. This prevents a recovered speaker from replaying a
            # sentence fragment that is no longer live.
            while True:
                try:
                    pending.extend(self._operator_audio_frames.get_nowait())
                except asyncio.QueueEmpty:
                    break
            if len(pending) > AUDIO_FRAME_BYTES * AUDIO_PLAYBACK_BUFFER_FRAMES:
                del pending[:-AUDIO_FRAME_BYTES * AUDIO_PLAYBACK_BUFFER_FRAMES]
                logger.warning("audio playback resync reason=application-backlog")
            data = bytes(pending[:AUDIO_FRAME_BYTES]) if pending else silence
            if pending:
                del pending[:AUDIO_FRAME_BYTES]
            if len(data) < AUDIO_FRAME_BYTES:
                data += bytes(AUDIO_FRAME_BYTES - len(data))
            stream.write(data)
            await self._drain_audio_stream(stream)
            self.audio_playback_healthy = True
            next_write_at += AUDIO_FRAME_MS / 1000
            now = loop.time()
            if now - next_write_at > AUDIO_FRAME_MS / 1000:
                next_write_at = now
                pending.clear()
                logger.warning("audio playback resync reason=writer-late")
            await asyncio.sleep(max(0, next_write_at - loop.time()))

    @staticmethod
    async def _drain_audio_stream(stream: asyncio.StreamWriter) -> None:
        """Bound a blocked audio device without asyncio.wait_for's cancel race."""
        drain_task = asyncio.create_task(stream.drain())
        try:
            done, _pending = await asyncio.wait(
                {drain_task}, timeout=AUDIO_PLAYBACK_WRITE_TIMEOUT_SECONDS
            )
            if not done:
                drain_task.cancel()
                await asyncio.gather(drain_task, return_exceptions=True)
                raise TimeoutError("audio playback writer stalled")
            await drain_task
        except asyncio.CancelledError:
            drain_task.cancel()
            await asyncio.gather(drain_task, return_exceptions=True)
            raise

    async def _full_duplex_audio_loop(
        self,
        source: rtc.AudioSource,
        sample_rate: int,
        channels: int,
        samples: int,
    ) -> None:
        delay = 1.0
        while True:
            process: asyncio.subprocess.Process | None = None
            playback_task: asyncio.Task[None] | None = None
            output_task: asyncio.Task[str] | None = None
            try:
                await asyncio.to_thread(self._require_aec_plugins)
                command = self._audio_duplex_command()
                process = await asyncio.create_subprocess_exec(
                    *command,
                    stdin=asyncio.subprocess.PIPE,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    limit=AUDIO_FRAME_BYTES * 4,
                )
                assert process.stdin and process.stdout
                output_task = asyncio.create_task(
                    self._capture_stream_output(process.stderr)
                )
                playback_task = asyncio.create_task(
                    self._write_duplex_playback(process.stdin)
                )
                self.aec_active = True
                logger.info(
                    "audio processing initialized aec=true ns=true agc=true "
                    "sample_rate=%d frame_ms=%d device_buffer_ms=%d",
                    sample_rate,
                    AUDIO_FRAME_MS,
                    AUDIO_DEVICE_BUFFER_MS,
                )
                while True:
                    data = await asyncio.wait_for(
                        process.stdout.readexactly(samples * channels * 2),
                        timeout=AUDIO_CAPTURE_STALL_TIMEOUT_SECONDS,
                    )
                    if source.queued_duration > AUDIO_LIVEKIT_QUEUE_MS / 1000:
                        source.clear_queue()
                        logger.warning("audio capture resync reason=livekit-backlog")
                    await source.capture_frame(
                        rtc.AudioFrame(data, sample_rate, channels, samples)
                    )
                    self.audio_capture_healthy = True
                    delay = 1.0
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                detail = ""
                if output_task and output_task.done():
                    result = output_task.result()
                    detail = f" detail={result[-500:]}" if result else ""
                logger.warning(
                    "audio duplex unavailable; retrying in %.1fs aec=false error=%s%s",
                    delay,
                    exc,
                    detail,
                )
            finally:
                self.aec_active = False
                self.audio_capture_healthy = False
                self.audio_playback_healthy = False
                if playback_task:
                    playback_task.cancel()
                    await asyncio.gather(playback_task, return_exceptions=True)
                await self._stop_process(process)
                if output_task:
                    if not output_task.done():
                        output_task.cancel()
                    await asyncio.gather(output_task, return_exceptions=True)
            await asyncio.sleep(delay)
            delay = min(15.0, delay * 2)

    async def _ffmpeg_audio_loop(
        self, source: rtc.AudioSource, sample_rate: int, channels: int, samples: int
    ) -> None:
        frame_size = samples * channels * 2
        delay = 1.0
        while True:
            process = None
            try:
                command = self._audio_capture_command(sample_rate, channels)
                process = await asyncio.create_subprocess_exec(
                    *command,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                logger.info(
                    "microphone capture started backend=%s",
                    command[0],
                )
                assert process.stdout
                while True:
                    data = await asyncio.wait_for(
                        process.stdout.readexactly(frame_size),
                        timeout=AUDIO_CAPTURE_STALL_TIMEOUT_SECONDS,
                    )
                    if source.queued_duration > AUDIO_LIVEKIT_QUEUE_MS / 1000:
                        source.clear_queue()
                        logger.warning("audio capture resync reason=livekit-backlog")
                    await source.capture_frame(
                        rtc.AudioFrame(data, sample_rate, channels, samples)
                    )
                    self.audio_capture_healthy = True
                    delay = 1.0
            except asyncio.IncompleteReadError:
                logger.warning("audio source ended; reconnecting")
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("audio source error; retrying error=%s", exc)
            finally:
                self.audio_capture_healthy = False
                await self._stop_process(process)
            await asyncio.sleep(delay)
            delay = min(15.0, delay * 2)

    def _audio_capture_command(
        self, sample_rate: int, channels: int
    ) -> list[str]:
        if self.config.simulator_audio_source_type == "device":
            audio_source = self.config.simulator_audio_source or "default"
            unavailable_reason = prepare_audio_source(audio_source)
            if unavailable_reason:
                raise ValueError(unavailable_reason)
            if audio_source.startswith("pulse:") and shutil.which("pacat"):
                return [
                    "pacat",
                    "--record",
                    "--raw",
                    f"--device={audio_source.removeprefix('pulse:')}",
                    f"--rate={sample_rate}",
                    "--format=s16le",
                    f"--channels={channels}",
                    f"--latency-msec={AUDIO_PULSE_CAPTURE_LATENCY_MS}",
                    f"--process-time-msec={AUDIO_PULSE_CAPTURE_PROCESS_MS}",
                    "--client-name=rovera-robot-edge",
                    "--stream-name=robot-microphone",
                ]
            if not audio_source.startswith("pulse:") and shutil.which("arecord"):
                return [
                    "arecord",
                    "--quiet",
                    f"--device={audio_source}",
                    "--file-type=raw",
                    "--format=S16_LE",
                    f"--rate={sample_rate}",
                    f"--channels={channels}",
                    f"--buffer-time={AUDIO_DEVICE_BUFFER_MS * 1000}",
                    f"--period-time={AUDIO_DEVICE_PERIOD_MS * 1000}",
                ]
        return [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            *self._audio_input_args(),
            "-vn",
            "-ac",
            str(channels),
            "-ar",
            str(sample_rate),
            "-f",
            "s16le",
            "pipe:1",
        ]

    def _audio_input_args(self) -> list[str]:
        if self.config.simulator_audio_source_type == "device":
            audio_source = self.config.simulator_audio_source or "default"
            unavailable_reason = prepare_audio_source(audio_source)
            if unavailable_reason:
                raise ValueError(unavailable_reason)
            if audio_source.startswith("pulse:"):
                return [
                    "-f",
                    "pulse",
                    "-sample_rate",
                    "48000",
                    "-channels",
                    "1",
                    "-fragment_size",
                    "1920",
                    "-i",
                    audio_source.removeprefix("pulse:"),
                ]
            return ["-f", "alsa", "-i", audio_source]
        return [
            "-stream_loop", "-1", "-re", "-i", self.config.simulator_audio_source
        ]

    def _audio_output_command(self) -> list[str]:
        unavailable_reason = prepare_audio_output(
            self.config.simulator_audio_output
        )
        if unavailable_reason:
            raise ValueError(unavailable_reason)
        output_args = audio_output_args(self.config.simulator_audio_output)
        output_format = output_args[1]
        resolved_output = output_args[-1]
        if output_format == "pulse" and shutil.which("pacat"):
            return [
                "pacat",
                "--playback",
                "--raw",
                f"--device={resolved_output}",
                "--rate=48000",
                "--format=s16le",
                "--channels=1",
                f"--latency-msec={AUDIO_DEVICE_BUFFER_MS}",
                f"--process-time-msec={AUDIO_DEVICE_PERIOD_MS}",
                "--client-name=rovera-robot-edge",
                "--stream-name=operator-voice",
            ]
        if output_format == "alsa" and shutil.which("aplay"):
            return [
                "aplay",
                "--quiet",
                f"--device={resolved_output}",
                "--file-type=raw",
                "--format=S16_LE",
                "--rate=48000",
                "--channels=1",
                f"--buffer-time={AUDIO_DEVICE_BUFFER_MS * 1000}",
                f"--period-time={AUDIO_DEVICE_PERIOD_MS * 1000}",
            ]
        logger.warning(
            "native low-latency audio player unavailable; using FFmpeg output=%s",
            self.config.simulator_audio_output,
        )
        return [
            "ffmpeg",
            "-hide_banner",
            "-nostdin",
            "-loglevel",
            "error",
            "-f",
            "s16le",
            "-ar",
            "48000",
            "-ac",
            "1",
            "-i",
            "pipe:0",
            "-ar",
            "48000",
            "-ac",
            "1",
            *output_args,
        ]

    @staticmethod
    def _queue_latest_audio_frame(
        frames: asyncio.Queue[bytes], data: bytes
    ) -> None:
        if frames.full():
            try:
                frames.get_nowait()
            except asyncio.QueueEmpty:
                pass
        frames.put_nowait(data)

    async def _audio_playback_loop(
        self, frames: asyncio.Queue[bytes]
    ) -> None:
        process: asyncio.subprocess.Process | None = None
        output_task: asyncio.Task[str] | None = None
        retry_at = 0.0
        retry_delay = 1.0

        async def stop_output() -> None:
            nonlocal process, output_task
            if process and process.stdin:
                process.stdin.close()
            await self._stop_process(process)
            if output_task:
                if not output_task.done():
                    output_task.cancel()
                await asyncio.gather(output_task, return_exceptions=True)
            process = None
            output_task = None

        try:
            while True:
                if process is None:
                    if time.monotonic() < retry_at:
                        await asyncio.sleep(retry_at - time.monotonic())
                    try:
                        command = self._audio_output_command()
                        process = await asyncio.create_subprocess_exec(
                            *command,
                            stdin=asyncio.subprocess.PIPE,
                            stdout=asyncio.subprocess.PIPE,
                            stderr=asyncio.subprocess.STDOUT,
                        )
                        output_task = asyncio.create_task(
                            self._capture_process_output(process)
                        )
                        # Give immediate device-open failures one event-loop
                        # turn to surface before accepting more PCM.
                        await asyncio.sleep(0)
                        if process.returncode is not None:
                            detail = await output_task
                            raise RuntimeError(detail or "Không mở được loa")
                        logger.info(
                            "speaker playback started output=%s backend=%s target_ms=%d",
                            self.config.simulator_audio_output,
                            command[0],
                            AUDIO_DEVICE_BUFFER_MS,
                        )
                        retry_delay = 1.0
                    except asyncio.CancelledError:
                        raise
                    except Exception as exc:
                        await stop_output()
                        logger.warning(
                            "speaker unavailable; retrying in %.1fs error=%s",
                            retry_delay,
                            exc,
                        )
                        retry_at = time.monotonic() + retry_delay
                        retry_delay = min(15.0, retry_delay * 2)
                        continue
                data = await frames.get()
                try:
                    if process.stdin is None:
                        raise BrokenPipeError("Audio player stdin unavailable")
                    process.stdin.write(data)
                    await self._drain_audio_stream(process.stdin)
                    self.audio_playback_healthy = True
                except asyncio.CancelledError:
                    raise
                except (BrokenPipeError, ConnectionError, OSError) as exc:
                    detail = ""
                    if output_task and output_task.done():
                        try:
                            detail = output_task.result()
                        except Exception as output_error:
                            detail = str(output_error)
                    logger.warning(
                        "speaker playback stopped; reconnecting error=%s",
                        detail or exc,
                    )
                    await stop_output()
                    retry_at = time.monotonic() + retry_delay
                    retry_delay = min(15.0, retry_delay * 2)
        finally:
            self.audio_playback_healthy = False
            await stop_output()

    async def _consume_audio(self, track: rtc.AudioTrack) -> None:
        stream = rtc.AudioStream(
            track,
            capacity=AUDIO_PLAYBACK_BUFFER_FRAMES,
            sample_rate=48_000,
            num_channels=1,
            frame_size_ms=AUDIO_FRAME_MS,
        )
        playback_frames = (
            self._operator_audio_frames
            if self._uses_full_duplex_aec()
            else asyncio.Queue(maxsize=AUDIO_PLAYBACK_BUFFER_FRAMES)
        )
        playback_task: asyncio.Task[None] | None = None
        if (
            not self._uses_full_duplex_aec()
            and self.config.simulator_audio_output_type == "device"
            and self.config.simulator_audio_output
        ):
            playback_task = asyncio.create_task(
                self._audio_playback_loop(playback_frames)
            )
        try:
            async for event in stream:
                samples = event.frame.data
                if samples:
                    self.audio_level = min(
                        1.0, max(abs(value) for value in samples) / 32768
                    )
                    if playback_task or self._uses_full_duplex_aec():
                        self._queue_latest_audio_frame(
                            playback_frames, bytes(samples)
                        )
        finally:
            if playback_task:
                playback_task.cancel()
                await asyncio.gather(playback_task, return_exceptions=True)

    async def disconnect(self) -> None:
        self.connected = False
        self.aec_active = False
        self.audio_capture_healthy = False
        self.audio_playback_healthy = False
        while not self._operator_audio_frames.empty():
            try:
                self._operator_audio_frames.get_nowait()
            except asyncio.QueueEmpty:
                break
        tasks, self.tasks = self.tasks, []
        for task in tasks:
            task.cancel()
        if tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*tasks, return_exceptions=True), timeout=4
                )
            except asyncio.TimeoutError:
                logger.warning("timed out while stopping media workers")
        room, self.room = self.room, None
        if room:
            try:
                await asyncio.wait_for(room.disconnect(), timeout=4)
            except asyncio.TimeoutError:
                logger.warning("timed out while disconnecting LiveKit room")
