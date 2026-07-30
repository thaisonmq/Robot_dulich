import asyncio
import logging
import math
import os
import shutil
import struct
import subprocess
import threading
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

from livekit import rtc
from livekit.rtc._proto import room_pb2 as proto_room

from simulator.config import SimulatorConfig
from simulator.media_devices import prepare_audio_source

logger = logging.getLogger("simulator.media")
VIDEO_FRAME_TIMEOUT_SECONDS = 10
VIDEO_JITTER_BUFFER_FRAMES = 2
VIDEO_PIPE_BUFFER_FRAMES = 4
VIDEO_PACER_BASE_BITRATE = 8_000_000
VIDEO_PACER_MAX_LATENCY_MS = 60


@dataclass(frozen=True)
class EncodedVideoPlan:
    """A video route that never sends raw Full HD frames through Python."""

    mode: str
    source_codec: str
    encoder: str
    ffmpeg_binary: str = "ffmpeg"


def video_pipe_buffer_limit(frame_size: int) -> int:
    """Keep asyncio from pausing FFmpeg dozens of times inside one raw frame."""
    return frame_size * VIDEO_PIPE_BUFFER_FRAMES


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
        self._detected_camera_format = ""
        self._encoder_cache: tuple[str, str] | None = None
        self._vaapi_rate_control = "auto"
        self._vaapi_low_power = False
        self._video_plan_lock = threading.Lock()
        self._prepared_video_key: tuple[object, ...] | None = None
        self._prepared_video_plan: EncodedVideoPlan | None = None

    async def connect(self) -> None:
        if not self.config.media_enabled:
            return
        if self.token_provider is None:
            raise RuntimeError("Center media token provider is not configured")
        await self.disconnect()
        self.room = rtc.Room()

        @self.room.on("track_subscribed")
        def on_track_subscribed(track, _publication, participant) -> None:
            if track.kind == rtc.TrackKind.KIND_AUDIO:
                logger.info("receiving user audio identity=%s", participant.identity)
                self.tasks.append(asyncio.create_task(self._consume_audio(track)))

        @self.room.on("disconnected")
        def on_disconnected(*_args) -> None:
            self.connected = False
            logger.warning("LiveKit disconnected; media publisher will reconnect")

        token = await self.token_provider("main")
        await self.room.connect(self.config.livekit_url, token)
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

        source_codec = self._probe_source_codec()
        if source_codec == "h264" and self.config.video_passthrough:
            mode = (
                "direct"
                if self.config.simulator_media_source_type in {"rtsp", "camera"}
                else "bridge"
            )
            logger.info(
                "video optimization selected mode=%s source_codec=h264 "
                "decode=false encode=false",
                mode,
            )
            return EncodedVideoPlan(mode, source_codec, "copy")

        encoder, binary = self._select_video_encoder()
        if encoder == "h264_vaapi":
            logger.info(
                "video optimization selected mode=bridge source_codec=%s "
                "encoder=%s rate_control=%s low_power=%s",
                source_codec,
                encoder,
                self._vaapi_rate_control,
                self._vaapi_low_power,
            )
        else:
            logger.info(
                "video optimization selected mode=bridge source_codec=%s encoder=%s",
                source_codec,
                encoder,
            )
        return EncodedVideoPlan("bridge", source_codec, encoder, binary)

    def _probe_source_codec(self) -> str:
        kind = self.config.simulator_media_source_type
        if kind == "camera":
            capture_format = self._camera_capture_format()
            self._detected_camera_format = capture_format
            normalized = capture_format.lower().replace(".", "")
            if normalized in {"h264", "avc", "avc1"}:
                return "h264"
            if normalized in {"mjpeg", "mjpg", "jpeg"}:
                return "mjpeg"
            return "rawvideo"

        if kind not in {"rtsp", "file"}:
            return "rawvideo"
        if shutil.which("ffprobe") is None:
            raise RuntimeError("ffprobe is required to inspect the video source")
        if kind == "rtsp":
            input_args = [
                "-rtsp_transport",
                self.config.rtsp_transport,
                "-i",
                self._resolved_rtsp_source(),
            ]
        else:
            input_args = ["-i", self.config.simulator_media_source]
        try:
            result = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    *input_args,
                    "-select_streams",
                    "v:0",
                    "-show_entries",
                    "stream=codec_name",
                    "-of",
                    "default=nokey=1:noprint_wrappers=1",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=8,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("video source codec probe timed out") from exc
        codec = result.stdout.strip().splitlines()
        if result.returncode != 0 or not codec:
            detail = result.stderr.strip().replace(
                self.config.simulator_media_source, "<media-source>"
            )
            raise RuntimeError(
                f"cannot detect video source codec: {detail[-300:] or 'no video stream'}"
            )
        return codec[0].strip().lower()

    def _camera_capture_format(self) -> str:
        configured = self.config.simulator_camera_format.strip()
        if configured:
            return configured
        device = (
            self.config.simulator_media_source
            or self.config.simulator_camera_device
        )
        if shutil.which("v4l2-ctl") is None:
            return ""
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--device", device, "--list-formats-ext"],
                capture_output=True,
                check=False,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        formats = result.stdout.upper()
        if result.returncode == 0 and (
            "'H264'" in formats or "'AVC1'" in formats or "H.264" in formats
        ):
            return "h264"
        return ""

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
        failures: list[str] = []
        for encoder in candidates:
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
            return [
                "rtspsrc",
                f"location={self._gst_quote(self._resolved_rtsp_source())}",
                f"protocols={self.config.rtsp_transport}",
                # Keep a small RTSP reorder buffer. Dropping an encoded H.264
                # delta frame corrupts the rest of its GOP, which appears as
                # scratches until the next keyframe.
                "latency=80",
                "drop-on-latency=false",
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
            device = (
                self.config.simulator_media_source
                or self.config.simulator_camera_device
            )
            return [
                "v4l2src",
                f"device={self._gst_quote(device)}",
                "do-timestamp=true",
                "!",
                (
                    "video/x-h264,"
                    f"width={self.config.simulator_camera_width},"
                    f"height={self.config.simulator_camera_height},"
                    f"framerate={self.config.simulator_camera_fps}/1"
                ),
                "!",
                "h264parse",
                "disable-passthrough=true",
                "config-interval=1",
            ]
        raise ValueError(f"direct H.264 is unsupported for source type: {kind}")

    def _bridge_h264_pipeline(self) -> list[str]:
        return [
            "fdsrc",
            "fd=0",
            "!",
            "tsdemux",
            # tsdemux defaults to 700 ms, which made an otherwise smooth USB
            # camera arrive almost one second late after the browser buffer.
            "latency=0",
            "!",
            "queue",
            # Apply back-pressure to FFmpeg instead of dropping encoded
            # delta frames and leaving the browser with an undecodable GOP.
            # One frame is enough to decouple the processes without letting
            # the clock-synchronised sink sit several frames behind live.
            "max-size-buffers=1",
            "max-size-bytes=0",
            "max-size-time=0",
            "!",
            "h264parse",
            "disable-passthrough=true",
            "config-interval=1",
        ]

    def _publisher_command(self, token: str, pipeline: list[str]) -> list[str]:
        return [
            "gstreamer-publisher",
            "--url",
            self.config.livekit_url,
            "--token",
            token,
            "--pacer-bitrate",
            str(max(self.config.video_bitrate, VIDEO_PACER_BASE_BITRATE)),
            "--pacer-max-latency-ms",
            str(VIDEO_PACER_MAX_LATENCY_MS),
            "--",
            *pipeline,
        ]

    def _video_encoder_args(
        self,
        encoder: str,
        vaapi_mode: str | None = None,
        *,
        vaapi_low_power: bool | None = None,
    ) -> list[str]:
        common = [
            "-g",
            str(self.config.video_fps),
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
                    "8",
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
                str(self.config.video_bitrate),
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
            str(self.config.video_bitrate),
        ]
        if encoder == "libx264":
            return [
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                *rate_control,
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
        command.extend([*self._video_input_args(), "-map", "0:v:0", "-an"])

        if plan.encoder == "copy":
            command.extend(
                ["-c:v", "copy", "-bsf:v", "h264_mp4toannexb"]
            )
        else:
            cadence_filter = (
                f"setpts=N/({self.config.video_fps}*TB)"
                if self.config.simulator_media_source_type == "camera"
                else f"fps={self.config.video_fps}"
            )
            filters = (
                f"{cadence_filter},"
                f"scale={self.config.video_width}:{self.config.video_height}:"
                "force_original_aspect_ratio=increase,"
                f"crop={self.config.video_width}:{self.config.video_height}"
            )
            if plan.encoder in {"h264_vaapi", "h264_rkmpp"}:
                filters += ",format=nv12"
            if plan.encoder == "h264_vaapi":
                filters += ",hwupload"
            command.extend(["-vf", filters, "-c:v", plan.encoder])
            command.extend(self._video_encoder_args(plan.encoder))
        command.extend(
            [
                "-muxdelay",
                "0",
                "-muxpreload",
                "0",
                # Send each muxed frame into the OS pipe immediately instead
                # of waiting for FFmpeg's AVIO buffer to fill on quiet scenes.
                "-flush_packets",
                "1",
                "-f",
                "mpegts",
                "pipe:1",
            ]
        )
        return command

    async def _encoded_video_loop(self, plan: EncodedVideoPlan) -> None:
        delay = 1.0
        while True:
            processes: list[asyncio.subprocess.Process] = []
            output_tasks: list[asyncio.Task[str]] = []
            process_names: list[str] = []
            token = ""
            try:
                assert self.token_provider is not None
                token = await self.token_provider("video")
                if plan.mode == "direct":
                    command = self._publisher_command(
                        token, self._direct_h264_pipeline()
                    )
                    process, output_task = await self._start_media_process(command)
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
                    )

                logger.info(
                    "encoded video publisher started mode=%s encoder=%s",
                    plan.mode,
                    plan.encoder,
                )
                waiters = [asyncio.create_task(process.wait()) for process in processes]
                done, pending = await asyncio.wait(
                    waiters, return_when=asyncio.FIRST_COMPLETED
                )
                for waiter in pending:
                    waiter.cancel()
                await asyncio.gather(*waiters, return_exceptions=True)
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
                detail = " | ".join(labeled_details).replace(
                    token, "<redacted>"
                )
                raise RuntimeError(
                    f"encoded video child exited codes={exit_codes} "
                    f"detail={detail or 'no detail'}"
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                safe_error = str(exc).replace(token, "<redacted>")
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
            delay = min(15.0, delay * 2)

    async def _start_bridge_media_processes(
        self,
        token: str,
        plan: EncodedVideoPlan,
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
                    self._bridge_h264_pipeline(),
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
                        self._capture_stream_output(publisher.stdout)
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
    ) -> tuple[asyncio.subprocess.Process, asyncio.Task[str]]:
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        return process, asyncio.create_task(
            MediaPublisher._capture_process_output(process)
        )

    @staticmethod
    async def _capture_process_output(
        process: asyncio.subprocess.Process,
    ) -> str:
        return await MediaPublisher._capture_stream_output(process.stdout)

    @staticmethod
    async def _capture_stream_output(
        stream: asyncio.StreamReader | None,
    ) -> str:
        if stream is None:
            return ""
        tail = ""
        while True:
            chunk = await stream.read(2048)
            if not chunk:
                return tail.strip()
            tail = (tail + chunk.decode(errors="replace"))[-4000:]

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

    def _video_input_args(self) -> list[str]:
        kind = self.config.simulator_media_source_type
        media_source = self.config.simulator_media_source
        if kind == "file":
            if not media_source:
                raise ValueError("SIMULATOR_MEDIA_SOURCE is required for file input")
            return ["-stream_loop", "-1", "-re", "-i", media_source]
        if kind == "rtsp":
            media_source = self._resolved_rtsp_source()
            # TCP already guarantees packet order, so a large RTP reorder queue
            # only makes stale video accumulate when decoding briefly falls
            # behind. Keep a small socket/input queue and ask FFmpeg to emit
            # decoded frames as soon as possible.
            low_latency = [
                "-fflags", "+genpts+discardcorrupt+nobuffer",
                "-flags", "low_delay",
                "-thread_queue_size", "64",
                "-rtsp_transport", self.config.rtsp_transport,
                "-buffer_size", "262144",
            ]
            if self.config.rtsp_transport == "tcp":
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
            device = media_source or self.config.simulator_camera_device
            arguments = [
                # Do not let stale USB frames accumulate when decode or encode
                # briefly takes longer than one capture interval.
                "-fflags", "+discardcorrupt+nobuffer",
                "-flags", "low_delay",
                "-thread_queue_size", "1",
                "-f", "v4l2",
                "-framerate", str(self.config.simulator_camera_fps),
                "-video_size",
                f"{self.config.simulator_camera_width}x{self.config.simulator_camera_height}",
            ]
            camera_format = (
                self._detected_camera_format
                or self.config.simulator_camera_format
            )
            if camera_format:
                arguments.extend(["-input_format", camera_format])
            return [*arguments, "-i", device]
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
        source = rtc.AudioSource(sample_rate, channels)
        track = rtc.LocalAudioTrack.create_audio_track("robot-microphone", source)
        await self.room.local_participant.publish_track(
            track, rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE)
        )
        self.tasks.append(asyncio.create_task(self._audio_loop(source, sample_rate, channels)))

    async def _audio_loop(self, source: rtc.AudioSource, sample_rate: int, channels: int) -> None:
        samples = 480
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

    async def _ffmpeg_audio_loop(
        self, source: rtc.AudioSource, sample_rate: int, channels: int, samples: int
    ) -> None:
        frame_size = samples * channels * 2
        delay = 1.0
        while True:
            process = None
            try:
                command = [
                    "ffmpeg", "-hide_banner", "-loglevel", "error",
                    *self._audio_input_args(),
                    "-vn", "-ac", str(channels), "-ar", str(sample_rate),
                    "-f", "s16le", "pipe:1",
                ]
                process = await asyncio.create_subprocess_exec(
                    *command,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                assert process.stdout
                while True:
                    data = await process.stdout.readexactly(frame_size)
                    await source.capture_frame(
                        rtc.AudioFrame(data, sample_rate, channels, samples)
                    )
                    delay = 1.0
            except asyncio.IncompleteReadError:
                logger.warning("audio source ended; reconnecting")
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("audio source error; retrying error=%s", exc)
            finally:
                await self._stop_process(process)
            await asyncio.sleep(delay)
            delay = min(15.0, delay * 2)

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
                    "-i",
                    audio_source.removeprefix("pulse:"),
                ]
            return ["-f", "alsa", "-i", audio_source]
        return [
            "-stream_loop", "-1", "-re", "-i", self.config.simulator_audio_source
        ]

    async def _consume_audio(self, track: rtc.AudioTrack) -> None:
        stream = rtc.AudioStream(track)
        async for event in stream:
            samples = event.frame.data
            if samples:
                self.audio_level = min(1.0, max(abs(value) for value in samples) / 32768)

    async def disconnect(self) -> None:
        self.connected = False
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
