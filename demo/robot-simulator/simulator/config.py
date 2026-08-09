from typing import Literal

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class SimulatorConfig(BaseSettings):
    center_api_url: str = "http://localhost:8888"
    center_robot_ws_url: str = "ws://localhost:8888/ws/robot/connect"
    center_tls_verify: bool = True
    center_tls_ca_file: str = ""
    robot_id: str = "ROBOT-001"
    robot_credential: str = ""
    robot_enrollment_token: str = ""
    robot_management_address: str = ""
    robot_username: str = ""
    robot_password: str = ""
    robot_state_file: str = ""
    map_id: str = "MAP-001"
    map_width_m: float = 16.0
    map_height_m: float = 10.0
    initial_x: float = 5.5
    initial_y: float = 6.0
    initial_yaw: float = 0.0
    simulation_hz: int = Field(default=30, ge=20, le=50)
    telemetry_hz: int = Field(default=8, ge=5, le=10)
    command_watchdog_ms: int = Field(default=400, ge=300, le=500)
    max_forward_speed: float = 0.4
    max_reverse_speed: float = 0.25
    max_angular_speed: float = 0.8
    # ``disabled`` is the fail-closed mode used while Rovera coexists with a
    # vendor stack that still owns /cmd_vel. It keeps camera, telemetry and
    # mapping online without pretending that Web motion was delivered.
    motion_backend: Literal["disabled", "simulator", "ros2"] = "simulator"
    navigation_backend: Literal["simulator", "ros2"] = "simulator"
    navigation_socket_path: str = "/var/lib/rovera/navigation/navigation.sock"
    navigation_mode_request_path: str = "/var/lib/rovera/navigation/mode-request.json"
    navigation_mode_switch_timeout_seconds: float = Field(default=60.0, ge=15.0, le=120.0)
    map_cache_dir: str = "/var/lib/rovera/maps"
    motion_socket_path: str = "/var/lib/rovera/control/motion.sock"
    motion_watchdog_ms: int = Field(default=250, ge=150, le=500)
    ros_max_forward_speed: float = Field(default=0.33, gt=0, le=1.0)
    ros_max_reverse_speed: float = Field(default=0.25, gt=0, le=1.0)
    ros_max_angular_speed: float = Field(default=0.8, gt=0, le=3.0)
    # Mapping needs slower chassis motion than teleoperation. Wheel slip while
    # turning is otherwise interpreted as laser pose motion and produces the
    # characteristic fan-shaped duplicate walls in the occupancy grid.
    mapping_max_forward_speed: float = Field(default=0.18, gt=0, le=0.4)
    mapping_max_reverse_speed: float = Field(default=0.14, gt=0, le=0.25)
    mapping_max_angular_speed: float = Field(default=0.40, gt=0, le=0.8)
    heartbeat_seconds: float = 2.0
    livekit_url: str = "ws://localhost:7880"
    simulator_media_source_type: str = "test"
    simulator_media_source: str = ""
    simulator_audio_source: str = ""
    simulator_audio_source_type: str = "silent"
    simulator_audio_output: str = ""
    simulator_audio_output_type: str = "disabled"
    simulator_camera_device: str = "/dev/video0"
    simulator_camera_format: str = ""
    # Zero means auto: select one of the discrete modes advertised by the
    # chosen V4L2 camera for the active video profile. Non-zero values remain
    # available as an explicit deployment override.
    simulator_camera_width: int = Field(default=0, ge=0, le=3840)
    simulator_camera_height: int = Field(default=0, ge=0, le=2160)
    simulator_camera_fps: int = Field(default=0, ge=0, le=120)
    device_ip: str = "127.0.0.1"
    camera_label: str = "Camera chính"
    microphone_label: str = "Microphone chính"
    speaker_label: str = "Loa chính"
    video_profile: str = "balanced"
    # Auto tries UDP first for latency and lets GStreamer fall back to TCP.
    rtsp_transport: str = "auto"
    # Preserve clean H.264 without decoding. Auto-normalization only transcodes
    # RTSP sources whose metadata cannot provide a safe, bounded frame cadence.
    # ``rtsp_normalize`` remains a force switch for known-bad vendor streams.
    rtsp_normalize: bool = False
    rtsp_auto_normalize: bool = True
    media_enabled: bool = True
    video_width: int = Field(default=1280, ge=640, le=1920)
    video_height: int = Field(default=720, ge=360, le=1080)
    video_fps: int = Field(default=20, ge=10, le=30)
    video_bitrate: int = Field(default=2_500_000, ge=500_000, le=12_000_000)
    video_encoder: str = "auto"
    video_pipeline: str = "auto"
    video_passthrough: bool = True
    video_ffmpeg_binary: str = "ffmpeg"
    simulator_rtsp_path: str = ""

    model_config = SettingsConfigDict(env_file=".env", extra="ignore")
