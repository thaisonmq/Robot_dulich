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
    heartbeat_seconds: float = 2.0
    livekit_url: str = "ws://localhost:7880"
    simulator_media_source_type: str = "test"
    simulator_media_source: str = ""
    simulator_audio_source: str = ""
    simulator_audio_source_type: str = "silent"
    simulator_camera_device: str = "/dev/video0"
    simulator_camera_format: str = ""
    simulator_camera_width: int = Field(default=1920, ge=320, le=3840)
    simulator_camera_height: int = Field(default=1080, ge=240, le=2160)
    simulator_camera_fps: int = Field(default=25, ge=5, le=60)
    device_ip: str = "127.0.0.1"
    camera_label: str = "Camera chính"
    microphone_label: str = "Microphone chính"
    video_profile: str = "full_hd"
    # Robot and its IP camera normally share the same LAN. UDP avoids TCP
    # head-of-line bursts that turn a steady 25 fps RTP stream into alternating
    # short/long frame intervals. Operators can still select TCP when required.
    rtsp_transport: str = "udp"
    media_enabled: bool = True
    video_width: int = Field(default=1920, ge=640, le=1920)
    video_height: int = Field(default=1080, ge=360, le=1080)
    video_fps: int = Field(default=25, ge=10, le=30)
    video_bitrate: int = Field(default=8_000_000, ge=500_000, le=12_000_000)
    video_encoder: str = "auto"
    video_pipeline: str = "auto"
    video_passthrough: bool = True
    video_ffmpeg_binary: str = "ffmpeg"
    simulator_rtsp_path: str = ""

    model_config = SettingsConfigDict(env_file=".env", extra="ignore")
