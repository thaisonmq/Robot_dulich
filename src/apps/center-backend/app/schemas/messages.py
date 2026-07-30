from datetime import datetime, timezone
from typing import Any, Literal
from uuid import UUID

from pydantic import BaseModel, Field, field_validator, model_validator


def normalize_management_address(value: str) -> str:
    address = value.strip().lower()
    for prefix in ("https://", "http://"):
        if address.startswith(prefix):
            address = address[len(prefix):]
            break
    address = address.rstrip("/")
    if (
        not address
        or any(character.isspace() for character in address)
        or "/" in address
        or "@" in address
    ):
        raise ValueError("Địa chỉ robot phải là IP hoặc hostname hợp lệ")
    return address


class RealtimeMessage(BaseModel):
    message_id: UUID
    schema_version: Literal["1.0"] = "1.0"
    message_type: str = Field(min_length=3, max_length=64)
    robot_id: str = Field(pattern=r"^[A-Z0-9-]{3,64}$")
    session_id: str = Field(default="", max_length=128)
    sequence: int = Field(ge=0)
    timestamp: datetime
    ttl_ms: int = Field(ge=0, le=30_000)
    payload: dict[str, Any]

    @field_validator("timestamp")
    @classmethod
    def timestamp_has_timezone(cls, value: datetime) -> datetime:
        if value.tzinfo is None:
            raise ValueError("timestamp must include timezone")
        return value

    def expired(self) -> bool:
        age_ms = (datetime.now(timezone.utc) - self.timestamp).total_seconds() * 1000
        return self.ttl_ms > 0 and age_ms > self.ttl_ms


class LoginRequest(BaseModel):
    email: str
    password: str


class RobotTokenRequest(BaseModel):
    robot_id: str = Field(pattern=r"^[A-Z0-9-]{3,64}$")
    credential: str = Field(min_length=16, max_length=512)


class RobotEnrollmentRequest(BaseModel):
    enrollment_token: str = Field(min_length=32, max_length=512)
    device_fingerprint: str = Field(min_length=3, max_length=255)


class RobotCredentialClaimRequest(BaseModel):
    management_address: str = Field(min_length=3, max_length=255)
    username: str = Field(min_length=1, max_length=120)
    password: str = Field(min_length=6, max_length=512)
    device_fingerprint: str = Field(min_length=3, max_length=255)

    @field_validator("management_address")
    @classmethod
    def normalize_address(cls, value: str) -> str:
        return normalize_management_address(value)

    @field_validator("username")
    @classmethod
    def normalize_username(cls, value: str) -> str:
        username = value.strip()
        if not username:
            raise ValueError("Tài khoản robot không được để trống")
        return username


class RobotQuickCreate(BaseModel):
    management_address: str = Field(min_length=3, max_length=255)
    username: str = Field(min_length=1, max_length=120)
    password: str = Field(min_length=6, max_length=512)

    @field_validator("management_address")
    @classmethod
    def normalize_address(cls, value: str) -> str:
        return normalize_management_address(value)

    @field_validator("username")
    @classmethod
    def normalize_username(cls, value: str) -> str:
        username = value.strip()
        if not username:
            raise ValueError("Tài khoản robot không được để trống")
        return username


class RobotCreate(BaseModel):
    robot_id: str = Field(pattern=r"^[A-Z0-9-]{3,64}$")
    name: str = Field(min_length=2, max_length=120)
    site_id: str = Field(min_length=2, max_length=64)
    map_id: str = Field(default="MAP-001", min_length=2, max_length=64)


class RobotUpdate(BaseModel):
    name: str = Field(min_length=2, max_length=120)
    site_id: str = Field(min_length=2, max_length=64)
    map_id: str = Field(min_length=2, max_length=64)
    enabled: bool = True
    management_address: str | None = Field(default=None, min_length=3, max_length=255)
    management_username: str | None = Field(default=None, min_length=1, max_length=120)
    management_password: str = Field(default="", max_length=512)

    @field_validator("management_address")
    @classmethod
    def normalize_address(cls, value: str | None) -> str | None:
        return normalize_management_address(value) if value is not None else None

    @field_validator("management_password")
    @classmethod
    def validate_optional_password(cls, value: str) -> str:
        if value and len(value) < 6:
            raise ValueError("Mật khẩu robot phải có ít nhất 6 ký tự")
        return value

    @field_validator("management_username")
    @classmethod
    def normalize_optional_username(cls, value: str | None) -> str | None:
        if value is None:
            return None
        username = value.strip()
        if not username:
            raise ValueError("Tài khoản robot không được để trống")
        return username


class SessionCreate(BaseModel):
    robot_id: str


class RobotConfigurationUpdate(BaseModel):
    device_ip: str = Field(min_length=1, max_length=255)
    video_source_type: Literal["rtsp", "camera", "file", "test"] = "rtsp"
    video_source: str = Field(min_length=1, max_length=2048)
    video_profile: Literal["full_hd", "balanced", "low_bandwidth"] = "full_hd"
    rtsp_transport: Literal["tcp", "udp"] = "tcp"
    camera_label: str = Field(default="Camera chính", min_length=1, max_length=120)
    audio_source_type: Literal["silent", "device", "file"] = "silent"
    audio_source: str = Field(default="", max_length=2048)
    microphone_label: str = Field(
        default="Microphone chính", min_length=1, max_length=120
    )

    @field_validator("video_source")
    @classmethod
    def validate_video_source(cls, value: str) -> str:
        return value.strip()

    @model_validator(mode="after")
    def validate_source_for_type(self) -> "RobotConfigurationUpdate":
        source = self.video_source.lower()
        if self.video_source_type == "rtsp" and not source.startswith(
            ("rtsp://", "rtsps://")
        ):
            raise ValueError("Nguồn RTSP phải bắt đầu bằng rtsp:// hoặc rtsps://")
        if self.video_source_type == "camera" and not self.video_source.startswith("/dev/"):
            raise ValueError("Camera USB phải dùng đường dẫn thiết bị /dev/video*")
        if self.audio_source_type != "silent" and not self.audio_source.strip():
            raise ValueError("Hãy chọn nguồn microphone")
        return self


class MediaProbeRequest(BaseModel):
    media_kind: Literal["video", "audio"]
    configuration: RobotConfigurationUpdate


class NavigationPreviewRequest(BaseModel):
    robot_id: str
    destination_id: str
    start: dict[str, float] | None = None


class NavigationGoalRequest(BaseModel):
    robot_id: str
    session_id: str
    route_id: str


class NavigationCancelRequest(BaseModel):
    robot_id: str
    session_id: str
