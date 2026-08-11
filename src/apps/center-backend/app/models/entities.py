from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from sqlalchemy import (
    JSON,
    Boolean,
    DateTime,
    Float,
    ForeignKey,
    String,
    Text,
    UniqueConstraint,
)
from sqlalchemy.orm import Mapped, mapped_column

from app.models.database import Base


def now() -> datetime:
    return datetime.now(timezone.utc)


class User(Base):
    __tablename__ = "users"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    email: Mapped[str] = mapped_column(String(255), unique=True, index=True)
    username: Mapped[str] = mapped_column(String(32), unique=True, index=True)
    password_hash: Mapped[str | None] = mapped_column(String(255))
    full_name: Mapped[str] = mapped_column(String(120))
    role: Mapped[str] = mapped_column(String(32), default="guest")
    active: Mapped[bool] = mapped_column(Boolean, default=True)
    email_verified: Mapped[bool] = mapped_column(Boolean, default=False)
    avatar_url: Mapped[str | None] = mapped_column(String(1024))
    must_change_password: Mapped[bool] = mapped_column(Boolean, default=False)
    created_by_id: Mapped[str | None] = mapped_column(
        ForeignKey("users.id"), index=True
    )
    last_login_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class AuthIdentity(Base):
    __tablename__ = "auth_identities"
    __table_args__ = (
        UniqueConstraint(
            "provider", "provider_subject", name="uq_auth_identity_provider_subject"
        ),
    )

    id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    user_id: Mapped[str] = mapped_column(
        ForeignKey("users.id"), index=True
    )
    provider: Mapped[str] = mapped_column(String(32))
    provider_subject: Mapped[str] = mapped_column(String(255))
    provider_email: Mapped[str | None] = mapped_column(String(255))
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    last_login_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))


class OAuthLoginCode(Base):
    __tablename__ = "oauth_login_codes"

    id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    code_hash: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    user_id: Mapped[str] = mapped_column(ForeignKey("users.id"), index=True)
    expires_at: Mapped[datetime] = mapped_column(DateTime(timezone=True))
    used_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class AccountAuditLog(Base):
    __tablename__ = "account_audit_logs"

    id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    actor_user_id: Mapped[str | None] = mapped_column(
        ForeignKey("users.id"), index=True
    )
    target_user_id: Mapped[str | None] = mapped_column(
        ForeignKey("users.id"), index=True
    )
    action: Mapped[str] = mapped_column(String(64), index=True)
    detail: Mapped[dict[str, Any]] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class Robot(Base):
    __tablename__ = "robots"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    name: Mapped[str] = mapped_column(String(120))
    site_id: Mapped[str] = mapped_column(String(64))
    map_id: Mapped[str] = mapped_column(String(64))
    active_map_version: Mapped[int | None] = mapped_column(default=None)
    status: Mapped[str] = mapped_column(String(32), default="offline")
    availability: Mapped[str] = mapped_column(String(32), default="available")
    software_version: Mapped[str] = mapped_column(String(32), default="sim-1.0")
    capabilities: Mapped[dict[str, Any]] = mapped_column(JSON, default=dict)
    battery_percent: Mapped[float] = mapped_column(Float, default=78)
    last_seen_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    credential_hash: Mapped[str | None] = mapped_column(String(64))
    enrollment_token_hash: Mapped[str | None] = mapped_column(String(64), index=True)
    enrollment_expires_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    enrolled_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    device_fingerprint: Mapped[str | None] = mapped_column(String(255))
    management_address: Mapped[str | None] = mapped_column(
        String(255), unique=True, index=True
    )
    management_username: Mapped[str | None] = mapped_column(String(120))
    management_password_hash: Mapped[str | None] = mapped_column(String(255))
    connection_method: Mapped[str] = mapped_column(String(32), default="token")
    enabled: Mapped[bool] = mapped_column(Boolean, default=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now, onupdate=now)


class RobotConnection(Base):
    __tablename__ = "robot_connections"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    connected_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    disconnected_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    remote_address: Mapped[str | None] = mapped_column(String(120))


class ControlSession(Base):
    __tablename__ = "control_sessions"
    session_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    user_id: Mapped[str] = mapped_column(String(255), index=True)
    status: Mapped[str] = mapped_column(String(32), default="active")
    started_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    ended_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    ended_by_user_id: Mapped[str | None] = mapped_column(String(36), index=True)
    end_reason: Mapped[str | None] = mapped_column(String(64))
    last_heartbeat_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class MapRecord(Base):
    __tablename__ = "maps"
    map_id: Mapped[str] = mapped_column(String(64), primary_key=True)
    name: Mapped[str] = mapped_column(String(120))
    image_url: Mapped[str] = mapped_column(String(255))
    width_pixels: Mapped[int]
    height_pixels: Mapped[int]
    resolution_m_per_pixel: Mapped[float]
    origin: Mapped[dict[str, Any]] = mapped_column(JSON)
    site_id: Mapped[str] = mapped_column(String(64), default="")
    floor_id: Mapped[str] = mapped_column(String(64), default="")
    notes: Mapped[str] = mapped_column(Text, default="")
    status: Mapped[str] = mapped_column(String(32), default="ACTIVE", index=True)
    active_version: Mapped[int | None] = mapped_column(default=None)
    deletion_status: Mapped[str | None] = mapped_column(String(32), default=None, index=True)
    deleted_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), index=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class MapVersion(Base):
    __tablename__ = "map_versions"
    __table_args__ = (
        UniqueConstraint("map_id", "version", name="uq_map_version_number"),
        UniqueConstraint("map_id", "checksum", name="uq_map_version_checksum"),
    )

    id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    status: Mapped[str] = mapped_column(String(32), default="DRAFT", index=True)
    checksum: Mapped[str] = mapped_column(String(64), index=True)
    storage_path: Mapped[str] = mapped_column(String(1024))
    resolution: Mapped[float]
    origin: Mapped[dict[str, Any]] = mapped_column(JSON)
    width: Mapped[int]
    height: Mapped[int]
    metadata_json: Mapped[dict[str, Any]] = mapped_column(JSON, default=dict)
    created_by_robot: Mapped[str] = mapped_column(String(64), index=True)
    sync_status: Mapped[str] = mapped_column(String(32), default="SYNCED", index=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )
    activated_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    deleted_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), index=True)


class MappingSession(Base):
    __tablename__ = "mapping_sessions"

    session_id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    user_id: Mapped[str] = mapped_column(String(36), index=True)
    status: Mapped[str] = mapped_column(String(32), default="STARTING", index=True)
    last_request_id: Mapped[str | None] = mapped_column(String(64), index=True)
    metadata_json: Mapped[dict[str, Any]] = mapped_column(JSON, default=dict)
    error_code: Mapped[str | None] = mapped_column(String(64))
    error_message: Mapped[str | None] = mapped_column(Text)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class RobotMapCache(Base):
    __tablename__ = "robot_map_caches"
    __table_args__ = (
        UniqueConstraint("robot_id", "map_id", "version", name="uq_robot_map_cache"),
    )

    id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    checksum: Mapped[str] = mapped_column(String(64))
    status: Mapped[str] = mapped_column(String(32), default="MISSING", index=True)
    local_status: Mapped[str] = mapped_column(String(32), default="MISSING")
    sync_status: Mapped[str] = mapped_column(String(32), default="SYNC_PENDING", index=True)
    active: Mapped[bool] = mapped_column(Boolean, default=False)
    progress_percent: Mapped[float] = mapped_column(Float, default=0)
    error_message: Mapped[str | None] = mapped_column(Text)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class MapDeletionAck(Base):
    __tablename__ = "map_deletion_acks"
    __table_args__ = (
        UniqueConstraint("map_id", "robot_id", name="uq_map_deletion_robot"),
    )

    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    map_id: Mapped[str] = mapped_column(String(64), index=True)
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    acknowledged_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class POI(Base):
    __tablename__ = "map_pois"

    poi_id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    name: Mapped[str] = mapped_column(String(120))
    x: Mapped[float]
    y: Mapped[float]
    yaw: Mapped[float] = mapped_column(Float, default=0)
    enabled: Mapped[bool] = mapped_column(Boolean, default=True)


class KeepoutZone(Base):
    __tablename__ = "keepout_zones"

    zone_id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    name: Mapped[str] = mapped_column(String(120))
    points: Mapped[list[dict[str, float]]] = mapped_column(JSON)


class SpeedZone(Base):
    __tablename__ = "speed_zones"

    zone_id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    version: Mapped[int]
    name: Mapped[str] = mapped_column(String(120))
    points: Mapped[list[dict[str, float]]] = mapped_column(JSON)
    max_speed_mps: Mapped[float]


class Destination(Base):
    __tablename__ = "destinations"
    destination_id: Mapped[str] = mapped_column(String(64), primary_key=True)
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    name: Mapped[str] = mapped_column(String(120))
    x: Mapped[float]
    y: Mapped[float]
    yaw: Mapped[float]
    enabled: Mapped[bool] = mapped_column(Boolean, default=True)


class NavigationRoute(Base):
    __tablename__ = "navigation_routes"
    route_id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    destination_id: Mapped[str] = mapped_column(String(64))
    points: Mapped[list[dict[str, float]]] = mapped_column(JSON)
    distance_m: Mapped[float]
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class NavigationMission(Base):
    __tablename__ = "navigation_missions"

    mission_id: Mapped[str] = mapped_column(
        String(36), primary_key=True, default=lambda: str(uuid4())
    )
    request_id: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    control_session_id: Mapped[str] = mapped_column(String(36), index=True)
    map_id: Mapped[str] = mapped_column(ForeignKey("maps.map_id"), index=True)
    map_version: Mapped[int]
    status: Mapped[str] = mapped_column(String(32), default="PLANNING", index=True)
    goal: Mapped[dict[str, Any]] = mapped_column(JSON)
    path: Mapped[list[dict[str, float]]] = mapped_column(JSON, default=list)
    distance_m: Mapped[float] = mapped_column(Float, default=0)
    error_code: Mapped[str | None] = mapped_column(String(64))
    error_message: Mapped[str | None] = mapped_column(Text)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class CommandReceipt(Base):
    __tablename__ = "command_receipts"

    request_id: Mapped[str] = mapped_column(String(64), primary_key=True)
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    command_type: Mapped[str] = mapped_column(String(64), index=True)
    expected_state: Mapped[str | None] = mapped_column(String(32))
    status: Mapped[str] = mapped_column(String(32), default="PENDING")
    response: Mapped[dict[str, Any]] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=now, onupdate=now
    )


class CommandLog(Base):
    __tablename__ = "command_logs"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    session_id: Mapped[str] = mapped_column(String(36), index=True)
    message_type: Mapped[str] = mapped_column(String(64))
    message_id: Mapped[str] = mapped_column(String(36))
    payload: Mapped[dict[str, Any]] = mapped_column(JSON)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class RobotEvent(Base):
    __tablename__ = "robot_events"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), index=True)
    event_type: Mapped[str] = mapped_column(String(64))
    detail: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)
