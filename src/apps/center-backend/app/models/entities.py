from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from sqlalchemy import JSON, Boolean, DateTime, Float, ForeignKey, String, Text
from sqlalchemy.orm import Mapped, mapped_column

from app.models.database import Base


def now() -> datetime:
    return datetime.now(timezone.utc)


class User(Base):
    __tablename__ = "users"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    email: Mapped[str] = mapped_column(String(255), unique=True, index=True)
    password_hash: Mapped[str] = mapped_column(String(255))
    role: Mapped[str] = mapped_column(String(32), default="operator")
    active: Mapped[bool] = mapped_column(Boolean, default=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=now)


class Robot(Base):
    __tablename__ = "robots"
    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid4()))
    robot_id: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    name: Mapped[str] = mapped_column(String(120))
    site_id: Mapped[str] = mapped_column(String(64))
    map_id: Mapped[str] = mapped_column(String(64))
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
