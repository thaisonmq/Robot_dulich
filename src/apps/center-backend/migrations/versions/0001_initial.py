"""initial telepresence schema"""
from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0001"
down_revision: str | None = None
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    op.create_table("users",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("email", sa.String(255), nullable=False, unique=True),
        sa.Column("password_hash", sa.String(255), nullable=False),
        sa.Column("role", sa.String(32), nullable=False),
        sa.Column("active", sa.Boolean(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False))
    op.create_table("robots",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False, unique=True),
        sa.Column("name", sa.String(120), nullable=False),
        sa.Column("site_id", sa.String(64), nullable=False),
        sa.Column("map_id", sa.String(64), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("availability", sa.String(32), nullable=False),
        sa.Column("software_version", sa.String(32), nullable=False),
        sa.Column("capabilities", sa.JSON(), nullable=False),
        sa.Column("battery_percent", sa.Float(), nullable=False),
        sa.Column("last_seen_at", sa.DateTime(timezone=True)),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False))
    op.create_table("robot_connections",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("connected_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("disconnected_at", sa.DateTime(timezone=True)),
        sa.Column("remote_address", sa.String(120)))
    op.create_table("control_sessions",
        sa.Column("session_id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("user_id", sa.String(255), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("ended_at", sa.DateTime(timezone=True)),
        sa.Column("last_heartbeat_at", sa.DateTime(timezone=True), nullable=False))
    op.create_table("maps",
        sa.Column("map_id", sa.String(64), primary_key=True),
        sa.Column("name", sa.String(120), nullable=False),
        sa.Column("image_url", sa.String(255), nullable=False),
        sa.Column("width_pixels", sa.Integer(), nullable=False),
        sa.Column("height_pixels", sa.Integer(), nullable=False),
        sa.Column("resolution_m_per_pixel", sa.Float(), nullable=False),
        sa.Column("origin", sa.JSON(), nullable=False))
    op.create_table("destinations",
        sa.Column("destination_id", sa.String(64), primary_key=True),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("name", sa.String(120), nullable=False),
        sa.Column("x", sa.Float(), nullable=False),
        sa.Column("y", sa.Float(), nullable=False),
        sa.Column("yaw", sa.Float(), nullable=False),
        sa.Column("enabled", sa.Boolean(), nullable=False))
    op.create_table("navigation_routes",
        sa.Column("route_id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("destination_id", sa.String(64), nullable=False),
        sa.Column("points", sa.JSON(), nullable=False),
        sa.Column("distance_m", sa.Float(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False))
    op.create_table("command_logs",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("session_id", sa.String(36), nullable=False),
        sa.Column("message_type", sa.String(64), nullable=False),
        sa.Column("message_id", sa.String(36), nullable=False),
        sa.Column("payload", sa.JSON(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False))
    op.create_table("robot_events",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("event_type", sa.String(64), nullable=False),
        sa.Column("detail", sa.Text(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False))


def downgrade() -> None:
    for table in ("robot_events", "command_logs", "navigation_routes", "destinations",
                  "maps", "control_sessions", "robot_connections", "robots", "users"):
        op.drop_table(table)
