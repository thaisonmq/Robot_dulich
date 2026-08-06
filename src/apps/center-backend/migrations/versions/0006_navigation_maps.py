"""versioned maps, mapping sessions and navigation missions"""

from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0006"
down_revision: str | None = "0005"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    with op.batch_alter_table("maps") as batch:
        batch.add_column(sa.Column("site_id", sa.String(64), nullable=False, server_default=""))
        batch.add_column(sa.Column("floor_id", sa.String(64), nullable=False, server_default=""))
        batch.add_column(sa.Column("notes", sa.Text(), nullable=False, server_default=""))
        batch.add_column(sa.Column("status", sa.String(32), nullable=False, server_default="ACTIVE"))
        batch.add_column(sa.Column("active_version", sa.Integer(), nullable=True))
        batch.add_column(sa.Column("created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()))
        batch.add_column(sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()))
        batch.create_index("ix_maps_status", ["status"], unique=False)

    op.create_table(
        "map_versions",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("version", sa.Integer(), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("checksum", sa.String(64), nullable=False),
        sa.Column("storage_path", sa.String(1024), nullable=False),
        sa.Column("resolution", sa.Float(), nullable=False),
        sa.Column("origin", sa.JSON(), nullable=False),
        sa.Column("width", sa.Integer(), nullable=False),
        sa.Column("height", sa.Integer(), nullable=False),
        sa.Column("metadata_json", sa.JSON(), nullable=False),
        sa.Column("created_by_robot", sa.String(64), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("activated_at", sa.DateTime(timezone=True), nullable=True),
        sa.UniqueConstraint("map_id", "version", name="uq_map_version_number"),
        sa.UniqueConstraint("map_id", "checksum", name="uq_map_version_checksum"),
    )
    op.create_index("ix_map_versions_map_id", "map_versions", ["map_id"])
    op.create_index("ix_map_versions_status", "map_versions", ["status"])
    op.create_index("ix_map_versions_checksum", "map_versions", ["checksum"])
    op.create_index("ix_map_versions_created_by_robot", "map_versions", ["created_by_robot"])

    op.create_table(
        "mapping_sessions",
        sa.Column("session_id", sa.String(36), primary_key=True),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("version", sa.Integer(), nullable=False),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("user_id", sa.String(36), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("last_request_id", sa.String(64), nullable=True),
        sa.Column("metadata_json", sa.JSON(), nullable=False),
        sa.Column("error_code", sa.String(64), nullable=True),
        sa.Column("error_message", sa.Text(), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
    )
    for column in ("map_id", "robot_id", "user_id", "status", "last_request_id"):
        op.create_index(f"ix_mapping_sessions_{column}", "mapping_sessions", [column])

    op.create_table(
        "robot_map_caches",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("version", sa.Integer(), nullable=False),
        sa.Column("checksum", sa.String(64), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("progress_percent", sa.Float(), nullable=False),
        sa.Column("error_message", sa.Text(), nullable=True),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        sa.UniqueConstraint("robot_id", "map_id", "version", name="uq_robot_map_cache"),
    )
    for column in ("robot_id", "map_id", "status"):
        op.create_index(f"ix_robot_map_caches_{column}", "robot_map_caches", [column])

    op.create_table(
        "map_pois",
        sa.Column("poi_id", sa.String(36), primary_key=True),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("version", sa.Integer(), nullable=False),
        sa.Column("name", sa.String(120), nullable=False),
        sa.Column("x", sa.Float(), nullable=False),
        sa.Column("y", sa.Float(), nullable=False),
        sa.Column("yaw", sa.Float(), nullable=False),
        sa.Column("enabled", sa.Boolean(), nullable=False),
    )
    op.create_index("ix_map_pois_map_id", "map_pois", ["map_id"])

    for table in ("keepout_zones", "speed_zones"):
        columns = [
            sa.Column("zone_id", sa.String(36), primary_key=True),
            sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
            sa.Column("version", sa.Integer(), nullable=False),
            sa.Column("name", sa.String(120), nullable=False),
            sa.Column("points", sa.JSON(), nullable=False),
        ]
        if table == "speed_zones":
            columns.append(sa.Column("max_speed_mps", sa.Float(), nullable=False))
        op.create_table(table, *columns)
        op.create_index(f"ix_{table}_map_id", table, ["map_id"])

    op.create_table(
        "navigation_missions",
        sa.Column("mission_id", sa.String(36), primary_key=True),
        sa.Column("request_id", sa.String(64), nullable=False, unique=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("control_session_id", sa.String(36), nullable=False),
        sa.Column("map_id", sa.String(64), sa.ForeignKey("maps.map_id"), nullable=False),
        sa.Column("map_version", sa.Integer(), nullable=False),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("goal", sa.JSON(), nullable=False),
        sa.Column("path", sa.JSON(), nullable=False),
        sa.Column("distance_m", sa.Float(), nullable=False),
        sa.Column("error_code", sa.String(64), nullable=True),
        sa.Column("error_message", sa.Text(), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
    )
    for column in ("request_id", "robot_id", "control_session_id", "map_id", "status"):
        op.create_index(f"ix_navigation_missions_{column}", "navigation_missions", [column], unique=column == "request_id")

    op.create_table(
        "command_receipts",
        sa.Column("request_id", sa.String(64), primary_key=True),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("command_type", sa.String(64), nullable=False),
        sa.Column("expected_state", sa.String(32), nullable=True),
        sa.Column("status", sa.String(32), nullable=False),
        sa.Column("response", sa.JSON(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
    )
    op.create_index("ix_command_receipts_robot_id", "command_receipts", ["robot_id"])
    op.create_index("ix_command_receipts_command_type", "command_receipts", ["command_type"])


def downgrade() -> None:
    for table in (
        "command_receipts", "navigation_missions", "speed_zones", "keepout_zones",
        "map_pois", "robot_map_caches", "mapping_sessions", "map_versions",
    ):
        op.drop_table(table)
    with op.batch_alter_table("maps") as batch:
        batch.drop_index("ix_maps_status")
        for column in ("updated_at", "created_at", "active_version", "status", "notes", "floor_id", "site_id"):
            batch.drop_column(column)
