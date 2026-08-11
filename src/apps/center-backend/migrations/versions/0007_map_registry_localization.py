"""map registry deletion, sync and per-robot active version"""

from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0007"
down_revision: str | None = "0006"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    with op.batch_alter_table("robots") as batch:
        batch.add_column(sa.Column("active_map_version", sa.Integer(), nullable=True))
    with op.batch_alter_table("maps") as batch:
        batch.add_column(sa.Column("deletion_status", sa.String(32), nullable=True))
        batch.add_column(sa.Column("deleted_at", sa.DateTime(timezone=True), nullable=True))
        batch.create_index("ix_maps_deletion_status", ["deletion_status"], unique=False)
        batch.create_index("ix_maps_deleted_at", ["deleted_at"], unique=False)
    with op.batch_alter_table("map_versions") as batch:
        batch.add_column(sa.Column("sync_status", sa.String(32), nullable=False, server_default="SYNCED"))
        batch.add_column(sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()))
        batch.add_column(sa.Column("deleted_at", sa.DateTime(timezone=True), nullable=True))
        batch.create_index("ix_map_versions_sync_status", ["sync_status"], unique=False)
        batch.create_index("ix_map_versions_deleted_at", ["deleted_at"], unique=False)
    with op.batch_alter_table("robot_map_caches") as batch:
        batch.add_column(sa.Column("local_status", sa.String(32), nullable=False, server_default="MISSING"))
        batch.add_column(sa.Column("sync_status", sa.String(32), nullable=False, server_default="SYNC_PENDING"))
        batch.add_column(sa.Column("active", sa.Boolean(), nullable=False, server_default=sa.false()))
        batch.create_index("ix_robot_map_caches_sync_status", ["sync_status"], unique=False)
    op.create_table(
        "map_deletion_acks",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("map_id", sa.String(64), nullable=False),
        sa.Column("robot_id", sa.String(64), nullable=False),
        sa.Column("acknowledged_at", sa.DateTime(timezone=True), nullable=False),
        sa.UniqueConstraint("map_id", "robot_id", name="uq_map_deletion_robot"),
    )
    op.create_index("ix_map_deletion_acks_map_id", "map_deletion_acks", ["map_id"])
    op.create_index("ix_map_deletion_acks_robot_id", "map_deletion_acks", ["robot_id"])


def downgrade() -> None:
    op.drop_table("map_deletion_acks")
    with op.batch_alter_table("robot_map_caches") as batch:
        batch.drop_index("ix_robot_map_caches_sync_status")
        batch.drop_column("active")
        batch.drop_column("sync_status")
        batch.drop_column("local_status")
    with op.batch_alter_table("map_versions") as batch:
        batch.drop_index("ix_map_versions_deleted_at")
        batch.drop_index("ix_map_versions_sync_status")
        batch.drop_column("deleted_at")
        batch.drop_column("updated_at")
        batch.drop_column("sync_status")
    with op.batch_alter_table("maps") as batch:
        batch.drop_index("ix_maps_deleted_at")
        batch.drop_index("ix_maps_deletion_status")
        batch.drop_column("deleted_at")
        batch.drop_column("deletion_status")
    with op.batch_alter_table("robots") as batch:
        batch.drop_column("active_map_version")
