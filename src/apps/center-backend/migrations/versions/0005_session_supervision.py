"""add control-session supervision audit fields"""

from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0005"
down_revision: str | None = "0004"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    with op.batch_alter_table("control_sessions") as batch:
        batch.add_column(
            sa.Column("ended_by_user_id", sa.String(36), nullable=True)
        )
        batch.add_column(sa.Column("end_reason", sa.String(64), nullable=True))
        batch.create_index(
            "ix_control_sessions_ended_by_user_id",
            ["ended_by_user_id"],
            unique=False,
        )


def downgrade() -> None:
    with op.batch_alter_table("control_sessions") as batch:
        batch.drop_index("ix_control_sessions_ended_by_user_id")
        batch.drop_column("end_reason")
        batch.drop_column("ended_by_user_id")
