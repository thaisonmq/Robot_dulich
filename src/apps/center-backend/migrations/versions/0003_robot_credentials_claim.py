"""add operator-friendly robot credential claim"""
from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0003"
down_revision: str | None = "0002"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    with op.batch_alter_table("robots") as batch:
        batch.add_column(sa.Column("management_address", sa.String(255), nullable=True))
        batch.add_column(sa.Column("management_username", sa.String(120), nullable=True))
        batch.add_column(
            sa.Column("management_password_hash", sa.String(255), nullable=True)
        )
        batch.add_column(
            sa.Column(
                "connection_method",
                sa.String(32),
                nullable=False,
                server_default="token",
            )
        )
        batch.create_index(
            "ix_robots_management_address",
            ["management_address"],
            unique=True,
        )


def downgrade() -> None:
    with op.batch_alter_table("robots") as batch:
        batch.drop_index("ix_robots_management_address")
        batch.drop_column("connection_method")
        batch.drop_column("management_password_hash")
        batch.drop_column("management_username")
        batch.drop_column("management_address")
