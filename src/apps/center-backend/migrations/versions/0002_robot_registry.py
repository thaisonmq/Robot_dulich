"""database-backed robot registry and enrollment"""
from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0002"
down_revision: str | None = "0001"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    with op.batch_alter_table("robots") as batch:
        batch.add_column(sa.Column("credential_hash", sa.String(64), nullable=True))
        batch.add_column(sa.Column("enrollment_token_hash", sa.String(64), nullable=True))
        batch.add_column(sa.Column("enrollment_expires_at", sa.DateTime(timezone=True), nullable=True))
        batch.add_column(sa.Column("enrolled_at", sa.DateTime(timezone=True), nullable=True))
        batch.add_column(sa.Column("device_fingerprint", sa.String(255), nullable=True))
        batch.add_column(
            sa.Column("enabled", sa.Boolean(), nullable=False, server_default=sa.true())
        )
        batch.create_index(
            "ix_robots_enrollment_token_hash", ["enrollment_token_hash"], unique=False
        )


def downgrade() -> None:
    with op.batch_alter_table("robots") as batch:
        batch.drop_index("ix_robots_enrollment_token_hash")
        batch.drop_column("enabled")
        batch.drop_column("device_fingerprint")
        batch.drop_column("enrolled_at")
        batch.drop_column("enrollment_expires_at")
        batch.drop_column("enrollment_token_hash")
        batch.drop_column("credential_hash")
