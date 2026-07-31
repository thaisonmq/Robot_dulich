"""add persistent user accounts, OAuth identities and account audit trail"""

from datetime import datetime, timezone
import re
from typing import Sequence

from alembic import op
import sqlalchemy as sa

revision: str = "0004"
down_revision: str | None = "0003"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def _legacy_username(email: str, user_id: str, used: set[str]) -> str:
    base = re.sub(r"[^a-z0-9._-]", "-", email.casefold().split("@", 1)[0])
    base = base.strip(".-_")[:24]
    if len(base) < 3:
        base = f"user-{user_id[:8]}"
    candidate = base
    counter = 2
    while candidate in used:
        suffix = f"-{counter}"
        candidate = f"{base[:32 - len(suffix)]}{suffix}"
        counter += 1
    used.add(candidate)
    return candidate


def upgrade() -> None:
    with op.batch_alter_table("users") as batch:
        batch.add_column(sa.Column("username", sa.String(32), nullable=True))
        batch.add_column(sa.Column("full_name", sa.String(120), nullable=True))
        batch.add_column(
            sa.Column(
                "email_verified",
                sa.Boolean(),
                nullable=False,
                server_default=sa.false(),
            )
        )
        batch.add_column(sa.Column("avatar_url", sa.String(1024), nullable=True))
        batch.add_column(
            sa.Column(
                "must_change_password",
                sa.Boolean(),
                nullable=False,
                server_default=sa.false(),
            )
        )
        batch.add_column(sa.Column("created_by_id", sa.String(36), nullable=True))
        batch.add_column(
            sa.Column("last_login_at", sa.DateTime(timezone=True), nullable=True)
        )
        batch.add_column(
            sa.Column(
                "updated_at",
                sa.DateTime(timezone=True),
                nullable=False,
                server_default=sa.func.now(),
            )
        )
        batch.alter_column(
            "password_hash",
            existing_type=sa.String(255),
            nullable=True,
        )

    connection = op.get_bind()
    rows = connection.execute(sa.text("SELECT id, email FROM users")).mappings()
    used: set[str] = set()
    for row in rows:
        username = _legacy_username(str(row["email"]), str(row["id"]), used)
        connection.execute(
            sa.text(
                "UPDATE users SET username = :username, full_name = :full_name "
                "WHERE id = :user_id"
            ),
            {
                "username": username,
                "full_name": str(row["email"]).split("@", 1)[0],
                "user_id": row["id"],
            },
        )

    with op.batch_alter_table("users") as batch:
        batch.alter_column(
            "username", existing_type=sa.String(32), nullable=False
        )
        batch.alter_column(
            "full_name", existing_type=sa.String(120), nullable=False
        )
        batch.create_unique_constraint("uq_users_username", ["username"])
        batch.create_index("ix_users_created_by_id", ["created_by_id"], unique=False)
        batch.create_foreign_key(
            "fk_users_created_by_id_users",
            "users",
            ["created_by_id"],
            ["id"],
        )

    op.create_table(
        "auth_identities",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column(
            "user_id", sa.String(36), sa.ForeignKey("users.id"), nullable=False
        ),
        sa.Column("provider", sa.String(32), nullable=False),
        sa.Column("provider_subject", sa.String(255), nullable=False),
        sa.Column("provider_email", sa.String(255), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            default=lambda: datetime.now(timezone.utc),
        ),
        sa.Column("last_login_at", sa.DateTime(timezone=True), nullable=True),
        sa.UniqueConstraint(
            "provider",
            "provider_subject",
            name="uq_auth_identity_provider_subject",
        ),
    )
    op.create_index(
        "ix_auth_identities_user_id", "auth_identities", ["user_id"], unique=False
    )

    op.create_table(
        "oauth_login_codes",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column("code_hash", sa.String(64), nullable=False, unique=True),
        sa.Column(
            "user_id", sa.String(36), sa.ForeignKey("users.id"), nullable=False
        ),
        sa.Column("expires_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("used_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            default=lambda: datetime.now(timezone.utc),
        ),
    )
    op.create_index(
        "ix_oauth_login_codes_code_hash",
        "oauth_login_codes",
        ["code_hash"],
        unique=True,
    )
    op.create_index(
        "ix_oauth_login_codes_user_id",
        "oauth_login_codes",
        ["user_id"],
        unique=False,
    )

    op.create_table(
        "account_audit_logs",
        sa.Column("id", sa.String(36), primary_key=True),
        sa.Column(
            "actor_user_id", sa.String(36), sa.ForeignKey("users.id"), nullable=True
        ),
        sa.Column(
            "target_user_id", sa.String(36), sa.ForeignKey("users.id"), nullable=True
        ),
        sa.Column("action", sa.String(64), nullable=False),
        sa.Column("detail", sa.JSON(), nullable=False),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            default=lambda: datetime.now(timezone.utc),
        ),
    )
    op.create_index(
        "ix_account_audit_logs_actor_user_id",
        "account_audit_logs",
        ["actor_user_id"],
        unique=False,
    )
    op.create_index(
        "ix_account_audit_logs_target_user_id",
        "account_audit_logs",
        ["target_user_id"],
        unique=False,
    )
    op.create_index(
        "ix_account_audit_logs_action",
        "account_audit_logs",
        ["action"],
        unique=False,
    )


def downgrade() -> None:
    op.drop_table("account_audit_logs")
    op.drop_table("oauth_login_codes")
    op.drop_table("auth_identities")
    with op.batch_alter_table("users") as batch:
        batch.drop_constraint("fk_users_created_by_id_users", type_="foreignkey")
        batch.drop_index("ix_users_created_by_id")
        batch.drop_constraint("uq_users_username", type_="unique")
        batch.alter_column(
            "password_hash",
            existing_type=sa.String(255),
            nullable=False,
        )
        batch.drop_column("updated_at")
        batch.drop_column("last_login_at")
        batch.drop_column("created_by_id")
        batch.drop_column("must_change_password")
        batch.drop_column("avatar_url")
        batch.drop_column("email_verified")
        batch.drop_column("full_name")
        batch.drop_column("username")
