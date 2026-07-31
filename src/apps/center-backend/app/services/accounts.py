import re
from typing import Any

from sqlalchemy import select
from sqlalchemy.orm import Session

from app.models.entities import AccountAuditLog, AuthIdentity, User

ROLE_PERMISSIONS: dict[str, tuple[str, ...]] = {
    "admin": (
        "account.self",
        "accounts.manage",
        "robots.view",
        "robots.manage",
        "robots.operate",
    ),
    "operator": (
        "account.self",
        "accounts.manage_guests",
        "robots.view",
        "robots.manage",
        "robots.operate",
        "sessions.supervise_guests",
    ),
    "guest": (
        "account.self",
        "robots.view",
        "robots.operate",
    ),
}


def account_view(
    user: User,
    *,
    providers: list[str] | None = None,
) -> dict[str, Any]:
    return {
        "id": user.id,
        "username": user.username,
        "email": user.email,
        "name": user.full_name,
        "full_name": user.full_name,
        "role": user.role,
        "active": user.active,
        "email_verified": user.email_verified,
        "avatar_url": user.avatar_url,
        "must_change_password": user.must_change_password,
        "password_enabled": user.password_hash is not None,
        "auth_providers": providers or [],
        "permissions": list(ROLE_PERMISSIONS.get(user.role, ())),
        "created_by_id": user.created_by_id,
        "last_login_at": user.last_login_at.isoformat() if user.last_login_at else None,
        "created_at": user.created_at.isoformat(),
        "updated_at": user.updated_at.isoformat(),
    }


def account_providers(database: Session, user_id: str) -> list[str]:
    return list(
        database.scalars(
            select(AuthIdentity.provider)
            .where(AuthIdentity.user_id == user_id)
            .order_by(AuthIdentity.provider)
        )
    )


def unique_username(database: Session, preferred: str) -> str:
    base = re.sub(r"[^a-z0-9._-]", "-", preferred.casefold()).strip(".-_")
    if len(base) < 3:
        base = "user"
    base = base[:28]
    candidate = base
    counter = 2
    while database.scalar(select(User.id).where(User.username == candidate)):
        suffix = f"-{counter}"
        candidate = f"{base[:32 - len(suffix)]}{suffix}"
        counter += 1
    return candidate


def record_account_event(
    database: Session,
    action: str,
    *,
    actor_user_id: str | None,
    target_user_id: str | None,
    detail: dict[str, Any] | None = None,
) -> None:
    database.add(
        AccountAuditLog(
            actor_user_id=actor_user_id,
            target_user_id=target_user_id,
            action=action,
            detail=detail or {},
        )
    )
