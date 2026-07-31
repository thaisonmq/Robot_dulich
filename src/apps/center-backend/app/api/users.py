import math

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy import func, or_, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session

from app.core.security import hash_password, require_roles
from app.models.database import get_db
from app.models.entities import AccountAuditLog, User
from app.schemas.messages import (
    AdminPasswordResetRequest,
    AdminUserCreateRequest,
    AdminUserUpdateRequest,
)
from app.services.accounts import (
    account_providers,
    account_view,
    record_account_event,
)

router = APIRouter(prefix="/api/admin/users", tags=["account-administration"])
account_manager_required = require_roles("admin", "operator")


def ensure_actor_can_manage(actor: User, target: User) -> None:
    if target.role == "admin":
        raise HTTPException(
            status_code=403,
            detail="Không thể thay đổi tài khoản quản trị cao nhất tại đây",
        )
    if actor.role == "operator" and target.role != "guest":
        raise HTTPException(
            status_code=403,
            detail="Nhân viên vận hành chỉ được quản lý tài khoản khách",
        )


@router.get("")
async def list_users(
    page: int = Query(default=1, ge=1),
    page_size: int = Query(default=10, ge=1, le=100),
    search: str = Query(default="", max_length=120),
    role: str = Query(default="all"),
    status: str = Query(default="all"),
    actor: User = Depends(account_manager_required),
    database: Session = Depends(get_db),
) -> dict:
    query = select(User).order_by(User.created_at.desc())
    if actor.role == "operator":
        query = query.where(User.role == "guest")
    normalized_search = search.strip().casefold()
    if normalized_search:
        pattern = f"%{normalized_search}%"
        query = query.where(
            or_(
                func.lower(User.username).like(pattern),
                func.lower(User.email).like(pattern),
                func.lower(User.full_name).like(pattern),
            )
        )
    if role in {"admin", "operator", "guest"}:
        query = query.where(User.role == role)
    if status == "active":
        query = query.where(User.active.is_(True))
    elif status == "inactive":
        query = query.where(User.active.is_(False))

    users = list(database.scalars(query))
    total = len(users)
    start = (page - 1) * page_size
    items = users[start:start + page_size]
    all_users_query = select(User)
    if actor.role == "operator":
        all_users_query = all_users_query.where(User.role == "guest")
    all_users = list(database.scalars(all_users_query))
    return {
        "items": [
            account_view(user, providers=account_providers(database, user.id))
            for user in items
        ],
        "page": page,
        "page_size": page_size,
        "total": total,
        "total_pages": max(1, math.ceil(total / page_size)),
        "summary": {
            "total": len(all_users),
            "admin": sum(user.role == "admin" for user in all_users),
            "operator": sum(user.role == "operator" for user in all_users),
            "guest": sum(user.role == "guest" for user in all_users),
            "inactive": sum(not user.active for user in all_users),
        },
    }


@router.post("", status_code=201)
async def create_user(
    body: AdminUserCreateRequest,
    actor: User = Depends(account_manager_required),
    database: Session = Depends(get_db),
) -> dict:
    if actor.role == "operator" and body.role != "guest":
        raise HTTPException(
            status_code=403,
            detail="Nhân viên vận hành chỉ được tạo tài khoản khách",
        )
    existing = database.scalar(
        select(User).where(
            or_(User.username == body.username, User.email == body.email)
        )
    )
    if existing is not None:
        raise HTTPException(
            status_code=409,
            detail=(
                "Tên đăng nhập đã được sử dụng"
                if existing.username == body.username
                else "Email đã được sử dụng"
            ),
        )
    user = User(
        username=body.username,
        email=body.email,
        password_hash=hash_password(body.password),
        full_name=body.full_name,
        role=body.role,
        active=True,
        email_verified=False,
        must_change_password=body.must_change_password,
        created_by_id=actor.id,
    )
    database.add(user)
    try:
        database.flush()
    except IntegrityError as exc:
        database.rollback()
        raise HTTPException(
            status_code=409,
            detail="Tên đăng nhập hoặc email đã được sử dụng",
        ) from exc
    record_account_event(
        database,
        f"{actor.role}.account_created",
        actor_user_id=actor.id,
        target_user_id=user.id,
        detail={"role": body.role},
    )
    database.commit()
    return account_view(user)


@router.patch("/{user_id}")
async def update_user(
    user_id: str,
    body: AdminUserUpdateRequest,
    actor: User = Depends(account_manager_required),
    database: Session = Depends(get_db),
) -> dict:
    user = database.get(User, user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy tài khoản")
    ensure_actor_can_manage(actor, user)
    if actor.role == "operator" and body.role not in {None, "guest"}:
        raise HTTPException(
            status_code=403,
            detail="Nhân viên vận hành không được nâng quyền tài khoản khách",
        )
    previous = {"role": user.role, "active": user.active}
    if body.full_name is not None:
        user.full_name = body.full_name
    if body.role is not None:
        user.role = body.role
    if body.active is not None:
        user.active = body.active
    record_account_event(
        database,
        f"{actor.role}.account_updated",
        actor_user_id=actor.id,
        target_user_id=user.id,
        detail={
            "previous": previous,
            "current": {"role": user.role, "active": user.active},
        },
    )
    database.commit()
    return account_view(
        user,
        providers=account_providers(database, user.id),
    )


@router.post("/{user_id}/reset-password")
async def reset_user_password(
    user_id: str,
    body: AdminPasswordResetRequest,
    actor: User = Depends(account_manager_required),
    database: Session = Depends(get_db),
) -> dict:
    user = database.get(User, user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy tài khoản")
    ensure_actor_can_manage(actor, user)
    user.password_hash = hash_password(body.new_password)
    user.must_change_password = body.must_change_password
    record_account_event(
        database,
        f"{actor.role}.password_reset",
        actor_user_id=actor.id,
        target_user_id=user.id,
        detail={"must_change_password": body.must_change_password},
    )
    database.commit()
    return {"status": "updated"}


@router.get("/{user_id}/activity")
async def user_activity(
    user_id: str,
    limit: int = Query(default=20, ge=1, le=100),
    actor: User = Depends(account_manager_required),
    database: Session = Depends(get_db),
) -> list[dict]:
    user = database.get(User, user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy tài khoản")
    ensure_actor_can_manage(actor, user)
    events = list(
        database.scalars(
            select(AccountAuditLog)
            .where(AccountAuditLog.target_user_id == user_id)
            .order_by(AccountAuditLog.created_at.desc())
            .limit(limit)
        )
    )
    return [
        {
            "id": event.id,
            "action": event.action,
            "detail": event.detail,
            "actor_user_id": event.actor_user_id,
            "created_at": event.created_at.isoformat(),
        }
        for event in events
    ]
