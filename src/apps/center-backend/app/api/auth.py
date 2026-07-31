import hashlib
import hmac
import secrets
from datetime import datetime, timedelta, timezone
from urllib.parse import urlencode

import httpx
import jwt
from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import RedirectResponse
from sqlalchemy import or_, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.core.security import (
    authenticated_user,
    create_access_token,
    hash_password,
    verify_password,
)
from app.models.database import get_db
from app.models.entities import AuthIdentity, OAuthLoginCode, User
from app.schemas.messages import (
    LoginRequest,
    OAuthExchangeRequest,
    PasswordChangeRequest,
    ProfileUpdateRequest,
    RegisterRequest,
)
from app.services.accounts import (
    account_providers,
    account_view,
    record_account_event,
    unique_username,
)

router = APIRouter(prefix="/api/auth", tags=["auth"])
GOOGLE_AUTHORIZE_URL = "https://accounts.google.com/o/oauth2/v2/auth"
GOOGLE_TOKEN_URL = "https://oauth2.googleapis.com/token"
GOOGLE_USERINFO_URL = "https://openidconnect.googleapis.com/v1/userinfo"
OAUTH_COOKIE = "rovera_google_oauth_state"
DUMMY_PASSWORD_HASH = hash_password("invalid-password-used-for-constant-work")


def login_payload(user: User, settings: Settings, database: Session) -> dict:
    return {
        "access_token": create_access_token(user.id, settings),
        "token_type": "bearer",
        "expires_in": settings.jwt_expire_minutes * 60,
        "user": account_view(
            user,
            providers=account_providers(database, user.id),
        ),
    }


def google_callback_url(settings: Settings) -> str:
    if settings.google_redirect_uri.strip():
        return settings.google_redirect_uri.strip()
    return f"{settings.frontend_public_url.rstrip('/')}/api/auth/google/callback"


def frontend_redirect(settings: Settings, path: str = "/") -> str:
    return f"{settings.frontend_public_url.rstrip('/')}{path}"


def oauth_error_redirect(settings: Settings, message: str) -> RedirectResponse:
    target = f"{frontend_redirect(settings)}?{urlencode({'oauth_error': message})}"
    response = RedirectResponse(target, status_code=303)
    response.delete_cookie(OAUTH_COOKIE, path="/api/auth/google")
    return response


def create_oauth_state(settings: Settings) -> str:
    now = datetime.now(timezone.utc)
    return jwt.encode(
        {
            "sub": secrets.token_urlsafe(24),
            "type": "google_oauth_state",
            "iat": now,
            "exp": now + timedelta(minutes=settings.oauth_state_expire_minutes),
        },
        settings.jwt_secret,
        algorithm=settings.jwt_algorithm,
    )


def validate_oauth_state(state: str, cookie_state: str, settings: Settings) -> None:
    if not state or not cookie_state or not hmac.compare_digest(state, cookie_state):
        raise ValueError("OAuth state mismatch")
    payload = jwt.decode(
        state,
        settings.jwt_secret,
        algorithms=[settings.jwt_algorithm],
    )
    if payload.get("type") != "google_oauth_state":
        raise ValueError("OAuth state type mismatch")


@router.post("/login")
async def login(
    body: LoginRequest,
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    identifier = body.login_identifier
    user = database.scalar(
        select(User).where(
            or_(User.username == identifier, User.email == identifier)
        )
    )
    password_matches = verify_password(
        body.password,
        user.password_hash if user and user.password_hash else DUMMY_PASSWORD_HASH,
    )
    if user is None or not user.active or not password_matches:
        raise HTTPException(
            status_code=401,
            detail="Tên đăng nhập, email hoặc mật khẩu không đúng",
        )
    user.last_login_at = datetime.now(timezone.utc)
    record_account_event(
        database,
        "auth.login",
        actor_user_id=user.id,
        target_user_id=user.id,
        detail={"method": "password"},
    )
    database.commit()
    return login_payload(user, settings, database)


@router.post("/register", status_code=201)
async def register(
    body: RegisterRequest,
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    existing = database.scalar(
        select(User).where(
            or_(User.username == body.username, User.email == body.email)
        )
    )
    if existing is not None:
        detail = (
            "Tên đăng nhập đã được sử dụng"
            if existing.username == body.username
            else "Email đã được sử dụng"
        )
        raise HTTPException(status_code=409, detail=detail)
    user = User(
        username=body.username,
        email=body.email,
        password_hash=hash_password(body.password),
        full_name=body.full_name,
        role="guest",
        active=True,
        email_verified=False,
        must_change_password=False,
        last_login_at=datetime.now(timezone.utc),
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
        "account.registered",
        actor_user_id=user.id,
        target_user_id=user.id,
        detail={"method": "password", "role": "guest"},
    )
    database.commit()
    return login_payload(user, settings, database)


@router.get("/me")
async def me(
    user: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    return account_view(user, providers=account_providers(database, user.id))


@router.patch("/me")
async def update_profile(
    body: ProfileUpdateRequest,
    user: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    previous_name = user.full_name
    user.full_name = body.full_name
    record_account_event(
        database,
        "account.profile_updated",
        actor_user_id=user.id,
        target_user_id=user.id,
        detail={"previous_name": previous_name},
    )
    database.commit()
    return account_view(user, providers=account_providers(database, user.id))


@router.post("/me/password")
async def change_password(
    body: PasswordChangeRequest,
    user: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    if user.password_hash and not verify_password(
        body.current_password, user.password_hash
    ):
        raise HTTPException(status_code=400, detail="Mật khẩu hiện tại không đúng")
    user.password_hash = hash_password(body.new_password)
    user.must_change_password = False
    record_account_event(
        database,
        "account.password_changed",
        actor_user_id=user.id,
        target_user_id=user.id,
    )
    database.commit()
    return {"status": "updated"}


@router.get("/google/status")
async def google_status(settings: Settings = Depends(get_settings)) -> dict:
    return {"enabled": settings.google_oauth_enabled}


@router.get("/google/login")
async def google_login(settings: Settings = Depends(get_settings)) -> RedirectResponse:
    if not settings.google_oauth_enabled:
        raise HTTPException(
            status_code=503,
            detail="Đăng nhập Google chưa được cấu hình",
        )
    state = create_oauth_state(settings)
    query = urlencode(
        {
            "client_id": settings.google_client_id,
            "redirect_uri": google_callback_url(settings),
            "response_type": "code",
            "scope": "openid email profile",
            "state": state,
            "prompt": "select_account",
        }
    )
    response = RedirectResponse(f"{GOOGLE_AUTHORIZE_URL}?{query}", status_code=302)
    response.set_cookie(
        OAUTH_COOKIE,
        state,
        max_age=settings.oauth_state_expire_minutes * 60,
        httponly=True,
        secure=google_callback_url(settings).startswith("https://"),
        samesite="lax",
        path="/api/auth/google",
    )
    return response


@router.get("/google/callback", name="google_callback")
async def google_callback(
    request: Request,
    code: str = "",
    state: str = "",
    error: str = "",
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> RedirectResponse:
    if error:
        return oauth_error_redirect(settings, "Google đã huỷ yêu cầu đăng nhập")
    if not settings.google_oauth_enabled:
        return oauth_error_redirect(settings, "Đăng nhập Google chưa được cấu hình")
    try:
        validate_oauth_state(
            state,
            request.cookies.get(OAUTH_COOKIE, ""),
            settings,
        )
        async with httpx.AsyncClient(timeout=15) as client:
            token_response = await client.post(
                GOOGLE_TOKEN_URL,
                data={
                    "code": code,
                    "client_id": settings.google_client_id,
                    "client_secret": settings.google_client_secret,
                    "redirect_uri": google_callback_url(settings),
                    "grant_type": "authorization_code",
                },
            )
            token_response.raise_for_status()
            access_token = str(token_response.json().get("access_token", ""))
            user_response = await client.get(
                GOOGLE_USERINFO_URL,
                headers={"Authorization": f"Bearer {access_token}"},
            )
            user_response.raise_for_status()
            profile = user_response.json()
    except (ValueError, jwt.PyJWTError, httpx.HTTPError):
        return oauth_error_redirect(
            settings,
            "Không thể xác minh tài khoản Google",
        )

    subject = str(profile.get("sub", ""))
    email = str(profile.get("email", "")).strip().casefold()
    email_verified = bool(profile.get("email_verified"))
    if not subject or not email or not email_verified:
        return oauth_error_redirect(
            settings,
            "Google chưa xác minh email của tài khoản này",
        )

    identity = database.scalar(
        select(AuthIdentity).where(
            AuthIdentity.provider == "google",
            AuthIdentity.provider_subject == subject,
        )
    )
    user = database.get(User, identity.user_id) if identity else None
    if user is None:
        user = database.scalar(select(User).where(User.email == email))
        if user is None:
            user = User(
                username=unique_username(database, email.split("@", 1)[0]),
                email=email,
                password_hash=None,
                full_name=str(profile.get("name") or email.split("@", 1)[0])[:120],
                role="guest",
                active=True,
                email_verified=True,
                avatar_url=str(profile.get("picture") or "")[:1024] or None,
            )
            database.add(user)
            try:
                database.flush()
            except IntegrityError:
                database.rollback()
                return oauth_error_redirect(
                    settings,
                    "Email Google đã được liên kết với tài khoản khác",
                )
            record_account_event(
                database,
                "account.registered",
                actor_user_id=user.id,
                target_user_id=user.id,
                detail={"method": "google", "role": "guest"},
            )
        identity = AuthIdentity(
            user_id=user.id,
            provider="google",
            provider_subject=subject,
            provider_email=email,
        )
        database.add(identity)

    if not user.active:
        return oauth_error_redirect(settings, "Tài khoản đã bị vô hiệu hoá")

    now = datetime.now(timezone.utc)
    user.email_verified = True
    user.avatar_url = str(profile.get("picture") or user.avatar_url or "")[:1024] or None
    user.last_login_at = now
    identity.last_login_at = now
    login_code = secrets.token_urlsafe(48)
    database.add(
        OAuthLoginCode(
            code_hash=hashlib.sha256(login_code.encode()).hexdigest(),
            user_id=user.id,
            expires_at=now
            + timedelta(seconds=settings.oauth_login_code_expire_seconds),
        )
    )
    record_account_event(
        database,
        "auth.login",
        actor_user_id=user.id,
        target_user_id=user.id,
        detail={"method": "google"},
    )
    database.commit()
    response = RedirectResponse(
        f"{frontend_redirect(settings, '/auth/google/callback')}?"
        f"{urlencode({'code': login_code})}",
        status_code=303,
    )
    response.delete_cookie(OAUTH_COOKIE, path="/api/auth/google")
    return response


@router.post("/google/exchange")
async def google_exchange(
    body: OAuthExchangeRequest,
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    code_hash = hashlib.sha256(body.code.encode()).hexdigest()
    login_code = database.scalar(
        select(OAuthLoginCode)
        .where(OAuthLoginCode.code_hash == code_hash)
        .with_for_update()
    )
    now = datetime.now(timezone.utc)
    expires_at = login_code.expires_at if login_code else None
    if expires_at and expires_at.tzinfo is None:
        expires_at = expires_at.replace(tzinfo=timezone.utc)
    if (
        login_code is None
        or login_code.used_at is not None
        or expires_at is None
        or expires_at <= now
    ):
        raise HTTPException(
            status_code=401,
            detail="Mã đăng nhập Google không hợp lệ hoặc đã hết hạn",
        )
    user = database.get(User, login_code.user_id)
    if user is None or not user.active:
        raise HTTPException(status_code=401, detail="Tài khoản không còn hoạt động")
    login_code.used_at = now
    database.commit()
    return login_payload(user, settings, database)
