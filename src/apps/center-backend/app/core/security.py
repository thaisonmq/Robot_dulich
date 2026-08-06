import base64
import hashlib
import hmac
import secrets
from datetime import datetime, timedelta, timezone

import jwt
from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.models.database import get_db
from app.models.entities import User

bearer = HTTPBearer(auto_error=False)
ROBOT_PASSWORD_ITERATIONS = 600_000
USER_ROLES = frozenset({"admin", "operator", "guest"})


def hash_password(password: str) -> str:
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac(
        "sha256", password.encode(), salt, ROBOT_PASSWORD_ITERATIONS
    )
    return "$".join(
        (
            "pbkdf2_sha256",
            str(ROBOT_PASSWORD_ITERATIONS),
            base64.urlsafe_b64encode(salt).decode(),
            base64.urlsafe_b64encode(digest).decode(),
        )
    )


def verify_password(password: str, encoded: str | None) -> bool:
    if not encoded:
        return False
    try:
        algorithm, iterations_text, salt_text, digest_text = encoded.split("$", 3)
        if algorithm != "pbkdf2_sha256":
            return False
        iterations = int(iterations_text)
        salt = base64.urlsafe_b64decode(salt_text.encode())
        expected = base64.urlsafe_b64decode(digest_text.encode())
        supplied = hashlib.pbkdf2_hmac(
            "sha256", password.encode(), salt, iterations
        )
    except (ValueError, TypeError):
        return False
    return hmac.compare_digest(supplied, expected)


hash_robot_password = hash_password
verify_robot_password = verify_password


def create_access_token(subject: str, settings: Settings) -> str:
    now = datetime.now(timezone.utc)
    return jwt.encode(
        {
            "sub": subject,
            "iat": now,
            "exp": now + timedelta(minutes=settings.jwt_expire_minutes),
            "type": "access",
        },
        settings.jwt_secret,
        algorithm=settings.jwt_algorithm,
    )


def create_robot_access_token(robot_id: str, settings: Settings) -> str:
    now = datetime.now(timezone.utc)
    return jwt.encode(
        {
            "sub": robot_id,
            "iat": now,
            "exp": now + timedelta(minutes=settings.robot_token_expire_minutes),
            "type": "robot",
        },
        settings.jwt_secret,
        algorithm=settings.jwt_algorithm,
    )


def decode_typed_token(token: str, settings: Settings, expected_type: str) -> str:
    try:
        payload = jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
        if payload.get("type") != expected_type:
            raise jwt.InvalidTokenError("wrong token type")
        return str(payload["sub"])
    except (jwt.PyJWTError, KeyError) as exc:
        raise HTTPException(status_code=401, detail="Token không hợp lệ hoặc đã hết hạn") from exc


def decode_token(token: str, settings: Settings) -> str:
    return decode_typed_token(token, settings, "access")


def decode_robot_token(token: str, settings: Settings) -> str:
    return decode_typed_token(token, settings, "robot")


def authenticated_user(
    credentials: HTTPAuthorizationCredentials | None = Depends(bearer),
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> User:
    if credentials is None:
        raise HTTPException(status_code=401, detail="Thiếu access token")
    user_id = decode_token(credentials.credentials, settings)
    user = database.get(User, user_id)
    if user is None or not user.active:
        raise HTTPException(
            status_code=401,
            detail="Tài khoản không tồn tại hoặc đã bị vô hiệu hoá",
        )
    return user


def current_user(user: User = Depends(authenticated_user)) -> str:
    return user.id


def operator_user_id(user: User = Depends(authenticated_user)) -> str:
    if user.role not in {"admin", "operator"}:
        raise HTTPException(
            status_code=403,
            detail="Tài khoản khách không có quyền xem hoặc thay đổi cấu hình kỹ thuật",
        )
    return user.id


def user_or_robot(
    credentials: HTTPAuthorizationCredentials | None = Depends(bearer),
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> tuple[str, str]:
    """Authenticate a browser user or an enrolled robot for map transfer APIs."""
    if credentials is None:
        raise HTTPException(status_code=401, detail="Thiếu access token")
    token = credentials.credentials
    try:
        user_id = decode_token(token, settings)
    except HTTPException:
        user_id = ""
    if user_id:
        user = database.get(User, user_id)
        if user is not None and user.active:
            return "user", user.id
    try:
        robot_id = decode_robot_token(token, settings)
    except HTTPException as exc:
        raise HTTPException(status_code=401, detail="Token không hợp lệ hoặc đã hết hạn") from exc
    from app.models.entities import Robot

    robot = database.query(Robot).filter(Robot.robot_id == robot_id).first()
    if robot is None or not robot.enabled or robot.credential_hash is None:
        raise HTTPException(status_code=401, detail="Robot chưa được đăng ký")
    return "robot", robot_id


def require_roles(*roles: str):
    allowed = frozenset(roles)

    def role_dependency(user: User = Depends(authenticated_user)) -> User:
        if user.role not in allowed:
            raise HTTPException(
                status_code=403,
                detail="Tài khoản không có quyền thực hiện thao tác này",
            )
        return user

    return role_dependency
