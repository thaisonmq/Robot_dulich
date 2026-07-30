import base64
import hashlib
import hmac
import secrets
from datetime import datetime, timedelta, timezone

import jwt
from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from app.core.config import Settings, get_settings

bearer = HTTPBearer(auto_error=False)
ROBOT_PASSWORD_ITERATIONS = 600_000


def hash_robot_password(password: str) -> str:
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


def verify_robot_password(password: str, encoded: str | None) -> bool:
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


def current_user(
    credentials: HTTPAuthorizationCredentials | None = Depends(bearer),
    settings: Settings = Depends(get_settings),
) -> str:
    if credentials is None:
        raise HTTPException(status_code=401, detail="Thiếu access token")
    return decode_token(credentials.credentials, settings)
