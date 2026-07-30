import hashlib
import hmac
import secrets
from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Query
from fastapi.security import HTTPAuthorizationCredentials
from sqlalchemy import select
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.core.security import (
    bearer,
    create_robot_access_token,
    decode_robot_token,
    hash_robot_password,
    verify_robot_password,
)
from app.models.database import get_db
from app.models.entities import Robot
from app.schemas.messages import (
    RobotCredentialClaimRequest,
    RobotEnrollmentRequest,
    RobotTokenRequest,
)
from app.services.hub import hub
from app.services.media import create_robot_media_token

router = APIRouter(prefix="/api/robot-auth", tags=["robot-auth"])
DUMMY_ROBOT_PASSWORD_HASH = hash_robot_password(
    "invalid-robot-password-used-only-for-constant-work"
)


@router.post("/token")
async def robot_token(
    body: RobotTokenRequest,
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    robot = database.scalar(select(Robot).where(Robot.robot_id == body.robot_id))
    supplied_hash = hashlib.sha256(body.credential.encode()).hexdigest()
    authenticated = (
        robot is not None
        and robot.enabled
        and robot.credential_hash is not None
        and hmac.compare_digest(supplied_hash, robot.credential_hash)
    )
    if not authenticated:
        raise HTTPException(status_code=401, detail="Thông tin xác thực robot không hợp lệ")
    return {
        "access_token": create_robot_access_token(body.robot_id, settings),
        "token_type": "bearer",
        "expires_in": settings.robot_token_expire_minutes * 60,
    }


@router.post("/enroll")
async def enroll_robot(
    body: RobotEnrollmentRequest,
    database: Session = Depends(get_db),
) -> dict:
    token_hash = hashlib.sha256(body.enrollment_token.encode()).hexdigest()
    robot = database.scalar(
        select(Robot)
        .where(Robot.enrollment_token_hash == token_hash)
        .with_for_update()
    )
    now = datetime.now(timezone.utc)
    expires_at = robot.enrollment_expires_at if robot else None
    if expires_at and expires_at.tzinfo is None:
        expires_at = expires_at.replace(tzinfo=timezone.utc)
    if (
        robot is None
        or not robot.enabled
        or expires_at is None
        or expires_at <= now
    ):
        raise HTTPException(
            status_code=401, detail="Liên kết ghép nối không hợp lệ hoặc đã hết hạn"
        )
    credential = secrets.token_urlsafe(48)
    robot.credential_hash = hashlib.sha256(credential.encode()).hexdigest()
    robot.enrollment_token_hash = None
    robot.enrollment_expires_at = None
    robot.enrolled_at = now
    robot.device_fingerprint = body.device_fingerprint
    robot.status = "offline"
    database.commit()
    hub.sync_registry_robot(
        robot.robot_id,
        robot.name,
        robot.site_id,
        robot.map_id,
        enabled=robot.enabled,
        enrolled=True,
        battery_percent=robot.battery_percent,
        last_seen_at=robot.last_seen_at,
    )
    return {
        "robot_id": robot.robot_id,
        "credential": credential,
        "message": "Ghép nối robot thành công",
    }


@router.post("/claim")
async def claim_robot_with_credentials(
    body: RobotCredentialClaimRequest,
    database: Session = Depends(get_db),
) -> dict:
    robot = database.scalar(
        select(Robot)
        .where(Robot.management_address == body.management_address)
        .with_for_update()
    )
    password_matches = verify_robot_password(
        body.password,
        robot.management_password_hash if robot else DUMMY_ROBOT_PASSWORD_HASH,
    )
    username_matches = bool(
        robot
        and robot.management_username
        and hmac.compare_digest(body.username, robot.management_username)
    )
    runtime = hub.robots.get(robot.robot_id) if robot else None
    if (
        robot is None
        or not robot.enabled
        or not username_matches
        or not password_matches
        or (runtime is not None and runtime.status == "online")
    ):
        raise HTTPException(
            status_code=401,
            detail="Không tìm thấy robot hoặc thông tin đăng nhập không đúng",
        )
    credential = secrets.token_urlsafe(48)
    robot.credential_hash = hashlib.sha256(credential.encode()).hexdigest()
    robot.enrollment_token_hash = None
    robot.enrollment_expires_at = None
    robot.enrolled_at = datetime.now(timezone.utc)
    robot.device_fingerprint = body.device_fingerprint
    robot.connection_method = "credentials"
    robot.status = "offline"
    database.commit()
    hub.sync_registry_robot(
        robot.robot_id,
        robot.name,
        robot.site_id,
        robot.map_id,
        enabled=robot.enabled,
        enrolled=True,
        battery_percent=robot.battery_percent,
        last_seen_at=robot.last_seen_at,
    )
    return {
        "robot_id": robot.robot_id,
        "credential": credential,
        "message": "Robot đã được nhận diện",
    }


@router.post("/media-token")
async def robot_media_token(
    purpose: Literal["main", "video"] = Query(default="main"),
    credentials: HTTPAuthorizationCredentials | None = Depends(bearer),
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    if credentials is None:
        raise HTTPException(status_code=401, detail="Thiếu robot access token")
    robot_id = decode_robot_token(credentials.credentials, settings)
    robot = database.scalar(select(Robot).where(Robot.robot_id == robot_id))
    if robot is None or not robot.enabled or robot.credential_hash is None:
        raise HTTPException(status_code=401, detail="Robot không còn được cấp quyền")
    return {
        "url": settings.robot_livekit_url,
        "room_name": f"robot-{robot_id}",
        "token": create_robot_media_token(settings, robot_id, purpose),
        "expires_in": 1800,
    }
