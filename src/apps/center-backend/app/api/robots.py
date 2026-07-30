import asyncio
import hashlib
import math
import secrets
from datetime import datetime, timedelta, timezone
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Query, Request, Response
from sqlalchemy import select
from sqlalchemy.orm import Session

from app.core.security import current_user, hash_robot_password
from app.core.config import Settings, get_settings
from app.models.database import get_db
from app.models.entities import MapRecord, Robot
from app.schemas.messages import (
    RobotConfigurationUpdate,
    MediaProbeRequest,
    RobotCreate,
    RobotQuickCreate,
    RobotUpdate,
)
from app.services.hub import hub
from app.services.media import create_preview_media_token

router = APIRouter(prefix="/api/robots", tags=["robots"])


def sync_robot(entity: Robot) -> dict:
    runtime = hub.sync_registry_robot(
        entity.robot_id,
        entity.name,
        entity.site_id,
        entity.map_id,
        enabled=entity.enabled,
        enrolled=entity.credential_hash is not None,
        battery_percent=entity.battery_percent,
        last_seen_at=entity.last_seen_at,
    )
    return {
        **hub.robot_view(runtime),
        "management_address": entity.management_address,
        "management_username": entity.management_username,
        "connection_method": entity.connection_method,
    }


def issue_enrollment(entity: Robot) -> tuple[str, datetime]:
    token = secrets.token_urlsafe(48)
    expires_at = datetime.now(timezone.utc) + timedelta(hours=24)
    entity.enrollment_token_hash = hashlib.sha256(token.encode()).hexdigest()
    entity.enrollment_expires_at = expires_at
    return token, expires_at


@router.get("")
async def list_robots(
    page: int = Query(default=1, ge=1),
    page_size: int = Query(default=6, ge=1, le=50),
    search: str = Query(default="", max_length=120),
    status_filter: str = Query(default="all", alias="status"),
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    entities = list(database.scalars(select(Robot).order_by(Robot.created_at.desc())))
    items = [sync_robot(entity) for entity in entities]
    normalized_search = search.strip().casefold()
    if normalized_search:
        items = [
            item for item in items
            if normalized_search in " ".join(
                (
                    item["robot_id"],
                    item["name"],
                    item["site_id"],
                    item.get("management_address") or "",
                )
            ).casefold()
        ]
    if status_filter != "all":
        if status_filter == "pending":
            items = [item for item in items if item["enrollment_status"] == "pending"]
        else:
            items = [item for item in items if item["status"] == status_filter]
    total = len(items)
    start = (page - 1) * page_size
    paged_items = items[start:start + page_size]
    all_views = [sync_robot(entity) for entity in entities]
    return {
        "items": paged_items,
        "page": page,
        "page_size": page_size,
        "total": total,
        "total_pages": max(1, math.ceil(total / page_size)),
        "summary": {
            "total": len(all_views),
            "online": sum(item["status"] == "online" for item in all_views),
            "available": sum(
                item["status"] == "online" and item["availability"] == "available"
                for item in all_views
            ),
            "pending": sum(
                item["enrollment_status"] == "pending" for item in all_views
            ),
        },
    }


@router.post("", status_code=201)
async def create_robot(
    body: RobotCreate,
    request: Request,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    if database.scalar(select(Robot).where(Robot.robot_id == body.robot_id)):
        raise HTTPException(status_code=409, detail="Mã robot đã tồn tại")
    if database.get(MapRecord, body.map_id) is None:
        raise HTTPException(status_code=422, detail="Map không tồn tại")
    entity = Robot(
        robot_id=body.robot_id,
        name=body.name,
        site_id=body.site_id,
        map_id=body.map_id,
        status="offline",
        availability="offline",
        capabilities={
            "video": True,
            "audio": True,
            "navigation": True,
            "teleoperation": True,
        },
    )
    token, expires_at = issue_enrollment(entity)
    database.add(entity)
    database.commit()
    view = sync_robot(entity)
    return {
        **view,
        "enrollment_token": token,
        "enrollment_expires_at": expires_at.isoformat(),
        "enrollment_endpoint": f"{str(request.base_url).rstrip('/')}/api/robot-auth/enroll",
    }


@router.post("/quick-add", status_code=201)
async def quick_add_robot(
    body: RobotQuickCreate,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    existing = database.scalar(
        select(Robot).where(Robot.management_address == body.management_address)
    )
    if existing is not None:
        raise HTTPException(
            status_code=409,
            detail="Địa chỉ này đã thuộc một robot trong hệ thống",
        )
    if database.get(MapRecord, "MAP-001") is None:
        raise HTTPException(status_code=422, detail="Map mặc định không tồn tại")
    robot_id = ""
    while not robot_id:
        candidate = f"ROBOT-{secrets.token_hex(3).upper()}"
        if database.scalar(select(Robot.id).where(Robot.robot_id == candidate)) is None:
            robot_id = candidate
    entity = Robot(
        robot_id=robot_id,
        name=f"Robot {body.management_address}",
        site_id="Chưa phân khu",
        map_id="MAP-001",
        status="offline",
        availability="offline",
        management_address=body.management_address,
        management_username=body.username,
        management_password_hash=hash_robot_password(body.password),
        connection_method="credentials",
        capabilities={
            "video": True,
            "audio": True,
            "navigation": True,
            "teleoperation": True,
        },
    )
    database.add(entity)
    database.commit()
    return sync_robot(entity)


@router.get("/{robot_id}")
async def get_robot(
    robot_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    entity = database.scalar(select(Robot).where(Robot.robot_id == robot_id))
    if entity is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    return sync_robot(entity)


@router.patch("/{robot_id}")
async def update_robot(
    robot_id: str,
    body: RobotUpdate,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    entity = database.scalar(select(Robot).where(Robot.robot_id == robot_id))
    if entity is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if database.get(MapRecord, body.map_id) is None:
        raise HTTPException(status_code=422, detail="Map không tồn tại")
    runtime = hub.robots.get(robot_id)
    is_online = bool(runtime and runtime.status == "online")
    if is_online and not body.enabled:
        raise HTTPException(
            status_code=409, detail="Hãy ngắt kết nối robot trước khi vô hiệu hoá"
        )
    if body.management_address is not None:
        duplicate = database.scalar(
            select(Robot).where(
                Robot.management_address == body.management_address,
                Robot.robot_id != robot_id,
            )
        )
        if duplicate is not None:
            raise HTTPException(
                status_code=409,
                detail="Địa chỉ này đã thuộc một robot khác",
            )
        entity.management_address = body.management_address
    if body.management_username is not None:
        entity.management_username = body.management_username
    if body.management_password:
        entity.management_password_hash = hash_robot_password(
            body.management_password
        )
        if not is_online:
            entity.credential_hash = None
            entity.enrolled_at = None
            entity.device_fingerprint = None
        entity.connection_method = "credentials"
    entity.name = body.name
    entity.site_id = body.site_id
    entity.map_id = body.map_id
    entity.enabled = body.enabled
    database.commit()
    return sync_robot(entity)


@router.post("/{robot_id}/enrollment")
async def renew_robot_enrollment(
    robot_id: str,
    request: Request,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    entity = database.scalar(select(Robot).where(Robot.robot_id == robot_id))
    if entity is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    runtime = hub.robots.get(robot_id)
    if runtime and runtime.status == "online":
        raise HTTPException(status_code=409, detail="Robot đang online")
    token, expires_at = issue_enrollment(entity)
    entity.credential_hash = None
    entity.enrolled_at = None
    entity.device_fingerprint = None
    database.commit()
    sync_robot(entity)
    return {
        "robot_id": robot_id,
        "enrollment_token": token,
        "enrollment_expires_at": expires_at.isoformat(),
        "enrollment_endpoint": f"{str(request.base_url).rstrip('/')}/api/robot-auth/enroll",
    }


@router.delete("/{robot_id}", status_code=204)
async def delete_robot(
    robot_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> Response:
    entity = database.scalar(select(Robot).where(Robot.robot_id == robot_id))
    if entity is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    runtime = hub.robots.get(robot_id)
    if runtime and (
        runtime.status == "online" or robot_id in hub.robot_session
    ):
        raise HTTPException(
            status_code=409, detail="Không thể xoá robot đang kết nối hoặc có phiên điều khiển"
        )
    database.delete(entity)
    database.commit()
    hub.robots.pop(robot_id, None)
    return Response(status_code=204)


async def configuration_from_simulator(
    robot_id: str,
    message_type: str,
    payload: dict,
    *,
    raise_on_negative: bool = True,
) -> dict:
    robot = hub.robots.get(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if robot.status != "online":
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    try:
        response = await hub.request_robot(robot_id, message_type, payload)
    except ConnectionError as exc:
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except asyncio.TimeoutError as exc:
        raise HTTPException(
            status_code=504, detail="Simulator không phản hồi yêu cầu cấu hình"
        ) from exc
    if response.get("ok") is False and raise_on_negative:
        raise HTTPException(
            status_code=502,
            detail=str(response.get("error") or "Simulator từ chối cấu hình"),
        )
    return {key: value for key, value in response.items() if key != "request_id"}


@router.get("/{robot_id}/configuration")
async def get_robot_configuration(
    robot_id: str,
    _: str = Depends(current_user),
) -> dict:
    return await configuration_from_simulator(robot_id, "configuration.get", {})


@router.patch("/{robot_id}/configuration")
async def update_robot_configuration(
    robot_id: str,
    body: RobotConfigurationUpdate,
    _: str = Depends(current_user),
) -> dict:
    return await configuration_from_simulator(
        robot_id, "configuration.update", body.model_dump()
    )


@router.post("/{robot_id}/diagnostics/connection")
async def test_robot_connection(
    robot_id: str,
    _: str = Depends(current_user),
) -> dict:
    return await configuration_from_simulator(robot_id, "diagnostics.ping", {})


@router.get("/{robot_id}/media-sources")
async def get_robot_media_sources(
    robot_id: str,
    media_kind: Literal["video", "audio", "all"] = Query(default="all"),
    _: str = Depends(current_user),
) -> dict:
    return await configuration_from_simulator(
        robot_id, "media.sources.get", {"media_kind": media_kind}
    )


@router.post("/{robot_id}/diagnostics/media")
async def test_robot_media(
    robot_id: str,
    body: MediaProbeRequest,
    _: str = Depends(current_user),
) -> dict:
    return await configuration_from_simulator(
        robot_id,
        "media.probe",
        body.model_dump(),
        raise_on_negative=False,
    )


@router.post("/{robot_id}/preview-token")
async def preview_robot_media(
    robot_id: str,
    user_id: str = Depends(current_user),
    settings: Settings = Depends(get_settings),
) -> dict:
    robot = hub.robots.get(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if robot.status != "online":
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    lease = await hub.create_preview_lease(
        robot_id, user_id, settings.media_lease_ttl_seconds
    )
    if lease is None:
        raise HTTPException(
            status_code=409,
            detail="Robot mất kết nối trước khi khởi động camera",
        )
    return {
        "url": settings.livekit_url,
        "room_name": f"robot-{robot_id}",
        "token": create_preview_media_token(settings, robot_id, user_id),
        "expires_in": 600,
        "lease_id": lease.lease_id,
    }


@router.post("/{robot_id}/preview/{lease_id}/heartbeat")
async def renew_robot_media_preview(
    robot_id: str,
    lease_id: str,
    user_id: str = Depends(current_user),
    settings: Settings = Depends(get_settings),
) -> dict:
    if not await hub.renew_preview_lease(
        robot_id, lease_id, user_id, settings.media_lease_ttl_seconds
    ):
        raise HTTPException(
            status_code=404,
            detail="Phiên xem trước không tồn tại hoặc robot đã mất kết nối",
        )
    return {"lease_id": lease_id, "status": "active"}


@router.delete("/{robot_id}/preview/{lease_id}", status_code=204)
async def stop_robot_media_preview(
    robot_id: str,
    lease_id: str,
    user_id: str = Depends(current_user),
) -> Response:
    if not await hub.close_preview_lease(robot_id, lease_id, user_id):
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên xem trước")
    return Response(status_code=204)


@router.post("/{robot_id}/connect")
async def connect_robot(robot_id: str, _: str = Depends(current_user)) -> dict:
    robot = hub.robots.get(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if robot.status != "online":
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    return {"robot_id": robot_id, "status": "ready"}


@router.post("/{robot_id}/disconnect")
async def disconnect_robot(robot_id: str, _: str = Depends(current_user)) -> dict:
    session_id = hub.robot_session.get(robot_id)
    if session_id:
        await hub.close_session(session_id)
    return {"robot_id": robot_id, "status": "disconnected"}
