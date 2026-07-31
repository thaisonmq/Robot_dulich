import asyncio
from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.core.security import authenticated_user, require_roles
from app.models.database import get_db
from app.models.entities import ControlSession, User
from app.schemas.messages import SessionCameraSelect, SessionCreate
from app.services.hub import CameraSourceRuntime, SessionRuntime, hub
from app.services.media import create_media_token, create_spectator_media_token

router = APIRouter(prefix="/api/sessions", tags=["sessions"])
supervisor_required = require_roles("admin", "operator")


def websocket_base(request: Request) -> str:
    ws_scheme = "wss" if request.url.scheme == "https" else "ws"
    return f"{ws_scheme}://{request.url.netloc}"


def session_payload(
    session: SessionRuntime,
    request: Request,
    settings: Settings,
    *,
    user_id: str,
    mode: str,
    controller: User | None = None,
) -> dict:
    media_token = (
        create_media_token(
            settings, session.robot_id, user_id, session.session_id
        )
        if mode == "control"
        else create_spectator_media_token(
            settings, session.robot_id, user_id, session.session_id
        )
    )
    base = websocket_base(request)
    return {
        "session_id": session.session_id,
        "robot_id": session.robot_id,
        "status": session.status if mode == "control" else "spectating",
        "mode": mode,
        "started_at": session.started_at.isoformat(),
        "expires_at": session.expires_at.isoformat(),
        "controller": (
            {
                "id": controller.id,
                "name": controller.full_name,
                "username": controller.username,
                "role": controller.role,
            }
            if controller
            else None
        ),
        "media": {
            "url": settings.livekit_url,
            "room_name": f"robot-{session.robot_id}",
            "token": media_token,
        },
        "control_websocket_url": (
            f"{base}/ws/user/control/{session.robot_id}"
            if mode == "control"
            else ""
        ),
        "telemetry_websocket_url": (
            f"{base}/ws/user/telemetry/{session.robot_id}"
        ),
    }


def session_owner(database: Session, session: SessionRuntime) -> User | None:
    return database.get(User, session.user_id)


def can_view_session(actor: User, owner: User | None, session: SessionRuntime) -> bool:
    return (
        actor.id == session.user_id
        or (
            actor.role in {"admin", "operator"}
            and owner is not None
            and owner.role == "guest"
        )
    )


def persist_session_end(
    database: Session,
    session_id: str,
    actor_id: str,
    reason: str,
) -> None:
    record = database.get(ControlSession, session_id)
    if record:
        record.status = "ended"
        record.ended_at = datetime.now(timezone.utc)
        record.ended_by_user_id = actor_id
        record.end_reason = reason
        database.commit()


def camera_payload(
    source: CameraSourceRuntime, index: int, *, detailed: bool
) -> dict:
    public = {
        "id": source.camera_id,
        "label": source.label if detailed else f"Camera {index + 1}",
        "selected": source.selected,
    }
    if detailed:
        public.update(
            {
                "source_type": source.source_type,
                "source": source.source,
            }
        )
    return public


async def load_camera_inventory(robot_id: str) -> dict:
    try:
        return await hub.request_robot(
            robot_id, "media.cameras.get", {}, timeout_seconds=2
        )
    except asyncio.TimeoutError:
        # Compatibility with edge agents released before the lightweight
        # camera-inventory message existed.
        return await hub.request_robot(
            robot_id,
            "media.sources.get",
            {"media_kind": "video"},
            timeout_seconds=8,
        )


async def select_robot_camera(
    session: SessionRuntime, source: CameraSourceRuntime
) -> dict:
    payload = {
        "source_type": source.source_type,
        "source": source.source,
        "label": source.label,
        "session_id": session.session_id,
    }
    try:
        return await hub.request_robot(
            session.robot_id,
            "media.source.select",
            payload,
            timeout_seconds=2,
        )
    except asyncio.TimeoutError:
        # Legacy agents can still switch live by applying the selected source
        # to their complete media configuration and restarting the publisher.
        current = await hub.request_robot(
            session.robot_id,
            "configuration.get",
            {},
            timeout_seconds=5,
        )
        configuration = {
            key: current[key]
            for key in (
                "device_ip",
                "video_source_type",
                "video_source",
                "video_profile",
                "rtsp_transport",
                "camera_label",
                "audio_source_type",
                "audio_source",
                "microphone_label",
            )
            if key in current
        }
        configuration.update(
            {
                "video_source_type": "camera",
                "video_source": source.source,
                "camera_label": source.label,
            }
        )
        return await hub.request_robot(
            session.robot_id,
            "configuration.update",
            configuration,
            timeout_seconds=8,
        )


@router.post("")
async def create_session(
    body: SessionCreate,
    request: Request,
    actor: User = Depends(authenticated_user),
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    try:
        session = await hub.create_session(
            body.robot_id, actor.id, settings.session_timeout_seconds
        )
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot") from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except RuntimeError as exc:
        raise HTTPException(
            status_code=409, detail="Robot đang có người điều khiển"
        ) from exc

    if not await hub.start_session_media(
        session, settings.media_lease_ttl_seconds
    ):
        await hub.close_session(
            session.session_id, actor.id, reason="robot_disconnected"
        )
        raise HTTPException(
            status_code=409,
            detail="Robot mất kết nối trước khi khởi động camera",
        )

    database.add(
        ControlSession(
            session_id=session.session_id,
            robot_id=session.robot_id,
            user_id=actor.id,
            status="active",
            started_at=session.started_at,
            last_heartbeat_at=session.started_at,
        )
    )
    database.commit()
    return session_payload(
        session,
        request,
        settings,
        user_id=actor.id,
        mode="control",
        controller=actor,
    )


@router.get("/active")
async def active_guest_sessions(
    _: User = Depends(supervisor_required),
    database: Session = Depends(get_db),
) -> list[dict]:
    now = datetime.now(timezone.utc)
    sessions: list[dict] = []
    for session in hub.sessions.values():
        if session.status != "active":
            continue
        owner = session_owner(database, session)
        if owner is None or owner.role != "guest":
            continue
        robot = hub.robots.get(session.robot_id)
        sessions.append(
            {
                "session_id": session.session_id,
                "robot_id": session.robot_id,
                "robot_name": robot.name if robot else session.robot_id,
                "status": session.status,
                "started_at": session.started_at.isoformat(),
                "expires_at": session.expires_at.isoformat(),
                "duration_seconds": max(
                    0, round((now - session.started_at).total_seconds())
                ),
                "controller": {
                    "id": owner.id,
                    "name": owner.full_name,
                    "username": owner.username,
                    "role": owner.role,
                },
            }
        )
    return sorted(sessions, key=lambda item: item["started_at"])


@router.post("/{session_id}/spectate")
async def spectate_guest_session(
    session_id: str,
    request: Request,
    supervisor: User = Depends(supervisor_required),
    settings: Settings = Depends(get_settings),
    database: Session = Depends(get_db),
) -> dict:
    session = hub.get_session(session_id)
    if session is None:
        raise HTTPException(
            status_code=404, detail="Phiên không tồn tại hoặc đã kết thúc"
        )
    owner = session_owner(database, session)
    if owner is None or owner.role != "guest":
        raise HTTPException(
            status_code=403,
            detail="Chỉ được giám sát phiên điều khiển của tài khoản khách",
        )
    return session_payload(
        session,
        request,
        settings,
        user_id=supervisor.id,
        mode="spectator",
        controller=owner,
    )


@router.post("/{session_id}/force-end")
async def force_end_guest_session(
    session_id: str,
    supervisor: User = Depends(supervisor_required),
    database: Session = Depends(get_db),
) -> dict:
    session = hub.get_session(session_id)
    if session is None:
        raise HTTPException(
            status_code=404, detail="Phiên không tồn tại hoặc đã kết thúc"
        )
    owner = session_owner(database, session)
    if owner is None or owner.role != "guest":
        raise HTTPException(
            status_code=403,
            detail="Chỉ được kết thúc cưỡng bức phiên của tài khoản khách",
        )
    await hub.forward_to_robot(
        session.robot_id,
        {
            "message_id": session.session_id,
            "schema_version": "1.0",
            "message_type": "control.stop",
            "robot_id": session.robot_id,
            "session_id": session.session_id,
            "sequence": session.last_sequence + 1,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 1000,
            "payload": {"reason": "force_ended_by_supervisor"},
        },
    )
    if not await hub.close_session(
        session_id, reason="force_ended_by_supervisor"
    ):
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên")
    persist_session_end(
        database,
        session_id,
        supervisor.id,
        "force_ended_by_supervisor",
    )
    return {"session_id": session_id, "status": "ended"}


@router.get("/{session_id}/cameras")
async def session_cameras(
    session_id: str,
    actor: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    session = hub.get_session(session_id)
    if session is None:
        raise HTTPException(
            status_code=404, detail="Phiên không tồn tại hoặc đã kết thúc"
        )
    owner = session_owner(database, session)
    if not can_view_session(actor, owner, session):
        raise HTTPException(status_code=403, detail="Không có quyền xem phiên này")
    try:
        response = await load_camera_inventory(session.robot_id)
    except ConnectionError as exc:
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except asyncio.TimeoutError as exc:
        raise HTTPException(
            status_code=504, detail="Robot không phản hồi danh sách camera"
        ) from exc
    cameras = hub.remember_camera_sources(
        session.robot_id,
        list(response.get("video_sources") or []),
        str(response.get("selected_source") or ""),
    )
    detailed = actor.role in {"admin", "operator"}
    return {
        "robot_id": session.robot_id,
        "items": [
            camera_payload(source, index, detailed=detailed)
            for index, source in enumerate(cameras)
        ],
    }


@router.put("/{session_id}/camera")
async def select_session_camera(
    session_id: str,
    body: SessionCameraSelect,
    actor: User = Depends(authenticated_user),
) -> dict:
    session = hub.get_session(session_id, actor.id)
    if session is None:
        raise HTTPException(
            status_code=403,
            detail="Chỉ người đang điều khiển mới được đổi camera",
        )
    source = hub.camera_source(session.robot_id, body.camera_id)
    if source is None:
        raise HTTPException(
            status_code=404, detail="Camera không tồn tại hoặc đã được rút"
        )
    try:
        response = await select_robot_camera(session, source)
    except ConnectionError as exc:
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except asyncio.TimeoutError as exc:
        raise HTTPException(
            status_code=504, detail="Robot không phản hồi lệnh đổi camera"
        ) from exc
    if response.get("ok") is False:
        raise HTTPException(
            status_code=502,
            detail=str(response.get("error") or "Robot không đổi được camera"),
        )
    selected = hub.select_camera_source(session.robot_id, body.camera_id)
    assert selected is not None
    sources = list(hub.camera_sources.get(session.robot_id, {}).values())
    return camera_payload(
        selected,
        sources.index(selected),
        detailed=actor.role in {"admin", "operator"},
    )


@router.get("/{session_id}")
async def get_session(
    session_id: str,
    actor: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    session = hub.get_session(session_id)
    owner = session_owner(database, session) if session else None
    if session is None or not can_view_session(actor, owner, session):
        raise HTTPException(
            status_code=404, detail="Phiên không tồn tại hoặc đã kết thúc"
        )
    return {
        "session_id": session.session_id,
        "robot_id": session.robot_id,
        "status": session.status,
        "started_at": session.started_at.isoformat(),
        "expires_at": session.expires_at.isoformat(),
    }


@router.delete("/{session_id}")
async def delete_session(
    session_id: str,
    actor: User = Depends(authenticated_user),
    database: Session = Depends(get_db),
) -> dict:
    session = hub.get_session(session_id, actor.id)
    if session:
        await hub.forward_to_robot(
            session.robot_id,
            {
                "message_id": session.session_id,
                "schema_version": "1.0",
                "message_type": "control.stop",
                "robot_id": session.robot_id,
                "session_id": session.session_id,
                "sequence": session.last_sequence + 1,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "ttl_ms": 1000,
                "payload": {"reason": "session_ended"},
            },
        )
    if not await hub.close_session(
        session_id, actor.id, reason="session_ended"
    ):
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên")
    persist_session_end(database, session_id, actor.id, "session_ended")
    return {"session_id": session_id, "status": "ended"}
