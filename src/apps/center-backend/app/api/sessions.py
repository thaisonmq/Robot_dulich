from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException, Request

from app.core.config import Settings, get_settings
from app.core.security import current_user
from app.schemas.messages import SessionCreate
from app.services.hub import hub
from app.services.media import create_media_token

router = APIRouter(prefix="/api/sessions", tags=["sessions"])


@router.post("")
async def create_session(
    body: SessionCreate,
    request: Request,
    user_id: str = Depends(current_user),
    settings: Settings = Depends(get_settings),
) -> dict:
    try:
        session = await hub.create_session(body.robot_id, user_id, settings.session_timeout_seconds)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot") from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail="Robot đang có người điều khiển") from exc

    if not await hub.start_session_media(session, settings.media_lease_ttl_seconds):
        await hub.close_session(session.session_id, user_id)
        raise HTTPException(
            status_code=409,
            detail="Robot mất kết nối trước khi khởi động camera",
        )

    token = create_media_token(settings, body.robot_id, user_id, session.session_id)
    ws_scheme = "wss" if request.url.scheme == "https" else "ws"
    ws_base = f"{ws_scheme}://{request.url.netloc}"
    return {
        "session_id": session.session_id,
        "robot_id": body.robot_id,
        "status": session.status,
        "expires_at": session.expires_at.isoformat(),
        "media": {
            "url": settings.livekit_url,
            "room_name": f"robot-{body.robot_id}",
            "token": token,
        },
        "control_websocket_url": f"{ws_base}/ws/user/control/{body.robot_id}",
        "telemetry_websocket_url": f"{ws_base}/ws/user/telemetry/{body.robot_id}",
    }


@router.get("/{session_id}")
async def get_session(session_id: str, user_id: str = Depends(current_user)) -> dict:
    session = hub.get_session(session_id, user_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Phiên không tồn tại hoặc đã kết thúc")
    return {
        "session_id": session.session_id,
        "robot_id": session.robot_id,
        "status": session.status,
        "started_at": session.started_at.isoformat(),
        "expires_at": session.expires_at.isoformat(),
    }


@router.delete("/{session_id}")
async def delete_session(session_id: str, user_id: str = Depends(current_user)) -> dict:
    session = hub.get_session(session_id, user_id)
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
    if not await hub.close_session(session_id, user_id):
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên")
    return {"session_id": session_id, "status": "ended"}
