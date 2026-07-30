from datetime import datetime, timezone
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException

from app.core.security import current_user
from app.schemas.messages import (
    NavigationCancelRequest,
    NavigationGoalRequest,
    NavigationPreviewRequest,
)
from app.services.hub import hub
from app.services.maps import destination_by_id

router = APIRouter(prefix="/api/navigation", tags=["navigation"])


@router.post("/preview")
async def preview(body: NavigationPreviewRequest, _: str = Depends(current_user)) -> dict:
    robot = hub.robots.get(body.robot_id)
    destination = destination_by_id(body.destination_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if destination is None or destination["map_id"] != robot.map_id:
        raise HTTPException(status_code=400, detail="Điểm đến không hợp lệ với bản đồ")
    return hub.create_route(body.robot_id, destination)


@router.post("/goal")
async def goal(body: NavigationGoalRequest, user_id: str = Depends(current_user)) -> dict:
    session = hub.get_session(body.session_id, user_id)
    route = hub.routes.get(body.route_id)
    if session is None or session.robot_id != body.robot_id:
        raise HTTPException(status_code=403, detail="Phiên điều khiển không hợp lệ")
    if route is None or route["robot_id"] != body.robot_id:
        raise HTTPException(status_code=404, detail="Không tìm thấy tuyến đường")
    message = {
        "message_id": str(uuid4()),
        "schema_version": "1.0",
        "message_type": "navigation.goal",
        "robot_id": body.robot_id,
        "session_id": body.session_id,
        "sequence": session.last_sequence + 1,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "ttl_ms": 5000,
        "payload": route,
    }
    if not await hub.forward_to_robot(body.robot_id, message):
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    return {"status": "accepted", **route}


@router.post("/cancel")
async def cancel(body: NavigationCancelRequest, user_id: str = Depends(current_user)) -> dict:
    session = hub.get_session(body.session_id, user_id)
    if session is None or session.robot_id != body.robot_id:
        raise HTTPException(status_code=403, detail="Phiên điều khiển không hợp lệ")
    message = {
        "message_id": str(uuid4()),
        "schema_version": "1.0",
        "message_type": "navigation.cancel",
        "robot_id": body.robot_id,
        "session_id": body.session_id,
        "sequence": session.last_sequence + 1,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "ttl_ms": 1000,
        "payload": {"reason": "user_cancelled"},
    }
    await hub.forward_to_robot(body.robot_id, message)
    return {"status": "cancelled"}
