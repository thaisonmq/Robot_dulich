import json
from datetime import datetime, timezone
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect, status

from app.core.config import Settings, get_settings
from app.core.security import decode_robot_token, decode_token
from app.models.database import SessionLocal
from app.models.entities import Robot, RobotConnection
from app.schemas.messages import RealtimeMessage
from app.services.hub import hub

router = APIRouter(tags=["websockets"])
MAX_MESSAGE_BYTES = 65_536


async def ws_error(socket: WebSocket, code: int, reason: str) -> None:
    await socket.close(code=code, reason=reason[:120])


@router.websocket("/ws/robot/connect")
async def robot_gateway(socket: WebSocket, settings: Settings = Depends(get_settings)) -> None:
    robot_id = socket.query_params.get("robot_id", "")
    authorization = socket.headers.get("authorization", "")
    token = authorization.removeprefix("Bearer ").strip()
    try:
        authenticated_robot_id = decode_robot_token(token, settings)
    except HTTPException:
        authenticated_robot_id = ""
    if not robot_id or authenticated_robot_id != robot_id:
        await socket.accept()
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "robot token rejected")
        return
    with SessionLocal() as database:
        entity = database.query(Robot).filter(Robot.robot_id == robot_id).first()
        if entity is None or not entity.enabled or entity.credential_hash is None:
            await socket.accept()
            await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "robot not registered")
            return
        hub.sync_registry_robot(
            entity.robot_id,
            entity.name,
            entity.site_id,
            entity.map_id,
            enabled=entity.enabled,
            enrolled=True,
            battery_percent=entity.battery_percent,
            last_seen_at=entity.last_seen_at,
        )
    await socket.accept()
    old = await hub.register_robot(robot_id, socket)
    if old and old is not socket:
        await ws_error(old, 4001, "replaced by a new authenticated connection")
    connection_id = str(uuid4())
    with SessionLocal.begin() as database:
        entity = database.query(Robot).filter(Robot.robot_id == robot_id).first()
        if entity:
            entity.status = "online"
            entity.availability = "available"
            entity.last_seen_at = datetime.now(timezone.utc)
        database.add(
            RobotConnection(
                id=connection_id,
                robot_id=robot_id,
                remote_address=socket.client.host if socket.client else None,
            )
        )
    await socket.send_json(
        {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "gateway.welcome",
            "robot_id": robot_id,
            "session_id": "",
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 0,
            "payload": {"heartbeat_interval_seconds": 2, "protocol": "1.0"},
        }
    )
    try:
        while True:
            raw = await socket.receive_text()
            if len(raw.encode()) > MAX_MESSAGE_BYTES:
                await ws_error(socket, status.WS_1009_MESSAGE_TOO_BIG, "message too large")
                break
            message = RealtimeMessage.model_validate_json(raw)
            if message.robot_id != robot_id:
                await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "robot_id mismatch")
                break
            hub.touch_robot(robot_id)
            data = message.model_dump(mode="json")
            if message.message_type == "robot.pose":
                hub.robots[robot_id].pose.update(message.payload)
            elif message.message_type == "robot.health":
                hub.robots[robot_id].health.update(message.payload)
            elif message.message_type in {
                "configuration.state", "diagnostics.result", "media.sources"
            }:
                request_id = str(message.payload.get("request_id", ""))
                if hub.resolve_robot_request(robot_id, request_id, message.payload):
                    continue
            if message.message_type != "robot.heartbeat":
                await hub.broadcast_telemetry(robot_id, data)
    except (WebSocketDisconnect, ValueError, json.JSONDecodeError):
        pass
    finally:
        await hub.unregister_robot(robot_id, socket)
        with SessionLocal.begin() as database:
            entity = database.query(Robot).filter(Robot.robot_id == robot_id).first()
            if entity:
                entity.status = "offline"
                entity.availability = "offline"
                entity.last_seen_at = hub.robots[robot_id].last_seen_at
            connection = database.get(RobotConnection, connection_id)
            if connection:
                connection.disconnected_at = datetime.now(timezone.utc)


def ws_user(socket: WebSocket, settings: Settings) -> str | None:
    token = socket.query_params.get("token", "")
    try:
        return decode_token(token, settings)
    except Exception:
        return None


@router.websocket("/ws/user/control/{robot_id}")
async def user_control(
    socket: WebSocket, robot_id: str, settings: Settings = Depends(get_settings)
) -> None:
    user_id = ws_user(socket, settings)
    session_id = socket.query_params.get("session_id", "")
    session = hub.get_session(session_id, user_id) if user_id else None
    await socket.accept()
    if not session or session.robot_id != robot_id:
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "invalid session")
        return
    session.control_connected = True
    try:
        while True:
            raw = await socket.receive_text()
            if len(raw.encode()) > MAX_MESSAGE_BYTES:
                await ws_error(socket, status.WS_1009_MESSAGE_TOO_BIG, "message too large")
                return
            message = RealtimeMessage.model_validate_json(raw)
            ack_status = "accepted"
            if message.robot_id != robot_id or message.session_id != session_id:
                ack_status = "invalid_session"
            elif message.expired():
                ack_status = "expired"
            elif message.sequence <= session.last_sequence:
                ack_status = "rejected"
            elif message.message_type not in {"control.velocity", "control.stop"}:
                ack_status = "rejected"
            else:
                session.last_sequence = message.sequence
                if not await hub.forward_to_robot(robot_id, message.model_dump(mode="json")):
                    ack_status = "robot_offline"
            await socket.send_json(
                {
                    "message_id": str(uuid4()),
                    "schema_version": "1.0",
                    "message_type": "command.ack",
                    "robot_id": robot_id,
                    "session_id": session_id,
                    "sequence": message.sequence,
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "ttl_ms": 0,
                    "payload": {
                        "command_message_id": str(message.message_id),
                        "status": ack_status,
                    },
                }
            )
    except (WebSocketDisconnect, ValueError, json.JSONDecodeError):
        pass
    finally:
        session.control_connected = False
        await hub.forward_to_robot(
            robot_id,
            {
                "message_id": str(uuid4()),
                "schema_version": "1.0",
                "message_type": "control.stop",
                "robot_id": robot_id,
                "session_id": session_id,
                "sequence": session.last_sequence + 1,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "ttl_ms": 1000,
                "payload": {"reason": "user_control_disconnected"},
            },
        )
        await hub.close_session(session_id, user_id)


@router.websocket("/ws/user/telemetry/{robot_id}")
async def user_telemetry(
    socket: WebSocket, robot_id: str, settings: Settings = Depends(get_settings)
) -> None:
    user_id = ws_user(socket, settings)
    session_id = socket.query_params.get("session_id", "")
    session = hub.get_session(session_id, user_id) if user_id else None
    await socket.accept()
    if not session or session.robot_id != robot_id:
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "invalid session")
        return
    hub.telemetry_sockets.setdefault(robot_id, set()).add(socket)
    robot = hub.robots[robot_id]
    await socket.send_json(
        {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "robot.pose",
            "robot_id": robot_id,
            "session_id": session_id,
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 0,
            "payload": robot.pose,
        }
    )
    try:
        while True:
            await socket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        hub.telemetry_sockets.get(robot_id, set()).discard(socket)
