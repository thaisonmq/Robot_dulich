import json
from datetime import datetime, timezone
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect, status

from app.core.config import Settings, get_settings
from app.core.security import decode_robot_token, decode_token
from app.models.database import SessionLocal
from app.models.entities import Robot, RobotConnection, User
from app.schemas.messages import RealtimeMessage
from app.services.hub import hub

router = APIRouter(tags=["websockets"])
MAX_MESSAGE_BYTES = 65_536


async def ws_error(socket: WebSocket, code: int, reason: str) -> None:
    try:
        await socket.close(code=code, reason=reason[:120])
    except (RuntimeError, OSError, WebSocketDisconnect):
        # Replacing a connection races with the old handler finishing. Starlette
        # raises RuntimeError if that handler has already sent its close frame;
        # an obsolete socket must never abort the new authenticated connection.
        pass


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
    connection_id = str(uuid4())
    registered = False
    await socket.accept()
    try:
        old = await hub.register_robot(robot_id, socket)
        registered = True
        if old and old is not socket:
            await ws_error(old, 4001, "replaced by a new authenticated connection")
        with SessionLocal.begin() as database:
            entity = database.query(Robot).filter(Robot.robot_id == robot_id).first()
            if entity:
                runtime = hub.robots[robot_id]
                entity.status = runtime.status
                entity.availability = runtime.availability
                entity.last_seen_at = runtime.last_seen_at
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
                "configuration.state",
                "diagnostics.result",
                "media.sources",
                "media.cameras",
                "media.source.state",
            }:
                request_id = str(message.payload.get("request_id", ""))
                if hub.resolve_robot_request(robot_id, request_id, message.payload):
                    continue
            if message.message_type != "robot.heartbeat":
                await hub.broadcast_telemetry(robot_id, data)
    except (WebSocketDisconnect, ValueError, json.JSONDecodeError, OSError):
        pass
    finally:
        if registered:
            await hub.unregister_robot(robot_id, socket)
        with SessionLocal.begin() as database:
            entity = database.query(Robot).filter(Robot.robot_id == robot_id).first()
            if entity:
                runtime = hub.robots[robot_id]
                entity.status = runtime.status
                entity.availability = runtime.availability
                entity.last_seen_at = runtime.last_seen_at
            connection = database.get(RobotConnection, connection_id)
            if connection:
                connection.disconnected_at = datetime.now(timezone.utc)


def ws_user(socket: WebSocket, settings: Settings) -> tuple[str, str] | None:
    token = socket.query_params.get("token", "")
    try:
        user_id = decode_token(token, settings)
    except Exception:
        return None
    with SessionLocal() as database:
        user = database.get(User, user_id)
        if user is None or not user.active:
            return None
        return user_id, user.role


def can_watch_session(
    user: tuple[str, str], session_user_id: str
) -> bool:
    user_id, role = user
    if user_id == session_user_id:
        return True
    if role not in {"admin", "operator"}:
        return False
    with SessionLocal() as database:
        owner = database.get(User, session_user_id)
        return owner is not None and owner.active and owner.role == "guest"


@router.websocket("/ws/user/control/{robot_id}")
async def user_control(
    socket: WebSocket, robot_id: str, settings: Settings = Depends(get_settings)
) -> None:
    user = ws_user(socket, settings)
    user_id = user[0] if user else None
    session_id = socket.query_params.get("session_id", "")
    # This identifier only lives in the page process, so duplicating a tab gets
    # a different controller identity even though sessionStorage is cloned.
    client_id = socket.query_params.get("client_id", "").strip()[:128]
    if not client_id:
        client_id = f"legacy-{uuid4()}"
    session = hub.get_session(session_id, user_id) if user_id else None
    await socket.accept()
    if not session or session.robot_id != robot_id:
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "invalid session")
        return
    claimed, old_control_socket = await hub.claim_control(
        session_id, client_id, socket
    )
    if not claimed:
        await ws_error(socket, 4009, "session is controlled by another tab")
        return
    session.control_connected = True
    session.control_ever_connected = True
    session.control_last_seen_at = datetime.now(timezone.utc)
    session.control_disconnected_at = None
    hub.session_sockets.setdefault(session_id, set()).add(socket)
    if old_control_socket is not None and old_control_socket is not socket:
        await ws_error(old_control_socket, 4001, "replaced by reconnected controller")
    # A restored browser tab starts its local sequence at zero. The new socket
    # replaces the old controller, so it also starts a fresh command sequence.
    session.last_sequence = -1
    await socket.send_json(
        {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "control.ready",
            "robot_id": robot_id,
            "session_id": session_id,
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 0,
            "payload": {"client_id": client_id},
        }
    )
    try:
        while True:
            raw = await socket.receive_text()
            if len(raw.encode()) > MAX_MESSAGE_BYTES:
                await ws_error(socket, status.WS_1009_MESSAGE_TOO_BIG, "message too large")
                return
            message = RealtimeMessage.model_validate_json(raw)
            session.control_last_seen_at = datetime.now(timezone.utc)
            ack_status = "accepted"
            if session.status != "active":
                ack_status = "invalid_session"
            elif message.robot_id != robot_id or message.session_id != session_id:
                ack_status = "invalid_session"
            elif message.expired():
                ack_status = "expired"
            elif message.sequence <= session.last_sequence:
                ack_status = "rejected"
            elif message.message_type not in {
                "control.velocity", "control.stop", "session.heartbeat"
            }:
                ack_status = "rejected"
            else:
                session.last_sequence = message.sequence
                if (
                    message.message_type != "session.heartbeat"
                    and not await hub.forward_to_robot(
                        robot_id, message.model_dump(mode="json")
                    )
                ):
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
        hub.session_sockets.get(session_id, set()).discard(socket)
        if await hub.release_control(session_id, socket):
            session.control_connected = False
            session.control_disconnected_at = datetime.now(timezone.utc)
        if session.status == "active" and not session.control_connected:
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


@router.websocket("/ws/user/telemetry/{robot_id}")
async def user_telemetry(
    socket: WebSocket, robot_id: str, settings: Settings = Depends(get_settings)
) -> None:
    user = ws_user(socket, settings)
    session_id = socket.query_params.get("session_id", "")
    session = hub.get_session(session_id) if user else None
    await socket.accept()
    if (
        not session
        or session.robot_id != robot_id
        or user is None
        or not can_watch_session(user, session.user_id)
    ):
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "invalid session")
        return
    hub.telemetry_sockets.setdefault(robot_id, set()).add(socket)
    hub.session_sockets.setdefault(session_id, set()).add(socket)
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
        hub.session_sockets.get(session_id, set()).discard(socket)
