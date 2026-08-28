import json
from datetime import datetime, timedelta, timezone
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect, status

from app.core.config import Settings, get_settings
from app.core.security import decode_robot_token, decode_token
from app.models.database import SessionLocal
from app.models.entities import (
    MappingSession,
    NavigationMission,
    Robot,
    RobotConnection,
    RobotMapCache,
    User,
)
from app.schemas.messages import RealtimeMessage
from app.services.hub import hub

router = APIRouter(tags=["websockets"])
MAX_MESSAGE_BYTES = 65_536


def runtime_capabilities_from_health(payload: dict) -> dict:
    motion_backend = str(payload.get("motion_backend", ""))
    navigation_backend = str(payload.get("navigation_backend", ""))
    runtime_mode = str(payload.get("mode", "")).upper()
    simulated = motion_backend == "simulator" and navigation_backend == "simulator"
    sensor_blockers: list[str] = []
    if navigation_backend == "ros2":
        # IDLE deliberately has neither Nav2 nor SLAM running. The mapping.start
        # command owns the safe transition to SLAM and its adapter performs the
        # authoritative scan/TF/safety preflight after startup. Treating the
        # expected absence of an adapter as a mapping blocker creates a deadlock.
        if runtime_mode != "IDLE":
            if not payload.get("scan_fresh", False):
                sensor_blockers.append("SCAN_STALE")
            if not payload.get("odometry_ready", False):
                sensor_blockers.append("ODOMETRY_UNAVAILABLE")
            if not payload.get("lidar_tf_ready", False):
                sensor_blockers.append("LIDAR_TF_UNAVAILABLE")
    elif not simulated:
        sensor_blockers.append("NAVIGATION_BACKEND_UNAVAILABLE")
    nav2_ready = str(payload.get("nav2", "UNAVAILABLE")).upper() == "READY"
    mapping_ready = simulated or (
        navigation_backend == "ros2" and not sensor_blockers
    )
    navigation_ready = simulated or (
        navigation_backend == "ros2"
        and not sensor_blockers
        and nav2_ready
    )
    source = (
        "simulator"
        if simulated
        else "robot"
        if motion_backend == "ros2" or navigation_backend == "ros2"
        else "unknown"
    )
    return {
        "motion_backend": motion_backend or "unknown",
        "navigation_backend": navigation_backend or "unknown",
        "mapping": mapping_ready,
        "navigation": navigation_ready,
        "mapping_blockers": sensor_blockers,
        "source": source,
    }


def persist_robot_runtime_event(robot_id: str, message_type: str, payload: dict) -> None:
    """Persist authoritative edge state without coupling media/control delivery to it."""
    with SessionLocal.begin() as database:
        if message_type == "mapping.status":
            session_id = str(payload.get("mapping_session_id", ""))
            session = database.get(MappingSession, session_id) if session_id else None
            if session is not None and session.robot_id == robot_id:
                session.status = str(payload.get("status", session.status)).upper()
                session.error_code = payload.get("error_code")
                session.error_message = payload.get("error_message")
        elif message_type == "map.cache.state":
            map_id = str(payload.get("map_id", ""))
            version = int(payload.get("version", 0) or 0)
            cache = database.query(RobotMapCache).filter(
                RobotMapCache.robot_id == robot_id,
                RobotMapCache.map_id == map_id,
                RobotMapCache.version == version,
            ).first()
            if cache is not None:
                cache.status = str(payload.get("status", cache.status)).upper()
                cache.progress_percent = float(payload.get("progress_percent", cache.progress_percent))
                cache.error_message = payload.get("error_message")
        elif message_type in {"navigation.status", "navigation.result", "robot.health"}:
            runtime_mode = str(payload.get("mode", "")).upper()
            runtime_state = str(
                payload.get("state")
                or payload.get("map_state")
                or payload.get("status")
                or ""
            ).upper()
            if runtime_mode == "MAPPING" and runtime_state in {
                "IDLE",
                "MAPPING_STARTING",
                "MAPPING_LOCALIZING",
                "MAPPING_RUNNING",
                "MAPPING_STOPPED_UNSAVED",
                "MAPPING_SAVING",
                "MAPPING_ERROR",
                "MAPPING",
                "PAUSED",
                "FINISHED",
                "CANCELED",
                "FAULT",
            }:
                mapping = (
                    database.query(MappingSession)
                    .filter(
                        MappingSession.robot_id == robot_id,
                        MappingSession.status.not_in(("FINISHED", "CANCELED", "FAULT", "MAPPING_ERROR")),
                    )
                    .order_by(MappingSession.created_at.desc())
                    .first()
                )
                if mapping is not None:
                    mapping_updated_at = mapping.updated_at
                    if mapping_updated_at.tzinfo is None:
                        mapping_updated_at = mapping_updated_at.replace(tzinfo=timezone.utc)
                    starting_grace = (
                        mapping.status == "STARTING"
                        and datetime.now(timezone.utc) - mapping_updated_at
                        <= timedelta(seconds=120)
                    )
                    if runtime_state == "IDLE" and not starting_grace:
                        # The mapping process restarted while Center still had
                        # an active session. Mark the orphan terminal so maps
                        # are not locked forever and the UI can start a clean
                        # continuation from a saved pose-graph.
                        mapping.status = "FAULT"
                        mapping.error_code = "MAPPING_RUNTIME_RESET"
                        mapping.error_message = "SLAM runtime đã reset trước khi phiên mapping kết thúc"
                    elif runtime_state != "IDLE":
                        mapping.status = runtime_state
                        if runtime_state in {
                            "MAPPING", "MAPPING_LOCALIZING", "MAPPING_RUNNING", "MAPPING_STOPPED_UNSAVED",
                            "MAPPING_SAVING", "PAUSED", "FINISHED", "CANCELED",
                        }:
                            mapping.error_code = None
                            mapping.error_message = None
                    if runtime_state == "FAULT":
                        mapping.error_code = str(payload.get("error_code") or "MAPPING_FAULT")
                        mapping.error_message = str(payload.get("error_message") or "ROS mapping runtime fault")
            mission_id = str(payload.get("mission_id", ""))
            mission = database.get(NavigationMission, mission_id) if mission_id else None
            if mission is not None and mission.robot_id == robot_id:
                status_value = runtime_state or str(mission.status).upper()
                status_value = {
                    "MOVING": "NAVIGATING",
                    "CANCELLED": "CANCELED",
                    "WAIT_FOR_DYNAMIC_CLEAR": "RECOVERY",
                    "WAITING_FOR_DYNAMIC_CLEAR": "RECOVERY",
                    "DYNAMIC_REPLAN": "RECOVERY",
                    "SENSOR_TIME_INVALID": "RECOVERY",
                    "VERIFYING": "RECOVERY",
                    "LOCALIZATION_REQUIRED": "LOCALIZATION_LOST",
                }.get(status_value, status_value)
                if status_value in {
                    "READY", "PLANNING", "NAVIGATING", "PAUSED", "BLOCKED",
                    "RECOVERY", "LOCALIZATION_LOST", "SUCCEEDED", "ARRIVED",
                    "CANCELED", "PLAN_FAILED", "FAILED", "FAULT",
                    "NARROW_PATH_DECISION", "MANUAL_BYPASS",
                    "COMPUTING_ALTERNATIVES", "ROUTE_SELECTION",
                }:
                    mission.status = status_value
                    mission.error_code = payload.get("error_code")
                    mission.error_message = payload.get("error_message")


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
                hub.robots[robot_id].capabilities.update(
                    runtime_capabilities_from_health(message.payload)
                )
            elif message.message_type in {
                "command.ack",
                "configuration.state",
                "diagnostics.result",
                "media.sources",
                "media.onvif.devices",
                "media.cameras",
                "media.source.state",
            }:
                request_id = str(message.payload.get("request_id", ""))
                if hub.resolve_robot_request(robot_id, request_id, message.payload):
                    continue
            if message.message_type in {
                "mapping.status",
                "map.cache.state",
                "navigation.status",
                "navigation.result",
                "robot.health",
            }:
                persist_robot_runtime_event(robot_id, message.message_type, message.payload)
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
                "control.velocity", "control.stop", "camera.ptz", "session.heartbeat"
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


@router.websocket("/ws/user/mapping/{mapping_session_id}")
async def user_mapping(
    socket: WebSocket,
    mapping_session_id: str,
    settings: Settings = Depends(get_settings),
) -> None:
    user = ws_user(socket, settings)
    with SessionLocal() as database:
        mapping = database.get(MappingSession, mapping_session_id)
        allowed = bool(
            user
            and mapping
            and (mapping.user_id == user[0] or user[1] in {"admin", "operator"})
        )
        robot_id = mapping.robot_id if mapping else ""
        initial = (
            {
                "mapping_session_id": mapping.session_id,
                "map_id": mapping.map_id,
                "version": mapping.version,
                "status": mapping.status,
            }
            if mapping
            else {}
        )
    await socket.accept()
    if not allowed:
        await ws_error(socket, status.WS_1008_POLICY_VIOLATION, "invalid mapping session")
        return
    hub.telemetry_sockets.setdefault(robot_id, set()).add(socket)
    await socket.send_json(
        {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "mapping.status",
            "robot_id": robot_id,
            "session_id": mapping_session_id,
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 0,
            "payload": initial,
        }
    )
    try:
        while True:
            await socket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        hub.telemetry_sockets.get(robot_id, set()).discard(socket)
