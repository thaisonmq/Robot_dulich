import asyncio
import logging
import random
import secrets
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any
from uuid import uuid4

from fastapi import WebSocket


logger = logging.getLogger("center.robot.commands")


@dataclass(slots=True)
class RobotRuntime:
    robot_id: str
    name: str
    site_id: str
    map_id: str
    status: str = "offline"
    availability: str = "available"
    battery_percent: float = 78
    enabled: bool = True
    enrolled: bool = False
    last_seen_at: datetime | None = None
    capabilities: dict[str, Any] = field(
        default_factory=lambda: {
            "media": ["video", "audio"],
            "control": ["velocity", "stop"],
            "navigation": True,
            # Fail closed until the edge reports which motion backend it uses.
            "source": "unknown",
        }
    )
    pose: dict[str, Any] = field(
        default_factory=lambda: {
            "map_id": "MAP-001",
            "x": 5.5,
            "y": 6.0,
            "yaw": 0.0,
            "linear_velocity": 0.0,
            "angular_velocity": 0.0,
        }
    )
    health: dict[str, Any] = field(
        default_factory=lambda: {
            "battery_percent": 78,
            "network_rtt_ms": 42,
            "packet_loss_percent": 0.2,
            "camera": "online",
            "audio": "online",
            "navigation": "idle",
        }
    )
@dataclass(slots=True)
class SessionRuntime:
    session_id: str
    robot_id: str
    user_id: str
    status: str
    started_at: datetime
    expires_at: datetime | None
    last_sequence: int = -1
    media_renewed_at: datetime | None = None
    control_connected: bool = False
    control_ever_connected: bool = False
    control_last_seen_at: datetime | None = None
    control_disconnected_at: datetime | None = None
    robot_disconnected_at: datetime | None = None
    ended_at: datetime | None = None
    end_reason: str | None = None


@dataclass(slots=True)
class CameraSourceRuntime:
    camera_id: str
    source_type: str
    source: str
    label: str
    selected: bool = False
    ptz: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class PreviewLeaseRuntime:
    lease_id: str
    robot_id: str
    user_id: str
    expires_at: datetime


class ConnectionHub:
    def __init__(self) -> None:
        self.lock = asyncio.Lock()
        self.robot_sockets: dict[str, WebSocket] = {}
        self.robot_send_locks: dict[str, asyncio.Lock] = {}
        self.pending_robot_requests: dict[
            str, tuple[str, asyncio.Future[dict[str, Any]]]
        ] = {}
        self.telemetry_sockets: dict[str, set[WebSocket]] = {}
        self.session_sockets: dict[str, set[WebSocket]] = {}
        self.control_sockets: dict[str, WebSocket] = {}
        self.control_clients: dict[str, str] = {}
        self.sessions: dict[str, SessionRuntime] = {}
        self.robot_session: dict[str, str] = {}
        self.camera_sources: dict[str, dict[str, CameraSourceRuntime]] = {}
        self.preview_leases: dict[str, PreviewLeaseRuntime] = {}
        self.robots: dict[str, RobotRuntime] = {}
        self.routes: dict[str, dict[str, Any]] = {}

    def sync_registry_robot(
        self,
        robot_id: str,
        name: str,
        site_id: str,
        map_id: str,
        *,
        enabled: bool = True,
        enrolled: bool = False,
        battery_percent: float = 78,
        last_seen_at: datetime | None = None,
    ) -> RobotRuntime:
        robot = self.robots.get(robot_id)
        if robot is None:
            robot = RobotRuntime(robot_id, name, site_id, map_id)
            self.robots[robot_id] = robot
        robot.name = name
        robot.site_id = site_id
        robot.map_id = map_id
        robot.enabled = enabled
        robot.enrolled = enrolled
        if robot_id not in self.robot_sockets:
            robot.battery_percent = battery_percent
            robot.last_seen_at = last_seen_at
            robot.status = "offline"
            robot.availability = "offline"
        return robot

    async def register_robot(self, robot_id: str, socket: WebSocket) -> WebSocket | None:
        async with self.lock:
            old = self.robot_sockets.get(robot_id)
            self.robot_sockets[robot_id] = socket
            self.robot_send_locks.setdefault(robot_id, asyncio.Lock())
            robot = self.robots[robot_id]
            robot.status = "online"
            robot.availability = "busy" if robot_id in self.robot_session else "available"
            robot.last_seen_at = datetime.now(timezone.utc)
            session_id = self.robot_session.get(robot_id)
            session = self.sessions.get(session_id) if session_id else None
            if session is not None and session.status == "active":
                session.robot_disconnected_at = None
            return old

    async def unregister_robot(
        self, robot_id: str, socket: WebSocket
    ) -> None:
        async with self.lock:
            if self.robot_sockets.get(robot_id) is socket:
                self.robot_sockets.pop(robot_id, None)
                robot = self.robots[robot_id]
                robot.status = "offline"
                robot.availability = "offline"
                for request_id, (pending_robot_id, future) in list(
                    self.pending_robot_requests.items()
                ):
                    if pending_robot_id == robot_id:
                        if not future.done():
                            future.set_exception(ConnectionError("robot_offline"))
                        self.pending_robot_requests.pop(request_id, None)
                session_id = self.robot_session.get(robot_id)
                session = self.sessions.get(session_id) if session_id else None
                if session is not None and session.status == "active":
                    session.robot_disconnected_at = datetime.now(timezone.utc)

    async def create_session(self, robot_id: str, user_id: str, timeout_seconds: int) -> SessionRuntime:
        async with self.lock:
            self._expire_sessions()
            robot = self.robots.get(robot_id)
            if robot is None:
                raise KeyError("robot_not_found")
            if robot.status != "online":
                raise ValueError("robot_offline")
            existing_session_id = self.robot_session.get(robot_id)
            if existing_session_id is not None:
                existing_session = self.sessions.get(existing_session_id)
                if existing_session is not None and existing_session.status == "active":
                    raise RuntimeError("robot_busy")
                self.robot_session.pop(robot_id, None)
            now = datetime.now(timezone.utc)
            session = SessionRuntime(
                session_id=str(uuid4()),
                robot_id=robot_id,
                user_id=user_id,
                status="active",
                started_at=now,
                expires_at=(
                    now + timedelta(seconds=timeout_seconds)
                    if timeout_seconds > 0
                    else None
                ),
            )
            self.sessions[session.session_id] = session
            self.robot_session[robot_id] = session.session_id
            robot.availability = "busy"
            return session

    def get_session(self, session_id: str, user_id: str | None = None) -> SessionRuntime | None:
        self._expire_sessions()
        session = self.sessions.get(session_id)
        if session and user_id and session.user_id != user_id:
            return None
        return session if session and session.status == "active" else None

    async def claim_control(
        self, session_id: str, client_id: str, socket: WebSocket
    ) -> tuple[bool, WebSocket | None]:
        """Keep the first browser tab in control while allowing its reconnects."""
        async with self.lock:
            old_socket = self.control_sockets.get(session_id)
            old_client_id = self.control_clients.get(session_id)
            if (
                old_socket is not None
                and old_socket is not socket
                and old_client_id != client_id
            ):
                return False, None
            self.control_sockets[session_id] = socket
            self.control_clients[session_id] = client_id
            return True, old_socket

    async def release_control(self, session_id: str, socket: WebSocket) -> bool:
        """Release ownership only when this is still the registered socket."""
        async with self.lock:
            if self.control_sockets.get(session_id) is not socket:
                return False
            self.control_sockets.pop(session_id, None)
            self.control_clients.pop(session_id, None)
            return True

    async def close_session(
        self,
        session_id: str,
        user_id: str | None = None,
        *,
        reason: str = "session_ended",
    ) -> bool:
        closed_session: SessionRuntime | None = None
        async with self.lock:
            session = self.sessions.get(session_id)
            if not session or (user_id and session.user_id != user_id):
                return False
            if session.status != "active":
                return False
            session.status = "ended"
            session.ended_at = datetime.now(timezone.utc)
            session.end_reason = reason
            closed_session = session
            self.robot_session.pop(session.robot_id, None)
            robot = self.robots.get(session.robot_id)
            if robot:
                robot.availability = "available" if robot.status == "online" else "offline"
        await self.set_media_lease(
            closed_session.robot_id,
            f"session:{closed_session.session_id}",
            active=False,
        )
        await self.notify_session_ended(closed_session)
        return True

    async def expire_unconnected_sessions(
        self, connect_timeout_seconds: int
    ) -> list[SessionRuntime]:
        """Release sessions whose browser never opened the control channel."""
        now = datetime.now(timezone.utc)
        closed_sessions: list[SessionRuntime] = []
        async with self.lock:
            for session in self.sessions.values():
                if (
                    session.status != "active"
                    or session.control_connected
                    or session.control_ever_connected
                    or (now - session.started_at).total_seconds()
                    < connect_timeout_seconds
                ):
                    continue
                session.status = "ended"
                session.ended_at = now
                session.end_reason = "control_connect_timeout"
                if self.robot_session.get(session.robot_id) == session.session_id:
                    self.robot_session.pop(session.robot_id, None)
                robot = self.robots.get(session.robot_id)
                if robot:
                    robot.availability = (
                        "available" if robot.status == "online" else "offline"
                    )
                closed_sessions.append(session)

        for session in closed_sessions:
            await self.set_media_lease(
                session.robot_id,
                f"session:{session.session_id}",
                active=False,
            )
            await self.notify_session_ended(session)
        return closed_sessions

    async def expire_disconnected_sessions(
        self, reconnect_timeout_seconds: int
    ) -> list[SessionRuntime]:
        """End sessions only after a controller or robot misses its reconnect grace."""
        now = datetime.now(timezone.utc)
        closed_sessions: list[SessionRuntime] = []
        async with self.lock:
            for session in self.sessions.values():
                if session.status != "active":
                    continue
                control_timed_out = (
                    session.control_ever_connected
                    and (
                        (
                            not session.control_connected
                            and session.control_disconnected_at is not None
                            and (now - session.control_disconnected_at).total_seconds()
                            >= reconnect_timeout_seconds
                        )
                        or (
                            session.control_connected
                            and session.control_last_seen_at is not None
                            and (now - session.control_last_seen_at).total_seconds()
                            >= reconnect_timeout_seconds
                        )
                    )
                )
                robot_timed_out = (
                    session.robot_disconnected_at is not None
                    and (now - session.robot_disconnected_at).total_seconds()
                    >= reconnect_timeout_seconds
                )
                if not control_timed_out and not robot_timed_out:
                    continue
                session.status = "ended"
                session.ended_at = now
                session.end_reason = (
                    "robot_reconnect_timeout"
                    if robot_timed_out
                    else "control_reconnect_timeout"
                )
                if self.robot_session.get(session.robot_id) == session.session_id:
                    self.robot_session.pop(session.robot_id, None)
                robot = self.robots.get(session.robot_id)
                if robot:
                    robot.availability = (
                        "available" if robot.status == "online" else "offline"
                    )
                closed_sessions.append(session)

        for session in closed_sessions:
            await self.set_media_lease(
                session.robot_id,
                f"session:{session.session_id}",
                active=False,
            )
            await self.notify_session_ended(session)
        return closed_sessions

    def _expire_sessions(self) -> None:
        now = datetime.now(timezone.utc)
        for session in self.sessions.values():
            if (
                session.status == "active"
                and session.expires_at is not None
                and session.expires_at <= now
            ):
                session.status = "expired"
                session.ended_at = now
                session.end_reason = "session_expired"
                self.robot_session.pop(session.robot_id, None)
                robot = self.robots.get(session.robot_id)
                if robot:
                    robot.availability = (
                        "available" if robot.status == "online" else "offline"
                    )

    async def notify_session_ended(self, session: SessionRuntime) -> None:
        message = {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "session.ended",
            "robot_id": session.robot_id,
            "session_id": session.session_id,
            "sequence": session.last_sequence + 1,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 0,
            "payload": {"reason": session.end_reason or "session_ended"},
        }
        sockets = set(self.session_sockets.get(session.session_id, set()))
        control_socket = self.control_sockets.get(session.session_id)
        if control_socket is not None:
            sockets.add(control_socket)
        for socket in sockets:
            try:
                await socket.send_json(message)
                await socket.close(code=4003, reason="session ended")
            except Exception:
                pass
        self.session_sockets.pop(session.session_id, None)
        self.control_sockets.pop(session.session_id, None)
        self.control_clients.pop(session.session_id, None)

    def remember_camera_sources(
        self,
        robot_id: str,
        sources: list[dict[str, Any]],
        selected_source: str = "",
    ) -> list[CameraSourceRuntime]:
        previous = self.camera_sources.get(robot_id, {})
        previous_by_source = {
            item.source: item for item in previous.values()
        }
        remembered: dict[str, CameraSourceRuntime] = {}
        for index, source in enumerate(sources):
            value = str(source.get("value", "")).strip()
            if not value:
                continue
            prior = previous_by_source.get(value)
            camera = CameraSourceRuntime(
                camera_id=(
                    prior.camera_id if prior else secrets.token_urlsafe(12)
                ),
                source_type=str(source.get("type", "camera")),
                source=value,
                label=str(source.get("label") or f"Camera {index + 1}"),
                selected=value == selected_source,
                ptz=dict(source.get("ptz") or {}),
            )
            remembered[camera.camera_id] = camera
        self.camera_sources[robot_id] = remembered
        return list(remembered.values())

    def camera_source(
        self, robot_id: str, camera_id: str
    ) -> CameraSourceRuntime | None:
        return self.camera_sources.get(robot_id, {}).get(camera_id)

    def select_camera_source(
        self, robot_id: str, camera_id: str
    ) -> CameraSourceRuntime | None:
        selected = self.camera_source(robot_id, camera_id)
        if selected is None:
            return None
        for source in self.camera_sources.get(robot_id, {}).values():
            source.selected = source.camera_id == camera_id
        return selected

    async def forward_to_robot(self, robot_id: str, message: dict[str, Any]) -> bool:
        socket = self.robot_sockets.get(robot_id)
        if socket is None:
            return False
        send_lock = self.robot_send_locks.setdefault(robot_id, asyncio.Lock())
        try:
            async with send_lock:
                if self.robot_sockets.get(robot_id) is not socket:
                    return False
                await socket.send_json(message)
            return True
        except Exception:
            return False

    async def set_media_lease(
        self,
        robot_id: str,
        lease_id: str,
        *,
        active: bool,
        ttl_seconds: int = 30,
        session_id: str = "",
    ) -> bool:
        return await self.forward_to_robot(
            robot_id,
            {
                "message_id": str(uuid4()),
                "schema_version": "1.0",
                "message_type": "media.start" if active else "media.stop",
                "robot_id": robot_id,
                "session_id": session_id,
                "sequence": 0,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "ttl_ms": 5_000,
                "payload": {
                    "lease_id": lease_id,
                    **({"ttl_seconds": ttl_seconds} if active else {}),
                },
            },
        )

    async def start_session_media(
        self, session: SessionRuntime, ttl_seconds: int
    ) -> bool:
        sent = await self.set_media_lease(
            session.robot_id,
            f"session:{session.session_id}",
            active=True,
            ttl_seconds=ttl_seconds,
            session_id=session.session_id,
        )
        if sent:
            session.media_renewed_at = datetime.now(timezone.utc)
        return sent

    async def renew_session_media_leases(
        self, ttl_seconds: int, renew_interval_seconds: int
    ) -> None:
        self._expire_sessions()
        now = datetime.now(timezone.utc)
        sessions = [
            session
            for session in self.sessions.values()
            if session.status == "active"
            and (
                session.media_renewed_at is None
                or (now - session.media_renewed_at).total_seconds()
                >= renew_interval_seconds
            )
        ]
        for session in sessions:
            await self.start_session_media(session, ttl_seconds)

    async def create_preview_lease(
        self, robot_id: str, user_id: str, ttl_seconds: int
    ) -> PreviewLeaseRuntime | None:
        lease = PreviewLeaseRuntime(
            lease_id=str(uuid4()),
            robot_id=robot_id,
            user_id=user_id,
            expires_at=datetime.now(timezone.utc) + timedelta(seconds=ttl_seconds),
        )
        self.preview_leases[lease.lease_id] = lease
        if not await self.set_media_lease(
            robot_id,
            f"preview:{lease.lease_id}",
            active=True,
            ttl_seconds=ttl_seconds,
        ):
            self.preview_leases.pop(lease.lease_id, None)
            return None
        return lease

    async def renew_preview_lease(
        self, robot_id: str, lease_id: str, user_id: str, ttl_seconds: int
    ) -> bool:
        self.expire_preview_leases()
        lease = self.preview_leases.get(lease_id)
        if (
            lease is None
            or lease.robot_id != robot_id
            or lease.user_id != user_id
        ):
            return False
        if not await self.set_media_lease(
            robot_id,
            f"preview:{lease_id}",
            active=True,
            ttl_seconds=ttl_seconds,
        ):
            return False
        lease.expires_at = datetime.now(timezone.utc) + timedelta(
            seconds=ttl_seconds
        )
        return True

    async def close_preview_lease(
        self, robot_id: str, lease_id: str, user_id: str
    ) -> bool:
        lease = self.preview_leases.get(lease_id)
        if (
            lease is None
            or lease.robot_id != robot_id
            or lease.user_id != user_id
        ):
            return False
        self.preview_leases.pop(lease_id, None)
        await self.set_media_lease(
            robot_id,
            f"preview:{lease_id}",
            active=False,
        )
        return True

    def expire_preview_leases(self) -> None:
        now = datetime.now(timezone.utc)
        for lease_id, lease in list(self.preview_leases.items()):
            if lease.expires_at <= now:
                self.preview_leases.pop(lease_id, None)

    async def request_robot(
        self,
        robot_id: str,
        message_type: str,
        payload: dict[str, Any],
        timeout_seconds: float = 5.0,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        if robot_id not in self.robot_sockets:
            raise ConnectionError("robot_offline")
        request_id = request_id or str(uuid4())
        if request_id in self.pending_robot_requests:
            raise RuntimeError("request_already_pending")
        request_started = time.monotonic()
        logger.info(
            "stage=COMMAND_RECEIVED request_id=%s robot_id=%s command=%s",
            request_id,
            robot_id,
            message_type,
        )
        future: asyncio.Future[dict[str, Any]] = asyncio.get_running_loop().create_future()
        self.pending_robot_requests[request_id] = (robot_id, future)
        message = {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": message_type,
            "robot_id": robot_id,
            "session_id": "",
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            # The wire contract caps TTL at 30 seconds. Long-running SLAM
            # commands may still use a longer ACK wait below; TTL only limits
            # how long a queued command may wait before the robot starts it.
            "ttl_ms": min(int(timeout_seconds * 1000), 30_000),
            "payload": {**payload, "request_id": request_id},
        }
        try:
            if not await self.forward_to_robot(robot_id, message):
                raise ConnectionError("robot_offline")
            logger.info(
                "stage=COMMAND_DISPATCHED request_id=%s robot_id=%s command=%s duration_ms=%.1f",
                request_id,
                robot_id,
                message_type,
                (time.monotonic() - request_started) * 1000.0,
            )
            response = await asyncio.wait_for(future, timeout=timeout_seconds)
            logger.info(
                "stage=ACK_RECEIVED request_id=%s robot_id=%s command=%s status=%s error_code=%s duration_ms=%.1f",
                request_id,
                robot_id,
                message_type,
                response.get("status"),
                response.get("error_code", ""),
                (time.monotonic() - request_started) * 1000.0,
            )
            return response
        except TimeoutError:
            logger.warning(
                "stage=ACK_TIMEOUT request_id=%s robot_id=%s command=%s duration_ms=%.1f",
                request_id,
                robot_id,
                message_type,
                (time.monotonic() - request_started) * 1000.0,
            )
            raise
        finally:
            self.pending_robot_requests.pop(request_id, None)

    def resolve_robot_request(
        self, robot_id: str, request_id: str, payload: dict[str, Any]
    ) -> bool:
        pending = self.pending_robot_requests.get(request_id)
        if not pending or pending[0] != robot_id:
            return False
        future = pending[1]
        if not future.done():
            future.set_result(payload)
        return True

    async def broadcast_telemetry(self, robot_id: str, message: dict[str, Any]) -> None:
        sockets = list(self.telemetry_sockets.get(robot_id, set()))
        stale: list[WebSocket] = []
        for socket in sockets:
            try:
                await socket.send_json(message)
            except Exception:
                stale.append(socket)
        for socket in stale:
            self.telemetry_sockets.get(robot_id, set()).discard(socket)

    def touch_robot(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.last_seen_at = datetime.now(timezone.utc)
        robot.status = "online"

    def create_route(self, robot_id: str, destination: dict[str, Any]) -> dict[str, Any]:
        pose = self.robots[robot_id].pose
        start = {"x": float(pose["x"]), "y": float(pose["y"])}
        end = {"x": float(destination["x"]), "y": float(destination["y"])}
        mid = {"x": round((start["x"] + end["x"]) / 2, 2), "y": round(start["y"], 2)}
        distance = abs(mid["x"] - start["x"]) + abs(end["x"] - mid["x"]) + abs(end["y"] - mid["y"])
        route = {
            "route_id": str(uuid4()),
            "robot_id": robot_id,
            "destination_id": destination["destination_id"],
            "points": [start, mid, end],
            "distance_m": round(distance, 2),
            "estimated_seconds": max(5, round(distance / 0.35)),
        }
        self.routes[route["route_id"]] = route
        return route

    def robot_view(self, robot: RobotRuntime) -> dict[str, Any]:
        session_id = self.robot_session.get(robot.robot_id)
        session = self.sessions.get(session_id) if session_id else None
        if session_id and (session is None or session.status != "active"):
            self.robot_session.pop(robot.robot_id, None)
            session_id = None
        robot.availability = (
            "offline"
            if robot.status != "online"
            else "busy" if session_id else "available"
        )
        return {
            "robot_id": robot.robot_id,
            "name": robot.name,
            "site_id": robot.site_id,
            "map_id": robot.map_id,
            "status": robot.status,
            "availability": robot.availability,
            "battery_percent": (
                robot.health.get("battery_percent", robot.battery_percent)
                if robot.status == "online"
                else robot.battery_percent
            ),
            "last_seen_at": robot.last_seen_at.isoformat() if robot.last_seen_at else None,
            "software_version": "sim-1.0",
            "capabilities": robot.capabilities,
            "network_rtt_ms": robot.health.get("network_rtt_ms", random.randint(35, 75)),
            "enabled": robot.enabled,
            "enrollment_status": "enrolled" if robot.enrolled else "pending",
        }


hub = ConnectionHub()
