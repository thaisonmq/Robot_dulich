import asyncio
import random
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any
from uuid import uuid4

from fastapi import WebSocket


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
            "source": "simulator",
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
    expires_at: datetime
    last_sequence: int = -1
    media_renewed_at: datetime | None = None
    control_connected: bool = False


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
        self.sessions: dict[str, SessionRuntime] = {}
        self.robot_session: dict[str, str] = {}
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
            return old

    async def unregister_robot(self, robot_id: str, socket: WebSocket) -> None:
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
                await self._close_session_for_robot(robot_id)

    async def create_session(self, robot_id: str, user_id: str, timeout_seconds: int) -> SessionRuntime:
        async with self.lock:
            self._expire_sessions()
            robot = self.robots.get(robot_id)
            if robot is None:
                raise KeyError("robot_not_found")
            if robot.status != "online":
                raise ValueError("robot_offline")
            if robot_id in self.robot_session:
                raise RuntimeError("robot_busy")
            session = SessionRuntime(
                session_id=str(uuid4()),
                robot_id=robot_id,
                user_id=user_id,
                status="active",
                started_at=datetime.now(timezone.utc),
                expires_at=datetime.now(timezone.utc) + timedelta(seconds=timeout_seconds),
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

    async def close_session(self, session_id: str, user_id: str | None = None) -> bool:
        closed_session: SessionRuntime | None = None
        async with self.lock:
            session = self.sessions.get(session_id)
            if not session or (user_id and session.user_id != user_id):
                return False
            session.status = "ended"
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
        return True

    async def _close_session_for_robot(self, robot_id: str) -> None:
        session_id = self.robot_session.pop(robot_id, None)
        if session_id and session_id in self.sessions:
            self.sessions[session_id].status = "ended"

    def _expire_sessions(self) -> None:
        now = datetime.now(timezone.utc)
        for session in self.sessions.values():
            if session.status == "active" and session.expires_at <= now:
                session.status = "expired"
                self.robot_session.pop(session.robot_id, None)

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
            and session.control_connected
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
    ) -> dict[str, Any]:
        if robot_id not in self.robot_sockets:
            raise ConnectionError("robot_offline")
        request_id = str(uuid4())
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
            "ttl_ms": int(timeout_seconds * 1000),
            "payload": {**payload, "request_id": request_id},
        }
        try:
            if not await self.forward_to_robot(robot_id, message):
                raise ConnectionError("robot_offline")
            return await asyncio.wait_for(future, timeout=timeout_seconds)
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
