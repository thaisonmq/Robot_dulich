from __future__ import annotations

import asyncio
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from simulator.motion import MotionSimulator
from simulator.navigation import NavigationSimulator


class NavigationBackendError(RuntimeError):
    def __init__(self, code: str, message: str, *, current_state: str = "FAULT") -> None:
        super().__init__(message)
        self.code = code
        self.current_state = current_state


class NavigationBackend(Protocol):
    async def execute(self, command: str, payload: dict[str, Any]) -> dict[str, Any]: ...

    def state(self) -> dict[str, Any]: ...

    async def manual_takeover(self) -> None: ...


@dataclass(slots=True)
class SimulatorNavigationBackend:
    navigation: NavigationSimulator
    motion: MotionSimulator
    current_state: str = "READY"
    loaded_map_id: str = "MAP-001"
    loaded_version: int = 1
    paused_points: list[dict[str, float]] | None = None

    async def execute(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        expected_state = str(payload.get("expected_state", "")).upper()
        compatible_mapping_start = (
            command == "mapping.start"
            and expected_state == "IDLE"
            and self.current_state in {"READY", "CANCELED", "FINISHED", "FAULT"}
        )
        if expected_state and expected_state != self.current_state and not compatible_mapping_start:
            raise NavigationBackendError(
                "STATE_CONFLICT",
                f"Expected {expected_state}, robot is {self.current_state}",
                current_state=self.current_state,
            )
        if command == "map.load":
            self.loaded_map_id = str(payload["map_id"])
            self.loaded_version = int(payload["version"])
            self.current_state = "READY"
            return {"status": "completed", "current_state": "READY", "progress_percent": 100}
        if command == "map.set_initial_pose":
            pose = dict(payload["pose"])
            self.motion.pose.x = float(pose["x"])
            self.motion.pose.y = float(pose["y"])
            self.motion.pose.yaw = float(pose.get("yaw", 0))
            self.current_state = "READY"
            return {"status": "completed", "current_state": "READY", "localized": True}
        if command == "navigation.compute_path":
            goal = dict(payload["goal"])
            start = {"x": self.motion.pose.x, "y": self.motion.pose.y}
            end = {"x": float(goal["x"]), "y": float(goal["y"])}
            distance = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
            steps = max(1, math.ceil(distance / 0.25))
            points = [
                {
                    "x": start["x"] + (end["x"] - start["x"]) * index / steps,
                    "y": start["y"] + (end["y"] - start["y"]) * index / steps,
                }
                for index in range(steps + 1)
            ]
            self.current_state = "READY"
            return {
                "status": "completed",
                "current_state": "READY",
                "points": points,
                "distance_m": round(distance, 3),
            }
        if command in {"navigation.start", "navigation.goal"}:
            if command == "navigation.start":
                goal = dict(payload["goal"])
                points = [
                    {"x": self.motion.pose.x, "y": self.motion.pose.y},
                    {"x": float(goal["x"]), "y": float(goal["y"])},
                ]
                route_id = str(payload.get("mission_id", ""))
            else:
                points = list(payload.get("points") or [])
                route_id = str(payload.get("route_id", ""))
            self.navigation.start(route_id, points)
            self.current_state = "NAVIGATING"
            return {"status": "accepted", "current_state": "NAVIGATING"}
        if command == "navigation.pause":
            if self.current_state not in {"NAVIGATING", "BLOCKED"}:
                raise NavigationBackendError("STATE_CONFLICT", "Navigation is not running", current_state=self.current_state)
            self.paused_points = list(self.navigation.points[self.navigation.point_index :])
            self.motion.stop("navigation_paused")
            self.navigation.status = "paused"
            self.current_state = "PAUSED"
            return {"status": "completed", "current_state": "PAUSED"}
        if command == "navigation.resume":
            if self.current_state != "PAUSED" or not self.paused_points:
                raise NavigationBackendError("STATE_CONFLICT", "Navigation is not paused", current_state=self.current_state)
            points = [{"x": self.motion.pose.x, "y": self.motion.pose.y}, *self.paused_points]
            self.navigation.start(self.navigation.route_id, points)
            self.current_state = "NAVIGATING"
            return {"status": "accepted", "current_state": "NAVIGATING"}
        if command == "navigation.cancel":
            self.navigation.cancel()
            self.motion.stop("navigation_cancelled")
            self.paused_points = None
            self.current_state = "CANCELED"
            return {"status": "completed", "current_state": "CANCELED"}
        if command.startswith("mapping."):
            transitions = {
                "mapping.start": "MAPPING",
                "mapping.pause": "PAUSED",
                "mapping.resume": "MAPPING",
                "mapping.save_draft": "SAVED_DRAFT",
                "mapping.finish": "FINISHED",
                "mapping.cancel": "CANCELED",
            }
            self.current_state = transitions[command]
            return {"status": "completed", "current_state": self.current_state}
        raise NavigationBackendError("UNSUPPORTED_COMMAND", f"Unsupported command: {command}")

    def state(self) -> dict[str, Any]:
        if self.navigation.status == "arrived":
            self.current_state = "ARRIVED"
        return {
            "state": self.current_state,
            "map_id": self.loaded_map_id,
            "map_version": self.loaded_version,
            "localized": True,
            "nav2": "READY",
        }

    async def manual_takeover(self) -> None:
        if self.current_state in {"NAVIGATING", "PAUSED", "BLOCKED"}:
            self.navigation.cancel()
            self.motion.stop("manual_takeover")
            self.paused_points = None
            self.current_state = "CANCELED"


class Ros2NavigationBackend:
    """JSON request/response gateway to the isolated rclpy navigation adapter."""

    def __init__(self, socket_path: str, *, timeout_seconds: float = 20.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_seconds = timeout_seconds
        self._state: dict[str, Any] = {
            "state": "FAULT",
            "localized": False,
            "nav2": "UNAVAILABLE",
        }
        self._lock = asyncio.Lock()

    def _response_timeout(self, command: str) -> float:
        if command in {"mapping.start", "mapping.save_draft", "mapping.finish"}:
            return max(self.timeout_seconds, 90.0)
        return self.timeout_seconds

    async def execute(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        request = {"command": command, "payload": payload}
        try:
            async with self._lock:
                reader, writer = await asyncio.wait_for(
                    asyncio.open_unix_connection(str(self.socket_path)), timeout=2.0
                )
                writer.write(json.dumps(request, separators=(",", ":")).encode() + b"\n")
                await writer.drain()
                raw = await asyncio.wait_for(
                    reader.readline(), timeout=self._response_timeout(command)
                )
                writer.close()
                await writer.wait_closed()
        except (OSError, asyncio.TimeoutError) as exc:
            self._state.update({"state": "FAULT", "nav2": "UNAVAILABLE"})
            raise NavigationBackendError(
                "NAVIGATION_ADAPTER_UNAVAILABLE",
                "ROS 2 navigation adapter is unavailable",
            ) from exc
        try:
            result = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise NavigationBackendError("INVALID_ADAPTER_RESPONSE", "ROS adapter returned invalid JSON") from exc
        if not isinstance(result, dict):
            raise NavigationBackendError("INVALID_ADAPTER_RESPONSE", "ROS adapter returned invalid payload")
        self._state.update(dict(result.get("state") or {}))
        if result.get("status") == "rejected":
            raise NavigationBackendError(
                str(result.get("error_code", "ROS_COMMAND_REJECTED")),
                str(result.get("error_message", "ROS command rejected")),
                current_state=str(result.get("current_state", self._state.get("state", "FAULT"))),
            )
        return result

    def state(self) -> dict[str, Any]:
        return dict(self._state)

    async def manual_takeover(self) -> None:
        mode = str(self._state.get("mode", "")).upper()
        current_state = str(self._state.get("state", "")).upper()
        # Manual exploration is the expected motion source while SLAM is in
        # MAPPING mode. It must not be mistaken for a Nav2 takeover, otherwise
        # the first joystick/web command moves the adapter to CANCELED and all
        # subsequent pause/save/finish mapping commands conflict.
        if mode == "MAPPING" or current_state not in {
            "NAVIGATING",
            "PAUSED",
            "BLOCKED",
        }:
            return
        try:
            await self.execute("navigation.cancel", {"reason": "manual_takeover"})
        except NavigationBackendError:
            # Manual motion must not wait for Nav2. motion-safety arbitrates the
            # source immediately; the failed cancel is surfaced in telemetry.
            self._state["state"] = "FAULT"


def build_navigation_backend(
    backend: str,
    navigation: NavigationSimulator,
    motion: MotionSimulator,
    socket_path: str,
) -> NavigationBackend:
    if backend == "ros2":
        return Ros2NavigationBackend(socket_path)
    return SimulatorNavigationBackend(navigation, motion)
