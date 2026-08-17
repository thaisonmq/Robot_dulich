from __future__ import annotations

import asyncio
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from simulator.motion import MotionSimulator
from simulator.navigation import NavigationSimulator


AUTO_SPEED_MODES = {"SLOW", "NORMAL", "FAST"}


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
    auto_speed_mode: str = "NORMAL"

    async def execute(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        expected_state = str(payload.get("expected_state", "")).upper()
        compatible_mapping_start = (
            command == "mapping.start"
            and expected_state == "IDLE"
            and self.current_state in {"READY", "CANCELED", "FINISHED", "FAULT", "MAPPING_ERROR"}
        )
        unconditional_safety_command = command in {
            "navigation.cancel", "navigation.speed_mode", "map.deactivate"
        }
        if (
            expected_state
            and expected_state != self.current_state
            and not compatible_mapping_start
            and not unconditional_safety_command
        ):
            raise NavigationBackendError(
                "STATE_CONFLICT",
                f"Expected {expected_state}, robot is {self.current_state}",
                current_state=self.current_state,
            )
        if command == "map.load":
            self.loaded_map_id = str(payload["map_id"])
            self.loaded_version = int(payload["version"])
            self.current_state = "READY"
            return {
                "status": "completed", "current_state": "READY", "progress_percent": 100,
                "state": {"localized": True, "localization_state": "READY", "localization_confidence": 1.0},
            }
        if command == "map.deactivate":
            self.navigation.cancel()
            self.motion.stop("map_deactivated")
            self.loaded_map_id = ""
            self.loaded_version = 0
            self.current_state = "NO_ACTIVE_MAP"
            return {"status": "completed", "current_state": "NO_ACTIVE_MAP"}
        if command == "map.set_initial_pose":
            pose = dict(payload["pose"])
            self.motion.pose.x = float(pose["x"])
            self.motion.pose.y = float(pose["y"])
            self.motion.pose.yaw = float(pose.get("yaw", 0))
            self.current_state = "READY"
            return {"status": "completed", "current_state": "READY", "localized": True}
        if command == "navigation.compute_path":
            self._validate_map_identity(payload)
            raise NavigationBackendError(
                "MAP_VALIDATION_UNAVAILABLE",
                "Simulator backend cannot validate a saved occupancy map; no direct-line fallback is allowed",
                current_state="READY",
            )
        if command in {"navigation.start", "navigation.goal"}:
            self._validate_map_identity(payload)
            if command == "navigation.start":
                points = list(payload.get("points") or [])
                if len(points) < 2:
                    raise NavigationBackendError(
                        "VALIDATED_ROUTE_REQUIRED",
                        "Simulator navigation.start requires validated preview geometry",
                        current_state="READY",
                    )
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
        if command == "navigation.speed_mode":
            mode = str(payload.get("mode", "")).upper()
            if mode not in AUTO_SPEED_MODES:
                raise NavigationBackendError(
                    "INVALID_SPEED_MODE",
                    "Auto navigation speed mode must be SLOW, NORMAL or FAST",
                    current_state=self.current_state,
                )
            self.auto_speed_mode = mode
            return {
                "status": "completed",
                "current_state": self.current_state,
                "mode": mode,
            }
        if command.startswith("mapping."):
            transitions = {
                "mapping.start": "MAPPING_RUNNING",
                "mapping.stop": "MAPPING_STOPPED_UNSAVED",
                "mapping.pause": "PAUSED",
                "mapping.resume": "MAPPING_RUNNING",
                "mapping.save": "FINISHED",
                "mapping.save_draft": "MAPPING_RUNNING",
                "mapping.finish": "FINISHED",
                "mapping.discard": "CANCELED",
                "mapping.cancel": "CANCELED",
            }
            self.current_state = transitions[command]
            return {"status": "completed", "current_state": self.current_state}
        raise NavigationBackendError("UNSUPPORTED_COMMAND", f"Unsupported command: {command}")

    def _validate_map_identity(self, payload: dict[str, Any]) -> None:
        map_id = str(payload.get("map_id") or self.loaded_map_id)
        version = int(payload.get("version") or self.loaded_version)
        if map_id != self.loaded_map_id or version != self.loaded_version:
            raise NavigationBackendError(
                "MAP_MISMATCH", "Map/version does not match the active saved map",
                current_state=self.current_state,
            )

    def state(self) -> dict[str, Any]:
        if self.navigation.status == "arrived":
            self.current_state = "ARRIVED"
        return {
            "state": self.current_state,
            "map_id": self.loaded_map_id,
            "map_version": self.loaded_version,
            "localized": True,
            "localization_state": "READY",
            "localization_confidence": 1.0,
            "nav2": "READY",
            "auto_speed_mode": self.auto_speed_mode,
            "mode": "MAPPING" if self.current_state.startswith("MAPPING_") else "NAVIGATION",
        }

    async def manual_takeover(self) -> None:
        if self.current_state in {"NAVIGATING", "PAUSED", "BLOCKED"}:
            self.navigation.cancel()
            self.motion.stop("manual_takeover")
            self.paused_points = None
            self.current_state = "CANCELED"


class Ros2NavigationBackend:
    """JSON request/response gateway to the isolated rclpy navigation adapter."""

    def __init__(
        self,
        socket_path: str,
        *,
        timeout_seconds: float = 20.0,
        mode_request_path: str = "/var/lib/rovera/navigation/mode-request.json",
        mode_switch_timeout_seconds: float = 60.0,
    ) -> None:
        self.socket_path = Path(socket_path)
        self.mode_request_path = Path(mode_request_path)
        self.timeout_seconds = timeout_seconds
        self.mode_switch_timeout_seconds = mode_switch_timeout_seconds
        self._state: dict[str, Any] = {
            "state": "FAULT",
            "localized": False,
            "nav2": "UNAVAILABLE",
        }
        self._lock = asyncio.Lock()

    def _response_timeout(self, command: str) -> float:
        if command in {"mapping.start", "mapping.save", "mapping.save_draft", "mapping.finish"}:
            return max(self.timeout_seconds, 90.0)
        if command == "navigation.alternatives":
            return max(self.timeout_seconds, 60.0)
        return self.timeout_seconds

    @staticmethod
    def _retryable_mapping_start(result: dict[str, Any]) -> bool:
        """Identify short-lived failures while a fresh SLAM runtime joins DDS."""
        code = str(result.get("error_code", "")).upper()
        if code in {"SCAN_STALE", "ODOMETRY_UNAVAILABLE", "LIDAR_TF_UNAVAILABLE"}:
            return True
        return (
            code == "MAPPING_AUTHORITY_CONFLICT"
            and "_NODE_NAME_UNKNOWN_" in str(result.get("error_message", ""))
        )

    @staticmethod
    def _required_mode(command: str) -> str | None:
        if command == "navigation.speed_mode":
            # Persist/apply through whichever adapter is already active. A
            # speed selection must never restart Nav2 or interrupt SLAM.
            return None
        if command.startswith("mapping."):
            return "MAPPING"
        if command.startswith("map.") or command.startswith("navigation."):
            return "NAVIGATION"
        return None

    def _write_mode_request(self, mode: str, command: str) -> str:
        self.mode_request_path.parent.mkdir(parents=True, exist_ok=True)
        request = {
            "mode": mode,
            "command": command,
            "requested_at_unix": time.time(),
            "request_id": f"{os.getpid()}-{time.monotonic_ns()}",
        }
        temporary = self.mode_request_path.with_name(
            f".{self.mode_request_path.name}.{os.getpid()}.tmp"
        )
        temporary.write_text(json.dumps(request, separators=(",", ":")))
        os.replace(temporary, self.mode_request_path)
        return str(request["request_id"])

    def _read_mode_status(self) -> dict[str, Any]:
        status_path = self.mode_request_path.with_name("mode-status.json")
        try:
            value = json.loads(status_path.read_text())
        except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
            return {}
        return value if isinstance(value, dict) else {}

    async def _wait_for_idle(self, command: str) -> None:
        request_id = await asyncio.to_thread(self._write_mode_request, "IDLE", command)
        status_path = self.mode_request_path.with_name("mode-status.json")
        deadline = time.monotonic() + self.mode_switch_timeout_seconds
        while time.monotonic() < deadline:
            await asyncio.sleep(0.25)
            try:
                status = json.loads(status_path.read_text())
            except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
                continue
            if str(status.get("request_id")) != request_id:
                continue
            if status.get("status") == "READY" and str(status.get("mode", "")).upper() == "IDLE":
                self._state.update(dict(status.get("state") or {}))
                return
            if status.get("status") == "FAULT":
                raise NavigationBackendError(
                    "MODE_SWITCH_FAILED",
                    str(status.get("error") or "Failed to stop navigation runtime"),
                    current_state=str(self._state.get("state", "FAULT")),
                )
        raise NavigationBackendError(
            "MODE_SWITCH_TIMEOUT",
            "Timed out stopping localization and Nav2 after map deactivation",
            current_state=str(self._state.get("state", "FAULT")),
        )

    async def _call_adapter(self, command: str, payload: dict, timeout: float) -> dict:
        request = {"command": command, "payload": payload}
        reader, writer = await asyncio.wait_for(
            asyncio.open_unix_connection(str(self.socket_path)), timeout=2.0
        )
        try:
            writer.write(json.dumps(request, separators=(",", ":")).encode() + b"\n")
            await writer.drain()
            raw = await asyncio.wait_for(reader.readline(), timeout=timeout)
        finally:
            writer.close()
            await writer.wait_closed()
        if not raw:
            raise OSError("navigation adapter closed without a response")
        try:
            result = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise NavigationBackendError(
                "INVALID_ADAPTER_RESPONSE", "ROS adapter returned invalid JSON"
            ) from exc
        if not isinstance(result, dict):
            raise NavigationBackendError(
                "INVALID_ADAPTER_RESPONSE", "ROS adapter returned invalid payload"
            )
        self._state.update(dict(result.get("state") or {}))
        return result

    async def _ensure_mode(self, command: str) -> bool:
        required = self._required_mode(command)
        if required is None:
            return False
        try:
            status = await self._call_adapter("system.status", {}, 3.0)
            current_mode = str((status.get("state") or {}).get("mode", "")).upper()
            nav2 = str((status.get("state") or {}).get("nav2", "")).upper()
            if current_mode == required and (required != "NAVIGATION" or nav2 == "READY"):
                return False
        except (OSError, asyncio.TimeoutError, NavigationBackendError):
            current_mode = ""
        request_id = await asyncio.to_thread(
            self._write_mode_request, required, command
        )
        deadline = time.monotonic() + self.mode_switch_timeout_seconds
        while time.monotonic() < deadline:
            await asyncio.sleep(0.5)
            supervisor = await asyncio.to_thread(self._read_mode_status)
            if str(supervisor.get("request_id", "")) == request_id:
                if str(supervisor.get("status", "")).upper() == "FAULT":
                    state = dict(supervisor.get("state") or {})
                    if state:
                        self._state.update(state)
                    raise NavigationBackendError(
                        "MODE_SWITCH_FAILED",
                        str(
                            supervisor.get("error")
                            or f"Failed to switch ROS runtime to {required}"
                        ),
                        current_state=str(self._state.get("state", "FAULT")),
                    )
            try:
                status = await self._call_adapter("system.status", {}, 3.0)
            except (OSError, asyncio.TimeoutError, NavigationBackendError):
                continue
            state = dict(status.get("state") or {})
            current_mode = str(state.get("mode", "")).upper()
            nav2 = str(state.get("nav2", "")).upper()
            if current_mode == required and (required != "NAVIGATION" or nav2 == "READY"):
                return True
        self._state.update({"state": "FAULT", "nav2": "UNAVAILABLE"})
        raise NavigationBackendError(
            "MODE_SWITCH_TIMEOUT",
            f"Timed out switching ROS runtime to {required}",
            current_state=str(self._state.get("state", "FAULT")),
        )

    async def execute(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            async with self._lock:
                switched = await self._ensure_mode(command)
                command_payload = dict(payload)
                if switched:
                    # A newly started adapter is authoritative. Do not send the
                    # optimistic browser state from the stack that was stopped.
                    command_payload["expected_state"] = str(
                        self._state.get("state")
                        or ("IDLE" if self._required_mode(command) == "MAPPING" else "READY")
                    ).upper()
                result = await self._call_adapter(
                    command, command_payload, self._response_timeout(command)
                )
                # A stopped ROS participant can remain visible in the DDS graph
                # for a few seconds, while scan/TF callbacks also need time to
                # warm up. Retry only these known transient failures after this
                # request actually switched to a fresh mapping runtime. A real,
                # named second map publisher is still rejected immediately.
                if switched and command == "mapping.start":
                    retry_deadline = time.monotonic() + min(
                        8.0, self.mode_switch_timeout_seconds
                    )
                    while (
                        result.get("status") == "rejected"
                        and self._retryable_mapping_start(result)
                        and time.monotonic() < retry_deadline
                    ):
                        await asyncio.sleep(0.5)
                        command_payload["expected_state"] = str(
                            result.get("current_state") or "IDLE"
                        ).upper()
                        result = await self._call_adapter(
                            command,
                            command_payload,
                            self._response_timeout(command),
                        )
                if command == "map.deactivate":
                    # The adapter first cancels motion and drops map state;
                    # then the host supervisor stops AMCL, map_server and Nav2.
                    await self._wait_for_idle(command)
        except (OSError, asyncio.TimeoutError) as exc:
            self._state.update({"state": "FAULT", "nav2": "UNAVAILABLE"})
            raise NavigationBackendError(
                "NAVIGATION_ADAPTER_UNAVAILABLE",
                "ROS 2 navigation adapter is unavailable",
            ) from exc
        if result.get("status") == "rejected":
            raise NavigationBackendError(
                str(result.get("error_code", "ROS_COMMAND_REJECTED")),
                str(result.get("error_message", "ROS command rejected")),
                current_state=str(result.get("current_state", self._state.get("state", "FAULT"))),
            )
        if command in {"mapping.save", "mapping.finish"}:
            # Returning the ACK and uploading the bundle stay in the edge
            # process. The host supervisor switches SLAM -> Nav2 immediately
            # afterwards without granting either container Docker access.
            await asyncio.to_thread(self._write_mode_request, "NAVIGATION", command)
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
            "NARROW_PATH_DECISION",
        }:
            return
        try:
            await self.execute(
                "navigation.manual_handoff",
                {"reason": "MANUAL_TAKEOVER"},
            )
        except NavigationBackendError:
            # Manual motion must not wait for Nav2. motion-safety arbitrates the
            # source immediately; the failed cancel is surfaced in telemetry.
            self._state["state"] = "FAULT"


def build_navigation_backend(
    backend: str,
    navigation: NavigationSimulator,
    motion: MotionSimulator,
    socket_path: str,
    *,
    motion_backend: str = "simulator",
) -> NavigationBackend:
    if backend == "ros2":
        return Ros2NavigationBackend(
            socket_path,
            mode_request_path=os.getenv(
                "NAVIGATION_MODE_REQUEST_PATH",
                "/var/lib/rovera/navigation/mode-request.json",
            ),
            mode_switch_timeout_seconds=float(
                os.getenv("NAVIGATION_MODE_SWITCH_TIMEOUT_SECONDS", "60")
            ),
        )
    if motion_backend == "ros2" or os.getenv("ROVERA_HARDWARE_MODE", "").lower() == "managed":
        raise NavigationBackendError(
            "NAVIGATION_BACKEND_UNSAFE",
            "Hardware motion requires NAVIGATION_BACKEND=ros2; simulator navigation is forbidden",
        )
    return SimulatorNavigationBackend(navigation, motion)
