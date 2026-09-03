from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Literal


MOTION_PROTOCOL_VERSION = 1
MAX_MOTION_DATAGRAM_BYTES = 1024
MotionMessageType = Literal["velocity", "stop", "estop_reset"]
OBSTACLE_FRONT = 1 << 0
OBSTACLE_REAR = 1 << 1
OBSTACLE_LEFT = 1 << 2
OBSTACLE_RIGHT = 1 << 3
OBSTACLE_DIRECTION_MASK = (
    OBSTACLE_FRONT | OBSTACLE_REAR | OBSTACLE_LEFT | OBSTACLE_RIGHT
)


def joy_input_active(
    axes: list[float] | tuple[float, ...],
    buttons: list[int] | tuple[int, ...],
    *,
    deadzone: float,
    axis_indices: tuple[int, ...] | None = None,
) -> bool:
    """Return whether a physical joystick is deliberately being operated."""
    selected_axes = (
        axes
        if axis_indices is None
        else (axes[index] for index in axis_indices if index < len(axes))
    )
    return any(button != 0 for button in buttons) or any(
        math.isfinite(axis) and abs(axis) > deadzone for axis in selected_axes
    )


@dataclass(slots=True)
class SafetyInterlock:
    """Fail-safe state for a periodically refreshed local safety signal."""

    watchdog_ms: int = 0
    sensor_stop_active: bool = False
    blocked_directions: int = 0
    last_update_monotonic: float = 0.0
    received_update: bool = False

    def __post_init__(self) -> None:
        if self.watchdog_ms < 0:
            raise ValueError("safety watchdog must not be negative")

    def update(self, stop_active: bool, now: float | None = None) -> None:
        self.sensor_stop_active = bool(stop_active)
        self._mark_updated(now)

    def update_directions(
        self, blocked_directions: int, now: float | None = None
    ) -> None:
        if blocked_directions < 0 or blocked_directions & ~OBSTACLE_DIRECTION_MASK:
            raise ValueError("invalid obstacle direction mask")
        self.blocked_directions = blocked_directions
        self._mark_updated(now)

    def _mark_updated(self, now: float | None) -> None:
        self.last_update_monotonic = time.monotonic() if now is None else now
        self.received_update = True

    def watchdog_expired(self, now: float | None = None) -> bool:
        if self.watchdog_ms == 0:
            return False
        if not self.received_update:
            return True
        now = time.monotonic() if now is None else now
        elapsed_ms = (now - self.last_update_monotonic) * 1000
        return elapsed_ms < 0 or elapsed_ms > self.watchdog_ms

    def locked(self, now: float | None = None) -> bool:
        return self.sensor_stop_active or self.watchdog_expired(now)

    def reason(self, now: float | None = None) -> str:
        if self.sensor_stop_active:
            return "obstacle"
        if self.watchdog_expired(now):
            return "watchdog"
        return ""

    def filter_velocity(
        self,
        linear_x: float,
        angular_z: float,
        now: float | None = None,
    ) -> tuple[float, float]:
        if self.locked(now):
            return 0.0, 0.0
        if linear_x > 0 and self.blocked_directions & OBSTACLE_FRONT:
            linear_x = 0.0
        elif linear_x < 0 and self.blocked_directions & OBSTACLE_REAR:
            linear_x = 0.0
        if angular_z > 0 and self.blocked_directions & OBSTACLE_LEFT:
            angular_z = 0.0
        elif angular_z < 0 and self.blocked_directions & OBSTACLE_RIGHT:
            angular_z = 0.0
        return linear_x, angular_z


@dataclass(slots=True)
class MeasuredZeroWindow:
    """Require fresh, near-zero measured motion for one continuous dwell."""

    linear_threshold: float = 0.015
    angular_threshold: float = 0.03
    dwell_seconds: float = 0.25
    stable_since: float | None = None

    def observe(
        self,
        *,
        now: float,
        fresh: bool,
        linear_velocity: float | None,
        angular_velocity: float | None,
    ) -> bool:
        valid = bool(
            fresh
            and linear_velocity is not None
            and angular_velocity is not None
            and math.isfinite(linear_velocity)
            and math.isfinite(angular_velocity)
            and abs(linear_velocity) <= self.linear_threshold
            and abs(angular_velocity) <= self.angular_threshold
        )
        if not valid:
            self.stable_since = None
            return False
        if self.stable_since is None or now < self.stable_since:
            self.stable_since = now
        return now - self.stable_since >= self.dwell_seconds


@dataclass(frozen=True, slots=True)
class MotionDatagram:
    protocol_version: int
    boot_id: str
    sequence: int
    message_type: MotionMessageType
    sent_monotonic_ns: int
    ttl_ms: int
    linear_x: float = 0.0
    angular_z: float = 0.0
    obstacle_avoidance_enabled: bool = True
    reason: str = ""

    def expired(self, now_ns: int | None = None) -> bool:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        return now_ns - self.sent_monotonic_ns > self.ttl_ms * 1_000_000


class LatestMotionSlot:
    """Single-slot buffer that drops duplicates and superseded commands."""

    def __init__(self) -> None:
        self._boot_id = ""
        self._last_sequence = -1
        self._command: MotionDatagram | None = None

    def stage(self, command: MotionDatagram) -> bool:
        if command.boot_id != self._boot_id:
            self._boot_id = command.boot_id
            self._last_sequence = -1
        if command.sequence <= self._last_sequence:
            return False
        self._last_sequence = command.sequence
        self._command = command
        return True

    def take(self) -> MotionDatagram | None:
        command = self._command
        self._command = None
        return command


def encode_motion_datagram(command: MotionDatagram) -> bytes:
    payload = json.dumps(
        asdict(command), separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    if len(payload) > MAX_MOTION_DATAGRAM_BYTES:
        raise ValueError("motion datagram is too large")
    return payload


def decode_motion_datagram(
    payload: bytes,
    *,
    now_ns: int | None = None,
) -> MotionDatagram:
    if not payload or len(payload) > MAX_MOTION_DATAGRAM_BYTES:
        raise ValueError("invalid motion datagram size")
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid motion datagram JSON") from exc
    if not isinstance(value, dict):
        raise ValueError("motion datagram must be an object")

    obstacle_avoidance_enabled = value.get("obstacle_avoidance_enabled", True)
    if not isinstance(obstacle_avoidance_enabled, bool):
        raise ValueError("invalid obstacle avoidance mode")
    try:
        command = MotionDatagram(
            protocol_version=int(value["protocol_version"]),
            boot_id=str(value["boot_id"]),
            sequence=int(value["sequence"]),
            message_type=str(value["message_type"]),  # type: ignore[arg-type]
            sent_monotonic_ns=int(value["sent_monotonic_ns"]),
            ttl_ms=int(value["ttl_ms"]),
            linear_x=float(value.get("linear_x", 0.0)),
            angular_z=float(value.get("angular_z", 0.0)),
            obstacle_avoidance_enabled=obstacle_avoidance_enabled,
            reason=str(value.get("reason", "")),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid motion datagram fields") from exc

    if command.protocol_version != MOTION_PROTOCOL_VERSION:
        raise ValueError("unsupported motion protocol version")
    if not command.boot_id or len(command.boot_id) > 64:
        raise ValueError("invalid motion boot id")
    if command.sequence < 0:
        raise ValueError("invalid motion sequence")
    if command.message_type not in {"velocity", "stop", "estop_reset"}:
        raise ValueError("invalid motion message type")
    if not 50 <= command.ttl_ms <= 2_000:
        raise ValueError("invalid motion TTL")
    if len(command.reason) > 160:
        raise ValueError("motion stop reason is too long")
    if not math.isfinite(command.linear_x) or not math.isfinite(command.angular_z):
        raise ValueError("motion velocity must be finite")

    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    # Containers share the host monotonic clock. Reject a clock value that is
    # implausibly far in the future instead of letting it bypass the watchdog.
    if command.sent_monotonic_ns - now_ns > 1_000_000_000:
        raise ValueError("motion timestamp is in the future")
    if command.expired(now_ns):
        raise ValueError("motion datagram expired")
    return command
