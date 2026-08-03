from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Literal


MOTION_PROTOCOL_VERSION = 1
MAX_MOTION_DATAGRAM_BYTES = 1024
MotionMessageType = Literal["velocity", "stop"]


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
    if command.message_type not in {"velocity", "stop"}:
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
