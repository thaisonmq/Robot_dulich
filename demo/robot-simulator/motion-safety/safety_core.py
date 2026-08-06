from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntFlag
from typing import Iterable


class Direction(IntFlag):
    NONE = 0
    FRONT = 1
    REAR = 2
    LEFT = 4
    RIGHT = 8


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    half_length: float = 0.15
    half_width: float = 0.05
    clearance: float = 0.10
    slow_extra: float = 0.20
    latency_seconds: float = 0.12
    braking_acceleration: float = 0.35
    clear_hysteresis_seconds: float = 0.40
    scan_timeout_seconds: float = 0.28


@dataclass(frozen=True, slots=True)
class ScanSample:
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    stop: bool
    speed_scale: float
    blocked: Direction
    nearest_clearance: float
    reason: str = ""


def stopping_clearance(speed: float, config: SafetyConfig) -> float:
    velocity = abs(speed)
    return (
        velocity * config.latency_seconds
        + velocity * velocity / (2 * config.braking_acceleration)
        + config.clearance
    )


def rectangle_clearance(
    x: float, y: float, *, half_length: float, half_width: float
) -> float:
    """Euclidean distance from a point to the rectangular physical footprint."""
    outside_x = max(abs(x) - half_length, 0.0)
    outside_y = max(abs(y) - half_width, 0.0)
    return math.hypot(outside_x, outside_y)


def point_direction(x: float, y: float, config: SafetyConfig) -> Direction:
    # Corners intentionally block both adjacent directions. This keeps turns
    # from sweeping a physical corner into an obstacle.
    direction = Direction.NONE
    if x >= config.half_length:
        direction |= Direction.FRONT
    elif x <= -config.half_length:
        direction |= Direction.REAR
    if y >= config.half_width:
        direction |= Direction.LEFT
    elif y <= -config.half_width:
        direction |= Direction.RIGHT
    if direction == Direction.NONE:
        direction = Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT
    return direction


def valid_scan_points(scan: ScanSample) -> Iterable[tuple[float, float]]:
    for index, distance in enumerate(scan.ranges):
        if (
            not math.isfinite(distance)
            or distance < scan.range_min
            or distance > scan.range_max
        ):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        yield distance * math.cos(angle), distance * math.sin(angle)


def evaluate_scan(
    scan: ScanSample,
    *,
    linear_x: float,
    angular_z: float,
    config: SafetyConfig,
) -> SafetyDecision:
    required = stopping_clearance(linear_x, config)
    slow_distance = required + config.slow_extra
    nearest = math.inf
    blocked = Direction.NONE
    slow_scale = 1.0
    valid = 0
    for x, y in valid_scan_points(scan):
        valid += 1
        clearance = rectangle_clearance(
            x,
            y,
            half_length=config.half_length,
            half_width=config.half_width,
        )
        nearest = min(nearest, clearance)
        direction = point_direction(x, y, config)
        if clearance <= required:
            blocked |= direction
        elif clearance < slow_distance:
            slow_scale = min(
                slow_scale,
                max(0.05, (clearance - required) / config.slow_extra),
            )

    if valid == 0:
        return SafetyDecision(True, 0.0, Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT, math.inf, "empty_scan")

    motion_blocked = (
        (linear_x > 0 and bool(blocked & Direction.FRONT))
        or (linear_x < 0 and bool(blocked & Direction.REAR))
        or (angular_z > 0 and bool(blocked & (Direction.LEFT | Direction.REAR)))
        or (angular_z < 0 and bool(blocked & (Direction.RIGHT | Direction.REAR)))
    )
    return SafetyDecision(
        stop=motion_blocked,
        speed_scale=0.0 if motion_blocked else slow_scale,
        blocked=blocked,
        nearest_clearance=nearest,
        reason="obstacle" if motion_blocked else "",
    )


def motion_blocked_by_mask(
    linear_x: float, angular_z: float, blocked: Direction
) -> bool:
    """Return whether a directional interlock blocks this velocity.

    The mask is shared by external obstacle programs and the scan evaluator:
    1=front, 2=rear, 4=left, 8=right. A mask never creates motion; it only
    removes components that point into a blocked direction.
    """

    return (
        (linear_x > 0 and bool(blocked & Direction.FRONT))
        or (linear_x < 0 and bool(blocked & Direction.REAR))
        or (angular_z > 0 and bool(blocked & Direction.LEFT))
        or (angular_z < 0 and bool(blocked & Direction.RIGHT))
    )


@dataclass(slots=True)
class StopHysteresis:
    clear_seconds: float
    stopped: bool = False
    clear_since: float | None = None

    def update(self, hazard: bool, now: float) -> bool:
        if hazard:
            self.stopped = True
            self.clear_since = None
            return True
        if not self.stopped:
            return False
        if self.clear_since is None:
            self.clear_since = now
            return True
        if now - self.clear_since < self.clear_seconds:
            return True
        self.stopped = False
        self.clear_since = None
        return False
