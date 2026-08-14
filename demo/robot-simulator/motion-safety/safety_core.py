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
    half_length: float = 0.20
    half_width: float = 0.18
    clearance: float = 0.10
    side_margin: float = 0.06
    rotation_margin: float = 0.04
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
    angular_scale: float = 1.0
    front_clearance: float = math.inf
    rear_clearance: float = math.inf
    left_clearance: float = math.inf
    right_clearance: float = math.inf


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


def point_inside_footprint(x: float, y: float, config: SafetyConfig) -> bool:
    """Return whether a scan return is physically inside the robot body.

    A 2D lidar can see the rear cover, cable loom, or mounting hardware. Those
    self-returns cannot be external obstacles and must not become an all-way
    stop. Points exactly on the footprint boundary remain safety inputs.
    """

    return abs(x) < config.half_length and abs(y) < config.half_width


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
    rotation_radius = math.hypot(config.half_length, config.half_width)
    nearest = math.inf
    front_clearance = math.inf
    rear_clearance = math.inf
    left_clearance = math.inf
    right_clearance = math.inf
    blocked = Direction.NONE
    slow_scale = 1.0
    valid = 0
    pure_rotation = abs(linear_x) < 0.02 and abs(angular_z) > 0.05
    left_path_turn_blocked = False
    right_path_turn_blocked = False
    rotation_blocked = False
    for x, y in valid_scan_points(scan):
        if point_inside_footprint(x, y, config):
            continue
        valid += 1
        clearance = rectangle_clearance(
            x,
            y,
            half_length=config.half_length,
            half_width=config.half_width,
        )
        nearest = min(nearest, clearance)
        point_mask = Direction.NONE

        # Translation uses a true swept rectangle. A side wall beyond the
        # lateral body + margin is not in the forward braking envelope even
        # when its point happens to have x > the front axle.
        if x >= config.half_length and abs(y) <= config.half_width + config.side_margin:
            forward_gap = x - config.half_length
            front_clearance = min(front_clearance, forward_gap)
            if forward_gap <= required:
                point_mask |= Direction.FRONT
            elif linear_x > 0 and forward_gap < slow_distance:
                slow_scale = min(
                    slow_scale,
                    max(0.05, (forward_gap - required) / config.slow_extra),
                )
        if (
            x <= -config.half_length
            and abs(y) <= config.half_width + config.side_margin
        ):
            rear_gap = -x - config.half_length
            rear_clearance = min(rear_clearance, rear_gap)
            if rear_gap <= required:
                point_mask |= Direction.REAR
            elif linear_x < 0 and rear_gap < slow_distance:
                slow_scale = min(
                    slow_scale,
                    max(0.05, (rear_gap - required) / config.slow_extra),
                )

        # Rotation is a separate chassis-corner sweep. It is deliberately not
        # reused as a forward-stop test: a corridor can permit straight travel
        # while being too narrow for an in-place turn.
        radial_gap = math.hypot(x, y) - rotation_radius
        if pure_rotation and radial_gap <= config.rotation_margin:
            rotation_blocked = True
            if y > 0:
                point_mask |= Direction.LEFT
            elif y < 0:
                point_mask |= Direction.RIGHT
            else:
                point_mask |= Direction.LEFT | Direction.RIGHT
        if (
            y >= config.half_width
            and -config.half_length <= x <= config.half_length + required
        ):
            left_clearance = min(left_clearance, y - config.half_width)
            if radial_gap <= config.rotation_margin:
                point_mask |= Direction.LEFT
                left_path_turn_blocked = True
        if (
            y <= -config.half_width
            and -config.half_length <= x <= config.half_length + required
        ):
            right_clearance = min(right_clearance, -y - config.half_width)
            if radial_gap <= config.rotation_margin:
                point_mask |= Direction.RIGHT
                right_path_turn_blocked = True
        blocked |= point_mask

    if valid == 0:
        return SafetyDecision(
            True,
            0.0,
            Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT,
            math.inf,
            "empty_scan",
            angular_scale=0.0,
        )

    translation_blocked = (
        (linear_x > 0 and bool(blocked & Direction.FRONT))
        or (linear_x < 0 and bool(blocked & Direction.REAR))
    )
    hard_stop = translation_blocked or (pure_rotation and rotation_blocked)
    turn_clamped = (
        not hard_stop
        and not pure_rotation
        and (
            (angular_z > 0.05 and left_path_turn_blocked)
            or (angular_z < -0.05 and right_path_turn_blocked)
        )
    )
    if translation_blocked:
        reason = "front_sweep_collision" if linear_x > 0 else "rear_sweep_collision"
    elif pure_rotation and rotation_blocked:
        reason = "rotation_sweep_collision"
    elif turn_clamped:
        reason = (
            "left_turn_clearance" if angular_z > 0 else "right_turn_clearance"
        )
    else:
        reason = ""
    return SafetyDecision(
        stop=hard_stop,
        speed_scale=0.0 if hard_stop else slow_scale,
        blocked=blocked,
        nearest_clearance=nearest,
        reason=reason,
        angular_scale=(0.0 if hard_stop or turn_clamped else slow_scale),
        front_clearance=front_clearance,
        rear_clearance=rear_clearance,
        left_clearance=left_clearance,
        right_clearance=right_clearance,
    )


def clip_motion_by_mask(
    linear_x: float,
    angular_z: float,
    blocked: Direction,
) -> tuple[float, float]:
    """Remove only velocity components aimed into an external interlock."""
    safe_linear = float(linear_x)
    safe_angular = float(angular_z)
    if (safe_linear > 0 and blocked & Direction.FRONT) or (
        safe_linear < 0 and blocked & Direction.REAR
    ):
        safe_linear = 0.0
    if (safe_angular > 0 and blocked & Direction.LEFT) or (
        safe_angular < 0 and blocked & Direction.RIGHT
    ):
        safe_angular = 0.0
    return safe_linear, safe_angular


def motion_blocked_by_mask(
    linear_x: float, angular_z: float, blocked: Direction
) -> bool:
    """Return whether a directional interlock blocks this velocity.

    The mask is shared by external obstacle programs and the scan evaluator:
    1=front, 2=rear, 4=left, 8=right. A mask never creates motion; it only
    removes components that point into a blocked direction.
    """

    safe_linear, safe_angular = clip_motion_by_mask(
        linear_x, angular_z, blocked
    )
    return safe_linear != linear_x or safe_angular != angular_z


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
