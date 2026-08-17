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
    half_width: float = 0.10
    clearance: float = 0.04
    side_margin: float = 0.04
    translation_lateral_margin: float = 0.01
    rotation_margin: float = 0.03
    slow_extra: float = 0.10
    latency_seconds: float = 0.12
    braking_acceleration: float = 0.60
    angular_braking_acceleration: float = 2.0
    rotation_preview_angle: float = 0.30
    trajectory_samples: int = 6
    clear_hysteresis_seconds: float = 0.20
    scan_timeout_seconds: float = 0.28
    laser_x: float = 0.0
    laser_y: float = 0.0
    laser_yaw: float = 0.0


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
    required_stop_distance: float = 0.0
    hard_stop: bool = False
    blocking_beam_index: int = -1
    blocking_point_x: float | None = None
    blocking_point_y: float | None = None
    blocking_range: float | None = None
    blocking_angle: float | None = None
    requested_rotation_direction: str = "NONE"
    measured_angular_velocity: float | None = None
    predicted_swept_clearance: float | None = None
    required_swept_clearance: float | None = None


def stopping_clearance(speed: float, config: SafetyConfig) -> float:
    velocity = abs(speed)
    return (
        velocity * config.latency_seconds
        + velocity * velocity / (2 * config.braking_acceleration)
        + config.clearance
    )


def maximum_safe_speed(clearance: float, config: SafetyConfig) -> float:
    """Invert ``stopping_clearance`` for a bumper-to-obstacle gap.

    This provides a continuous velocity cap.  It avoids interpreting an
    upstream requested velocity as the chassis' current velocity, which can
    otherwise permanently latch a stationary robot at zero output.
    """

    available = max(0.0, float(clearance) - config.clearance)
    acceleration = max(1e-3, float(config.braking_acceleration))
    latency = max(0.0, float(config.latency_seconds))
    return max(
        0.0,
        -acceleration * latency
        + math.sqrt(
            acceleration * acceleration * latency * latency
            + 2.0 * acceleration * available
        ),
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


def _valid_scan_beams(
    scan: ScanSample,
    config: SafetyConfig,
) -> Iterable[tuple[int, float, float, float, float]]:
    for index, distance in enumerate(scan.ranges):
        if (
            not math.isfinite(distance)
            or distance < scan.range_min
            or distance > scan.range_max
        ):
            continue
        sensor_angle = scan.angle_min + index * scan.angle_increment
        sensor_x = distance * math.cos(sensor_angle)
        sensor_y = distance * math.sin(sensor_angle)
        cosine = math.cos(config.laser_yaw)
        sine = math.sin(config.laser_yaw)
        point_x = config.laser_x + cosine * sensor_x - sine * sensor_y
        point_y = config.laser_y + sine * sensor_x + cosine * sensor_y
        yield index, float(distance), math.atan2(point_y, point_x), point_x, point_y


def valid_scan_points(
    scan: ScanSample,
    config: SafetyConfig | None = None,
) -> Iterable[tuple[float, float]]:
    applied = SafetyConfig() if config is None else config
    for _, _, _, point_x, point_y in _valid_scan_beams(scan, applied):
        yield point_x, point_y


def _point_in_pose_footprint(
    point_x: float,
    point_y: float,
    *,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    half_length: float,
    half_width: float,
    margin: float,
) -> bool:
    """Test a scan point against the physical rectangle at one future pose."""
    delta_x = point_x - pose_x
    delta_y = point_y - pose_y
    cosine = math.cos(pose_yaw)
    sine = math.sin(pose_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return (
        abs(local_x) <= half_length + margin
        and abs(local_y) <= half_width + margin
    )


def _rotation_direction_blocked(
    points: tuple[tuple[float, float], ...],
    *,
    angular_z: float,
    measured_angular_z: float | None,
    config: SafetyConfig,
) -> bool:
    """Check only the commanded rotation sweep, not a directionless circle."""
    effective_angular = float(angular_z)
    if (
        measured_angular_z is not None
        and abs(float(measured_angular_z)) > abs(effective_angular)
    ):
        effective_angular = float(measured_angular_z)
    speed = abs(effective_angular)
    if speed <= 0.05:
        return False
    angular_decel = max(1e-3, float(config.angular_braking_acceleration))
    stop_angle = (
        speed * config.latency_seconds
        + speed * speed / (2.0 * angular_decel)
    )
    direction = 1.0 if effective_angular > 0 else -1.0
    samples = max(2, int(config.trajectory_samples))
    for sample in range(1, samples + 1):
        # Preview enough of a sustained in-place command to cover the long
        # rectangular corner.  Braking angle alone is only a few degrees at
        # low speed and would incorrectly approve rotation in a corridor that
        # the 0.30 x 0.20 m body cannot rotate inside.
        yaw = direction * max(config.rotation_preview_angle, stop_angle) * sample / samples
        if any(
            _point_in_pose_footprint(
                x,
                y,
                pose_x=0.0,
                pose_y=0.0,
                pose_yaw=yaw,
                half_length=config.half_length,
                half_width=config.half_width,
                margin=config.rotation_margin,
            )
            for x, y in points
        ):
            return True
    return False


def _rotation_blocker_diagnostic(
    scan: ScanSample,
    *,
    angular_z: float,
    measured_angular_z: float | None,
    config: SafetyConfig,
) -> dict[str, float | int | str | None]:
    """Reproduce the accepted sweep only to identify its blocking beam."""
    effective_angular = float(angular_z)
    if (
        measured_angular_z is not None
        and abs(float(measured_angular_z)) > abs(effective_angular)
    ):
        effective_angular = float(measured_angular_z)
    speed = abs(effective_angular)
    if speed <= 0.05:
        return {}
    angular_decel = max(1e-3, float(config.angular_braking_acceleration))
    stop_angle = (
        speed * config.latency_seconds
        + speed * speed / (2.0 * angular_decel)
    )
    direction = 1.0 if effective_angular > 0 else -1.0
    samples = max(2, int(config.trajectory_samples))
    for sample in range(1, samples + 1):
        yaw = (
            direction
            * max(config.rotation_preview_angle, stop_angle)
            * sample
            / samples
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for index, distance, angle, point_x, point_y in _valid_scan_beams(
            scan, config
        ):
            if point_inside_footprint(point_x, point_y, config):
                continue
            local_x = cosine * point_x + sine * point_y
            local_y = -sine * point_x + cosine * point_y
            if (
                abs(local_x) <= config.half_length + config.rotation_margin
                and abs(local_y) <= config.half_width + config.rotation_margin
            ):
                predicted = rectangle_clearance(
                    local_x,
                    local_y,
                    half_length=config.half_length,
                    half_width=config.half_width,
                )
                return {
                    "blocking_beam_index": index,
                    "blocking_point_x": point_x,
                    "blocking_point_y": point_y,
                    "blocking_range": float(distance),
                    "blocking_angle": angle,
                    "requested_rotation_direction": (
                        "LEFT" if angular_z > 0.0 else "RIGHT"
                    ),
                    "measured_angular_velocity": measured_angular_z,
                    "predicted_swept_clearance": predicted,
                    "required_swept_clearance": config.rotation_margin,
                }
    return {}


def _translation_blocker_diagnostic(
    scan: ScanSample,
    *,
    linear_x: float,
    required: float,
    config: SafetyConfig,
) -> dict[str, float | int | str | None]:
    forward = linear_x > 0.0
    best: tuple[float, int, float, float, float, float] | None = None
    lateral_limit = config.half_width + config.translation_lateral_margin
    for index, distance, angle, point_x, point_y in _valid_scan_beams(scan, config):
        if point_inside_footprint(point_x, point_y, config):
            continue
        if abs(point_y) > lateral_limit:
            continue
        gap = (
            point_x - config.half_length
            if forward and point_x >= config.half_length
            else -point_x - config.half_length
            if not forward and point_x <= -config.half_length
            else math.inf
        )
        if gap <= required and (best is None or gap < best[0]):
            best = (gap, index, distance, angle, point_x, point_y)
    if best is None:
        return {}
    gap, index, distance, angle, point_x, point_y = best
    return {
        "blocking_beam_index": index,
        "blocking_point_x": point_x,
        "blocking_point_y": point_y,
        "blocking_range": distance,
        "blocking_angle": angle,
        "predicted_swept_clearance": gap,
        "required_swept_clearance": required,
    }


def _arc_turn_blocked(
    points: tuple[tuple[float, float], ...],
    *,
    linear_x: float,
    angular_z: float,
    travel_distance: float,
    config: SafetyConfig,
) -> bool:
    """Check 4-8 rectangular future poses along the requested arc."""
    if abs(linear_x) <= 0.02 or abs(angular_z) <= 0.05:
        return False
    samples = max(4, min(8, int(config.trajectory_samples)))
    curvature = angular_z / linear_x
    direction = 1.0 if linear_x > 0 else -1.0
    for sample in range(1, samples + 1):
        distance = direction * travel_distance * sample / samples
        yaw = curvature * distance
        if abs(curvature) <= 1e-6:
            pose_x, pose_y = distance, 0.0
        else:
            radius = 1.0 / curvature
            pose_x = radius * math.sin(yaw)
            pose_y = radius * (1.0 - math.cos(yaw))
        if any(
            _point_in_pose_footprint(
                x,
                y,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=yaw,
                half_length=config.half_length,
                half_width=config.half_width,
                margin=config.side_margin,
            )
            for x, y in points
        ):
            return True
    return False


def evaluate_scan(
    scan: ScanSample,
    *,
    linear_x: float,
    angular_z: float,
    config: SafetyConfig,
    measured_linear_x: float | None = None,
    measured_angular_z: float | None = None,
) -> SafetyDecision:
    braking_speed = (
        linear_x if measured_linear_x is None else measured_linear_x
    )
    required = stopping_clearance(braking_speed, config)
    nearest = math.inf
    front_clearance = math.inf
    rear_clearance = math.inf
    left_clearance = math.inf
    right_clearance = math.inf
    blocked = Direction.NONE
    slow_scale = 1.0
    valid = 0
    points = tuple(
        (x, y)
        for x, y in valid_scan_points(scan, config)
        if not point_inside_footprint(x, y, config)
    )
    for x, y in points:
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
        if (
            x >= config.half_length
            and abs(y) <= config.half_width + config.translation_lateral_margin
        ):
            forward_gap = x - config.half_length
            front_clearance = min(front_clearance, forward_gap)
            if forward_gap <= required:
                point_mask |= Direction.FRONT
        if (
            x <= -config.half_length
            and abs(y) <= config.half_width + config.translation_lateral_margin
        ):
            rear_gap = -x - config.half_length
            rear_clearance = min(rear_clearance, rear_gap)
            if rear_gap <= required:
                point_mask |= Direction.REAR

        if (
            y >= config.half_width
            and -config.half_length <= x <= config.half_length + required
        ):
            left_clearance = min(left_clearance, y - config.half_width)
        if (
            y <= -config.half_width
            and -config.half_length <= x <= config.half_length + required
        ):
            right_clearance = min(right_clearance, -y - config.half_width)
        blocked |= point_mask

    # The hard-stop envelope is based on measured chassis velocity.  Outside
    # it, cap the requested component to a speed that can still stop inside
    # the available bumper clearance.  The cap is continuous as the gap
    # closes and never creates the old 5%-command dead zone.
    requested_speed = abs(float(linear_x))
    requested_gap = front_clearance if linear_x > 0 else rear_clearance
    if requested_speed > 1e-4 and math.isfinite(requested_gap):
        safe_speed = maximum_safe_speed(requested_gap, config)
        slow_scale = min(1.0, safe_speed / requested_speed)

    if valid == 0:
        return SafetyDecision(
            True,
            0.0,
            Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT,
            math.inf,
            "empty_scan",
            angular_scale=0.0,
            required_stop_distance=required,
            hard_stop=True,
        )

    translation_blocked = bool(
        (linear_x > 0 and bool(blocked & Direction.FRONT))
        or (linear_x < 0 and bool(blocked & Direction.REAR))
    )
    effective_angular_z = (
        float(measured_angular_z)
        if measured_angular_z is not None
        and abs(float(measured_angular_z)) > abs(float(angular_z))
        else float(angular_z)
    )
    turn_clamped = _rotation_direction_blocked(
        points,
        angular_z=angular_z,
        measured_angular_z=measured_angular_z,
        config=config,
    )
    if not translation_blocked and not turn_clamped and _arc_turn_blocked(
        points,
        linear_x=linear_x,
        angular_z=effective_angular_z,
        travel_distance=required,
        config=config,
    ):
        # Straight motion is still valid; remove only the turn that bends the
        # rectangular footprint into the obstacle.
        turn_clamped = True
    if turn_clamped:
        blocked |= Direction.LEFT if effective_angular_z > 0 else Direction.RIGHT
    blocker_diagnostic = (
        _translation_blocker_diagnostic(
            scan,
            linear_x=linear_x,
            required=required,
            config=config,
        )
        if translation_blocked
        else
        _rotation_blocker_diagnostic(
            scan,
            angular_z=angular_z,
            measured_angular_z=measured_angular_z,
            config=config,
        )
        if turn_clamped
        else {}
    )
    hard_stop = translation_blocked and (
        abs(angular_z) <= 0.05 or turn_clamped
    ) or (abs(linear_x) <= 0.02 and turn_clamped)
    if translation_blocked:
        reason = "front_sweep_collision" if linear_x > 0 else "rear_sweep_collision"
    elif abs(linear_x) <= 0.02 and turn_clamped:
        reason = "rotation_sweep_collision"
    elif turn_clamped:
        reason = (
            "left_turn_clearance" if angular_z > 0 else "right_turn_clearance"
        )
    else:
        reason = ""
    return SafetyDecision(
        stop=hard_stop,
        speed_scale=0.0 if translation_blocked else slow_scale,
        blocked=blocked,
        nearest_clearance=nearest,
        reason=reason,
        angular_scale=(0.0 if turn_clamped else 1.0),
        front_clearance=front_clearance,
        rear_clearance=rear_clearance,
        left_clearance=left_clearance,
        right_clearance=right_clearance,
        required_stop_distance=required,
        hard_stop=hard_stop,
        **blocker_diagnostic,
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
