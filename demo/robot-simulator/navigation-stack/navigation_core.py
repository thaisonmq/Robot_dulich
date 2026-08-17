from __future__ import annotations

import json
import logging
import math
import os
import statistics
from collections import deque
from dataclasses import dataclass
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Iterable

import yaml
from PIL import Image


@dataclass(frozen=True, slots=True)
class GoalValidation:
    valid: bool
    code: str = ""
    message: str = ""


@dataclass(frozen=True, slots=True)
class ExecutablePathValidation:
    valid: bool
    code: str = ""
    segment_index: int = -1
    sample_x: float | None = None
    sample_y: float | None = None
    sample_yaw: float | None = None
    cell_cost: int | None = None
    samples_checked: int = 0
    collision_x: float | None = None
    collision_y: float | None = None
    collision_cells: tuple[tuple[float, float, int], ...] = ()


def validate_executable_grid_path(
    points: Iterable[dict[str, float]],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    data: Iterable[int],
    half_length: float,
    half_width: float,
    allow_unknown: bool = False,
    lethal_threshold: int = 99,
    inscribed_threshold: int | None = None,
) -> ExecutablePathValidation:
    """Validate the oriented physical footprint over every path segment.

    Segment centers are sampled at no more than half a costmap cell. At each
    sample an exact separating-axis test checks the robot rectangle against
    every intersecting costmap cell. This catches wall collisions that a free
    centerline alone cannot detect, including on rotated occupancy grids.
    """
    route = [
        {"x": float(point["x"]), "y": float(point["y"])}
        for point in points
    ]
    costs = [int(value) for value in data]
    width = int(width)
    height = int(height)
    resolution = float(resolution)
    if (
        len(route) < 2
        or width <= 0
        or height <= 0
        or resolution <= 0.0
        or len(costs) != width * height
    ):
        return ExecutablePathValidation(False, "PATH_OR_COSTMAP_INVALID")

    grid_cosine = math.cos(float(origin_yaw))
    grid_sine = math.sin(float(origin_yaw))
    grid_axis_x = (grid_cosine, grid_sine)
    grid_axis_y = (-grid_sine, grid_cosine)
    cell_half = resolution / 2.0
    footprint_radius = math.hypot(float(half_length), float(half_width))
    candidate_radius = footprint_radius + math.sqrt(2.0) * cell_half
    sample_spacing = resolution / 2.0
    samples_checked = 0
    first_collision: tuple[str, int, float, float, float, int, float, float] | None = None
    collision_cells: dict[tuple[int, int], tuple[float, float, int]] = {}

    def grid_coordinates(world_x: float, world_y: float) -> tuple[float, float]:
        delta_x = world_x - float(origin_x)
        delta_y = world_y - float(origin_y)
        return (
            grid_cosine * delta_x + grid_sine * delta_y,
            -grid_sine * delta_x + grid_cosine * delta_y,
        )

    def cell_intersects_footprint(
        column: int,
        row: int,
        center_x: float,
        center_y: float,
        path_yaw: float,
    ) -> bool:
        local_cell_x = (column + 0.5) * resolution
        local_cell_y = (row + 0.5) * resolution
        cell_x = (
            float(origin_x)
            + grid_cosine * local_cell_x
            - grid_sine * local_cell_y
        )
        cell_y = (
            float(origin_y)
            + grid_sine * local_cell_x
            + grid_cosine * local_cell_y
        )
        delta = (cell_x - center_x, cell_y - center_y)
        robot_axis_x = (math.cos(path_yaw), math.sin(path_yaw))
        robot_axis_y = (-robot_axis_x[1], robot_axis_x[0])

        def dot(left: tuple[float, float], right: tuple[float, float]) -> float:
            return left[0] * right[0] + left[1] * right[1]

        for axis in (robot_axis_x, robot_axis_y, grid_axis_x, grid_axis_y):
            distance = abs(dot(delta, axis))
            robot_projection = (
                float(half_length) * abs(dot(robot_axis_x, axis))
                + float(half_width) * abs(dot(robot_axis_y, axis))
            )
            cell_projection = cell_half * (
                abs(dot(grid_axis_x, axis)) + abs(dot(grid_axis_y, axis))
            )
            if distance > robot_projection + cell_projection + 1e-9:
                return False
        return True

    for segment_index, (left, right) in enumerate(zip(route, route[1:])):
        delta_x = right["x"] - left["x"]
        delta_y = right["y"] - left["y"]
        distance = math.hypot(delta_x, delta_y)
        if distance <= 1e-9:
            continue
        yaw = math.atan2(delta_y, delta_x)
        sample_count = max(1, math.ceil(distance / sample_spacing))
        for sample_index in range(sample_count + 1):
            ratio = sample_index / sample_count
            sample_x = left["x"] + delta_x * ratio
            sample_y = left["y"] + delta_y * ratio
            local_x, local_y = grid_coordinates(sample_x, sample_y)
            minimum_column = math.floor((local_x - candidate_radius) / resolution)
            maximum_column = math.floor((local_x + candidate_radius) / resolution)
            minimum_row = math.floor((local_y - candidate_radius) / resolution)
            maximum_row = math.floor((local_y + candidate_radius) / resolution)
            samples_checked += 1
            if (
                minimum_column < 0
                or minimum_row < 0
                or maximum_column >= width
                or maximum_row >= height
            ):
                return ExecutablePathValidation(
                    False,
                    "PATH_FOOTPRINT_OUTSIDE_COSTMAP",
                    segment_index,
                    sample_x,
                    sample_y,
                    yaw,
                    None,
                    samples_checked,
                )
            center_column = math.floor(local_x / resolution)
            center_row = math.floor(local_y / resolution)
            center_cost = costs[center_row * width + center_column]
            center_blocked = (
                inscribed_threshold is not None
                and center_cost >= int(inscribed_threshold)
            ) or (center_cost < 0 and not allow_unknown)
            if center_blocked:
                local_cell_x = (center_column + 0.5) * resolution
                local_cell_y = (center_row + 0.5) * resolution
                collision_x = (
                    float(origin_x)
                    + grid_cosine * local_cell_x
                    - grid_sine * local_cell_y
                )
                collision_y = (
                    float(origin_y)
                    + grid_sine * local_cell_x
                    + grid_cosine * local_cell_y
                )
                code = (
                    "PATH_UNKNOWN_COLLISION"
                    if center_cost < 0
                    else "PATH_FOOTPRINT_COLLISION"
                )
                collision_cells[(center_column, center_row)] = (
                    collision_x,
                    collision_y,
                    center_cost,
                )
                if first_collision is None:
                    first_collision = (
                        code,
                        segment_index,
                        sample_x,
                        sample_y,
                        yaw,
                        center_cost,
                        collision_x,
                        collision_y,
                    )
            for row in range(minimum_row, maximum_row + 1):
                for column in range(minimum_column, maximum_column + 1):
                    cost = costs[row * width + column]
                    blocked = cost >= int(lethal_threshold) or (
                        cost < 0 and not allow_unknown
                    )
                    if not blocked or not cell_intersects_footprint(
                        column, row, sample_x, sample_y, yaw
                    ):
                        continue
                    local_cell_x = (column + 0.5) * resolution
                    local_cell_y = (row + 0.5) * resolution
                    collision_x = (
                        float(origin_x)
                        + grid_cosine * local_cell_x
                        - grid_sine * local_cell_y
                    )
                    collision_y = (
                        float(origin_y)
                        + grid_sine * local_cell_x
                        + grid_cosine * local_cell_y
                    )
                    code = (
                        "PATH_UNKNOWN_COLLISION"
                        if cost < 0
                        else "PATH_FOOTPRINT_COLLISION"
                    )
                    collision_cells[(column, row)] = (
                        collision_x,
                        collision_y,
                        cost,
                    )
                    if first_collision is None:
                        first_collision = (
                            code,
                            segment_index,
                            sample_x,
                            sample_y,
                            yaw,
                            cost,
                            collision_x,
                            collision_y,
                        )
    if first_collision is not None:
        (
            code,
            segment_index,
            sample_x,
            sample_y,
            sample_yaw,
            cell_cost,
            collision_x,
            collision_y,
        ) = first_collision
        return ExecutablePathValidation(
            False,
            code,
            segment_index,
            sample_x,
            sample_y,
            sample_yaw,
            cell_cost,
            samples_checked,
            collision_x,
            collision_y,
            tuple(collision_cells.values()),
        )
    if samples_checked == 0:
        return ExecutablePathValidation(False, "PATH_HAS_NO_LENGTH")
    return ExecutablePathValidation(True, samples_checked=samples_checked)


@dataclass(frozen=True, slots=True)
class ClockCorrection:
    accepted: bool
    corrected_nanoseconds: int | None
    raw_skew_seconds: float
    state: str
    reason: str = ""


class SensorClockEstimator:
    """Estimate one source-clock offset without replacing capture time.

    The Yahboom MCU stamps scan, odometry and IMU from the same clock.  The
    smallest arrival-minus-source sample in a rolling window is the best
    available offset estimate because transport delay can only make a packet
    arrive later.  Correcting by that offset preserves the relative capture
    time between all three sensor streams.
    """

    def __init__(
        self,
        *,
        minimum_sync_samples: int = 5,
        window_samples: int = 120,
        sync_spread_seconds: float = 0.08,
        jump_tolerance_seconds: float = 0.50,
        maximum_corrected_age_seconds: float = 0.25,
        future_tolerance_seconds: float = 0.02,
        invalid_debounce_samples: int = 3,
    ) -> None:
        self.minimum_sync_samples = max(2, int(minimum_sync_samples))
        self.offset_samples: deque[int] = deque(maxlen=max(
            self.minimum_sync_samples, int(window_samples)
        ))
        self.sync_spread_nanoseconds = int(max(0.001, sync_spread_seconds) * 1e9)
        self.jump_tolerance_nanoseconds = int(max(0.01, jump_tolerance_seconds) * 1e9)
        self.maximum_corrected_age_nanoseconds = int(
            max(0.01, maximum_corrected_age_seconds) * 1e9
        )
        self.future_tolerance_nanoseconds = int(max(0.0, future_tolerance_seconds) * 1e9)
        self.invalid_debounce_samples = max(1, int(invalid_debounce_samples))
        self.offset_nanoseconds: int | None = None
        self.invalid_streak = 0
        self.time_invalid = False
        self.rejected_packets = 0
        self.last_source_nanoseconds: dict[str, int] = {}
        self.last_corrected_nanoseconds: dict[str, int] = {}
        self.last_raw_skew_seconds: dict[str, float] = {}
        self.last_reason = ""

    @property
    def state(self) -> str:
        if self.time_invalid:
            return "SENSOR_TIME_INVALID"
        if self.offset_nanoseconds is None:
            return "CLOCK_SYNCING"
        return "SYNCED"

    def _reject(self, raw_skew_seconds: float, reason: str) -> ClockCorrection:
        self.invalid_streak += 1
        self.rejected_packets += 1
        self.last_reason = reason
        if self.invalid_streak >= self.invalid_debounce_samples:
            # A persistent jump is normally an MCU reconnect. Relearn the
            # common offset; never force those packets onto the host clock.
            self.offset_nanoseconds = None
            self.offset_samples.clear()
            self.last_source_nanoseconds.clear()
            self.last_corrected_nanoseconds.clear()
            self.time_invalid = True
        return ClockCorrection(False, None, raw_skew_seconds, self.state, reason)

    def observe(
        self,
        sensor: str,
        *,
        source_nanoseconds: int,
        arrival_nanoseconds: int,
    ) -> ClockCorrection:
        source_nanoseconds = int(source_nanoseconds)
        arrival_nanoseconds = int(arrival_nanoseconds)
        raw_skew_seconds = (arrival_nanoseconds - source_nanoseconds) / 1e9
        self.last_raw_skew_seconds[sensor] = raw_skew_seconds
        if source_nanoseconds <= 0:
            return self._reject(raw_skew_seconds, "ZERO_TIMESTAMP")
        previous_source = self.last_source_nanoseconds.get(sensor)
        if previous_source is not None and source_nanoseconds <= previous_source:
            return self._reject(raw_skew_seconds, "NON_MONOTONIC_TIMESTAMP")
        self.last_source_nanoseconds[sensor] = source_nanoseconds

        candidate_offset = arrival_nanoseconds - source_nanoseconds
        if self.offset_nanoseconds is None:
            self.offset_samples.append(candidate_offset)
            if len(self.offset_samples) < self.minimum_sync_samples:
                return ClockCorrection(
                    False, None, raw_skew_seconds, self.state, "CLOCK_SYNCING"
                )
            recent = list(self.offset_samples)[-self.minimum_sync_samples:]
            if max(recent) - min(recent) > self.sync_spread_nanoseconds:
                self.rejected_packets += 1
                self.last_reason = "CLOCK_OFFSET_UNSTABLE"
                return ClockCorrection(
                    False, None, raw_skew_seconds, self.state, self.last_reason
                )
            self.offset_nanoseconds = min(recent)

        residual = candidate_offset - self.offset_nanoseconds
        if residual < -self.sync_spread_nanoseconds or residual > self.jump_tolerance_nanoseconds:
            return self._reject(raw_skew_seconds, "CLOCK_OFFSET_JUMP")

        self.offset_samples.append(candidate_offset)
        # Track slow clock drift using the lower envelope, without allowing a
        # delayed packet to drag capture time toward callback time.
        self.offset_nanoseconds = min(self.offset_samples)
        corrected = source_nanoseconds + self.offset_nanoseconds
        corrected_age = arrival_nanoseconds - corrected
        if corrected_age > self.maximum_corrected_age_nanoseconds:
            return self._reject(raw_skew_seconds, "CORRECTED_TIMESTAMP_STALE")
        if corrected_age < -self.future_tolerance_nanoseconds:
            return self._reject(raw_skew_seconds, "CORRECTED_TIMESTAMP_FUTURE")
        previous_corrected = self.last_corrected_nanoseconds.get(sensor)
        if previous_corrected is not None and corrected <= previous_corrected:
            return self._reject(raw_skew_seconds, "NON_MONOTONIC_CORRECTED_TIMESTAMP")

        self.invalid_streak = 0
        self.time_invalid = False
        self.last_reason = ""
        self.last_corrected_nanoseconds[sensor] = corrected
        return ClockCorrection(True, corrected, raw_skew_seconds, self.state)


@dataclass(frozen=True, slots=True)
class PoseStability:
    sample_count: int
    duration_seconds: float
    xy_spread: float
    median_deviation: float
    yaw_circular_variance: float
    yaw_spread: float

    def passes(
        self,
        *,
        minimum_samples: int,
        minimum_duration_seconds: float,
        maximum_xy_spread: float,
        maximum_median_deviation: float,
        maximum_yaw_variance: float,
        maximum_yaw_spread: float,
    ) -> bool:
        return (
            self.sample_count >= minimum_samples
            and self.duration_seconds >= minimum_duration_seconds
            and self.xy_spread <= maximum_xy_spread
            and self.median_deviation <= maximum_median_deviation
            and self.yaw_circular_variance <= maximum_yaw_variance
            and self.yaw_spread <= maximum_yaw_spread
        )


def pose_stability(samples: Iterable[tuple[float, float, float, float]]) -> PoseStability:
    values = list(samples)
    if not values:
        return PoseStability(0, 0.0, math.inf, math.inf, 1.0, math.inf)
    x_values = [sample[1] for sample in values]
    y_values = [sample[2] for sample in values]
    yaws = [sample[3] for sample in values]
    median_x = statistics.median(x_values)
    median_y = statistics.median(y_values)
    deviations = [
        math.hypot(x - median_x, y - median_y)
        for x, y in zip(x_values, y_values)
    ]
    cosine = sum(math.cos(yaw) for yaw in yaws) / len(yaws)
    sine = sum(math.sin(yaw) for yaw in yaws) / len(yaws)
    resultant = math.hypot(cosine, sine)
    mean_yaw = math.atan2(sine, cosine)
    yaw_deviations = [
        abs(math.atan2(math.sin(yaw - mean_yaw), math.cos(yaw - mean_yaw)))
        for yaw in yaws
    ]
    return PoseStability(
        sample_count=len(values),
        duration_seconds=max(0.0, values[-1][0] - values[0][0]),
        xy_spread=max(deviations),
        median_deviation=statistics.median(deviations),
        yaw_circular_variance=max(0.0, min(1.0, 1.0 - resultant)),
        yaw_spread=max(yaw_deviations),
    )


@dataclass(frozen=True, slots=True)
class ScanMapMatch:
    score: float
    matched_beams: int
    valid_beams: int
    residual_beams: int
    median_residual: float
    p90_residual: float
    mean_residual: float


@dataclass(frozen=True, slots=True)
class PlanningScanFilter:
    ranges: list[float]
    total_beams: int
    valid_beams: int
    static_map_matches: int
    dynamic_points_kept: int
    raycast_unavailable: int


@dataclass(frozen=True, slots=True)
class HeadingDiversity:
    observed_bins: tuple[int, ...]
    span_radians: float


@dataclass(frozen=True, slots=True)
class CorridorAssessment:
    left_clearance: float
    right_clearance: float
    available_width: float
    required_width: float
    front_clearance: float
    can_go_straight: bool
    can_rotate: bool
    hard_required_width: float = 0.0
    auto_required_width: float = 0.0
    classification: str = "CLEAR"
    reason: str = ""
    localization_uncertainty: float = 0.0
    physically_passable: bool = True


def heading_diversity(
    headings: Iterable[float],
    *,
    bin_count: int = 8,
) -> HeadingDiversity:
    """Summarize independent physical scan headings on a circle."""
    count = max(1, int(bin_count))
    normalized = [float(value) % (2.0 * math.pi) for value in headings]
    bins = tuple(sorted({
        min(count - 1, int(value / (2.0 * math.pi) * count))
        for value in normalized
    }))
    span = 0.0
    for left in normalized:
        for right in normalized:
            span = max(
                span,
                abs(math.atan2(math.sin(left - right), math.cos(left - right))),
            )
    return HeadingDiversity(bins, span)


def evaluate_corridor(
    points: Iterable[tuple[float, float]],
    *,
    half_length: float,
    half_width: float,
    side_margin: float,
    front_clearance_required: float,
    lookahead: float = 0.80,
    rotation_margin: float | None = None,
    hard_side_margin: float = 0.02,
    localization_uncertainty: float = 0.0,
) -> CorridorAssessment:
    """Evaluate straight and in-place-rotation envelopes independently.

    Points are expressed in a frame whose +X axis follows the immediate path.
    Side walls constrain width, while only points inside the forward swept
    rectangle constrain forward braking. This distinction is what lets a
    physically valid corridor remain traversable without weakening collision
    safety for an object in the path.
    """
    length = max(0.0, float(half_length))
    width = max(0.0, float(half_width))
    margin = max(0.0, float(side_margin))
    forward_required = max(0.0, float(front_clearance_required))
    forward_limit = length + max(forward_required, float(lookahead))
    values = [
        (float(x), float(y))
        for x, y in points
        if math.isfinite(float(x)) and math.isfinite(float(y))
        and not (abs(float(x)) < length and abs(float(y)) < width)
    ]

    side_window = [
        (x, y) for x, y in values
        if -length <= x <= forward_limit
    ]
    left_center = min((y for _, y in side_window if y >= width), default=math.inf)
    right_center = min((-y for _, y in side_window if y <= -width), default=math.inf)
    left_clearance = (
        max(0.0, left_center - width) if math.isfinite(left_center) else math.inf
    )
    right_clearance = (
        max(0.0, right_center - width) if math.isfinite(right_center) else math.inf
    )
    available_width = (
        left_center + right_center
        if math.isfinite(left_center) and math.isfinite(right_center)
        else math.inf
    )
    hard_margin = max(0.0, float(hard_side_margin))
    uncertainty = max(0.0, float(localization_uncertainty))
    hard_required_width = 2.0 * width + 2.0 * hard_margin
    auto_required_width = 2.0 * width + 2.0 * margin + uncertainty
    required_width = auto_required_width

    front_clearance = min(
        (
            x - length
            for x, y in values
            if x >= length and abs(y) <= width + margin
        ),
        default=math.inf,
    )
    hard_side_clear = (
        left_clearance >= hard_margin and right_clearance >= hard_margin
    )
    auto_side_requirement = margin + uncertainty / 2.0
    auto_side_clear = (
        left_clearance >= auto_side_requirement
        and right_clearance >= auto_side_requirement
    )
    front_clear = front_clearance > forward_required
    physically_passable = (
        hard_side_clear
        and available_width >= hard_required_width
        and front_clear
    )
    if not physically_passable:
        classification = "PHYSICALLY_BLOCKED"
        reason = (
            "FRONT_CLEARANCE"
            if not front_clear
            else "HARD_WIDTH_OR_SIDE_MARGIN"
        )
    elif (
        not auto_side_clear
        or available_width < auto_required_width
    ):
        classification = "NARROW_OR_UNCERTAIN"
        reason = "AUTO_CLEARANCE_OR_LOCALIZATION_UNCERTAINTY"
    else:
        classification = "CLEAR"
        reason = "AUTO_CLEARANCE_CONFIRMED"
    can_go_straight = classification == "CLEAR"

    rotate_margin = margin if rotation_margin is None else max(
        0.0, float(rotation_margin)
    )
    rotation_radius = math.hypot(length, width) + rotate_margin
    nearest_radius = min((math.hypot(x, y) for x, y in values), default=math.inf)
    return CorridorAssessment(
        left_clearance=left_clearance,
        right_clearance=right_clearance,
        available_width=available_width,
        required_width=required_width,
        front_clearance=front_clearance,
        can_go_straight=can_go_straight,
        can_rotate=nearest_radius > rotation_radius,
        hard_required_width=hard_required_width,
        auto_required_width=auto_required_width,
        classification=classification,
        reason=reason,
        localization_uncertainty=uncertainty,
        physically_passable=physically_passable,
    )


def _resample_path(
    path: Iterable[dict[str, float]],
    *,
    spacing: float = 0.10,
) -> list[tuple[float, float]]:
    points = [(float(item["x"]), float(item["y"])) for item in path]
    if len(points) < 2:
        return points
    output = [points[0]]
    step = max(0.02, float(spacing))
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        distance = math.hypot(bx - ax, by - ay)
        samples = max(1, math.ceil(distance / step))
        output.extend(
            (
                ax + (bx - ax) * index / samples,
                ay + (by - ay) * index / samples,
            )
            for index in range(1, samples + 1)
        )
    return output


def path_overlap_ratio(
    left: Iterable[dict[str, float]],
    right: Iterable[dict[str, float]],
    *,
    distance_tolerance: float = 0.15,
) -> float:
    """Symmetric shared-corridor ratio for meaningful route distinctness."""
    left_points = _resample_path(left)
    right_points = _resample_path(right)
    if not left_points or not right_points:
        return 0.0
    tolerance = max(0.01, float(distance_tolerance))

    def covered(source: list[tuple[float, float]], target: list[tuple[float, float]]) -> float:
        matches = sum(
            1
            for x, y in source
            if min(math.hypot(x - tx, y - ty) for tx, ty in target) <= tolerance
        )
        return matches / len(source)

    return round(min(covered(left_points, right_points), covered(right_points, left_points)), 4)


def environment_flag(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}


class NavigationDebugLog:
    """Small rotating navigation-only log guarded by one environment flag."""

    def __init__(
        self,
        *,
        enabled: bool,
        path: str | Path,
        max_bytes: int = 20 * 1024 * 1024,
        backup_count: int = 5,
    ) -> None:
        self.enabled = bool(enabled)
        self.path = Path(path)
        self.logger: logging.Logger | None = None
        if not self.enabled:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        logger = logging.getLogger(f"rovera.navigation.{id(self)}")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        handler = RotatingFileHandler(
            self.path,
            maxBytes=max(1024, int(max_bytes)),
            backupCount=max(1, int(backup_count)),
            encoding="utf-8",
        )
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(message)s", datefmt="%Y-%m-%dT%H:%M:%S%z"
        ))
        logger.addHandler(handler)
        self.logger = logger

    @staticmethod
    def _value(value: Any) -> str:
        if isinstance(value, float):
            if not math.isfinite(value):
                return "null"
            return f"{value:.4f}".rstrip("0").rstrip(".")
        if isinstance(value, (dict, list, tuple)):
            return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
        if value is None:
            return "null"
        if isinstance(value, bool):
            return str(value).lower()
        return json.dumps(str(value), ensure_ascii=False)

    def event(self, event: str, **fields: Any) -> str:
        if not self.enabled or self.logger is None:
            return ""
        message = " ".join(
            [f"[NAV][{str(event).upper()}]"]
            + [f"{key}={self._value(value)}" for key, value in fields.items()]
        )
        self.logger.info(message)
        return message

    def close(self) -> None:
        if self.logger is None:
            return
        for handler in list(self.logger.handlers):
            handler.close()
            self.logger.removeHandler(handler)
        self.logger = None


def _scan_endpoints(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    lower_range: float,
    upper_range: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    step: int = 1,
) -> Iterable[tuple[int, float, float, float]]:
    for index, raw_distance in enumerate(ranges):
        if index % max(1, int(step)):
            continue
        distance = float(raw_distance)
        if (
            not math.isfinite(distance)
            or distance < lower_range
            or distance > upper_range
        ):
            continue
        angle = laser_yaw + angle_min + index * angle_increment
        yield (
            index,
            distance,
            laser_x + distance * math.cos(angle),
            laser_y + distance * math.sin(angle),
        )


def filter_static_map_scan(
    saved_map: "SavedOccupancyMap",
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    expected_range_tolerance: float,
    minimum_usable_range: float = 0.20,
    maximum_usable_range: float = 6.0,
) -> PlanningScanFilter:
    """Mask only beams whose measured range agrees with a saved-map raycast.

    Infinite ranges remain clearing rays when the global ObstacleLayer enables
    ``inf_is_valid``. AMCL, the local costmap and motion safety retain the
    original scan and therefore never lose a physical obstacle return.
    """
    output = [float(value) for value in ranges]
    lower = max(float(range_min), float(minimum_usable_range))
    upper = min(float(range_max), float(maximum_usable_range))
    valid = 0
    static_matches = 0
    raycast_unavailable = 0
    for index, distance, _, _ in _scan_endpoints(
        output,
        angle_min=float(angle_min),
        angle_increment=float(angle_increment),
        lower_range=lower,
        upper_range=upper,
        laser_x=float(laser_x),
        laser_y=float(laser_y),
        laser_yaw=float(laser_yaw),
    ):
        valid += 1
        angle = float(laser_yaw) + float(angle_min) + index * float(angle_increment)
        expected = saved_map.raycast_static_range(
            float(laser_x),
            float(laser_y),
            angle,
            minimum_range=lower,
            maximum_range=upper,
        )
        if expected is None:
            # Unknown/out-of-map rays are not evidence that a physical return
            # is static. Keeping it is the fail-safe outcome.
            raycast_unavailable += 1
            continue
        if abs(distance - expected) <= float(expected_range_tolerance):
            output[index] = math.inf
            static_matches += 1
    return PlanningScanFilter(
        ranges=output,
        total_beams=len(output),
        valid_beams=valid,
        static_map_matches=static_matches,
        dynamic_points_kept=max(0, valid - static_matches),
        raycast_unavailable=raycast_unavailable,
    )


def classify_planning_failure(
    *,
    tf_ready: bool,
    costmap_ready: bool,
    start_cost: int | None,
    goal_cost: int | None,
    route_crosses_unknown: bool,
) -> str:
    """Best available diagnosis for Humble's error-code-free planner result."""
    if not tf_ready:
        return "TF_ERROR"
    if not costmap_ready or start_cost is None or goal_cost is None:
        return "COSTMAP_NOT_READY"
    if start_cost < 0 or goal_cost < 0 or route_crosses_unknown:
        return "UNKNOWN_SPACE"
    if start_cost >= 99:
        return "START_BLOCKED"
    if goal_cost >= 99:
        return "GOAL_BLOCKED"
    return "NO_VALID_PATH"


def scan_to_map_match(
    saved_map: "SavedOccupancyMap",
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    maximum_beams: int = 90,
    minimum_usable_range: float = 0.20,
    maximum_usable_range: float = 6.0,
    endpoint_tolerance: float = 0.12,
) -> ScanMapMatch:
    """Compare scan endpoints and retain their actual map residuals."""
    measurements = list(ranges)
    if not measurements:
        return ScanMapMatch(0.0, 0, 0, 0, math.inf, math.inf, math.inf)
    step = max(1, math.ceil(len(measurements) / max(1, int(maximum_beams))))
    lower = max(float(range_min), float(minimum_usable_range))
    upper = min(float(range_max), float(maximum_usable_range))
    matched = 0
    valid = 0
    residuals: list[float] = []
    for _, _, endpoint_x, endpoint_y in _scan_endpoints(
        measurements,
        angle_min=angle_min,
        angle_increment=angle_increment,
        lower_range=lower,
        upper_range=upper,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=laser_yaw,
        step=step,
    ):
        valid += 1
        residual = saved_map.nearest_occupied_distance(
            endpoint_x,
            endpoint_y,
            maximum_distance=float(endpoint_tolerance),
        )
        if residual is not None:
            matched += 1
            residuals.append(residual)
    ordered = sorted(residuals)
    p90_index = max(0, math.ceil(0.9 * len(ordered)) - 1)
    return ScanMapMatch(
        score=0.0 if valid == 0 else round(matched / valid, 4),
        matched_beams=matched,
        valid_beams=valid,
        residual_beams=len(ordered),
        median_residual=(statistics.median(ordered) if ordered else math.inf),
        p90_residual=(ordered[p90_index] if ordered else math.inf),
        mean_residual=(statistics.fmean(ordered) if ordered else math.inf),
    )


def rotation_swept_clearance(
    point_x: float,
    point_y: float,
    *,
    half_length: float,
    half_width: float,
) -> float:
    """Distance from a scan point to the chassis' complete rotation sweep."""
    swept_radius = math.hypot(float(half_length), float(half_width))
    return math.hypot(float(point_x), float(point_y)) - swept_radius


def mask_scan_self_returns(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    half_length: float,
    half_width: float,
) -> tuple[list[float], int]:
    """Mask only LiDAR endpoints physically inside the calibrated chassis."""
    output = [float(value) for value in ranges]
    masked = 0
    for index, distance in enumerate(output):
        if (
            not math.isfinite(distance)
            or distance < float(range_min)
            or distance > float(range_max)
        ):
            continue
        angle = float(laser_yaw) + float(angle_min) + index * float(angle_increment)
        point_x = float(laser_x) + distance * math.cos(angle)
        point_y = float(laser_y) + distance * math.sin(angle)
        if abs(point_x) < float(half_length) and abs(point_y) < float(half_width):
            output[index] = math.nan
            masked += 1
    return output, masked


@dataclass(slots=True)
class SavedOccupancyMap:
    """Exact saved-map grid used for goal validation (never downsampled)."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    occupancy: list[int]

    @classmethod
    def load(cls, map_yaml: str | Path) -> "SavedOccupancyMap":
        yaml_path = Path(map_yaml)
        metadata = yaml.safe_load(yaml_path.read_text())
        if not isinstance(metadata, dict):
            raise ValueError("map.yaml must contain an object")
        image_path = Path(str(metadata["image"]))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image = Image.open(image_path).convert("L")
        resolution = float(metadata["resolution"])
        origin = list(metadata["origin"])
        if resolution <= 0 or len(origin) < 3:
            raise ValueError("map.yaml has invalid geometry")
        occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
        free_threshold = float(metadata.get("free_thresh", 0.196))
        negate = int(metadata.get("negate", 0))
        mode = str(metadata.get("mode", "trinary")).lower()
        if mode not in {"trinary", "scale", "raw"}:
            raise ValueError(f"unsupported map mode: {mode}")

        # PIL rows start at the top while OccupancyGrid row zero is the lower
        # image edge. Store the same row-major orientation ROS publishes.
        pixels = list(image.getdata())
        occupancy: list[int] = []
        for ros_row in range(image.height):
            image_row = image.height - 1 - ros_row
            for column in range(image.width):
                value = int(pixels[image_row * image.width + column])
                if mode == "raw":
                    occupancy.append(-1 if value == 255 else min(100, value))
                    continue
                probability = value / 255.0 if negate else (255 - value) / 255.0
                if probability >= occupied_threshold:
                    occupancy.append(100)
                elif probability <= free_threshold:
                    occupancy.append(0)
                elif mode == "scale":
                    occupancy.append(round(
                        (probability - free_threshold)
                        / (occupied_threshold - free_threshold)
                        * 100
                    ))
                else:
                    occupancy.append(-1)
        return cls(
            width=image.width,
            height=image.height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            occupancy=occupancy,
        )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        return column, row

    def value_at(self, column: int, row: int) -> int:
        return self.occupancy[row * self.width + column]

    def cell_center(self, column: int, row: int) -> tuple[float, float]:
        """Return the world-coordinate center of an OccupancyGrid cell."""
        local_x = (column + 0.5) * self.resolution
        local_y = (row + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def occupied_within(self, x: float, y: float, distance_m: float) -> bool:
        """Return whether a saved occupied cell already explains a live hit."""
        return self.nearest_occupied_distance(
            x, y, maximum_distance=distance_m
        ) is not None

    def nearest_occupied_distance(
        self,
        x: float,
        y: float,
        *,
        maximum_distance: float,
    ) -> float | None:
        """Return metric endpoint residual to the nearest occupied cell."""
        cell = self.world_to_cell(x, y)
        if cell is None or maximum_distance < 0:
            return None
        column, row = cell
        radius = math.ceil(maximum_distance / self.resolution)
        nearest = math.inf
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                check_column = column + offset_x
                check_row = row + offset_y
                if not (
                    0 <= check_column < self.width
                    and 0 <= check_row < self.height
                ):
                    continue
                center_x, center_y = self.cell_center(check_column, check_row)
                distance = math.hypot(center_x - x, center_y - y)
                if (
                    distance <= maximum_distance
                    and self.value_at(check_column, check_row) >= 65
                ):
                    nearest = min(nearest, distance)
        return nearest if math.isfinite(nearest) else None

    def raycast_static_range(
        self,
        x: float,
        y: float,
        angle: float,
        *,
        minimum_range: float,
        maximum_range: float,
    ) -> float | None:
        """Return expected range to the first known occupied map cell.

        Reaching unknown space or leaving the map before an occupied cell is
        deliberately inconclusive so callers retain the live obstacle point.
        """
        lower = max(0.0, float(minimum_range))
        upper = max(lower, float(maximum_range))
        # Half-cell sampling cannot skip a grid cell along either axis and
        # keeps the per-scan raycast bounded on the Pi.
        step = max(0.005, self.resolution * 0.5)
        cosine, sine = math.cos(angle), math.sin(angle)
        distance = lower
        visited: set[tuple[int, int]] = set()
        while distance <= upper:
            cell = self.world_to_cell(
                float(x) + distance * cosine,
                float(y) + distance * sine,
            )
            if cell is None:
                return None
            if cell not in visited:
                visited.add(cell)
                value = self.value_at(*cell)
                if value < 0:
                    return None
                if value >= 65:
                    center_x, center_y = self.cell_center(*cell)
                    projected = (
                        (center_x - float(x)) * cosine
                        + (center_y - float(y)) * sine
                    )
                    return max(lower, projected)
            distance += step
        return None

    def segment_crosses_unknown(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> bool:
        """Sample a direct route for failure diagnostics, not path creation."""
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        samples = max(1, math.ceil(distance / max(0.001, self.resolution * 0.5)))
        for index in range(samples + 1):
            ratio = index / samples
            cell = self.world_to_cell(
                start_x + (goal_x - start_x) * ratio,
                start_y + (goal_y - start_y) * ratio,
            )
            if cell is None or self.value_at(*cell) < 0:
                return True
        return False

    def validate_goal(
        self,
        x: float,
        y: float,
        *,
        clearance_m: float,
        allow_unknown: bool = False,
        lethal_world_cells: Iterable[tuple[float, float]] = (),
    ) -> GoalValidation:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return GoalValidation(False, "GOAL_OUTSIDE_MAP", "Điểm đến nằm ngoài bản đồ.")
        column, row = cell
        value = self.value_at(column, row)
        if value < 0 and not allow_unknown:
            return GoalValidation(False, "GOAL_UNKNOWN", "Không thể đi đến vùng chưa được lập bản đồ.")
        if value >= 65:
            return GoalValidation(False, "GOAL_OCCUPIED", "Điểm này nằm trong vật cản.")

        radius = max(0, math.ceil(clearance_m / self.resolution))
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if math.hypot(offset_x, offset_y) * self.resolution > clearance_m:
                    continue
                check_x = column + offset_x
                check_y = row + offset_y
                if not (0 <= check_x < self.width and 0 <= check_y < self.height):
                    return GoalValidation(False, "GOAL_CLEARANCE", "Robot không đủ khoảng trống tại điểm đến.")
                nearby = self.value_at(check_x, check_y)
                if nearby >= 65 or (nearby < 0 and not allow_unknown):
                    return GoalValidation(False, "GOAL_CLEARANCE", "Robot không đủ khoảng trống tại điểm đến.")
        for obstacle_x, obstacle_y in lethal_world_cells:
            if math.hypot(obstacle_x - x, obstacle_y - y) <= clearance_m:
                return GoalValidation(False, "GOAL_LETHAL", "Điểm đến đang có vật cản động nguy hiểm.")
        return GoalValidation(True)

    def validate_footprint(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        half_length: float,
        half_width: float,
        padding: float = 0.0,
        allow_unknown: bool = False,
        lethal_world_cells: Iterable[tuple[float, float]] = (),
        code_prefix: str = "GOAL",
    ) -> GoalValidation:
        """Validate the same oriented rectangular body envelope Nav2 uses.

        Goal click validation historically checked only a small circle around
        the center point, so Nav2 could reject poses the adapter had already
        called safe. Test occupied map cells as squares against the oriented
        robot rectangle so preflight retains the complete configured footprint.
        """
        prefix = str(code_prefix).strip().upper() or "GOAL"
        length = max(0.0, float(half_length) + float(padding))
        width = max(0.0, float(half_width) + float(padding))
        if length <= 0 or width <= 0:
            raise ValueError("footprint half dimensions must be positive")

        robot_x_axis = (math.cos(yaw), math.sin(yaw))
        robot_y_axis = (-math.sin(yaw), math.cos(yaw))
        map_x_axis = (math.cos(self.origin_yaw), math.sin(self.origin_yaw))
        map_y_axis = (-math.sin(self.origin_yaw), math.cos(self.origin_yaw))

        corners = [
            (
                x + sx * length * robot_x_axis[0] + sy * width * robot_y_axis[0],
                y + sx * length * robot_x_axis[1] + sy * width * robot_y_axis[1],
            )
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        center_cell = self.world_to_cell(x, y)
        if center_cell is None or any(self.world_to_cell(*corner) is None for corner in corners):
            subject = "Vị trí hiện tại" if prefix == "START" else "Điểm đến"
            return GoalValidation(
                False,
                f"{prefix}_FOOTPRINT_OUTSIDE_MAP",
                f"{subject} không đủ chỗ cho toàn bộ thân robot trong bản đồ.",
            )

        half_cell = self.resolution / 2.0

        def rectangles_intersect(cell_x: float, cell_y: float) -> bool:
            delta = (cell_x - x, cell_y - y)
            for axis in (robot_x_axis, robot_y_axis, map_x_axis, map_y_axis):
                separation = abs(delta[0] * axis[0] + delta[1] * axis[1])
                robot_projection = (
                    length * abs(robot_x_axis[0] * axis[0] + robot_x_axis[1] * axis[1])
                    + width * abs(robot_y_axis[0] * axis[0] + robot_y_axis[1] * axis[1])
                )
                cell_projection = half_cell * (
                    abs(map_x_axis[0] * axis[0] + map_x_axis[1] * axis[1])
                    + abs(map_y_axis[0] * axis[0] + map_y_axis[1] * axis[1])
                )
                if separation > robot_projection + cell_projection:
                    return False
            return True

        center_column, center_row = center_cell
        search_radius = math.ceil(
            (math.hypot(length, width) + math.sqrt(2.0) * half_cell)
            / self.resolution
        ) + 1
        for row in range(center_row - search_radius, center_row + search_radius + 1):
            for column in range(
                center_column - search_radius,
                center_column + search_radius + 1,
            ):
                if not (0 <= column < self.width and 0 <= row < self.height):
                    continue
                cell_x, cell_y = self.cell_center(column, row)
                if not rectangles_intersect(cell_x, cell_y):
                    continue
                value = self.value_at(column, row)
                if value >= 65 or (value < 0 and not allow_unknown):
                    subject = "Vị trí hiện tại" if prefix == "START" else "Điểm đến"
                    obstacle = "vùng chưa lập bản đồ" if value < 0 else "vật cản"
                    return GoalValidation(
                        False,
                        f"{prefix}_FOOTPRINT_BLOCKED",
                        f"{subject} không đủ khoảng trống cho toàn bộ thân robot ({obstacle} nằm trong footprint).",
                    )

        for obstacle_x, obstacle_y in lethal_world_cells:
            delta_x = float(obstacle_x) - x
            delta_y = float(obstacle_y) - y
            local_x = delta_x * robot_x_axis[0] + delta_y * robot_x_axis[1]
            local_y = delta_x * robot_y_axis[0] + delta_y * robot_y_axis[1]
            if abs(local_x) <= length and abs(local_y) <= width:
                subject = "Vị trí hiện tại" if prefix == "START" else "Điểm đến"
                return GoalValidation(
                    False,
                    f"{prefix}_FOOTPRINT_BLOCKED",
                    f"{subject} đang có vật cản động nằm trong footprint của robot.",
                )
        return GoalValidation(True)

    def nearest_valid_goal(
        self,
        x: float,
        y: float,
        *,
        clearance_m: float,
        max_distance_m: float,
        allow_unknown: bool = False,
        lethal_world_cells: Iterable[tuple[float, float]] = (),
        yaw: float = 0.0,
        footprint_half_length: float | None = None,
        footprint_half_width: float | None = None,
        footprint_padding: float = 0.0,
    ) -> tuple[float, float] | None:
        """Find the closest safe cell center to a requested in-map goal.

        Operator clicks are inherently imprecise, especially on the compact
        control map. Snapping is deliberately bounded so an unsafe click can
        never turn into a materially different destination.
        """
        requested_cell = self.world_to_cell(x, y)
        if requested_cell is None or max_distance_m < 0:
            return None
        dynamic_obstacles = tuple(lethal_world_cells)
        radius = max(0, math.ceil(max_distance_m / self.resolution))
        requested_column, requested_row = requested_cell
        candidates: list[tuple[float, int, int, float, float]] = []
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                column = requested_column + offset_x
                row = requested_row + offset_y
                if not (0 <= column < self.width and 0 <= row < self.height):
                    continue
                candidate_x, candidate_y = self.cell_center(column, row)
                distance = math.hypot(candidate_x - x, candidate_y - y)
                if distance > max_distance_m:
                    continue
                validation = self.validate_goal(
                    candidate_x,
                    candidate_y,
                    clearance_m=clearance_m,
                    allow_unknown=allow_unknown,
                    lethal_world_cells=dynamic_obstacles,
                )
                if not validation.valid:
                    continue
                if footprint_half_length is not None and footprint_half_width is not None:
                    validation = self.validate_footprint(
                        candidate_x,
                        candidate_y,
                        yaw,
                        half_length=footprint_half_length,
                        half_width=footprint_half_width,
                        padding=footprint_padding,
                        allow_unknown=allow_unknown,
                        lethal_world_cells=dynamic_obstacles,
                    )
                if validation.valid:
                    candidates.append((distance, row, column, candidate_x, candidate_y))
        if not candidates:
            return None
        _, _, _, candidate_x, candidate_y = min(candidates)
        return candidate_x, candidate_y


def localization_confidence(
    covariance: list[float] | tuple[float, ...],
    *,
    stability_score: float,
    scan_map_score: float,
    scan_map_threshold: float,
    scan_fresh: bool,
    tf_stable: bool,
    odometry_healthy: bool,
    sensor_time_valid: bool,
) -> float:
    """Normalize AMCL uncertainty and independent health gates to [0, 1]."""
    if (
        len(covariance) < 36
        or not scan_fresh
        or not tf_stable
        or not odometry_healthy
        or not sensor_time_valid
        or scan_map_threshold <= 0
    ):
        return 0.0
    variance = max(0.0, float(covariance[0])) + max(0.0, float(covariance[7]))
    variance += max(0.0, float(covariance[35])) * 0.5
    covariance_score = math.exp(-variance * 2.0)
    stability = max(0.0, min(1.0, float(stability_score)))
    map_score = max(0.0, min(1.0, float(scan_map_score) / scan_map_threshold))
    return round(
        max(0.0, min(1.0, covariance_score * stability * map_score)), 4
    )


def compact_lethal_cells(
    message: Any,
    *,
    threshold: int = 100,
    max_cells: int = 600,
) -> list[dict[str, float]]:
    """Return only lethal cells, not the broad inflation-cost gradient."""
    output: list[dict[str, float]] = []
    width = int(message.info.width)
    origin = message.info.origin.position
    orientation = getattr(message.info.origin, "orientation", None)
    yaw = 0.0 if orientation is None else math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    resolution = float(message.info.resolution)
    for index, raw in enumerate(message.data):
        if int(raw) < threshold:
            continue
        row, column = divmod(index, width)
        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        output.append({
            "x": round(float(origin.x) + cosine * local_x - sine * local_y, 3),
            "y": round(float(origin.y) + sine * local_x + cosine * local_y, 3),
        })
        if len(output) >= max_cells:
            break
    return output
