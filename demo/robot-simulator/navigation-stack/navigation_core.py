from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
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
    """Compare a bounded sample of scan endpoints with occupied map cells."""
    measurements = list(ranges)
    if not measurements:
        return ScanMapMatch(0.0, 0, 0)
    step = max(1, math.ceil(len(measurements) / max(1, int(maximum_beams))))
    lower = max(float(range_min), float(minimum_usable_range))
    upper = min(float(range_max), float(maximum_usable_range))
    matched = 0
    valid = 0
    for index in range(0, len(measurements), step):
        distance = float(measurements[index])
        if not math.isfinite(distance) or distance < lower or distance > upper:
            continue
        angle = laser_yaw + angle_min + index * angle_increment
        endpoint_x = laser_x + distance * math.cos(angle)
        endpoint_y = laser_y + distance * math.sin(angle)
        valid += 1
        if saved_map.occupied_within(endpoint_x, endpoint_y, endpoint_tolerance):
            matched += 1
    return ScanMapMatch(
        score=0.0 if valid == 0 else round(matched / valid, 4),
        matched_beams=matched,
        valid_beams=valid,
    )


def navigation_abort_state(recoveries: int) -> str:
    """Classify an exhausted Nav2 action without hiding technical failures."""
    return "BLOCKED" if max(0, int(recoveries)) > 0 else "FAILED"


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
        cell = self.world_to_cell(x, y)
        if cell is None or distance_m < 0:
            return False
        column, row = cell
        radius = math.ceil(distance_m / self.resolution)
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
                if (
                    math.hypot(center_x - x, center_y - y) <= distance_m
                    and self.value_at(check_column, check_row) >= 65
                ):
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
