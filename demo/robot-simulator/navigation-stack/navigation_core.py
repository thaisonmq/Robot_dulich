from __future__ import annotations

import json
import heapq
import logging
import math
import os
import statistics
from collections import deque
from dataclasses import dataclass, field
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


@dataclass(frozen=True, slots=True)
class RotationSweepValidation:
    valid: bool
    code: str = ""
    samples_checked: int = 0
    collision_yaw: float | None = None


@dataclass(frozen=True, slots=True)
class RouteMetadata:
    total_length: float
    minimum_passage_width: float
    minimum_static_clearance: float
    minimum_turn_clearance: float
    turn_count: int
    total_turn_angle: float
    narrow_segments: tuple[dict[str, float], ...]
    estimated_time: float
    turn_safe: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "total_length": round(self.total_length, 4),
            "minimum_passage_width": _finite_or_none(
                self.minimum_passage_width
            ),
            "minimum_static_clearance": _finite_or_none(
                self.minimum_static_clearance
            ),
            "minimum_turn_clearance": _finite_or_none(
                self.minimum_turn_clearance
            ),
            "turn_count": self.turn_count,
            "total_turn_angle": round(self.total_turn_angle, 4),
            "narrow_segments": [dict(item) for item in self.narrow_segments],
            "estimated_time": round(self.estimated_time, 3),
            "turn_safe": self.turn_safe,
        }


@dataclass(frozen=True, slots=True)
class StopTurnRoute:
    points: tuple[dict[str, float], ...]
    metadata: RouteMetadata
    heading_bins: tuple[int, ...]


def _finite_or_none(value: float) -> float | None:
    return round(float(value), 4) if math.isfinite(float(value)) else None


def _angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def canonicalize_stop_turn_path(
    points: Iterable[dict[str, float]],
    *,
    angular_tolerance: float = math.radians(1.0),
) -> list[dict[str, float]]:
    """Remove duplicate and collinear points without rounding corner geometry."""
    route: list[dict[str, float]] = []
    for point in points:
        candidate = {"x": float(point["x"]), "y": float(point["y"])}
        if route and math.hypot(
            candidate["x"] - route[-1]["x"],
            candidate["y"] - route[-1]["y"],
        ) <= 1e-8:
            continue
        route.append(candidate)
    if len(route) < 3:
        return route
    output = [route[0]]
    for index in range(1, len(route) - 1):
        previous = output[-1]
        current = route[index]
        following = route[index + 1]
        incoming = math.atan2(
            current["y"] - previous["y"], current["x"] - previous["x"]
        )
        outgoing = math.atan2(
            following["y"] - current["y"], following["x"] - current["x"]
        )
        if abs(_angle_delta(outgoing, incoming)) <= angular_tolerance:
            continue
        output.append(current)
    output.append(route[-1])
    return output


def densify_straight_segment(
    start: dict[str, float],
    end: dict[str, float],
    *,
    spacing: float,
) -> list[dict[str, float]]:
    """Sample one straight segment densely without changing its geometry.

    RPP prunes a path to its nearest pose.  A path containing only its two
    endpoints can therefore switch to the far endpoint while the start pose
    is already behind the robot, making the local path point backwards.  A
    dense collinear path keeps a nearby forward sample available throughout.
    """
    start_x, start_y = float(start["x"]), float(start["y"])
    end_x, end_y = float(end["x"]), float(end["y"])
    distance = math.hypot(end_x - start_x, end_y - start_y)
    if distance <= 1e-9:
        return [{"x": start_x, "y": start_y}]
    sample_count = max(1, math.ceil(distance / max(0.01, float(spacing))))
    return [
        {
            "x": start_x + (end_x - start_x) * index / sample_count,
            "y": start_y + (end_y - start_y) * index / sample_count,
        }
        for index in range(sample_count + 1)
    ]


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
    costs = data if isinstance(data, list) else [int(value) for value in data]
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


def _squared_edt_1d(values: list[float]) -> list[float]:
    """Exact squared Euclidean distance transform for one grid axis."""
    finite_sites = [index for index, value in enumerate(values) if math.isfinite(value)]
    if not finite_sites:
        return [math.inf] * len(values)
    vertices = [finite_sites[0]]
    boundaries = [-math.inf, math.inf]
    for site in finite_sites[1:]:
        while True:
            previous = vertices[-1]
            crossing = (
                (values[site] + site * site)
                - (values[previous] + previous * previous)
            ) / (2.0 * (site - previous))
            if crossing > boundaries[-2]:
                break
            vertices.pop()
            boundaries.pop(-2)
        vertices.append(site)
        boundaries.insert(-1, crossing)
    output = [0.0] * len(values)
    vertex_index = 0
    for coordinate in range(len(values)):
        while boundaries[vertex_index + 1] < coordinate:
            vertex_index += 1
        site = vertices[vertex_index]
        output[coordinate] = (
            (coordinate - site) * (coordinate - site) + values[site]
        )
    return output


def exact_euclidean_distance_transform(
    blocked: Iterable[bool],
    *,
    width: int,
    height: int,
) -> list[float]:
    """Return exact cell-center distances to blocked cells in O(width*height)."""
    mask = [bool(value) for value in blocked]
    if width <= 0 or height <= 0 or len(mask) != width * height:
        raise ValueError("blocked mask dimensions do not match")
    row_pass: list[float] = [math.inf] * len(mask)
    for row in range(height):
        start = row * width
        transformed = _squared_edt_1d([
            0.0 if mask[start + column] else math.inf
            for column in range(width)
        ])
        row_pass[start:start + width] = transformed
    squared: list[float] = [math.inf] * len(mask)
    for column in range(width):
        transformed = _squared_edt_1d([
            row_pass[row * width + column] for row in range(height)
        ])
        for row, value in enumerate(transformed):
            squared[row * width + column] = value
    return [math.sqrt(value) if math.isfinite(value) else math.inf for value in squared]


@dataclass(slots=True)
class MapNavigationGeometry:
    """Static, map-version-scoped geometry shared by planning and ranking."""

    width: int
    height: int
    resolution: float
    blocked: tuple[bool, ...]
    distance_cells: tuple[float, ...]
    static_clearance: tuple[float, ...]
    robot_navigable_mask: tuple[bool, ...]
    component_ids: tuple[int, ...]
    turn_safe_mask: tuple[bool, ...]
    identity: str
    _robot_shape: tuple[float, float, float] = field(default=(0.15, 0.10, 0.0))

    @classmethod
    def build(
        cls,
        saved_map: "SavedOccupancyMap",
        *,
        half_length: float = 0.15,
        half_width: float = 0.10,
        padding: float = 0.0,
    ) -> "MapNavigationGeometry":
        blocked = tuple(value < 0 or value >= 65 for value in saved_map.occupancy)
        distances = exact_euclidean_distance_transform(
            blocked, width=saved_map.width, height=saved_map.height
        )
        half_cell_diagonal = saved_map.resolution / math.sqrt(2.0)
        clearance = tuple(
            math.inf
            if not math.isfinite(distance)
            else max(0.0, distance * saved_map.resolution - half_cell_diagonal)
            for distance in distances
        )
        minimum_radius = float(half_width) + float(padding)
        turn_radius = math.hypot(
            float(half_length) + float(padding),
            float(half_width) + float(padding),
        )
        navigable = tuple(
            not is_blocked and cell_clearance + 1e-9 >= minimum_radius
            for is_blocked, cell_clearance in zip(blocked, clearance)
        )
        turn_safe = tuple(
            not is_blocked and cell_clearance + 1e-9 >= turn_radius
            for is_blocked, cell_clearance in zip(blocked, clearance)
        )
        components = cls._components(
            navigable, width=saved_map.width, height=saved_map.height
        )
        identity_payload = (
            f"{saved_map.width}:{saved_map.height}:{saved_map.resolution}:"
            + ",".join(str(value) for value in saved_map.occupancy)
        )
        import hashlib
        return cls(
            width=saved_map.width,
            height=saved_map.height,
            resolution=saved_map.resolution,
            blocked=blocked,
            distance_cells=tuple(distances),
            static_clearance=clearance,
            robot_navigable_mask=navigable,
            component_ids=components,
            turn_safe_mask=turn_safe,
            identity=hashlib.sha256(identity_payload.encode()).hexdigest(),
            _robot_shape=(float(half_length), float(half_width), float(padding)),
        )

    @staticmethod
    def _components(
        mask: tuple[bool, ...], *, width: int, height: int
    ) -> tuple[int, ...]:
        components = [-1] * len(mask)
        component = 0
        for start, free in enumerate(mask):
            if not free or components[start] >= 0:
                continue
            components[start] = component
            pending = deque([start])
            while pending:
                index = pending.popleft()
                row, column = divmod(index, width)
                for delta_x, delta_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    next_column = column + delta_x
                    next_row = row + delta_y
                    if not (
                        0 <= next_column < width and 0 <= next_row < height
                    ):
                        continue
                    neighbor = next_row * width + next_column
                    if mask[neighbor] and components[neighbor] < 0:
                        components[neighbor] = component
                        pending.append(neighbor)
            component += 1
        return tuple(components)

    def index(self, column: int, row: int) -> int:
        return row * self.width + column

    def clearance_at_cell(self, column: int, row: int) -> float:
        if not (0 <= column < self.width and 0 <= row < self.height):
            return 0.0
        return self.static_clearance[self.index(column, row)]

    def same_component(
        self, left: tuple[int, int], right: tuple[int, int]
    ) -> bool:
        left_id = self.component_ids[self.index(*left)]
        right_id = self.component_ids[self.index(*right)]
        return left_id >= 0 and left_id == right_id


def validate_rotation_sweep(
    saved_map: "SavedOccupancyMap",
    x: float,
    y: float,
    start_yaw: float,
    end_yaw: float,
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
    allow_unknown: bool = False,
) -> RotationSweepValidation:
    """Exact sampled rectangular footprint check for an in-place rotation.

    The angular interval is bounded so the furthest chassis corner travels no
    more than half a map cell between samples.
    """
    delta = _angle_delta(float(end_yaw), float(start_yaw))
    radius = math.hypot(
        float(half_length) + float(padding),
        float(half_width) + float(padding),
    )
    maximum_step = (
        math.pi
        if radius <= 1e-9
        else min(math.pi, saved_map.resolution * 0.5 / radius)
    )
    sample_count = max(1, math.ceil(abs(delta) / max(1e-6, maximum_step)))
    for index in range(sample_count + 1):
        yaw = float(start_yaw) + delta * index / sample_count
        validation = saved_map.validate_footprint(
            float(x),
            float(y),
            yaw,
            half_length=float(half_length),
            half_width=float(half_width),
            padding=float(padding),
            allow_unknown=allow_unknown,
            code_prefix="TURN",
        )
        if not validation.valid:
            return RotationSweepValidation(
                False, "TURN_SWEEP_COLLISION", index + 1, yaw
            )
    return RotationSweepValidation(True, samples_checked=sample_count + 1)


def validate_stop_turn_route(
    saved_map: "SavedOccupancyMap",
    points: Iterable[dict[str, float]],
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
) -> ExecutablePathValidation:
    route = canonicalize_stop_turn_path(points)
    if len(route) < 2:
        return ExecutablePathValidation(False, "PATH_HAS_NO_LENGTH")
    translation = validate_executable_grid_path(
        route,
        width=saved_map.width,
        height=saved_map.height,
        resolution=saved_map.resolution,
        origin_x=saved_map.origin_x,
        origin_y=saved_map.origin_y,
        origin_yaw=saved_map.origin_yaw,
        data=saved_map.occupancy,
        half_length=float(half_length) + float(padding),
        half_width=float(half_width) + float(padding),
        allow_unknown=False,
        lethal_threshold=65,
    )
    if not translation.valid:
        return translation
    samples = translation.samples_checked
    for corner_index in range(1, len(route) - 1):
        incoming = math.atan2(
            route[corner_index]["y"] - route[corner_index - 1]["y"],
            route[corner_index]["x"] - route[corner_index - 1]["x"],
        )
        outgoing = math.atan2(
            route[corner_index + 1]["y"] - route[corner_index]["y"],
            route[corner_index + 1]["x"] - route[corner_index]["x"],
        )
        rotation = validate_rotation_sweep(
            saved_map,
            route[corner_index]["x"],
            route[corner_index]["y"],
            incoming,
            outgoing,
            half_length=half_length,
            half_width=half_width,
            padding=padding,
        )
        samples += rotation.samples_checked
        if not rotation.valid:
            return ExecutablePathValidation(
                False,
                rotation.code,
                corner_index,
                route[corner_index]["x"],
                route[corner_index]["y"],
                rotation.collision_yaw,
                samples_checked=samples,
            )
    return ExecutablePathValidation(True, samples_checked=samples)


def route_geometry_metadata(
    saved_map: "SavedOccupancyMap",
    geometry: MapNavigationGeometry,
    points: Iterable[dict[str, float]],
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
    linear_speed: float = 0.20,
    angular_speed: float = 0.60,
) -> RouteMetadata:
    route = canonicalize_stop_turn_path(points)
    total_length = 0.0
    minimum_clearance = math.inf
    minimum_passage_width = math.inf
    minimum_turn_clearance = math.inf
    narrow_segments: list[dict[str, float]] = []
    turn_count = 0
    total_turn_angle = 0.0
    turn_safe = True
    rotation_diameter = 2.0 * math.hypot(
        half_length + padding, half_width + padding
    )
    for segment_index, (left, right) in enumerate(zip(route, route[1:])):
        distance = math.hypot(right["x"] - left["x"], right["y"] - left["y"])
        segment_yaw = math.atan2(
            right["y"] - left["y"], right["x"] - left["x"]
        )
        total_length += distance
        count = max(1, math.ceil(distance / max(0.001, saved_map.resolution / 2.0)))
        segment_width = math.inf
        for sample in range(count + 1):
            ratio = sample / count
            x = left["x"] + (right["x"] - left["x"]) * ratio
            y = left["y"] + (right["y"] - left["y"]) * ratio
            cell = saved_map.world_to_cell(x, y)
            clearance = 0.0 if cell is None else geometry.clearance_at_cell(*cell)
            minimum_clearance = min(minimum_clearance, clearance)
            ray_limit = saved_map.resolution * math.hypot(
                saved_map.width, saved_map.height
            ) + saved_map.resolution
            left_width = saved_map.raycast_static_range(
                x,
                y,
                segment_yaw + math.pi / 2.0,
                minimum_range=0.0,
                maximum_range=ray_limit,
                unknown_is_blocked=True,
            )
            right_width = saved_map.raycast_static_range(
                x,
                y,
                segment_yaw - math.pi / 2.0,
                minimum_range=0.0,
                maximum_range=ray_limit,
                unknown_is_blocked=True,
            )
            passage_width = (
                0.0
                if left_width is None or right_width is None
                else left_width + right_width
            )
            segment_width = min(segment_width, passage_width)
        minimum_passage_width = min(minimum_passage_width, segment_width)
        if segment_width < rotation_diameter:
            narrow_segments.append({
                "segment_index": segment_index,
                "passage_width": round(segment_width, 4),
                "length": round(distance, 4),
            })
    for index in range(1, len(route) - 1):
        incoming = math.atan2(
            route[index]["y"] - route[index - 1]["y"],
            route[index]["x"] - route[index - 1]["x"],
        )
        outgoing = math.atan2(
            route[index + 1]["y"] - route[index]["y"],
            route[index + 1]["x"] - route[index]["x"],
        )
        angle = abs(_angle_delta(outgoing, incoming))
        if angle <= 1e-6:
            continue
        turn_count += 1
        total_turn_angle += angle
        cell = saved_map.world_to_cell(route[index]["x"], route[index]["y"])
        clearance = 0.0 if cell is None else geometry.clearance_at_cell(*cell)
        turn_clearance = clearance - math.hypot(
            half_length + padding, half_width + padding
        )
        minimum_turn_clearance = min(minimum_turn_clearance, turn_clearance)
        turn_safe = turn_safe and validate_rotation_sweep(
            saved_map,
            route[index]["x"],
            route[index]["y"],
            incoming,
            outgoing,
            half_length=half_length,
            half_width=half_width,
            padding=padding,
        ).valid
    return RouteMetadata(
        total_length=total_length,
        minimum_passage_width=minimum_passage_width,
        minimum_static_clearance=minimum_clearance,
        minimum_turn_clearance=minimum_turn_clearance,
        turn_count=turn_count,
        total_turn_angle=total_turn_angle,
        narrow_segments=tuple(narrow_segments),
        estimated_time=(
            total_length / max(0.01, linear_speed)
            + total_turn_angle / max(0.01, angular_speed)
        ),
        turn_safe=turn_safe,
    )


class StopTurnStateLatticePlanner:
    """24-heading lattice containing only forward and in-place primitives."""

    HEADING_BINS = 24

    def __init__(
        self,
        saved_map: "SavedOccupancyMap",
        geometry: MapNavigationGeometry,
        *,
        half_length: float = 0.15,
        half_width: float = 0.10,
        padding: float = 0.0,
        primitive_length: float | None = None,
        linear_speed: float = 0.20,
        angular_speed: float = 0.60,
        max_expansions: int = 250_000,
    ) -> None:
        self.saved_map = saved_map
        self.geometry = geometry
        self.half_length = float(half_length)
        self.half_width = float(half_width)
        self.padding = float(padding)
        self.primitive_length = (
            max(saved_map.resolution, float(primitive_length))
            if primitive_length is not None
            else saved_map.resolution * 2.0
        )
        self.linear_speed = max(0.01, float(linear_speed))
        self.angular_speed = max(0.01, float(angular_speed))
        self.max_expansions = max(1, int(max_expansions))
        self.heading_step = 2.0 * math.pi / self.HEADING_BINS

    def heading_bin(self, yaw: float) -> int:
        return int(round(float(yaw) / self.heading_step)) % self.HEADING_BINS

    def heading(self, heading_bin: int) -> float:
        return (int(heading_bin) % self.HEADING_BINS) * self.heading_step

    def _pose_key(self, x: float, y: float, heading_bin: int) -> tuple[int, int, int]:
        quantum = self.saved_map.resolution * 0.5
        return (
            round((float(x) - self.saved_map.origin_x) / quantum),
            round((float(y) - self.saved_map.origin_y) / quantum),
            int(heading_bin) % self.HEADING_BINS,
        )

    @staticmethod
    def _excluded(
        x: float, y: float, exclusions: tuple[tuple[float, float, float], ...]
    ) -> bool:
        return any(
            math.hypot(float(x) - center_x, float(y) - center_y) <= radius
            for center_x, center_y, radius in exclusions
        )

    @staticmethod
    def _segment_excluded(
        left: dict[str, float],
        right: dict[str, float],
        exclusions: tuple[tuple[float, float, float], ...],
    ) -> bool:
        delta_x = right["x"] - left["x"]
        delta_y = right["y"] - left["y"]
        denominator = delta_x * delta_x + delta_y * delta_y
        for center_x, center_y, radius in exclusions:
            ratio = 0.0 if denominator <= 1e-12 else max(
                0.0,
                min(1.0, (
                    (center_x - left["x"]) * delta_x
                    + (center_y - left["y"]) * delta_y
                ) / denominator),
            )
            if math.hypot(
                left["x"] + ratio * delta_x - center_x,
                left["y"] + ratio * delta_y - center_y,
            ) <= radius:
                return True
        return False

    def _translation_valid(
        self, left: dict[str, float], right: dict[str, float]
    ) -> bool:
        return validate_executable_grid_path(
            (left, right),
            width=self.saved_map.width,
            height=self.saved_map.height,
            resolution=self.saved_map.resolution,
            origin_x=self.saved_map.origin_x,
            origin_y=self.saved_map.origin_y,
            origin_yaw=self.saved_map.origin_yaw,
            data=self.saved_map.occupancy,
            half_length=self.half_length + self.padding,
            half_width=self.half_width + self.padding,
            allow_unknown=False,
            lethal_threshold=65,
        ).valid

    def _grid_seed(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        exclusions: tuple[tuple[float, float, float], ...],
    ) -> list[tuple[int, int]]:
        """Fast topology search; exact SE(2) checks remain authoritative."""
        start_index = self.geometry.index(*start_cell)
        goal_index = self.geometry.index(*goal_cell)
        queue: list[tuple[float, int]] = [(0.0, start_index)]
        costs = {start_index: 0.0}
        parents: dict[int, int | None] = {start_index: None}
        while queue:
            _, index = heapq.heappop(queue)
            if index == goal_index:
                break
            row, column = divmod(index, self.saved_map.width)
            for delta_x, delta_y in (
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1),
            ):
                next_column = column + delta_x
                next_row = row + delta_y
                if not (
                    0 <= next_column < self.saved_map.width
                    and 0 <= next_row < self.saved_map.height
                ):
                    continue
                next_index = self.geometry.index(next_column, next_row)
                if (
                    next_index not in {start_index, goal_index}
                    and not self.geometry.robot_navigable_mask[next_index]
                ):
                    continue
                if delta_x and delta_y:
                    side_a = self.geometry.index(column + delta_x, row)
                    side_b = self.geometry.index(column, row + delta_y)
                    if not (
                        self.geometry.robot_navigable_mask[side_a]
                        and self.geometry.robot_navigable_mask[side_b]
                    ):
                        continue
                world_x, world_y = self.saved_map.cell_center(next_column, next_row)
                if self._excluded(world_x, world_y, exclusions):
                    continue
                step = math.hypot(delta_x, delta_y)
                next_cost = costs[index] + step
                if next_cost + 1e-9 >= costs.get(next_index, math.inf):
                    continue
                costs[next_index] = next_cost
                parents[next_index] = index
                heuristic = math.hypot(
                    goal_cell[0] - next_column, goal_cell[1] - next_row
                )
                heapq.heappush(queue, (next_cost + heuristic, next_index))
        if goal_index not in parents:
            return []
        output: list[tuple[int, int]] = []
        cursor: int | None = goal_index
        while cursor is not None:
            row, column = divmod(cursor, self.saved_map.width)
            output.append((column, row))
            cursor = parents[cursor]
        output.reverse()
        return output

    def _canonical_route_from_seed(
        self,
        seed: list[tuple[int, int]],
        start: dict[str, float],
        goal: dict[str, float],
    ) -> list[dict[str, float]]:
        if len(seed) < 2:
            return []
        raw = [{"x": float(start["x"]), "y": float(start["y"])}]
        raw.extend(
            {"x": point[0], "y": point[1]}
            for point in (
                self.saved_map.cell_center(column, row)
                for column, row in seed[1:-1]
            )
        )
        raw.append({"x": float(goal["x"]), "y": float(goal["y"])})
        translation_cache: dict[tuple[int, int], bool] = {}
        turn_cache: dict[tuple[int, int], bool] = {}
        failed: set[tuple[int, int]] = set()

        def solve(index: int, incoming_yaw: float) -> list[int] | None:
            incoming_bin = self.heading_bin(incoming_yaw)
            memo_key = (index, incoming_bin)
            if memo_key in failed:
                return None
            for following in range(len(raw) - 1, index, -1):
                outgoing_yaw = math.atan2(
                    raw[following]["y"] - raw[index]["y"],
                    raw[following]["x"] - raw[index]["x"],
                )
                if index > 0 and abs(_angle_delta(outgoing_yaw, incoming_yaw)) > math.radians(1.0):
                    turn_key = (index, self.heading_bin(outgoing_yaw))
                    turn_valid = turn_cache.get(turn_key)
                    if turn_valid is None:
                        turn_valid = validate_rotation_sweep(
                            self.saved_map,
                            raw[index]["x"],
                            raw[index]["y"],
                            incoming_yaw,
                            outgoing_yaw,
                            half_length=self.half_length,
                            half_width=self.half_width,
                            padding=self.padding,
                        ).valid
                        turn_cache[turn_key] = turn_valid
                    if not turn_valid:
                        continue
                edge = (index, following)
                edge_valid = translation_cache.get(edge)
                if edge_valid is None:
                    edge_valid = self._translation_valid(raw[index], raw[following])
                    translation_cache[edge] = edge_valid
                if not edge_valid:
                    continue
                if following == len(raw) - 1:
                    return [index, following]
                suffix = solve(following, outgoing_yaw)
                if suffix is not None:
                    return [index, *suffix]
            failed.add(memo_key)
            return None

        initial_yaw = float(start.get("yaw", 0.0))
        indices = solve(0, initial_yaw)
        if indices is None:
            return []
        return canonicalize_stop_turn_path(raw[index] for index in indices)

    def _route_result(
        self,
        route: list[dict[str, float]],
        *,
        start_yaw: float | None = None,
        goal_yaw: float | None = None,
    ) -> StopTurnRoute | None:
        if len(route) < 2:
            return None
        first_heading = math.atan2(
            route[1]["y"] - route[0]["y"],
            route[1]["x"] - route[0]["x"],
        )
        if start_yaw is not None and not validate_rotation_sweep(
            self.saved_map,
            route[0]["x"],
            route[0]["y"],
            float(start_yaw),
            first_heading,
            half_length=self.half_length,
            half_width=self.half_width,
            padding=self.padding,
        ).valid:
            return None
        last_heading = math.atan2(
            route[-1]["y"] - route[-2]["y"],
            route[-1]["x"] - route[-2]["x"],
        )
        if goal_yaw is not None and not validate_rotation_sweep(
            self.saved_map,
            route[-1]["x"],
            route[-1]["y"],
            last_heading,
            float(goal_yaw),
            half_length=self.half_length,
            half_width=self.half_width,
            padding=self.padding,
        ).valid:
            return None
        validation = validate_stop_turn_route(
            self.saved_map,
            route,
            half_length=self.half_length,
            half_width=self.half_width,
            padding=self.padding,
        )
        if not validation.valid:
            return None
        metadata = route_geometry_metadata(
            self.saved_map,
            self.geometry,
            route,
            half_length=self.half_length,
            half_width=self.half_width,
            padding=self.padding,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
        )
        headings = tuple(
            self.heading_bin(math.atan2(
                right["y"] - left["y"], right["x"] - left["x"]
            ))
            for left, right in zip(route, route[1:])
        )
        return StopTurnRoute(tuple(route), metadata, headings)

    def plan(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        *,
        exclusions: Iterable[tuple[float, float, float]] = (),
    ) -> StopTurnRoute | None:
        start_x, start_y = float(start["x"]), float(start["y"])
        goal_x, goal_y = float(goal["x"]), float(goal["y"])
        start_cell = self.saved_map.world_to_cell(start_x, start_y)
        goal_cell = self.saved_map.world_to_cell(goal_x, goal_y)
        if start_cell is None or goal_cell is None:
            return None
        # This conservative component test catches physically disconnected
        # clicks quickly. The oriented lattice below remains the authority for
        # narrow straight passages that are not turn-safe.
        if not self.geometry.same_component(start_cell, goal_cell):
            return None
        forbidden = tuple(
            (float(x), float(y), max(0.0, float(radius)))
            for x, y, radius in exclusions
        )
        start_yaw = float(start.get("yaw", 0.0))
        goal_yaw = (
            float(goal["yaw"]) if "yaw" in goal and goal["yaw"] is not None else None
        )
        simple_routes: list[StopTurnRoute] = []
        for simple in (
            [
                {"x": start_x, "y": start_y},
                {"x": goal_x, "y": goal_y},
            ],
            [
                {"x": start_x, "y": start_y},
                {"x": goal_x, "y": start_y},
                {"x": goal_x, "y": goal_y},
            ],
            [
                {"x": start_x, "y": start_y},
                {"x": start_x, "y": goal_y},
                {"x": goal_x, "y": goal_y},
            ],
        ):
            canonical_simple = canonicalize_stop_turn_path(simple)
            if any(
                self._segment_excluded(left, right, forbidden)
                for left, right in zip(canonical_simple, canonical_simple[1:])
            ):
                continue
            result = self._route_result(
                canonical_simple, start_yaw=start_yaw, goal_yaw=goal_yaw
            )
            if result is not None:
                simple_routes.append(result)
        if simple_routes:
            return min(simple_routes, key=self.ranking_key)
        seed = self._grid_seed(start_cell, goal_cell, forbidden)
        seeded_route = self._canonical_route_from_seed(seed, start, goal)
        if seeded_route:
            result = self._route_result(
                seeded_route, start_yaw=start_yaw, goal_yaw=goal_yaw
            )
            if result is not None:
                return result
        start_heading = self.heading_bin(float(start.get("yaw", 0.0)))
        start_key = self._pose_key(start_x, start_y, start_heading)
        positions: dict[tuple[int, int, int], tuple[float, float]] = {
            start_key: (start_x, start_y)
        }
        parents: dict[
            tuple[int, int, int], tuple[int, int, int] | None
        ] = {start_key: None}
        costs = {start_key: 0.0}
        queue: list[tuple[float, int, tuple[int, int, int]]] = []
        sequence = 0
        heapq.heappush(
            queue,
            (
                math.hypot(goal_x - start_x, goal_y - start_y)
                / self.linear_speed,
                sequence,
                start_key,
            ),
        )
        edge_cache: dict[tuple[tuple[int, int, int], tuple[int, int, int]], bool] = {}
        rotation_cache: dict[tuple[tuple[int, int, int], int], bool] = {}
        finish_key: tuple[int, int, int] | None = None
        expansions = 0
        while queue and expansions < self.max_expansions:
            _, _, state = heapq.heappop(queue)
            state_cost = costs[state]
            x, y = positions[state]
            heading_bin = state[2]
            yaw = self.heading(heading_bin)
            expansions += 1
            goal_distance = math.hypot(goal_x - x, goal_y - y)
            if goal_distance <= self.primitive_length * 2.5:
                goal_heading = math.atan2(goal_y - y, goal_x - x)
                rotation = validate_rotation_sweep(
                    self.saved_map,
                    x,
                    y,
                    yaw,
                    goal_heading,
                    half_length=self.half_length,
                    half_width=self.half_width,
                    padding=self.padding,
                )
                direct = {"x": goal_x, "y": goal_y}
                if (
                    rotation.valid
                    and not self._excluded(goal_x, goal_y, forbidden)
                    and self._translation_valid({"x": x, "y": y}, direct)
                ):
                    goal_bin = self.heading_bin(goal_heading)
                    finish_key = self._pose_key(goal_x, goal_y, goal_bin)
                    positions[finish_key] = (goal_x, goal_y)
                    parents[finish_key] = state
                    break

            next_x = x + self.primitive_length * math.cos(yaw)
            next_y = y + self.primitive_length * math.sin(yaw)
            next_cell = self.saved_map.world_to_cell(next_x, next_y)
            if (
                next_cell is not None
                and not self._excluded(next_x, next_y, forbidden)
            ):
                next_key = self._pose_key(next_x, next_y, heading_bin)
                edge = (state, next_key)
                valid = edge_cache.get(edge)
                if valid is None:
                    valid = self._translation_valid(
                        {"x": x, "y": y}, {"x": next_x, "y": next_y}
                    )
                    edge_cache[edge] = valid
                if valid:
                    next_cost = state_cost + self.primitive_length / self.linear_speed
                    if next_cost + 1e-9 < costs.get(next_key, math.inf):
                        costs[next_key] = next_cost
                        parents[next_key] = state
                        positions[next_key] = (next_x, next_y)
                        sequence += 1
                        heuristic = math.hypot(goal_x - next_x, goal_y - next_y) / self.linear_speed
                        heapq.heappush(
                            queue, (next_cost + heuristic, sequence, next_key)
                        )

            for direction in (-1, 1):
                next_heading = (heading_bin + direction) % self.HEADING_BINS
                next_key = self._pose_key(x, y, next_heading)
                cache_key = (state, next_heading)
                valid = rotation_cache.get(cache_key)
                if valid is None:
                    valid = validate_rotation_sweep(
                        self.saved_map,
                        x,
                        y,
                        yaw,
                        self.heading(next_heading),
                        half_length=self.half_length,
                        half_width=self.half_width,
                        padding=self.padding,
                    ).valid
                    rotation_cache[cache_key] = valid
                if not valid:
                    continue
                next_cost = state_cost + self.heading_step / self.angular_speed
                if next_cost + 1e-9 >= costs.get(next_key, math.inf):
                    continue
                costs[next_key] = next_cost
                parents[next_key] = state
                positions[next_key] = (x, y)
                sequence += 1
                heuristic = math.hypot(goal_x - x, goal_y - y) / self.linear_speed
                heapq.heappush(queue, (next_cost + heuristic, sequence, next_key))
        if finish_key is None:
            return None
        states: list[tuple[int, int, int]] = []
        cursor: tuple[int, int, int] | None = finish_key
        while cursor is not None:
            states.append(cursor)
            cursor = parents[cursor]
        states.reverse()
        route = canonicalize_stop_turn_path(
            {"x": positions[state][0], "y": positions[state][1]}
            for state in states
        )
        return self._route_result(route, start_yaw=start_yaw, goal_yaw=goal_yaw)

    def ranking_key(
        self, route: StopTurnRoute
    ) -> tuple[float, float, int, float, float]:
        metadata = route.metadata
        # Clearance beyond that required to rotate the complete rectangular
        # footprint is equally safe for this robot.  Saturating at that
        # geometry-derived threshold prevents an open room's orientation from
        # creating needless right-angle/S-shaped routes merely to maximize an
        # already ample ray width.  Below the threshold, wider routes still
        # rank first as required.
        rotation_radius = math.hypot(
            self.half_length + self.padding,
            self.half_width + self.padding,
        )
        return (
            -min(metadata.minimum_passage_width, 2.0 * rotation_radius),
            -min(metadata.minimum_static_clearance, rotation_radius),
            metadata.turn_count,
            metadata.total_turn_angle,
            metadata.total_length,
        )

    def plan_candidates(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        *,
        maximum_candidates: int = 3,
        overlap_threshold: float = 0.80,
    ) -> list[StopTurnRoute]:
        primary = self.plan(start, goal)
        if primary is None:
            return []
        candidates = [primary]
        attempts: list[tuple[float, float, float]] = []
        points = list(primary.points)
        if len(points) >= 2:
            sampled = _resample_path(points, spacing=max(0.10, self.primitive_length))
            radius = max(0.20, 2.0 * self.half_width + 2.0 * self.padding)
            for fraction in (0.50, 0.35, 0.65, 0.25, 0.75):
                index = min(len(sampled) - 1, max(0, round((len(sampled) - 1) * fraction)))
                x, y = sampled[index]
                if math.hypot(x - float(start["x"]), y - float(start["y"])) > radius * 1.5 and math.hypot(
                    x - float(goal["x"]), y - float(goal["y"])
                ) > radius * 1.5:
                    attempts.append((x, y, radius))
        for exclusion in attempts:
            if len(candidates) >= max(1, int(maximum_candidates)):
                break
            candidate = self.plan(start, goal, exclusions=(exclusion,))
            if candidate is None:
                continue
            candidate_points = list(candidate.points)
            if any(
                path_overlap_ratio(candidate_points, list(existing.points))
                >= float(overlap_threshold)
                for existing in candidates
            ):
                continue
            candidates.append(candidate)
        return sorted(candidates, key=self.ranking_key)


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


@dataclass(slots=True)
class UnwrappedYawProgress:
    """Accumulate physical yaw from odometry without command integration."""

    last_yaw: float | None = None
    total: float = 0.0

    def reset(self, yaw: float | None = None) -> None:
        self.last_yaw = None if yaw is None else float(yaw)
        self.total = 0.0

    def update(self, yaw: float) -> float:
        value = float(yaw)
        if self.last_yaw is not None:
            self.total += abs(_angle_delta(value, self.last_yaw))
        self.last_yaw = value
        return self.total


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

    side_window = [(x, y) for x, y in values if -length <= x <= forward_limit]
    # Pair walls in the same longitudinal bin. Combining a left return near
    # the robot with a right return at a different X creates a fictitious
    # corridor width. Medians reject isolated glass/noise returns.
    bin_size = 0.10
    side_bins: dict[int, dict[str, list[float]]] = {}
    for point_x, point_y in side_window:
        key = math.floor((point_x + length) / bin_size)
        group = side_bins.setdefault(key, {"left": [], "right": []})
        if point_y >= width:
            group["left"].append(point_y)
        elif point_y <= -width:
            group["right"].append(-point_y)
    paired = [
        (statistics.median(group["left"]), statistics.median(group["right"]))
        for group in side_bins.values()
        if group["left"] and group["right"]
    ]
    paired.sort(key=lambda item: item[0] + item[1])
    if paired:
        # A low percentile stays conservative while a single isolated return
        # cannot define the whole route width.
        selected = paired[min(len(paired) - 1, max(0, len(paired) // 5))]
        left_center, right_center = selected
    else:
        left_center = min(
            (y for _, y in side_window if y >= width), default=math.nan
        )
        right_center = min(
            (-y for _, y in side_window if y <= -width), default=math.nan
        )
    left_clearance = (
        max(0.0, left_center - width) if math.isfinite(left_center) else 0.0
    )
    right_clearance = (
        max(0.0, right_center - width) if math.isfinite(right_center) else 0.0
    )
    available_width = (
        left_center + right_center
        if math.isfinite(left_center) and math.isfinite(right_center)
        else 0.0
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
    both_sides_observed = bool(paired)
    physically_passable = (
        hard_side_clear
        and available_width >= hard_required_width
        and front_clear
    )
    if not both_sides_observed and front_clear:
        # Missing live wall data is uncertainty, not infinite clearance and
        # not proof of a physical blockage. The prevalidated static route is
        # authoritative; motion-safety still handles every live return.
        physically_passable = True
        classification = "NARROW_OR_UNCERTAIN"
        reason = "LIVE_SIDE_INCOMPLETE"
    elif not physically_passable:
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
    can_go_straight = physically_passable

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


def deskew_scan_points(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    time_increment: float,
    scan_time: float,
    linear_velocity: float,
    angular_velocity: float,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> list[tuple[int, float, float]]:
    """Transform each moving-scan return into the final-beam laser frame."""
    measurements = [float(value) for value in ranges]
    if not measurements:
        return []
    increment = float(time_increment)
    if increment <= 0.0:
        increment = max(0.0, float(scan_time)) / max(1, len(measurements) - 1)
    duration = increment * max(0, len(measurements) - 1)
    velocity = float(linear_velocity)
    yaw_rate = float(angular_velocity)

    def base_pose(timestamp: float) -> tuple[float, float, float]:
        yaw = yaw_rate * timestamp
        if abs(yaw_rate) <= 1e-9:
            return velocity * timestamp, 0.0, yaw
        radius = velocity / yaw_rate
        return radius * math.sin(yaw), radius * (1.0 - math.cos(yaw)), yaw

    reference_x, reference_y, reference_yaw = base_pose(duration)
    reference_laser_x = (
        reference_x
        + math.cos(reference_yaw) * float(laser_x)
        - math.sin(reference_yaw) * float(laser_y)
    )
    reference_laser_y = (
        reference_y
        + math.sin(reference_yaw) * float(laser_x)
        + math.cos(reference_yaw) * float(laser_y)
    )
    reference_laser_yaw = reference_yaw + float(laser_yaw)
    reference_cosine = math.cos(reference_laser_yaw)
    reference_sine = math.sin(reference_laser_yaw)
    output: list[tuple[int, float, float]] = []
    for index, distance in enumerate(measurements):
        if (
            not math.isfinite(distance)
            or distance < float(range_min)
            or distance > float(range_max)
        ):
            continue
        pose_x, pose_y, pose_yaw = base_pose(index * increment)
        beam = float(angle_min) + index * float(angle_increment) + float(laser_yaw)
        local_x = float(laser_x) + distance * math.cos(beam)
        local_y = float(laser_y) + distance * math.sin(beam)
        world_x = pose_x + math.cos(pose_yaw) * local_x - math.sin(pose_yaw) * local_y
        world_y = pose_y + math.sin(pose_yaw) * local_x + math.cos(pose_yaw) * local_y
        delta_x = world_x - reference_laser_x
        delta_y = world_y - reference_laser_y
        corrected_x = reference_cosine * delta_x + reference_sine * delta_y
        corrected_y = -reference_sine * delta_x + reference_cosine * delta_y
        output.append((index, corrected_x, corrected_y))
    return output


def deskew_laser_scan_ranges(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    time_increment: float,
    scan_time: float,
    linear_velocity: float,
    angular_velocity: float,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> tuple[list[float], int]:
    """Re-bin deskewed endpoints while retaining original unmatched beams."""
    output = [float(value) for value in ranges]
    corrected = [math.inf] * len(output)
    count = 0
    for _, point_x, point_y in deskew_scan_points(
        output,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        time_increment=time_increment,
        scan_time=scan_time,
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=laser_yaw,
    ):
        angle = math.atan2(point_y, point_x)
        if abs(float(angle_increment)) <= 1e-12:
            continue
        target = round((angle - float(angle_min)) / float(angle_increment))
        if not (0 <= target < len(corrected)):
            continue
        distance = math.hypot(point_x, point_y)
        if float(range_min) <= distance <= float(range_max):
            corrected[target] = min(corrected[target], distance)
            count += 1
    for index, distance in enumerate(corrected):
        if math.isfinite(distance):
            output[index] = distance
    return output, count


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
    navigation_geometry: MapNavigationGeometry | None = field(
        default=None, repr=False
    )

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
        saved_map = cls(
            width=image.width,
            height=image.height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            occupancy=occupancy,
        )
        saved_map.navigation_geometry = MapNavigationGeometry.build(saved_map)
        return saved_map

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
        unknown_is_blocked: bool = False,
    ) -> float | None:
        """Return expected range to the first known occupied map cell.

        Reaching unknown space or leaving the map before an occupied cell is
        deliberately inconclusive so callers retain the live obstacle point.
        """
        lower = max(0.0, float(minimum_range))
        upper = max(lower, float(maximum_range))
        # Amanatides-Woo traversal visits every crossed cell exactly once.
        # Unlike the former half-cell Python march, its work is proportional
        # to crossed grid cells and it cannot skip a thin wall.
        map_cosine = math.cos(self.origin_yaw)
        map_sine = math.sin(self.origin_yaw)
        delta_x = float(x) - self.origin_x
        delta_y = float(y) - self.origin_y
        local_origin_x = map_cosine * delta_x + map_sine * delta_y
        local_origin_y = -map_sine * delta_x + map_cosine * delta_y
        local_angle = float(angle) - self.origin_yaw
        direction_x = math.cos(local_angle)
        direction_y = math.sin(local_angle)
        ray_x = local_origin_x + lower * direction_x
        ray_y = local_origin_y + lower * direction_y
        column = math.floor(ray_x / self.resolution)
        row = math.floor(ray_y / self.resolution)
        if not (0 <= column < self.width and 0 <= row < self.height):
            return None
        step_x = 1 if direction_x > 0 else -1
        step_y = 1 if direction_y > 0 else -1
        delta_t_x = (
            math.inf if abs(direction_x) <= 1e-12
            else self.resolution / abs(direction_x)
        )
        delta_t_y = (
            math.inf if abs(direction_y) <= 1e-12
            else self.resolution / abs(direction_y)
        )
        next_boundary_x = (
            (column + 1) * self.resolution
            if direction_x > 0 else column * self.resolution
        )
        next_boundary_y = (
            (row + 1) * self.resolution
            if direction_y > 0 else row * self.resolution
        )
        max_t_x = (
            math.inf if abs(direction_x) <= 1e-12
            else (next_boundary_x - local_origin_x) / direction_x
        )
        max_t_y = (
            math.inf if abs(direction_y) <= 1e-12
            else (next_boundary_y - local_origin_y) / direction_y
        )
        entered_at = lower
        while entered_at <= upper:
            if not (0 <= column < self.width and 0 <= row < self.height):
                return entered_at if unknown_is_blocked else None
            value = self.value_at(column, row)
            if value < 0:
                return entered_at if unknown_is_blocked else None
            if value >= 65:
                return max(lower, entered_at)
            if max_t_x < max_t_y:
                entered_at = max_t_x
                max_t_x += delta_t_x
                column += step_x
            elif max_t_y < max_t_x:
                entered_at = max_t_y
                max_t_y += delta_t_y
                row += step_y
            else:
                entered_at = max_t_x
                max_t_x += delta_t_x
                max_t_y += delta_t_y
                column += step_x
                row += step_y
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
        reachable_from: tuple[float, float] | None = None,
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
        reachable_component: int | None = None
        if reachable_from is not None and self.navigation_geometry is not None:
            start_cell = self.world_to_cell(*reachable_from)
            if start_cell is None:
                return None
            reachable_component = self.navigation_geometry.component_ids[
                self.navigation_geometry.index(*start_cell)
            ]
            if reachable_component < 0:
                return None
        candidates: list[tuple[float, int, int, float, float]] = []
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                column = requested_column + offset_x
                row = requested_row + offset_y
                if not (0 <= column < self.width and 0 <= row < self.height):
                    continue
                candidate_x, candidate_y = self.cell_center(column, row)
                if (
                    reachable_component is not None
                    and self.navigation_geometry is not None
                    and self.navigation_geometry.component_ids[
                        self.navigation_geometry.index(column, row)
                    ] != reachable_component
                ):
                    continue
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
