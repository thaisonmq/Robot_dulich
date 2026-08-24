from __future__ import annotations

import bisect
import json
import heapq
import logging
import math
import os
import statistics
import time
from collections import defaultdict, deque
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
    initial_turn_angle: float
    internal_turn_angle: float
    final_turn_angle: float
    execution_total_turn_angle: float
    narrow_segments: tuple[dict[str, float], ...]
    estimated_time: float
    turn_safe: bool
    # Smallest lateral gap from either side of the physical chassis to Saved
    # Map structure while translating.  Total passage width alone is not
    # sufficient: a 35 cm corridor is still unusable when a 20 cm robot is
    # planned only a few millimetres from one wall.
    minimum_side_clearance: float = math.inf

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
            "minimum_side_clearance": _finite_or_none(
                self.minimum_side_clearance
            ),
            "turn_count": self.turn_count,
            "total_turn_angle": round(self.total_turn_angle, 4),
            "initial_turn_angle": round(self.initial_turn_angle, 4),
            "internal_turn_angle": round(self.internal_turn_angle, 4),
            "final_turn_angle": round(self.final_turn_angle, 4),
            "execution_total_turn_angle": round(
                self.execution_total_turn_angle, 4
            ),
            "narrow_segments": [dict(item) for item in self.narrow_segments],
            "estimated_time": round(self.estimated_time, 3),
            "turn_safe": self.turn_safe,
        }


@dataclass(frozen=True, slots=True)
class StopTurnRoute:
    points: tuple[dict[str, float], ...]
    metadata: RouteMetadata
    heading_bins: tuple[int, ...]
    segment_directions: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class StartEscape:
    """One bounded straight motion that leaves initial map overlap."""

    start: dict[str, float]
    end: dict[str, float]
    yaw: float
    distance: float
    initial_overlap_cells: tuple[tuple[int, int], ...]
    samples_checked: int
    motion_direction: int = 1


@dataclass(frozen=True, slots=True)
class PoseContinuity:
    """Map-pose motion compared with the same physical motion in odom."""

    consistent: bool
    translation_residual: float
    yaw_residual: float
    map_translation: float
    odom_translation: float
    map_yaw_delta: float
    odom_yaw_delta: float


@dataclass(frozen=True, slots=True)
class PlannerResult:
    """Structured result shared by initial planning and every recovery flow."""

    status: str
    route: StopTurnRoute | None = None
    reason: str = ""
    message: str = ""
    start_escape: StartEscape | None = None
    expansions: int = 0
    elapsed_seconds: float = 0.0

    @property
    def success(self) -> bool:
        return self.status == "SUCCESS" and self.route is not None


@dataclass(frozen=True, slots=True)
class DynamicObstacle:
    id: int
    center_x: float
    center_y: float
    radius: float
    bounds: tuple[float, float, float, float]
    first_seen: float
    last_seen: float
    observation_count: int
    confidence: float
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    speed: float = 0.0
    motion_state: str = "UNCONFIRMED"
    stationary_since: float | None = None
    motion_anchor_x: float | None = None
    motion_anchor_y: float | None = None
    motion_anchor_time: float | None = None


class DynamicObstacleOverlay:
    """TTL-scoped clustered runtime obstacles; never mutates the Saved Map."""

    def __init__(
        self,
        *,
        ttl_seconds: float = 2.0,
        cluster_distance: float = 0.12,
        association_distance: float = 0.20,
        observation_radius: float = 0.025,
        motion_threshold: float = 0.12,
        stationary_confirmation_seconds: float = 1.0,
    ) -> None:
        self.ttl_seconds = max(0.05, float(ttl_seconds))
        self.cluster_distance = max(0.001, float(cluster_distance))
        self.association_distance = max(
            self.cluster_distance, float(association_distance)
        )
        self.observation_radius = max(0.0, float(observation_radius))
        self.motion_threshold = max(0.01, float(motion_threshold))
        self.stationary_confirmation_seconds = max(
            0.10, float(stationary_confirmation_seconds)
        )
        self._next_id = 1
        self._obstacles: dict[int, DynamicObstacle] = {}

    @staticmethod
    def _clusters(
        points: tuple[tuple[float, float], ...], distance: float
    ) -> list[list[tuple[float, float]]]:
        """Return distance-connected components using a metric spatial hash.

        The previous implementation compared every pending point with every
        other point. A 600-cell local costmap therefore monopolized the Python
        callback group long enough for safety and sensor-time heartbeats to be
        reported stale. Cells are now compared only with their eight adjacent
        buckets while preserving the same transitive clustering semantics.
        """
        if not points:
            return []
        cell_size = max(1e-6, float(distance))

        def bucket(point: tuple[float, float]) -> tuple[int, int]:
            return (
                math.floor(point[0] / cell_size),
                math.floor(point[1] / cell_size),
            )

        buckets: dict[tuple[int, int], set[int]] = defaultdict(set)
        for index, point in enumerate(points):
            buckets[bucket(point)].add(index)
        remaining = set(range(len(points)))
        clusters: list[list[tuple[float, float]]] = []
        while remaining:
            seed = remaining.pop()
            buckets[bucket(points[seed])].discard(seed)
            indices = [seed]
            pending = [seed]
            while pending:
                current = pending.pop()
                cell_x, cell_y = bucket(points[current])
                candidates: set[int] = set()
                for offset_x in (-1, 0, 1):
                    for offset_y in (-1, 0, 1):
                        candidates.update(
                            buckets.get(
                                (cell_x + offset_x, cell_y + offset_y), ()
                            )
                        )
                joined = [
                    index
                    for index in candidates
                    if math.hypot(
                        points[index][0] - points[current][0],
                        points[index][1] - points[current][1],
                    ) <= distance
                ]
                for index in joined:
                    remaining.remove(index)
                    buckets[bucket(points[index])].discard(index)
                    indices.append(index)
                    pending.append(index)
            clusters.append([points[index] for index in indices])
        return clusters

    def expire(self, now: float) -> None:
        cutoff = float(now) - self.ttl_seconds
        self._obstacles = {
            obstacle_id: obstacle
            for obstacle_id, obstacle in self._obstacles.items()
            if obstacle.last_seen >= cutoff
        }

    def observe(
        self,
        points: Iterable[tuple[float, float]],
        *,
        now: float,
        saved_map: "SavedOccupancyMap | None" = None,
        static_tolerance: float = 0.08,
    ) -> tuple[DynamicObstacle, ...]:
        timestamp = float(now)
        self.expire(timestamp)
        filtered = tuple(
            (float(x), float(y))
            for x, y in points
            if math.isfinite(float(x)) and math.isfinite(float(y))
            and (
                saved_map is None
                or not saved_map.occupied_within(
                    float(x), float(y), max(0.0, float(static_tolerance))
                )
            )
        )
        # A costmap callback is one observation frame. Several disconnected
        # lethal-cell clusters in that same frame may all fall inside one
        # track's association radius; allowing each cluster to update the same
        # track made ``observation_count`` count clusters instead of independent
        # frames. Besides overstating confidence, one noisy costmap could then
        # immediately acquire planning/stop authority. Keep association
        # one-to-one inside this frame. Unmatched nearby clusters remain
        # separate tentative tracks and may be associated on a later frame.
        observed_track_ids: set[int] = set()
        for cluster in self._clusters(filtered, self.cluster_distance):
            minimum_x = min(point[0] for point in cluster)
            maximum_x = max(point[0] for point in cluster)
            minimum_y = min(point[1] for point in cluster)
            maximum_y = max(point[1] for point in cluster)
            center_x = sum(point[0] for point in cluster) / len(cluster)
            center_y = sum(point[1] for point in cluster) / len(cluster)
            radius = max(
                self.observation_radius,
                max(
                    math.hypot(x - center_x, y - center_y)
                    for x, y in cluster
                ) + self.observation_radius,
            )
            associated = min(
                (
                    obstacle
                    for obstacle in self._obstacles.values()
                    if obstacle.id not in observed_track_ids
                    and math.hypot(
                        obstacle.center_x - center_x,
                        obstacle.center_y - center_y,
                    ) <= self.association_distance + obstacle.radius + radius
                ),
                key=lambda obstacle: math.hypot(
                    obstacle.center_x - center_x,
                    obstacle.center_y - center_y,
                ),
                default=None,
            )
            if associated is None:
                obstacle_id = self._next_id
                self._next_id += 1
                first_seen = timestamp
                count = 1
                velocity_x = 0.0
                velocity_y = 0.0
                speed = 0.0
                motion_state = "UNCONFIRMED"
                stationary_since = timestamp
                motion_anchor_x = center_x
                motion_anchor_y = center_y
                motion_anchor_time = timestamp
            else:
                obstacle_id = associated.id
                first_seen = associated.first_seen
                count = associated.observation_count + 1
                motion_anchor_x = (
                    associated.center_x
                    if associated.motion_anchor_x is None
                    else associated.motion_anchor_x
                )
                motion_anchor_y = (
                    associated.center_y
                    if associated.motion_anchor_y is None
                    else associated.motion_anchor_y
                )
                motion_anchor_time = (
                    associated.last_seen
                    if associated.motion_anchor_time is None
                    else associated.motion_anchor_time
                )
                velocity_x = associated.velocity_x
                velocity_y = associated.velocity_y
                elapsed = timestamp - motion_anchor_time
                # Costmap endpoints can jump one 2.5 cm cell each frame even
                # for a fixed chair leg. Estimate motion over a longer window
                # so raster/pose jitter is not mistaken for a walking person.
                if elapsed >= 0.50:
                    velocity_x = (center_x - motion_anchor_x) / elapsed
                    velocity_y = (center_y - motion_anchor_y) / elapsed
                    motion_anchor_x = center_x
                    motion_anchor_y = center_y
                    motion_anchor_time = timestamp
                speed = math.hypot(velocity_x, velocity_y)
                if count >= 3 and speed >= self.motion_threshold:
                    motion_state = "MOVING"
                    stationary_since = None
                else:
                    stationary_since = (
                        associated.stationary_since
                        if associated.motion_state != "MOVING"
                        and associated.stationary_since is not None
                        else timestamp
                    )
                    motion_state = (
                        "STATIONARY"
                        if count >= 3
                        and timestamp - stationary_since
                        >= self.stationary_confirmation_seconds
                        else "UNCONFIRMED"
                    )
                # A fixed exponential center follows a moving person without
                # growing an unbounded historical trail, while still damping
                # one-cell costmap jitter for a stationary chair.
                weight = 0.50
                center_x = associated.center_x * (1.0 - weight) + center_x * weight
                center_y = associated.center_y * (1.0 - weight) + center_y * weight
            observed_track_ids.add(obstacle_id)
            self._obstacles[obstacle_id] = DynamicObstacle(
                obstacle_id,
                center_x,
                center_y,
                radius,
                (minimum_x, minimum_y, maximum_x, maximum_y),
                first_seen,
                timestamp,
                count,
                min(1.0, count / 3.0),
                velocity_x,
                velocity_y,
                speed,
                motion_state,
                stationary_since,
                motion_anchor_x,
                motion_anchor_y,
                motion_anchor_time,
            )
        return self.snapshot(timestamp)

    def observe_confirmed_blocker(
        self,
        points: Iterable[tuple[float, float]],
        *,
        now: float,
    ) -> tuple[DynamicObstacle, ...]:
        """Retain a live blocker confirmed by the motion controller.

        Ordinary scan points close to Saved Map occupancy are filtered so
        static walls never become dynamic obstacles. A fresh controller stop
        plus corridor evidence is stronger: even when the blocker overlaps a
        stale map pixel it must become a temporary planning exclusion, or the
        recovery planner will repeatedly return the route that just failed.
        """
        return self.observe(points, now=now, saved_map=None)

    def snapshot(self, now: float) -> tuple[DynamicObstacle, ...]:
        self.expire(now)
        return tuple(sorted(self._obstacles.values(), key=lambda item: item.id))

    def exclusions(
        self,
        now: float,
        *,
        inflation: float = 0.0,
        minimum_observations: int = 1,
    ) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            (
                obstacle.center_x,
                obstacle.center_y,
                obstacle.radius + max(0.0, float(inflation)),
            )
            for obstacle in self.snapshot(now)
            if obstacle.observation_count >= max(1, int(minimum_observations))
        )


@dataclass(slots=True)
class TurnBlockTracker:
    """Atomic-sequence turn-block timer with a clear dwell."""

    clear_dwell_seconds: float
    blocked_since: float | None = None
    clear_since: float | None = None
    last_sequence: int = -1

    def update(self, *, sequence: int, blocked: bool, now: float) -> bool:
        if int(sequence) <= self.last_sequence:
            return self.blocked_since is not None
        self.last_sequence = int(sequence)
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = float(now)
            self.clear_since = None
            return True
        if self.blocked_since is None:
            return False
        if self.clear_since is None:
            self.clear_since = float(now)
            return True
        if float(now) - self.clear_since < max(0.0, self.clear_dwell_seconds):
            return True
        self.blocked_since = None
        self.clear_since = None
        return False


@dataclass(frozen=True, slots=True)
class ActiveSegment:
    """Immutable control geometry for one stop-turn straight segment.

    The preview route remains useful to the UI, but it is deliberately not
    consulted after this object is created.  Re-anchoring creates a new
    instance and token so every control/feedback path observes one geometry.
    """

    planned_start: dict[str, float]
    effective_start: dict[str, float]
    endpoint: dict[str, float]
    fixed_heading: float
    segment_length: float
    segment_index: int
    route_id: str
    segment_token: int
    narrow: bool
    motion_direction: int

    @classmethod
    def create(
        cls,
        *,
        planned_start: dict[str, float],
        effective_start: dict[str, float],
        endpoint: dict[str, float],
        segment_index: int,
        route_id: str,
        segment_token: int,
        narrow: bool = False,
        motion_direction: int = 1,
    ) -> "ActiveSegment":
        planned = {
            "x": float(planned_start["x"]),
            "y": float(planned_start["y"]),
        }
        effective = {
            "x": float(effective_start["x"]),
            "y": float(effective_start["y"]),
        }
        end = {"x": float(endpoint["x"]), "y": float(endpoint["y"])}
        delta_x = end["x"] - effective["x"]
        delta_y = end["y"] - effective["y"]
        length = math.hypot(delta_x, delta_y)
        if length <= 1e-9:
            raise ValueError("Active segment must have non-zero length")
        direction = -1 if int(motion_direction) < 0 else 1
        travel_heading = math.atan2(delta_y, delta_x)
        return cls(
            planned_start=planned,
            effective_start=effective,
            endpoint=end,
            fixed_heading=(
                travel_heading
                if direction > 0
                else _angle_delta(travel_heading + math.pi, 0.0)
            ),
            segment_length=length,
            segment_index=int(segment_index),
            route_id=str(route_id),
            segment_token=int(segment_token),
            narrow=bool(narrow),
            motion_direction=direction,
        )


@dataclass(frozen=True, slots=True)
class StraightSegmentProgress:
    segment_length: float
    along_track: float
    remaining_longitudinal: float
    signed_cross_track: float
    endpoint_distance: float
    passed_endpoint: bool


@dataclass(frozen=True, slots=True)
class StraightControlDecision:
    angular: float
    forward_allowed: bool
    heading_error: float
    signed_cross_track: float
    reason: str = ""


@dataclass(frozen=True, slots=True)
class SegmentWatchdogDecision:
    exceeded: bool
    elapsed_limit: float
    travel_limit: float
    reason: str = ""


def position_within_tolerance(
    pose: dict[str, float],
    destination: dict[str, float],
    tolerance: float,
) -> bool:
    """Return position-only arrival without interpreting either pose yaw."""
    try:
        pose_x = float(pose["x"])
        pose_y = float(pose["y"])
        destination_x = float(destination["x"])
        destination_y = float(destination["y"])
        limit = max(0.0, float(tolerance))
    except (KeyError, TypeError, ValueError):
        return False
    return bool(
        all(math.isfinite(value) for value in (
            pose_x, pose_y, destination_x, destination_y, limit
        ))
        and math.hypot(pose_x - destination_x, pose_y - destination_y) <= limit
    )


def controller_abort_is_live_blockage(
    *,
    error_code: Any,
    error_msg: str,
    atomic_motion_safety_block: bool,
    dynamic_route_intersection: bool,
    controller_zero_linear: bool,
    repeated_zero_linear_abort: bool,
    corridor_sample_fresh: bool,
    corridor_front_clearance: float,
    corridor_blockage_limit: float,
) -> bool:
    """Classify explicit/fresh blockage evidence without inferring from status.

    Older Humble FollowPath results contain no diagnostics. In that case a
    fresh near-front corridor sample or repeated zero-linear abort supplies
    the missing controller-side evidence. None of these inputs proves a
    static disconnection; they only select runtime wait/replan behavior.
    """
    normalized_message = str(error_msg or "").strip().upper()
    normalized_code = str(error_code or "").strip().upper()
    diagnostic_block = bool(
        normalized_code in {
            "106", "NO_VALID_CONTROL", "COLLISION", "BLOCKED",
        }
        or any(term in normalized_message for term in (
            "COLLISION",
            "OBSTACLE",
            "BLOCKED",
            "NO VALID CONTROL",
            "NO VALID TRAJECTORY",
            "COSTMAP COLLISION",
        ))
    )
    front_clearance = float(corridor_front_clearance)
    blockage_limit = max(0.0, float(corridor_blockage_limit))
    fresh_near_front = bool(
        controller_zero_linear
        and corridor_sample_fresh
        and math.isfinite(front_clearance)
        and front_clearance <= blockage_limit
    )
    return bool(
        atomic_motion_safety_block
        or dynamic_route_intersection
        or diagnostic_block
        or (controller_zero_linear and repeated_zero_linear_abort)
        or fresh_near_front
    )


def dynamic_block_requires_alternative(reason: str) -> bool:
    """Return whether recovery must never resume the blocked geometry.

    Moving tracked objects may clear and permit the preserved route. A hard
    controller/corridor collision is different evidence: it invalidates that
    geometry for the current encounter and permits only one planned detour.
    """
    value = str(reason or "")
    return bool(
        value.startswith("CONTROLLER_ABORT")
        or value in {
            "MOTION_SAFETY_DYNAMIC_BLOCK",
            "CONFIRMED_DYNAMIC_ROUTE_BLOCK",
            "LIVE_ROUTE_CLEARANCE_INSUFFICIENT",
        }
    )


def straight_segment_progress(
    start: dict[str, float],
    end: dict[str, float],
    current_pose: dict[str, float],
    *,
    overshoot_epsilon: float = 1e-6,
) -> StraightSegmentProgress:
    """Project a pose onto a fixed finite segment without path look-ahead."""
    start_x, start_y = float(start["x"]), float(start["y"])
    end_x, end_y = float(end["x"]), float(end["y"])
    delta_x, delta_y = end_x - start_x, end_y - start_y
    length = math.hypot(delta_x, delta_y)
    if length <= 1e-9:
        raise ValueError("Straight segment must have non-zero length")
    unit_x, unit_y = delta_x / length, delta_y / length
    pose_delta_x = float(current_pose["x"]) - start_x
    pose_delta_y = float(current_pose["y"]) - start_y
    along = pose_delta_x * unit_x + pose_delta_y * unit_y
    cross = unit_x * pose_delta_y - unit_y * pose_delta_x
    remaining = length - along
    endpoint_distance = math.hypot(
        float(current_pose["x"]) - end_x,
        float(current_pose["y"]) - end_y,
    )
    epsilon = max(0.0, float(overshoot_epsilon))
    return StraightSegmentProgress(
        segment_length=length,
        along_track=along,
        remaining_longitudinal=remaining,
        signed_cross_track=cross,
        endpoint_distance=endpoint_distance,
        passed_endpoint=along > length + epsilon,
    )


def straight_heading_lock(
    segment: ActiveSegment,
    current_pose: dict[str, float],
    *,
    heading_kp: float,
    cross_track_kp: float,
    maximum_angular: float,
    heading_deadband: float,
    cross_track_deadband: float,
    hard_heading_error: float,
    hard_cross_track: float,
) -> StraightControlDecision:
    """Return bounded line-lock steering independent of RPP curvature."""
    progress = straight_segment_progress(
        segment.effective_start,
        segment.endpoint,
        current_pose,
    )
    heading_error = _angle_delta(
        segment.fixed_heading,
        float(current_pose.get("yaw", segment.fixed_heading)),
    )
    if abs(heading_error) > abs(float(hard_heading_error)):
        return StraightControlDecision(
            0.0,
            False,
            heading_error,
            progress.signed_cross_track,
            "HEADING_ERROR_HARD_LIMIT",
        )
    if abs(progress.signed_cross_track) > abs(float(hard_cross_track)):
        return StraightControlDecision(
            0.0,
            False,
            heading_error,
            progress.signed_cross_track,
            "CROSS_TRACK_HARD_LIMIT",
        )
    heading_term = (
        0.0
        if abs(heading_error) <= abs(float(heading_deadband))
        else float(heading_kp) * heading_error
    )
    cross_term = (
        0.0
        if abs(progress.signed_cross_track) <= abs(float(cross_track_deadband))
        else -float(cross_track_kp) * progress.signed_cross_track
    )
    maximum = abs(float(maximum_angular))
    angular = max(-maximum, min(maximum, heading_term + cross_term))
    return StraightControlDecision(
        angular,
        True,
        heading_error,
        progress.signed_cross_track,
    )


def endpoint_braking_speed_limit(
    remaining_longitudinal: float,
    *,
    deceleration: float,
    reaction_time: float,
) -> float:
    """Maximum speed whose conservative stop envelope fits before the end."""
    remaining = max(0.0, float(remaining_longitudinal))
    decel = max(1e-6, float(deceleration))
    reaction = max(0.0, float(reaction_time))
    # Solve v*t + v^2/(2*a) = remaining for its non-negative root.
    return decel * (math.sqrt(reaction * reaction + 2.0 * remaining / decel) - reaction)


def segment_travel_watchdog(
    *,
    segment_length: float,
    elapsed: float,
    positive_travel: float,
    expected_speed: float,
    settle_allowance: float,
    travel_factor: float,
    minimum_travel_slack: float,
    time_factor: float,
) -> SegmentWatchdogDecision:
    """Bound controller lifetime and travel relative to active geometry."""
    length = max(0.0, float(segment_length))
    speed = max(0.05, abs(float(expected_speed)))
    elapsed_limit = (
        length / speed * max(1.0, float(time_factor))
        + max(0.0, float(settle_allowance))
    )
    travel_limit = (
        length * max(1.0, float(travel_factor))
        + max(0.0, float(minimum_travel_slack))
    )
    if float(positive_travel) > travel_limit:
        return SegmentWatchdogDecision(
            True, elapsed_limit, travel_limit, "POSITIVE_TRAVEL_LIMIT"
        )
    if float(elapsed) > elapsed_limit:
        return SegmentWatchdogDecision(
            True, elapsed_limit, travel_limit, "ELAPSED_TIME_LIMIT"
        )
    return SegmentWatchdogDecision(False, elapsed_limit, travel_limit)


def turn_hysteresis_transition(
    phase: str,
    heading_error: float,
    *,
    completion_tolerance: float,
    reentry_tolerance: float,
    stable_elapsed: float,
    stable_dwell: float,
) -> str:
    """Pure TURN/TURN_SETTLING transition with distinct enter/exit bands.

    The completion tolerance is the threshold for entering the zero-command
    settling phase.  Once there, the wider re-entry tolerance is the stable
    band: small passive drift must neither restart the motors nor prevent the
    segment from completing forever.
    """
    error = abs(float(heading_error))
    completion = abs(float(completion_tolerance))
    reentry = max(completion, abs(float(reentry_tolerance)))
    if phase == "TURN":
        return "TURN_SETTLING" if error <= completion else "TURN"
    if phase != "TURN_SETTLING":
        raise ValueError(f"Unsupported turn phase {phase!r}")
    if error > reentry:
        return "TURN"
    if float(stable_elapsed) >= max(0.0, float(stable_dwell)):
        return "STRAIGHT_PREPARE"
    return "TURN_SETTLING"


def post_turn_reanchor_requires_turn(
    heading_delta: float,
    cross_track: float,
    *,
    straight_entry_heading_limit: float,
    straight_entry_cross_track_limit: float,
) -> bool:
    """Decide whether a validated post-turn re-anchor needs another turn.

    The fixed-heading straight controller is authoritative inside these
    bounded entry limits.  This keeps a small localization/chassis shift from
    producing TURN -> re-anchor -> TURN for the same physical segment.
    """
    return bool(
        abs(float(heading_delta)) > abs(float(straight_entry_heading_limit))
        or abs(float(cross_track))
        > abs(float(straight_entry_cross_track_limit))
    )


def _finite_or_none(value: float) -> float | None:
    return round(float(value), 4) if math.isfinite(float(value)) else None


def _angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def execution_pose_continuity(
    previous_map: dict[str, float],
    current_map: dict[str, float],
    previous_odom: dict[str, float],
    current_odom: dict[str, float],
    *,
    maximum_translation_residual: float,
    maximum_yaw_residual: float,
) -> PoseContinuity:
    """Reject map-frame corrections that cannot be explained by odometry.

    Relative translation is expressed in the previous chassis frame before
    comparing the two coordinate systems. A fixed map->odom transform then
    cancels out, while an AMCL hypothesis jump remains visible as residual
    motion that the wheels did not report.
    """

    def local_translation(
        previous: dict[str, float], current: dict[str, float]
    ) -> tuple[float, float]:
        delta_x = float(current["x"]) - float(previous["x"])
        delta_y = float(current["y"]) - float(previous["y"])
        yaw = float(previous.get("yaw", 0.0))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    map_x, map_y = local_translation(previous_map, current_map)
    odom_x, odom_y = local_translation(previous_odom, current_odom)
    translation_residual = math.hypot(map_x - odom_x, map_y - odom_y)
    map_yaw_delta = _angle_delta(
        float(current_map.get("yaw", 0.0)),
        float(previous_map.get("yaw", 0.0)),
    )
    odom_yaw_delta = _angle_delta(
        float(current_odom.get("yaw", 0.0)),
        float(previous_odom.get("yaw", 0.0)),
    )
    yaw_residual = abs(_angle_delta(map_yaw_delta, odom_yaw_delta))
    return PoseContinuity(
        consistent=(
            translation_residual <= max(0.0, maximum_translation_residual)
            and yaw_residual <= max(0.0, maximum_yaw_residual)
        ),
        translation_residual=translation_residual,
        yaw_residual=yaw_residual,
        map_translation=math.hypot(map_x, map_y),
        odom_translation=math.hypot(odom_x, odom_y),
        map_yaw_delta=map_yaw_delta,
        odom_yaw_delta=odom_yaw_delta,
    )


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
    allow_monotonic_initial_overlap: bool = False,
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
    permitted_initial_overlap: set[tuple[int, int]] | None = None
    previous_overlap_count = 0
    last_overlap: tuple[
        tuple[str, int, float, float, float, int, float, float],
        dict[tuple[int, int], tuple[float, float, int]],
    ] | None = None

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
            sample_first_collision: tuple[
                str, int, float, float, float, int, float, float
            ] | None = None
            sample_collision_cells: dict[
                tuple[int, int], tuple[float, float, int]
            ] = {}
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
                sample_collision_cells[(center_column, center_row)] = (
                    collision_x,
                    collision_y,
                    center_cost,
                )
                if sample_first_collision is None:
                    sample_first_collision = (
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
                    sample_collision_cells[(column, row)] = (
                        collision_x,
                        collision_y,
                        cost,
                    )
                    if sample_first_collision is None:
                        sample_first_collision = (
                            code,
                            segment_index,
                            sample_x,
                            sample_y,
                            yaw,
                            cost,
                            collision_x,
                            collision_y,
                        )
            if allow_monotonic_initial_overlap:
                sample_cells = set(sample_collision_cells)
                if permitted_initial_overlap is None:
                    permitted_initial_overlap = sample_cells
                    previous_overlap_count = len(sample_cells)
                elif (
                    not sample_cells.issubset(permitted_initial_overlap)
                    or len(sample_cells) > previous_overlap_count
                ):
                    assert sample_first_collision is not None
                    first_collision = sample_first_collision
                    collision_cells = sample_collision_cells
                    break
                else:
                    previous_overlap_count = len(sample_cells)
                if sample_first_collision is not None:
                    last_overlap = (
                        sample_first_collision,
                        sample_collision_cells,
                    )
                else:
                    last_overlap = None
            elif sample_first_collision is not None:
                collision_cells.update(sample_collision_cells)
                if first_collision is None:
                    first_collision = sample_first_collision
        if first_collision is not None and allow_monotonic_initial_overlap:
            break
    if (
        allow_monotonic_initial_overlap
        and first_collision is None
        and last_overlap is not None
    ):
        # An escape must end at a collision-free footprint. Merely retaining
        # the same overlap all the way to the endpoint is not executable.
        first_collision, collision_cells = last_overlap
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
    direction: int = 0,
) -> RotationSweepValidation:
    """Exact sampled rectangular footprint check for an in-place rotation.

    The angular interval is bounded so the furthest chassis corner travels no
    more than half a map cell between samples.
    """
    delta = _angle_delta(float(end_yaw), float(start_yaw))
    if int(direction) > 0 and delta < 0.0:
        delta += 2.0 * math.pi
    elif int(direction) < 0 and delta > 0.0:
        delta -= 2.0 * math.pi
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


def choose_turn_direction(
    angular_error: float,
    *,
    left_static_safe: bool,
    right_static_safe: bool,
    left_live_safe: bool,
    right_live_safe: bool,
) -> int:
    """Choose shortest safe direction, then the statically/live-safe opposite."""
    preferred = 1 if float(angular_error) >= 0.0 else -1
    safe = {
        1: bool(left_static_safe and left_live_safe),
        -1: bool(right_static_safe and right_live_safe),
    }
    if safe[preferred]:
        return preferred
    if safe[-preferred]:
        return -preferred
    return 0


def preferred_turn_bay_directions(
    start: dict[str, float], goal: dict[str, float]
) -> tuple[int, ...]:
    """Prefer the straight relocation that progresses toward the destination.

    A positive direction is forward along the current chassis heading and a
    negative direction is reverse with the chassis heading held fixed. Reverse
    is eligible only when it lies in the destination half-plane; it is never a
    fallback for a destination ahead of the chassis.
    """
    yaw = float(start.get("yaw", 0.0))
    goal_delta_x = float(goal["x"]) - float(start["x"])
    goal_delta_y = float(goal["y"]) - float(start["y"])
    forward_progress = (
        goal_delta_x * math.cos(yaw) + goal_delta_y * math.sin(yaw)
    )
    return (1,) if forward_progress >= 0.0 else (-1, 1)


def validate_rotation_sweep_neighborhood(
    saved_map: "SavedOccupancyMap",
    x: float,
    y: float,
    start_yaw: float,
    end_yaw: float,
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
    robustness_radius: float = 0.0,
    allow_unknown: bool = False,
) -> RotationSweepValidation:
    """Validate a turn at the nominal point and a small pose-error ring.

    A route corner that is valid only at one exact floating-point position is
    not executable on the real chassis: localization and braking leave a few
    millimetres of position error.  The ring is intentionally independent of
    footprint padding so straight travel in a narrow corridor keeps the
    directional rectangular-footprint clearance it already proved.
    """
    radius = max(0.0, float(robustness_radius))
    offsets = [(0.0, 0.0)]
    if radius > 0.0:
        diagonal = radius / math.sqrt(2.0)
        offsets.extend((
            (radius, 0.0),
            (-radius, 0.0),
            (0.0, radius),
            (0.0, -radius),
            (diagonal, diagonal),
            (diagonal, -diagonal),
            (-diagonal, diagonal),
            (-diagonal, -diagonal),
        ))
    samples = 0
    for offset_x, offset_y in offsets:
        validation = validate_rotation_sweep(
            saved_map,
            float(x) + offset_x,
            float(y) + offset_y,
            start_yaw,
            end_yaw,
            half_length=half_length,
            half_width=half_width,
            padding=padding,
            allow_unknown=allow_unknown,
        )
        samples += validation.samples_checked
        if not validation.valid:
            return RotationSweepValidation(
                False,
                "TURN_SWEEP_NOT_ROBUST",
                samples,
                validation.collision_yaw,
            )
    return RotationSweepValidation(True, samples_checked=samples)


def validate_stop_turn_route(
    saved_map: "SavedOccupancyMap",
    points: Iterable[dict[str, float]],
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
    turn_robustness_radius: float = 0.0,
    segment_directions: Iterable[int] | None = None,
) -> ExecutablePathValidation:
    route = canonicalize_stop_turn_path(points)
    if len(route) < 2:
        return ExecutablePathValidation(False, "PATH_HAS_NO_LENGTH")
    directions = (
        tuple(1 for _ in range(len(route) - 1))
        if segment_directions is None
        else tuple(-1 if int(value) < 0 else 1 for value in segment_directions)
    )
    if len(directions) != len(route) - 1:
        return ExecutablePathValidation(False, "SEGMENT_DIRECTIONS_INVALID")
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
        if directions[corner_index - 1] < 0:
            incoming = _angle_delta(incoming + math.pi, 0.0)
        if directions[corner_index] < 0:
            outgoing = _angle_delta(outgoing + math.pi, 0.0)
        rotation = validate_rotation_sweep_neighborhood(
            saved_map,
            route[corner_index]["x"],
            route[corner_index]["y"],
            incoming,
            outgoing,
            half_length=half_length,
            half_width=half_width,
            padding=padding,
            robustness_radius=turn_robustness_radius,
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
    start_yaw: float | None = None,
    goal_yaw: float | None = None,
    segment_directions: Iterable[int] | None = None,
) -> RouteMetadata:
    route = canonicalize_stop_turn_path(points)
    directions = (
        tuple(1 for _ in range(max(0, len(route) - 1)))
        if segment_directions is None
        else tuple(-1 if int(value) < 0 else 1 for value in segment_directions)
    )
    if len(directions) != max(0, len(route) - 1):
        raise ValueError("segment_directions must match the canonical route")
    total_length = 0.0
    minimum_clearance = math.inf
    minimum_passage_width = math.inf
    minimum_side_clearance = math.inf
    minimum_turn_clearance = math.inf
    narrow_segments: list[dict[str, float]] = []
    internal_turn_count = 0
    internal_turn_angle = 0.0
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
            side_clearance = (
                -float(half_width)
                if left_width is None or right_width is None
                else min(left_width, right_width) - float(half_width)
            )
            minimum_side_clearance = min(
                minimum_side_clearance, side_clearance
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
        if directions[index - 1] < 0:
            incoming = _angle_delta(incoming + math.pi, 0.0)
        if directions[index] < 0:
            outgoing = _angle_delta(outgoing + math.pi, 0.0)
        angle = abs(_angle_delta(outgoing, incoming))
        if angle <= 1e-6:
            continue
        internal_turn_count += 1
        internal_turn_angle += angle
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
    initial_turn_angle = 0.0
    final_turn_angle = 0.0
    if len(route) >= 2:
        first_heading = math.atan2(
            route[1]["y"] - route[0]["y"],
            route[1]["x"] - route[0]["x"],
        )
        if directions[0] < 0:
            first_heading = _angle_delta(first_heading + math.pi, 0.0)
        if start_yaw is not None:
            initial_turn_angle = abs(_angle_delta(first_heading, start_yaw))
        last_heading = math.atan2(
            route[-1]["y"] - route[-2]["y"],
            route[-1]["x"] - route[-2]["x"],
        )
        if directions[-1] < 0:
            last_heading = _angle_delta(last_heading + math.pi, 0.0)
        if goal_yaw is not None:
            final_turn_angle = abs(_angle_delta(goal_yaw, last_heading))
    executed_turn_angles = (
        (initial_turn_angle if initial_turn_angle > 1e-6 else 0.0),
        internal_turn_angle,
        (final_turn_angle if final_turn_angle > 1e-6 else 0.0),
    )
    execution_total_turn_angle = sum(executed_turn_angles)
    turn_count = (
        internal_turn_count
        + int(initial_turn_angle > 1e-6)
        + int(final_turn_angle > 1e-6)
    )
    return RouteMetadata(
        total_length=total_length,
        minimum_passage_width=minimum_passage_width,
        minimum_static_clearance=minimum_clearance,
        minimum_turn_clearance=minimum_turn_clearance,
        turn_count=turn_count,
        total_turn_angle=execution_total_turn_angle,
        initial_turn_angle=initial_turn_angle,
        internal_turn_angle=internal_turn_angle,
        final_turn_angle=final_turn_angle,
        execution_total_turn_angle=execution_total_turn_angle,
        narrow_segments=tuple(narrow_segments),
        estimated_time=(
            total_length / max(0.01, linear_speed)
            + execution_total_turn_angle / max(0.01, angular_speed)
            # Every executed rotation has a stop/settle boundary. One
            # footprint length at nominal speed is a geometry-scaled lower
            # bound for that overhead and applies to initial/final rotations
            # as well as internal corners.
            + turn_count
            * (2.0 * (half_length + padding))
            / max(0.01, linear_speed)
        ),
        turn_safe=turn_safe,
        minimum_side_clearance=minimum_side_clearance,
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
        turn_robustness_radius: float = 0.01,
        turn_bay_max_distance: float = 0.80,
        hard_side_margin: float = 0.0,
        preferred_side_margin: float = 0.05,
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
        self.turn_robustness_radius = max(0.0, float(turn_robustness_radius))
        self.turn_bay_max_distance = max(0.0, float(turn_bay_max_distance))
        self.hard_side_margin = max(0.0, float(hard_side_margin))
        self.preferred_side_margin = max(
            self.hard_side_margin, float(preferred_side_margin)
        )
        self.heading_step = 2.0 * math.pi / self.HEADING_BINS

    def _turn_valid(
        self,
        x: float,
        y: float,
        start_yaw: float,
        end_yaw: float,
        *,
        robust: bool = True,
    ) -> bool:
        return validate_rotation_sweep_neighborhood(
            self.saved_map,
            x,
            y,
            start_yaw,
            end_yaw,
            half_length=self.half_length,
            half_width=self.half_width + self.hard_side_margin,
            padding=self.padding,
            robustness_radius=(self.turn_robustness_radius if robust else 0.0),
        ).valid

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
            half_width=(
                self.half_width + self.padding + self.hard_side_margin
            ),
            allow_unknown=False,
            lethal_threshold=65,
        ).valid

    def _segment_has_center_clearance(
        self,
        left: dict[str, float],
        right: dict[str, float],
        minimum_center_clearance: float | None,
    ) -> bool:
        """Keep seed simplification inside the clearance band it searched.

        Without this check, A* can find a centered series of cells and the
        line-of-sight simplifier can immediately cut back across the cells
        beside a wall, recreating the short but unexecutable route.
        """
        if minimum_center_clearance is None:
            return True
        delta_x = float(right["x"]) - float(left["x"])
        delta_y = float(right["y"]) - float(left["y"])
        distance = math.hypot(delta_x, delta_y)
        count = max(
            1,
            math.ceil(distance / max(0.001, self.saved_map.resolution / 2.0)),
        )
        for sample in range(1, count):
            ratio = sample / count
            cell = self.saved_map.world_to_cell(
                float(left["x"]) + delta_x * ratio,
                float(left["y"]) + delta_y * ratio,
            )
            if (
                cell is None
                or self.geometry.clearance_at_cell(*cell) + 1e-9
                < float(minimum_center_clearance)
            ):
                return False
        return True

    def _segment_has_side_clearance(
        self,
        left: dict[str, float],
        right: dict[str, float],
        minimum_side_clearance: float | None,
    ) -> bool:
        """Check the directional body-to-wall gap before simplifying a seed.

        Euclidean cell clearance is useful for finding a topology, but it is
        conservative around raster-cell corners and is not equivalent to the
        lateral clearance of an oriented rectangular chassis.  A shortcut can
        therefore miss the requested margin by millimetres even though
        retaining one seed waypoint keeps the vehicle centered.  Use the same
        lateral ray metric as final route metadata while choosing shortcuts.
        """
        if minimum_side_clearance is None:
            return True
        delta_x = float(right["x"]) - float(left["x"])
        delta_y = float(right["y"]) - float(left["y"])
        distance = math.hypot(delta_x, delta_y)
        yaw = math.atan2(delta_y, delta_x)
        count = max(
            1,
            math.ceil(distance / max(0.001, self.saved_map.resolution / 2.0)),
        )
        ray_limit = self.saved_map.resolution * math.hypot(
            self.saved_map.width, self.saved_map.height
        ) + self.saved_map.resolution
        for sample in range(count + 1):
            ratio = sample / count
            x = float(left["x"]) + delta_x * ratio
            y = float(left["y"]) + delta_y * ratio
            left_width = self.saved_map.raycast_static_range(
                x,
                y,
                yaw + math.pi / 2.0,
                minimum_range=0.0,
                maximum_range=ray_limit,
                unknown_is_blocked=True,
            )
            right_width = self.saved_map.raycast_static_range(
                x,
                y,
                yaw - math.pi / 2.0,
                minimum_range=0.0,
                maximum_range=ray_limit,
                unknown_is_blocked=True,
            )
            if (
                left_width is None
                or right_width is None
                or min(left_width, right_width) - self.half_width + 1e-9
                < float(minimum_side_clearance)
            ):
                return False
        return True

    def _turn_bay_candidate(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        exclusions: tuple[tuple[float, float, float], ...],
        deadline_monotonic: float | None,
    ) -> StopTurnRoute | None:
        """Try a bounded goal-aligned relocation before turning at a bay."""
        if self.turn_bay_max_distance <= 0.0:
            return None
        start_yaw = float(start.get("yaw", 0.0))
        goal_yaw = (
            float(goal["yaw"])
            if "yaw" in goal and goal["yaw"] is not None
            else None
        )
        step = max(self.saved_map.resolution, self.primitive_length)
        attempts = math.floor(self.turn_bay_max_distance / step)
        goal_cell = self.saved_map.world_to_cell(
            float(goal["x"]), float(goal["y"])
        )
        if goal_cell is None:
            return None

        def reverse_is_needed_for_turn(
            route: list[dict[str, float]],
        ) -> bool:
            """A reverse bay is legal only when the next turn cannot occur here."""
            if len(route) < 3:
                return False
            bay = route[1]
            following = route[2]
            target_heading = math.atan2(
                float(following["y"]) - float(bay["y"]),
                float(following["x"]) - float(bay["x"]),
            )
            return not any(
                validate_rotation_sweep(
                    self.saved_map,
                    float(start["x"]),
                    float(start["y"]),
                    start_yaw,
                    target_heading,
                    half_length=self.half_length,
                    half_width=self.half_width,
                    padding=self.padding,
                    direction=turn_direction,
                ).valid
                for turn_direction in (1, -1)
            )

        for direction in preferred_turn_bay_directions(start, goal):
            for index in range(1, attempts + 1):
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return None
                distance = direction * index * step
                bay = {
                    "x": float(start["x"]) + distance * math.cos(start_yaw),
                    "y": float(start["y"]) + distance * math.sin(start_yaw),
                    "yaw": start_yaw,
                }
                if self._segment_excluded(start, bay, exclusions):
                    continue

                simple_continuations = (
                    [bay, dict(goal)],
                    [
                        bay,
                        {"x": float(goal["x"]), "y": float(bay["y"])},
                        dict(goal),
                    ],
                    [
                        bay,
                        {"x": float(bay["x"]), "y": float(goal["y"])},
                        dict(goal),
                    ],
                )
                for continuation in simple_continuations:
                    combined = canonicalize_stop_turn_path(
                        [dict(start), *continuation]
                    )
                    if len(combined) != len(continuation) + 1:
                        continue
                    if any(
                        self._segment_excluded(left, right, exclusions)
                        for left, right in zip(combined, combined[1:])
                    ):
                        continue
                    if direction < 0 and not reverse_is_needed_for_turn(combined):
                        continue
                    result = self._route_result(
                        combined,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                        segment_directions=(
                            direction,
                            *(1 for _ in range(len(combined) - 2)),
                        ),
                    )
                    if result is not None:
                        return result

                bay_cell = self.saved_map.world_to_cell(bay["x"], bay["y"])
                if bay_cell is None:
                    continue
                for seed_clearance in self._clearance_levels():
                    seed = self._grid_seed(
                        bay_cell,
                        goal_cell,
                        exclusions,
                        deadline_monotonic,
                        minimum_center_clearance=seed_clearance,
                    )
                    continuation = self._canonical_route_from_seed(
                        seed,
                        bay,
                        goal,
                        exclusions,
                        deadline_monotonic,
                        minimum_center_clearance=seed_clearance,
                    )
                    if not continuation:
                        continue
                    combined = canonicalize_stop_turn_path([
                        dict(start),
                        *continuation,
                    ])
                    if len(combined) != len(continuation) + 1:
                        continue
                    if direction < 0 and not reverse_is_needed_for_turn(combined):
                        continue
                    result = self._route_result(
                        combined,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                        segment_directions=(
                            direction,
                            *(1 for _ in range(len(combined) - 2)),
                        ),
                    )
                    if result is not None:
                        return result
        return None

    def _grid_seed(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        exclusions: tuple[tuple[float, float, float], ...],
        deadline_monotonic: float | None = None,
        minimum_center_clearance: float | None = None,
    ) -> list[tuple[int, int]]:
        """Fast topology search; exact SE(2) checks remain authoritative."""
        start_index = self.geometry.index(*start_cell)
        goal_index = self.geometry.index(*goal_cell)
        queue: list[tuple[float, int]] = [(0.0, start_index)]
        costs = {start_index: 0.0}
        parents: dict[int, int | None] = {start_index: None}
        while queue:
            if (
                deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic
            ):
                return []
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
                    and (
                        not self.geometry.robot_navigable_mask[next_index]
                        or (
                            minimum_center_clearance is not None
                            and self.geometry.static_clearance[next_index] + 1e-9
                            < float(minimum_center_clearance)
                        )
                    )
                ):
                    continue
                if delta_x and delta_y:
                    side_a = self.geometry.index(column + delta_x, row)
                    side_b = self.geometry.index(column, row + delta_y)
                    if not (
                        self.geometry.robot_navigable_mask[side_a]
                        and self.geometry.robot_navigable_mask[side_b]
                        and (
                            minimum_center_clearance is None
                            or (
                                self.geometry.static_clearance[side_a] + 1e-9
                                >= float(minimum_center_clearance)
                                and self.geometry.static_clearance[side_b] + 1e-9
                                >= float(minimum_center_clearance)
                            )
                        )
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

    def _minimum_turn_grid_seed(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        exclusions: tuple[tuple[float, float, float], ...],
        deadline_monotonic: float | None = None,
        minimum_center_clearance: float | None = None,
    ) -> list[tuple[int, int]]:
        """Find a width-valid topology seed with few direction changes.

        The ordinary grid seed minimizes distance. That can select a short
        staircase around an obstacle even when a slightly longer route has
        one fewer stop-turn boundary. Direction is therefore part of this
        search state and the lexicographic cost minimizes grid turns before
        distance. Exact footprint, rotation-sweep, and route ranking checks
        remain authoritative after the seed is canonicalized.
        """
        directions = (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        )
        start_index = self.geometry.index(*start_cell)
        goal_index = self.geometry.index(*goal_cell)
        start_state = (start_index, -1)
        costs: dict[tuple[int, int], tuple[int, float]] = {
            start_state: (0, 0.0)
        }
        parents: dict[
            tuple[int, int], tuple[int, int] | None
        ] = {start_state: None}
        queue: list[
            tuple[float, float, float, int, tuple[int, int]]
        ] = []
        sequence = 0
        initial_heuristic = math.hypot(
            goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1]
        )
        heapq.heappush(
            queue,
            (0.0, initial_heuristic, 0.0, sequence, start_state),
        )
        goal_state: tuple[int, int] | None = None
        while queue:
            if (
                deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic
            ):
                return []
            turns, _, distance, _, state = heapq.heappop(queue)
            best_cost = costs.get(state)
            if best_cost is None or (int(turns), distance) != best_cost:
                continue
            index, previous_direction = state
            if index == goal_index:
                goal_state = state
                break
            row, column = divmod(index, self.saved_map.width)
            for direction_index, (delta_x, delta_y) in enumerate(directions):
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
                    and (
                        not self.geometry.robot_navigable_mask[next_index]
                        or (
                            minimum_center_clearance is not None
                            and self.geometry.static_clearance[next_index] + 1e-9
                            < float(minimum_center_clearance)
                        )
                    )
                ):
                    continue
                if delta_x and delta_y:
                    side_a = self.geometry.index(column + delta_x, row)
                    side_b = self.geometry.index(column, row + delta_y)
                    if not (
                        self.geometry.robot_navigable_mask[side_a]
                        and self.geometry.robot_navigable_mask[side_b]
                        and (
                            minimum_center_clearance is None
                            or (
                                self.geometry.static_clearance[side_a] + 1e-9
                                >= float(minimum_center_clearance)
                                and self.geometry.static_clearance[side_b] + 1e-9
                                >= float(minimum_center_clearance)
                            )
                        )
                    ):
                        continue
                world_x, world_y = self.saved_map.cell_center(
                    next_column, next_row
                )
                if self._excluded(world_x, world_y, exclusions):
                    continue
                next_turns = int(turns) + int(
                    previous_direction >= 0
                    and previous_direction != direction_index
                )
                next_distance = distance + math.hypot(delta_x, delta_y)
                next_state = (next_index, direction_index)
                next_cost = (next_turns, next_distance)
                if next_cost >= costs.get(next_state, (math.inf, math.inf)):
                    continue
                costs[next_state] = next_cost
                parents[next_state] = state
                heuristic = math.hypot(
                    goal_cell[0] - next_column,
                    goal_cell[1] - next_row,
                )
                sequence += 1
                heapq.heappush(
                    queue,
                    (
                        float(next_turns),
                        next_distance + heuristic,
                        next_distance,
                        sequence,
                        next_state,
                    ),
                )
        if goal_state is None:
            return []
        output: list[tuple[int, int]] = []
        cursor: tuple[int, int] | None = goal_state
        while cursor is not None:
            row, column = divmod(cursor[0], self.saved_map.width)
            output.append((column, row))
            cursor = parents[cursor]
        output.reverse()
        return output

    def _clearance_levels(self) -> tuple[float, ...]:
        """Center-clearance bands, widest first, down to the hard reserve."""
        preferred = self.half_width + self.padding + self.preferred_side_margin
        hard = self.half_width + self.padding + self.hard_side_margin
        if preferred <= hard + 1e-9:
            return (hard,)
        # One-centimetre bands are finer than the 5 cm Saved Map cells while
        # keeping recovery planning bounded. The first exact-valid band wins;
        # distance and minimum-turn seeds compete only within that width class.
        step = 0.01
        count = max(1, math.ceil((preferred - hard) / step))
        levels = [max(hard, preferred - index * step) for index in range(count + 1)]
        levels[-1] = hard
        return tuple(dict.fromkeys(round(value, 6) for value in levels))

    def _single_turn_visibility_candidate(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        exclusions: tuple[tuple[float, float, float], ...],
        *,
        maximum_total_length: float,
        maximum_turn_count: int,
        minimum_center_clearance: float,
        start_yaw: float,
        goal_yaw: float | None,
        deadline_monotonic: float | None = None,
    ) -> StopTurnRoute | None:
        """Move one turn waypoint off the grid seed when both legs are visible.

        Eight-connected seeds can represent an oblique leg only as alternating
        grid headings. A useful one-turn waypoint may therefore lie beside,
        rather than on, both the shortest-distance and minimum-grid-turn
        seeds. Search turn-safe cell centers inside a bounded path-length
        ellipse, then retain only exact-valid routes in the same clearance
        band. The length bound is supplied by the caller so removing one stop
        cannot justify an arbitrarily large detour.
        """
        start_x = float(start["x"])
        start_y = float(start["y"])
        goal_x = float(goal["x"])
        goal_y = float(goal["y"])
        required_side_clearance = max(
            0.0,
            float(minimum_center_clearance)
            - self.half_width
            - self.padding,
        )
        best: StopTurnRoute | None = None
        for row in range(self.saved_map.height):
            for column in range(self.saved_map.width):
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return best
                index = self.geometry.index(column, row)
                if (
                    not self.geometry.turn_safe_mask[index]
                    or self.geometry.static_clearance[index] + 1e-9
                    < float(minimum_center_clearance)
                ):
                    continue
                waypoint_x, waypoint_y = self.saved_map.cell_center(column, row)
                if (
                    math.hypot(waypoint_x - start_x, waypoint_y - start_y)
                    + math.hypot(goal_x - waypoint_x, goal_y - waypoint_y)
                    > float(maximum_total_length) + 1e-9
                ):
                    continue
                waypoint = {"x": waypoint_x, "y": waypoint_y}
                if (
                    self._excluded(waypoint_x, waypoint_y, exclusions)
                    or self._segment_excluded(start, waypoint, exclusions)
                    or self._segment_excluded(waypoint, goal, exclusions)
                    or not self._segment_has_center_clearance(
                        start,
                        waypoint,
                        minimum_center_clearance,
                    )
                    or not self._segment_has_center_clearance(
                        waypoint,
                        goal,
                        minimum_center_clearance,
                    )
                ):
                    continue
                result = self._route_result(
                    [
                        {"x": start_x, "y": start_y},
                        waypoint,
                        {"x": goal_x, "y": goal_y},
                    ],
                    start_yaw=start_yaw,
                    goal_yaw=goal_yaw,
                )
                if (
                    result is None
                    or result.metadata.turn_count > int(maximum_turn_count)
                    or result.metadata.minimum_side_clearance + 1e-9
                    < required_side_clearance
                ):
                    continue
                if best is None or self.ranking_key(result) < self.ranking_key(best):
                    best = result
        return best

    def _reduce_one_route_corner(
        self,
        baseline: StopTurnRoute,
        exclusions: tuple[tuple[float, float, float], ...],
        *,
        maximum_total_length: float,
        minimum_center_clearance: float,
        start_yaw: float,
        goal_yaw: float | None,
        deadline_monotonic: float | None = None,
    ) -> StopTurnRoute | None:
        """Replace two adjacent waypoints with one exact-valid waypoint.

        Grid visibility simplification can leave a short sequence of shallow
        turns around a raster obstacle. Merely deleting either waypoint may
        be invalid even though moving their shared corner slightly produces a
        safe route with one fewer stop/turn cycle. Search turn-safe cell
        centers inside the same clearance band and retain the shortest bounded
        exact-valid replacement.
        """
        points = [dict(point) for point in baseline.points]
        if len(points) < 4:
            return None
        required_side_clearance = max(
            0.0,
            float(minimum_center_clearance)
            - self.half_width
            - self.padding,
        )
        best: StopTurnRoute | None = None
        for first_removed in range(1, len(points) - 2):
            left = points[first_removed - 1]
            right = points[first_removed + 2]
            visible_replacements: list[tuple[float, dict[str, float]]] = []
            old_local_length = sum(
                math.hypot(
                    float(segment_right["x"]) - float(segment_left["x"]),
                    float(segment_right["y"]) - float(segment_left["y"]),
                )
                for segment_left, segment_right in zip(
                    points[first_removed - 1:first_removed + 2],
                    points[first_removed:first_removed + 3],
                )
            )
            for row in range(self.saved_map.height):
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return best
                for column in range(self.saved_map.width):
                    index = self.geometry.index(column, row)
                    if (
                        not self.geometry.turn_safe_mask[index]
                        or self.geometry.static_clearance[index] + 1e-9
                        < float(minimum_center_clearance)
                    ):
                        continue
                    waypoint_x, waypoint_y = self.saved_map.cell_center(column, row)
                    replacement_local_length = (
                        math.hypot(
                            waypoint_x - float(left["x"]),
                            waypoint_y - float(left["y"]),
                        )
                        + math.hypot(
                            float(right["x"]) - waypoint_x,
                            float(right["y"]) - waypoint_y,
                        )
                    )
                    if replacement_local_length > old_local_length + 0.30 + 1e-9:
                        continue
                    waypoint = {"x": waypoint_x, "y": waypoint_y}
                    if (
                        self._excluded(waypoint_x, waypoint_y, exclusions)
                        or self._segment_excluded(left, waypoint, exclusions)
                        or self._segment_excluded(waypoint, right, exclusions)
                        or not self._segment_has_center_clearance(
                            left, waypoint, minimum_center_clearance
                        )
                        or not self._segment_has_center_clearance(
                            waypoint, right, minimum_center_clearance
                        )
                    ):
                        continue
                    visible_replacements.append(
                        (replacement_local_length, waypoint)
                    )
            # Full swept-footprint and rotation validation is deliberately
            # bounded. The visibility pass above can yield thousands of cell
            # centers in an open room; shortest replacements are the useful
            # candidates and keep the stop-turn planning budget deterministic.
            visible_replacements.sort(key=lambda item: item[0])
            for _, waypoint in visible_replacements[:96]:
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return best
                candidate = [
                    *points[:first_removed],
                    waypoint,
                    *points[first_removed + 2:],
                ]
                result = self._route_result(
                    candidate,
                    start_yaw=start_yaw,
                    goal_yaw=goal_yaw,
                )
                if (
                    result is None
                    or result.metadata.turn_count
                    >= baseline.metadata.turn_count
                    or result.metadata.total_length
                    > float(maximum_total_length) + 1e-9
                    or result.metadata.minimum_side_clearance + 1e-9
                    < required_side_clearance
                ):
                    continue
                if best is None or self.ranking_key(result) < self.ranking_key(best):
                    best = result
        return best

    def _canonical_route_from_seed(
        self,
        seed: list[tuple[int, int]],
        start: dict[str, float],
        goal: dict[str, float],
        exclusions: tuple[tuple[float, float, float], ...] = (),
        deadline_monotonic: float | None = None,
        minimum_center_clearance: float | None = None,
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
            if (
                deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic
            ):
                return None
            incoming_bin = self.heading_bin(incoming_yaw)
            memo_key = (index, incoming_bin)
            if memo_key in failed:
                return None
            for following in range(len(raw) - 1, index, -1):
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return None
                outgoing_yaw = math.atan2(
                    raw[following]["y"] - raw[index]["y"],
                    raw[following]["x"] - raw[index]["x"],
                )
                if self._segment_excluded(raw[index], raw[following], exclusions):
                    continue
                if not self._segment_has_center_clearance(
                    raw[index],
                    raw[following],
                    minimum_center_clearance,
                ):
                    continue
                edge = (index, following)
                if index > 0 and abs(_angle_delta(outgoing_yaw, incoming_yaw)) > math.radians(1.0):
                    if self._excluded(raw[index]["x"], raw[index]["y"], exclusions):
                        continue
                    turn_key = (index, self.heading_bin(outgoing_yaw))
                    turn_valid = turn_cache.get(turn_key)
                    if turn_valid is None:
                        turn_valid = self._turn_valid(
                            raw[index]["x"],
                            raw[index]["y"],
                            incoming_yaw,
                            outgoing_yaw,
                            robust=False,
                        )
                        turn_cache[turn_key] = turn_valid
                    if not turn_valid:
                        continue
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
        segment_directions: Iterable[int] | None = None,
    ) -> StopTurnRoute | None:
        if len(route) < 2:
            return None
        directions = (
            tuple(1 for _ in range(len(route) - 1))
            if segment_directions is None
            else tuple(
                -1 if int(value) < 0 else 1
                for value in segment_directions
            )
        )
        if len(directions) != len(route) - 1:
            return None
        first_heading = math.atan2(
            route[1]["y"] - route[0]["y"],
            route[1]["x"] - route[0]["x"],
        )
        if directions[0] < 0:
            first_heading = _angle_delta(first_heading + math.pi, 0.0)
        if start_yaw is not None and not validate_rotation_sweep(
            self.saved_map,
            route[0]["x"],
            route[0]["y"],
            float(start_yaw),
            first_heading,
            half_length=self.half_length,
            # The initial pose is measured, not selected by the planner. Match
            # the executor's exact physical-footprint turn check here; applying
            # the translation side reserve can reject a safe current pose and
            # force the lattice to exhaust its time budget. Planned internal
            # corners still retain hard_side_margin below.
            half_width=self.half_width,
            padding=self.padding,
        ).valid:
            return None
        last_heading = math.atan2(
            route[-1]["y"] - route[-2]["y"],
            route[-1]["x"] - route[-2]["x"],
        )
        if directions[-1] < 0:
            last_heading = _angle_delta(last_heading + math.pi, 0.0)
        if goal_yaw is not None and not validate_rotation_sweep(
            self.saved_map,
            route[-1]["x"],
            route[-1]["y"],
            last_heading,
            float(goal_yaw),
            half_length=self.half_length,
            # Final in-place execution uses the same physical sweep authority.
            half_width=self.half_width,
            padding=self.padding,
        ).valid:
            return None
        validation = validate_stop_turn_route(
            self.saved_map,
            route,
            half_length=self.half_length,
            half_width=self.half_width + self.hard_side_margin,
            padding=self.padding,
            segment_directions=directions,
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
            start_yaw=start_yaw,
            goal_yaw=goal_yaw,
            segment_directions=directions,
        )
        headings = tuple(
            self.heading_bin(
                math.atan2(
                    right["y"] - left["y"], right["x"] - left["x"]
                ) + (math.pi if direction < 0 else 0.0)
            )
            for direction, (left, right) in zip(
                directions, zip(route, route[1:])
            )
        )
        return StopTurnRoute(tuple(route), metadata, headings, directions)

    def _widen_route_with_one_waypoint(
        self,
        baseline: StopTurnRoute,
        exclusions: tuple[tuple[float, float, float], ...],
        *,
        required_side_clearance: float,
        start_yaw: float,
        goal_yaw: float | None,
        deadline_monotonic: float | None = None,
    ) -> StopTurnRoute | None:
        """Undo an over-aggressive shortcut when it loses lateral margin.

        A centered grid seed can be simplified into one shallow diagonal that
        saves a corner but clips a narrow passage by a few millimetres. Search
        one bounded corrective waypoint around the worst segment. Exact body,
        turn-sweep and lateral-clearance checks remain authoritative.
        """
        required = max(0.0, float(required_side_clearance))
        if baseline.metadata.minimum_side_clearance + 1e-9 >= required:
            return baseline
        points = [dict(point) for point in baseline.points]
        best: StopTurnRoute | None = None
        maximum_length = baseline.metadata.total_length + 2.0 * (
            self.half_length + self.padding
        )
        ray_limit = self.saved_map.resolution * math.hypot(
            self.saved_map.width, self.saved_map.height
        ) + self.saved_map.resolution
        offset_step = max(0.005, self.saved_map.resolution / 10.0)
        maximum_offset = min(0.08, 4.0 * self.saved_map.resolution)

        for segment_index, (left, right) in enumerate(zip(points, points[1:])):
            delta_x = float(right["x"]) - float(left["x"])
            delta_y = float(right["y"]) - float(left["y"])
            distance = math.hypot(delta_x, delta_y)
            if distance <= max(0.10, 2.0 * self.saved_map.resolution):
                continue
            yaw = math.atan2(delta_y, delta_x)
            normal_x = -delta_y / distance
            normal_y = delta_x / distance
            count = max(
                1,
                math.ceil(
                    distance / max(0.001, self.saved_map.resolution / 2.0)
                ),
            )
            worst: tuple[float, float, float] | None = None
            for sample in range(count + 1):
                ratio = sample / count
                x = float(left["x"]) + delta_x * ratio
                y = float(left["y"]) + delta_y * ratio
                left_width = self.saved_map.raycast_static_range(
                    x,
                    y,
                    yaw + math.pi / 2.0,
                    minimum_range=0.0,
                    maximum_range=ray_limit,
                    unknown_is_blocked=True,
                )
                right_width = self.saved_map.raycast_static_range(
                    x,
                    y,
                    yaw - math.pi / 2.0,
                    minimum_range=0.0,
                    maximum_range=ray_limit,
                    unknown_is_blocked=True,
                )
                clearance = (
                    -self.half_width
                    if left_width is None or right_width is None
                    else min(left_width, right_width) - self.half_width
                )
                if worst is None or clearance < worst[0]:
                    centering_offset = (
                        0.0
                        if left_width is None or right_width is None
                        else (left_width - right_width) / 2.0
                    )
                    worst = (clearance, ratio, centering_offset)
            if worst is None or worst[0] + 1e-9 >= required:
                continue

            ratio_candidates = [index / 20.0 for index in range(1, 20)]
            ratio_candidates.sort(key=lambda value: abs(value - worst[1]))
            predicted = max(-maximum_offset, min(maximum_offset, worst[2]))
            offsets = [predicted]
            steps = max(1, math.ceil(maximum_offset / offset_step))
            offsets.extend(
                direction * index * offset_step
                for index in range(1, steps + 1)
                for direction in (1.0, -1.0)
            )
            offsets = list(dict.fromkeys(round(value, 6) for value in offsets))
            offsets.sort(key=lambda value: abs(value - predicted))

            for ratio in ratio_candidates:
                if (
                    deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic
                ):
                    return best
                if min(ratio, 1.0 - ratio) * distance < 0.10:
                    continue
                base_x = float(left["x"]) + delta_x * ratio
                base_y = float(left["y"]) + delta_y * ratio
                for offset in offsets:
                    if abs(offset) < 1e-9:
                        continue
                    waypoint = {
                        "x": base_x + normal_x * offset,
                        "y": base_y + normal_y * offset,
                    }
                    if (
                        self._excluded(waypoint["x"], waypoint["y"], exclusions)
                        or self._segment_excluded(left, waypoint, exclusions)
                        or self._segment_excluded(waypoint, right, exclusions)
                        or not self._segment_has_side_clearance(
                            left, waypoint, required
                        )
                        or not self._segment_has_side_clearance(
                            waypoint, right, required
                        )
                    ):
                        continue
                    candidate_points = canonicalize_stop_turn_path([
                        *points[: segment_index + 1],
                        waypoint,
                        *points[segment_index + 1 :],
                    ])
                    candidate = self._route_result(
                        candidate_points,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                    )
                    if (
                        candidate is None
                        or candidate.metadata.total_length
                        > maximum_length + 1e-9
                        or candidate.metadata.minimum_side_clearance + 1e-9
                        < required
                    ):
                        continue
                    if best is None or self.ranking_key(candidate) < self.ranking_key(best):
                        best = candidate
        return best

    def plan(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        *,
        exclusions: Iterable[tuple[float, float, float]] = (),
        deadline_monotonic: float | None = None,
    ) -> StopTurnRoute | None:
        self._last_plan_expansions = 0
        self._last_plan_limit = ""
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
        direct_points = [
            {"x": start_x, "y": start_y},
            {"x": goal_x, "y": goal_y},
        ]
        candidate_pool: list[StopTurnRoute] = []
        if not self._segment_excluded(direct_points[0], direct_points[1], forbidden):
            direct = self._route_result(
                direct_points, start_yaw=start_yaw, goal_yaw=goal_yaw
            )
            if direct is not None:
                candidate_pool.append(direct)

        for simple in (
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
                candidate_pool.append(result)
        seeded_result: StopTurnRoute | None = None
        for seed_clearance in self._clearance_levels():
            level_results: list[StopTurnRoute] = []
            seen_routes: set[tuple[tuple[float, float], ...]] = set()
            for seed_search in (
                self._minimum_turn_grid_seed,
                self._grid_seed,
            ):
                seed = seed_search(
                    start_cell,
                    goal_cell,
                    forbidden,
                    deadline_monotonic,
                    minimum_center_clearance=seed_clearance,
                )
                seeded_route = self._canonical_route_from_seed(
                    seed,
                    start,
                    goal,
                    forbidden,
                    deadline_monotonic,
                    minimum_center_clearance=seed_clearance,
                )
                if not seeded_route:
                    continue
                route_key = tuple(
                    (round(point["x"], 6), round(point["y"], 6))
                    for point in seeded_route
                )
                if route_key in seen_routes:
                    continue
                seen_routes.add(route_key)
                result = self._route_result(
                    seeded_route,
                    start_yaw=start_yaw,
                    goal_yaw=goal_yaw,
                )
                if result is not None:
                    level_results.append(result)
            if level_results:
                baseline = min(level_results, key=self.ranking_key)
                maximum_reduced_length = (
                    min(
                        result.metadata.total_length
                        for result in level_results
                    )
                    + 2.0 * (self.half_length + self.padding)
                )
                if len(baseline.points) > 3:
                    # One removed stop has a geometry-scaled execution
                    # overhead equivalent to one chassis length of travel.
                    # Use that as the maximum extra distance worth searching.
                    single_turn = self._single_turn_visibility_candidate(
                        start,
                        goal,
                        forbidden,
                        maximum_total_length=(
                            maximum_reduced_length
                        ),
                        maximum_turn_count=baseline.metadata.turn_count - 1,
                        minimum_center_clearance=seed_clearance,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                        deadline_monotonic=deadline_monotonic,
                    )
                    if single_turn is not None:
                        level_results.append(single_turn)
                reduced = min(level_results, key=self.ranking_key)
                while len(reduced.points) > 3:
                    replacement = self._reduce_one_route_corner(
                        reduced,
                        forbidden,
                        maximum_total_length=maximum_reduced_length,
                        minimum_center_clearance=seed_clearance,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                        deadline_monotonic=deadline_monotonic,
                    )
                    if replacement is None:
                        break
                    level_results.append(replacement)
                    reduced = replacement
                candidate_pool.extend(level_results)
                seeded_result = min(level_results, key=self.ranking_key)
                break
        # The 8-connected topology seed exposes oblique alternatives to both
        # Manhattan routes.  The full heading lattice is only needed if that
        # exact candidate cannot be built.
        if seeded_result is not None:
            selected = min(candidate_pool, key=self.ranking_key)
            if (
                selected.metadata.minimum_side_clearance + 1e-9
                < self.preferred_side_margin
            ):
                widened = self._widen_route_with_one_waypoint(
                    selected,
                    forbidden,
                    required_side_clearance=self.preferred_side_margin,
                    start_yaw=start_yaw,
                    goal_yaw=goal_yaw,
                    deadline_monotonic=deadline_monotonic,
                )
                if widened is not None:
                    return widened
            return selected
        turn_bay_result = self._turn_bay_candidate(
            start,
            goal,
            forbidden,
            deadline_monotonic,
        )
        if turn_bay_result is not None:
            candidate_pool.append(turn_bay_result)
            return min(candidate_pool, key=self.ranking_key)
        if (
            deadline_monotonic is not None
            and time.monotonic() >= deadline_monotonic
        ):
            return min(candidate_pool, key=self.ranking_key) if candidate_pool else None
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
        expansions = 0
        while (
            queue
            and expansions < self.max_expansions
            and (
                deadline_monotonic is None
                or time.monotonic() < deadline_monotonic
            )
        ):
            _, _, state = heapq.heappop(queue)
            state_cost = costs[state]
            x, y = positions[state]
            heading_bin = state[2]
            yaw = self.heading(heading_bin)
            expansions += 1
            self._last_plan_expansions = expansions
            goal_distance = math.hypot(goal_x - x, goal_y - y)
            if goal_distance <= self.primitive_length * 2.5:
                goal_heading = math.atan2(goal_y - y, goal_x - x)
                rotation_valid = self._turn_valid(
                    x,
                    y,
                    yaw,
                    goal_heading,
                    robust=True,
                )
                direct = {"x": goal_x, "y": goal_y}
                if (
                    rotation_valid
                    and not self._excluded(goal_x, goal_y, forbidden)
                    and not self._segment_excluded(
                        {"x": x, "y": y}, direct, forbidden
                    )
                    and self._translation_valid({"x": x, "y": y}, direct)
                ):
                    states: list[tuple[int, int, int]] = []
                    cursor: tuple[int, int, int] | None = state
                    while cursor is not None:
                        states.append(cursor)
                        cursor = parents[cursor]
                    states.reverse()
                    candidate = canonicalize_stop_turn_path([
                        *(
                            {"x": positions[item][0], "y": positions[item][1]}
                            for item in states
                        ),
                        direct,
                    ])
                    lattice_result = self._route_result(
                        candidate,
                        start_yaw=start_yaw,
                        goal_yaw=goal_yaw,
                    )
                    if lattice_result is not None:
                        candidate_pool.append(lattice_result)
                        return min(candidate_pool, key=self.ranking_key)

            next_x = x + self.primitive_length * math.cos(yaw)
            next_y = y + self.primitive_length * math.sin(yaw)
            next_cell = self.saved_map.world_to_cell(next_x, next_y)
            if (
                next_cell is not None
                and not self._excluded(next_x, next_y, forbidden)
                and not self._segment_excluded(
                    {"x": x, "y": y},
                    {"x": next_x, "y": next_y},
                    forbidden,
                )
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
                if self._excluded(x, y, forbidden):
                    continue
                next_heading = (heading_bin + direction) % self.HEADING_BINS
                next_key = self._pose_key(x, y, next_heading)
                cache_key = (state, next_heading)
                valid = rotation_cache.get(cache_key)
                if valid is None:
                    valid = self._turn_valid(
                        x,
                        y,
                        yaw,
                        self.heading(next_heading),
                        robust=False,
                    )
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
        if expansions >= self.max_expansions:
            self._last_plan_limit = "SEARCH_EXPANSION_LIMIT"
        elif (
            deadline_monotonic is not None
            and time.monotonic() >= deadline_monotonic
        ):
            self._last_plan_limit = "SEARCH_TIME_BUDGET_EXCEEDED"
        return min(candidate_pool, key=self.ranking_key) if candidate_pool else None

    def plan_result(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        *,
        exclusions: Iterable[tuple[float, float, float]] = (),
        planning_time_budget: float | None = None,
        allow_start_escape: bool = False,
        maximum_start_escape_distance: float = 0.60,
        live_start_clear: bool = True,
    ) -> PlannerResult:
        """Plan with explicit start/goal/search/dynamic failure semantics."""
        started = time.monotonic()
        deadline = (
            None
            if planning_time_budget is None
            else started + max(0.0, float(planning_time_budget))
        )
        start_cell = self.saved_map.world_to_cell(
            float(start["x"]), float(start["y"])
        )
        if start_cell is None:
            return PlannerResult(
                "START_OUTSIDE_MAP",
                reason="START_OUTSIDE_MAP",
                message="Current robot center is outside the Saved Map",
            )
        goal_cell = self.saved_map.world_to_cell(float(goal["x"]), float(goal["y"]))
        if goal_cell is None:
            return PlannerResult(
                "GOAL_OUTSIDE_MAP",
                reason="GOAL_OUTSIDE_MAP",
                message="Goal is outside the Saved Map",
            )
        initial_same_component = self.geometry.same_component(start_cell, goal_cell)
        if not self.geometry.robot_navigable_mask[
            self.geometry.index(*goal_cell)
        ]:
            return PlannerResult(
                "GOAL_INVALID",
                reason="GOAL_INVALID",
                message="Goal has no translation-safe robot footprint",
            )
        start_validation = self.saved_map.validate_footprint(
            float(start["x"]),
            float(start["y"]),
            float(start.get("yaw", 0.0)),
            half_length=self.half_length,
            half_width=self.half_width,
            padding=self.padding,
            allow_unknown=False,
            code_prefix="START",
        )
        escape: StartEscape | None = None
        planning_start = dict(start)
        if not start_validation.valid:
            if start_validation.code == "START_FOOTPRINT_OUTSIDE_MAP":
                return PlannerResult(
                    "START_OUTSIDE_MAP",
                    reason="START_OUTSIDE_MAP",
                    message=start_validation.message,
                )
            escape = find_start_escape(
                self.saved_map,
                start,
                half_length=self.half_length,
                half_width=self.half_width,
                padding=self.padding,
                maximum_distance=maximum_start_escape_distance,
                # Start-overlap recovery is forward-only. Reverse motion is
                # reserved for turn-bay recovery after a turn is proven
                # blocked and the destination lies behind the chassis.
                directions=(1,),
            )
            escape_cell = (
                None
                if escape is None
                else self.saved_map.world_to_cell(
                    float(escape.end["x"]), float(escape.end["y"])
                )
            )
            if not allow_start_escape:
                return PlannerResult(
                    "START_STATIC_OVERLAP",
                    reason="START_STATIC_OVERLAP",
                    message="Saved Map raster overlaps the initial footprint",
                    start_escape=escape,
                    elapsed_seconds=time.monotonic() - started,
                )
            if not live_start_clear or escape is None:
                return PlannerResult(
                    "START_ESCAPE_UNAVAILABLE",
                    reason=(
                        "DYNAMICALLY_BLOCKED"
                        if not live_start_clear
                        else "START_ESCAPE_UNAVAILABLE"
                    ),
                    message="No bounded straight start escape is executable",
                    start_escape=escape,
                    elapsed_seconds=time.monotonic() - started,
                )
            if (
                not initial_same_component
                and (
                    escape_cell is None
                    or not self.geometry.same_component(escape_cell, goal_cell)
                )
            ):
                return PlannerResult(
                    "GOAL_DISCONNECTED",
                    reason="GOAL_DISCONNECTED",
                    message="Goal is outside the translation-reachable component",
                    start_escape=escape,
                )
            planning_start = {
                "x": escape.end["x"],
                "y": escape.end["y"],
                "yaw": escape.yaw,
            }
        elif not initial_same_component:
            return PlannerResult(
                "GOAL_DISCONNECTED",
                reason="GOAL_DISCONNECTED",
                message="Goal is outside the translation-reachable component",
            )
        forbidden = tuple(
            (float(x), float(y), max(0.0, float(radius)))
            for x, y, radius in exclusions
        )
        planning_start_cell = self.saved_map.world_to_cell(
            float(planning_start["x"]), float(planning_start["y"])
        )
        if forbidden and planning_start_cell is not None:
            static_seed = self._grid_seed(
                planning_start_cell,
                goal_cell,
                (),
                deadline,
                minimum_center_clearance=(
                    self.half_width + self.padding + self.hard_side_margin
                ),
            )
            dynamic_seed = self._grid_seed(
                planning_start_cell,
                goal_cell,
                forbidden,
                deadline,
                minimum_center_clearance=(
                    self.half_width + self.padding + self.hard_side_margin
                ),
            )
            if (
                static_seed
                and not dynamic_seed
                and (deadline is None or time.monotonic() < deadline)
            ):
                return PlannerResult(
                    "DYNAMICALLY_BLOCKED",
                    reason="DYNAMICALLY_BLOCKED",
                    message="Temporary obstacles disconnect every current route",
                    start_escape=escape,
                    elapsed_seconds=time.monotonic() - started,
                )
        route = self.plan(
            planning_start,
            goal,
            exclusions=forbidden,
            deadline_monotonic=deadline,
        )
        elapsed = time.monotonic() - started
        expansions = int(getattr(self, "_last_plan_expansions", 0))
        if route is not None:
            return PlannerResult(
                "SUCCESS",
                route=route,
                start_escape=escape,
                expansions=expansions,
                elapsed_seconds=elapsed,
            )
        limit = str(getattr(self, "_last_plan_limit", ""))
        if limit:
            return PlannerResult(
                limit,
                reason=limit,
                message="Stop-turn search reached its configured bound",
                start_escape=escape,
                expansions=expansions,
                elapsed_seconds=elapsed,
            )
        if forbidden:
            static_route = self.plan(
                planning_start,
                goal,
                exclusions=(),
                deadline_monotonic=deadline,
            )
            if static_route is not None:
                return PlannerResult(
                    "DYNAMICALLY_BLOCKED",
                    reason="DYNAMICALLY_BLOCKED",
                    message="Temporary obstacles block every current route",
                    start_escape=escape,
                    expansions=expansions,
                    elapsed_seconds=time.monotonic() - started,
                )
        start_yaw = float(planning_start.get("yaw", 0.0))
        forward_probe = {
            "x": float(planning_start["x"]) + self.primitive_length * math.cos(start_yaw),
            "y": float(planning_start["y"]) + self.primitive_length * math.sin(start_yaw),
        }
        forward_valid = self._translation_valid(planning_start, forward_probe)
        left_valid = self._turn_valid(
            float(planning_start["x"]),
            float(planning_start["y"]),
            start_yaw,
            start_yaw + self.heading_step,
            robust=False,
        )
        right_valid = self._turn_valid(
            float(planning_start["x"]),
            float(planning_start["y"]),
            start_yaw,
            start_yaw - self.heading_step,
            robust=False,
        )
        if not forward_valid and not left_valid and not right_valid:
            return PlannerResult(
                "START_TURN_BLOCKED_STATIC",
                reason="START_TURN_BLOCKED_STATIC",
                message="Start cannot translate forward or begin either static turn",
                start_escape=escape,
                expansions=expansions,
                elapsed_seconds=elapsed,
            )
        return PlannerResult(
            "NO_EXACT_STOP_TURN_ROUTE",
            reason="NO_EXACT_STOP_TURN_ROUTE",
            message="No exact-valid straight/stop/turn route was found",
            start_escape=escape,
            expansions=expansions,
            elapsed_seconds=elapsed,
        )

    def ranking_key(
        self, route: StopTurnRoute
    ) -> tuple[float, float, float, int, float, float, float, float, float]:
        metadata = route.metadata
        # Exact footprint and turn-sweep validation are hard filters before
        # ranking. Prefer a centered comfort-clear route before considering
        # execution time: total passage width cannot reveal that the centerline
        # is only a few millimetres from one wall.
        rotation_radius = math.hypot(
            self.half_length + self.padding,
            self.half_width + self.padding + self.hard_side_margin,
        )
        required_side_clearance = self.preferred_side_margin
        required_passage = 2.0 * (
            self.half_width + self.padding + required_side_clearance
        )
        passage_shortfall = max(
            0.0,
            required_passage - metadata.minimum_passage_width,
        ) / max(1e-9, required_passage)
        side_shortfall = max(
            0.0,
            required_side_clearance - metadata.minimum_side_clearance,
        ) / max(1e-9, required_side_clearance)
        turn_shortfall = max(
            0.0,
            -metadata.minimum_turn_clearance,
        ) / max(1e-9, rotation_radius)
        safety_shortfall = passage_shortfall + side_shortfall + turn_shortfall
        safety_band = 0.0 if safety_shortfall <= 1e-9 else 1.0
        # Each corner is a physical stop -> in-place turn -> settle cycle. Add
        # a stronger operator-preference penalty than the nominal timing model
        # while retaining a distance/time bound: this avoids both zig-zags and
        # very long right-angle detours selected merely to remove one corner.
        dominant_axis_deviation = (
            0.0
            if not route.heading_bins
            else sum(
                min(int(heading) % 6, 6 - int(heading) % 6)
                for heading in route.heading_bins
            ) / len(route.heading_bins) * self.heading_step
        )
        # Dominant map axes are a tie-breaker, not permission to take a large
        # detour merely to make every line horizontal or vertical.
        turn_preferred_time = metadata.estimated_time + 2.5 * metadata.turn_count
        return (
            safety_band,
            safety_shortfall,
            turn_preferred_time,
            metadata.turn_count,
            metadata.total_length,
            dominant_axis_deviation,
            metadata.total_turn_angle,
            -min(
                metadata.minimum_side_clearance,
                required_side_clearance,
            ),
            -min(metadata.minimum_passage_width, required_passage),
        )

    def plan_candidates(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        *,
        maximum_candidates: int = 3,
        overlap_threshold: float = 0.80,
        planning_time_budget: float | None = None,
        exclusions: Iterable[tuple[float, float, float]] = (),
        primary_route: StopTurnRoute | None = None,
    ) -> list[StopTurnRoute]:
        deadline = (
            None
            if planning_time_budget is None
            else time.monotonic() + max(0.0, float(planning_time_budget))
        )
        dynamic_exclusions = tuple(exclusions)
        primary = primary_route or self.plan(
            start,
            goal,
            exclusions=dynamic_exclusions,
            deadline_monotonic=deadline,
        )
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
            if deadline is not None and time.monotonic() >= deadline:
                break
            candidate = self.plan(
                start,
                goal,
                exclusions=(*dynamic_exclusions, exclusion),
                deadline_monotonic=deadline,
            )
            if candidate is None:
                continue
            if not validate_stop_turn_route(
                self.saved_map,
                candidate.points,
                half_length=self.half_length,
                half_width=self.half_width + self.hard_side_margin,
                padding=self.padding,
                turn_robustness_radius=self.turn_robustness_radius,
                segment_directions=(
                    candidate.segment_directions or None
                ),
            ).valid:
                # Optional routes must retain a small position-error reserve at
                # every corner. The primary remains available for genuinely
                # narrow maps where no such optional alternative exists.
                continue
            candidate_points = list(candidate.points)
            if any(
                path_overlap_ratio(candidate_points, list(existing.points))
                >= float(overlap_threshold)
                for existing in candidates
            ):
                continue
            candidates.append(candidate)
        # plan() already enforces the direct-route primary rule.  Exclusion
        # attempts provide optional alternatives and must not displace it.
        return [primary, *sorted(candidates[1:], key=self.ranking_key)]


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
class LocalizationRaycastConsistency:
    comparable_beams: int
    static_matches: int
    dynamic_occlusions: int
    map_contradictions: int
    inconclusive_map_hits: int
    static_match_ratio: float
    dynamic_occlusion_ratio: float
    contradiction_ratio: float
    median_error: float
    p90_error: float
    mean_error: float

    @property
    def matched_beams(self) -> int:
        """Backward-compatible name for exact saved-map matches."""
        return self.static_matches

    @property
    def match_ratio(self) -> float:
        """Backward-compatible name for the static-match ratio."""
        return self.static_match_ratio


@dataclass(frozen=True, slots=True)
class LocalizationVerification:
    accepted: bool
    reason: str


@dataclass(frozen=True, slots=True)
class LocalizationEvidenceFrame:
    """One scan evaluated against one AMCL pose hypothesis."""

    timestamp: float
    pose_x: float
    pose_y: float
    pose_yaw: float
    scan_score: float
    valid_beams: int
    residual_beams: int
    median_residual: float
    p90_residual: float
    mean_residual: float
    raycast_comparable_beams: int
    raycast_static_matches: int
    raycast_dynamic_occlusions: int
    raycast_map_contradictions: int
    raycast_inconclusive_map_hits: int
    raycast_static_match_ratio: float
    raycast_dynamic_occlusion_ratio: float
    raycast_contradiction_ratio: float
    raycast_median_error: float
    raycast_p90_error: float


@dataclass(frozen=True, slots=True)
class LocalizationConsensus:
    accepted: bool
    reason: str
    total_frames: int
    required_frames: int
    passing_frames: int
    agreeing_frames: int
    scan_score: float = 0.0
    valid_beams: int = 0
    residual_beams: int = 0
    median_residual: float = math.inf
    p90_residual: float = math.inf
    mean_residual: float = math.inf
    raycast_comparable_beams: int = 0
    raycast_static_matches: int = 0
    raycast_dynamic_occlusions: int = 0
    raycast_map_contradictions: int = 0
    raycast_inconclusive_map_hits: int = 0
    raycast_static_match_ratio: float = 0.0
    raycast_dynamic_occlusion_ratio: float = 0.0
    raycast_contradiction_ratio: float = 0.0
    raycast_median_error: float = math.inf
    raycast_p90_error: float = math.inf


@dataclass(frozen=True, slots=True)
class ParticleCloudUniqueness:
    accepted: bool
    reason: str
    particle_count: int
    cluster_count: int
    best_weight: float
    alternative_weight: float
    dominance_ratio: float
    best_x: float | None = None
    best_y: float | None = None


@dataclass(frozen=True, slots=True)
class GlobalScanUniqueness:
    """Independent coarse Saved-Map search around one stationary scan.

    AMCL may resample every particle into one locally plausible hypothesis;
    that collapse is not proof that another part of a repeated map is worse.
    This result compares the current scan against free poses across the map
    (or an explicitly hinted broad region) without using AMCL's particles.
    """

    accepted: bool
    reason: str
    evaluated_candidates: int
    usable_beams: int
    best_score: float
    alternative_score: float
    score_margin: float
    score_ratio: float
    best_x: float | None = None
    best_y: float | None = None
    best_yaw: float | None = None
    alternative_x: float | None = None
    alternative_y: float | None = None
    alternative_yaw: float | None = None
    candidate_position_error: float = math.inf
    candidate_yaw_error: float = math.inf


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
    normalized = sorted(
        float(value) % (2.0 * math.pi) for value in headings
    )
    bins = tuple(sorted({
        min(count - 1, int(value / (2.0 * math.pi) * count))
        for value in normalized
    }))
    if len(normalized) < 2:
        return HeadingDiversity(bins, 0.0)

    # The largest geodesic separation from one angle is attained next to its
    # antipode. Searching those neighbours keeps this exact calculation
    # O(n log n), instead of comparing an ever-growing list all-pairs.
    size = len(normalized)
    extended = normalized + [value + 2.0 * math.pi for value in normalized]
    span = 0.0
    for index, left in enumerate(normalized):
        insertion = bisect.bisect_left(
            extended,
            left + math.pi,
            index + 1,
            index + size,
        )
        for candidate_index in (insertion - 1, insertion):
            if not index < candidate_index < index + size:
                continue
            right = extended[candidate_index]
            span = max(
                span,
                abs(math.atan2(math.sin(left - right), math.cos(left - right))),
            )
    return HeadingDiversity(bins, span)


def bounded_heading_evidence(
    headings: Iterable[float],
    heading: float,
    *,
    bin_count: int = 8,
) -> list[float]:
    """Keep at most one physical heading observation per angular bin.

    Repeated stationary scans remain independent consensus frames, but they
    are not independent headings. This bounded collection owns only angular
    diversity and therefore must never grow with timer or scan count.
    """
    count = max(1, int(bin_count))
    by_bin: dict[int, float] = {}
    for value in (*tuple(headings), float(heading)):
        normalized = float(value) % (2.0 * math.pi)
        heading_bin = min(
            count - 1,
            int(normalized / (2.0 * math.pi) * count),
        )
        by_bin[heading_bin] = float(value)
    return [by_bin[index] for index in sorted(by_bin)]


def heading_position_spread(
    positions: Iterable[tuple[float, float]],
) -> float:
    """Maximum candidate-position separation across independent headings."""
    values = [(float(x), float(y)) for x, y in positions]
    return max(
        (
            math.hypot(left_x - right_x, left_y - right_y)
            for left_x, left_y in values
            for right_x, right_y in values
        ),
        default=0.0,
    )


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
    translation_lateral_margin: float = 0.01,
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
            if x >= length
            and abs(y) <= width + max(0.0, float(translation_lateral_margin))
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


def dynamic_exclusions_intersect_route(
    points: Iterable[dict[str, float]],
    exclusions: Iterable[tuple[float, float, float]],
    *,
    horizon: float = 2.0,
) -> bool:
    """Check only the bounded upcoming route, never unrelated/behind objects."""
    route = list(points)
    remaining_horizon = max(0.0, float(horizon))
    dynamic = tuple(exclusions)
    for left, right in zip(route, route[1:]):
        left_x, left_y = float(left["x"]), float(left["y"])
        delta_x = float(right["x"]) - left_x
        delta_y = float(right["y"]) - left_y
        length = math.hypot(delta_x, delta_y)
        if length <= 1e-9:
            continue
        used = min(length, remaining_horizon)
        ratio_end = used / length
        end_x = left_x + delta_x * ratio_end
        end_y = left_y + delta_y * ratio_end
        segment_x = end_x - left_x
        segment_y = end_y - left_y
        denominator = segment_x * segment_x + segment_y * segment_y
        for center_x, center_y, radius in dynamic:
            ratio = 0.0 if denominator <= 1e-12 else max(
                0.0,
                min(
                    1.0,
                    (
                        (float(center_x) - left_x) * segment_x
                        + (float(center_y) - left_y) * segment_y
                    ) / denominator,
                ),
            )
            if math.hypot(
                left_x + ratio * segment_x - float(center_x),
                left_y + ratio * segment_y - float(center_y),
            ) <= float(radius):
                return True
        remaining_horizon -= used
        if remaining_horizon <= 1e-9:
            break
    return False


def dynamic_trajectory_conflict_ttc(
    points: Iterable[dict[str, float]],
    obstacle: DynamicObstacle,
    *,
    robot_speed: float,
    footprint_inflation: float,
    ttc_horizon: float = 3.0,
    sample_period: float = 0.10,
) -> float | None:
    """Return the first predicted swept-route conflict time for a moving track.

    A confirmed costmap cluster merely lying somewhere on the next two metres
    is not sufficient reason to cancel the controller. Project both the
    chassis along the maintained route and the obstacle with its measured map
    velocity, then grant proactive stop authority only when their inflated
    footprints overlap inside the bounded TTC horizon. Stationary tracks stay
    under controller/corridor and independent motion-safety authority.
    """
    route = list(points)
    if len(route) < 2 or obstacle.motion_state != "MOVING":
        return None
    speed = abs(float(robot_speed))
    horizon = max(0.0, float(ttc_horizon))
    if speed <= 1e-6 or horizon <= 0.0:
        return None

    segments: list[tuple[float, float, float, float, float]] = []
    total_length = 0.0
    for left, right in zip(route, route[1:]):
        left_x, left_y = float(left["x"]), float(left["y"])
        delta_x = float(right["x"]) - left_x
        delta_y = float(right["y"]) - left_y
        length = math.hypot(delta_x, delta_y)
        if length <= 1e-9:
            continue
        segments.append((left_x, left_y, delta_x, delta_y, length))
        total_length += length
    if not segments:
        return None

    inflated_radius = max(
        0.0, float(obstacle.radius) + max(0.0, float(footprint_inflation))
    )
    period = max(0.02, float(sample_period))
    steps = max(1, int(math.ceil(horizon / period)))
    segment_index = 0
    segment_start_distance = 0.0
    for step in range(steps + 1):
        timestamp = min(horizon, step * period)
        route_distance = min(total_length, speed * timestamp)
        while (
            segment_index + 1 < len(segments)
            and route_distance
            > segment_start_distance + segments[segment_index][4]
        ):
            segment_start_distance += segments[segment_index][4]
            segment_index += 1
        left_x, left_y, delta_x, delta_y, length = segments[segment_index]
        ratio = max(
            0.0,
            min(1.0, (route_distance - segment_start_distance) / length),
        )
        robot_x = left_x + ratio * delta_x
        robot_y = left_y + ratio * delta_y
        obstacle_x = obstacle.center_x + obstacle.velocity_x * timestamp
        obstacle_y = obstacle.center_y + obstacle.velocity_y * timestamp
        if math.hypot(robot_x - obstacle_x, robot_y - obstacle_y) <= inflated_radius:
            return round(timestamp, 3)
    return None


def turn_braking_speed_limit(
    heading_error: float,
    *,
    completion_tolerance: float,
    angular_deceleration: float,
    reaction_time: float,
) -> float:
    """Maximum angular speed that can settle without crossing the target."""
    remaining = max(
        0.0, abs(float(heading_error)) - abs(float(completion_tolerance))
    )
    if remaining <= 0.0:
        return 0.0
    deceleration = max(0.01, abs(float(angular_deceleration)))
    reaction = max(0.0, float(reaction_time))
    # Solve v * reaction + v^2 / (2a) <= remaining angle.
    return max(
        0.0,
        -deceleration * reaction
        + math.sqrt(
            (deceleration * reaction) ** 2
            + 2.0 * deceleration * remaining
        ),
    )


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


def scan_raycast_consistency(
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
    match_tolerance: float = 0.15,
    minimum_reliable_structure_span: float = 0.75,
) -> LocalizationRaycastConsistency:
    """Classify measured ranges against first-hit ranges from the saved map.

    A nearer measurement is a dynamic occlusion, not direct evidence that the
    pose is wrong. A measurement beyond a conclusive saved-map hit is a map
    contradiction and is the strong negative evidence used by localization.
    Rays without a conclusive static-map hit are omitted from the denominator.
    """
    measurements = list(ranges)
    if not measurements:
        return LocalizationRaycastConsistency(
            comparable_beams=0,
            static_matches=0,
            dynamic_occlusions=0,
            map_contradictions=0,
            inconclusive_map_hits=0,
            static_match_ratio=0.0,
            dynamic_occlusion_ratio=0.0,
            contradiction_ratio=0.0,
            median_error=math.inf,
            p90_error=math.inf,
            mean_error=math.inf,
        )
    step = max(1, math.ceil(len(measurements) / max(1, int(maximum_beams))))
    lower = max(float(range_min), float(minimum_usable_range))
    upper = min(float(range_max), float(maximum_usable_range))
    errors: list[float] = []
    static_matches = 0
    dynamic_occlusions = 0
    map_contradictions = 0
    inconclusive_map_hits = 0
    for index, distance, _, _ in _scan_endpoints(
        measurements,
        angle_min=float(angle_min),
        angle_increment=float(angle_increment),
        lower_range=lower,
        upper_range=upper,
        laser_x=float(laser_x),
        laser_y=float(laser_y),
        laser_yaw=float(laser_yaw),
        step=step,
    ):
        angle = float(laser_yaw) + float(angle_min) + index * float(angle_increment)
        hit = saved_map.raycast_static_hit(
            float(laser_x),
            float(laser_y),
            angle,
            minimum_range=lower,
            maximum_range=upper,
        )
        if hit is None:
            continue
        expected, hit_column, hit_row = hit
        delta = distance - expected
        error = abs(delta)
        if error <= float(match_tolerance):
            static_matches += 1
            errors.append(error)
        elif delta < 0.0:
            dynamic_occlusions += 1
            errors.append(error)
        elif (
            hit_column is not None
            and hit_row is not None
            and saved_map.is_reliable_static_structure(
                hit_column,
                hit_row,
                minimum_span=float(minimum_reliable_structure_span),
            )
        ):
            map_contradictions += 1
            errors.append(error)
        else:
            # A missing isolated pixel or short mapped object is not strong
            # enough evidence that the pose is wrong. It remains visible in
            # diagnostics but cannot dilute or inflate the conclusive ratios.
            inconclusive_map_hits += 1
    ordered = sorted(errors)
    p90_index = max(0, math.ceil(0.9 * len(ordered)) - 1)
    comparable = len(ordered)
    return LocalizationRaycastConsistency(
        comparable_beams=comparable,
        static_matches=static_matches,
        dynamic_occlusions=dynamic_occlusions,
        map_contradictions=map_contradictions,
        inconclusive_map_hits=inconclusive_map_hits,
        static_match_ratio=(
            0.0 if comparable == 0
            else round(static_matches / comparable, 4)
        ),
        dynamic_occlusion_ratio=(
            0.0 if comparable == 0
            else round(dynamic_occlusions / comparable, 4)
        ),
        contradiction_ratio=(
            0.0 if comparable == 0
            else round(map_contradictions / comparable, 4)
        ),
        median_error=(statistics.median(ordered) if ordered else math.inf),
        p90_error=(ordered[p90_index] if ordered else math.inf),
        mean_error=(statistics.fmean(ordered) if ordered else math.inf),
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


TRINARY_UNKNOWN_GRAYSCALE = 205


def image_grayscale_values(image: Image.Image) -> list[int]:
    """Read pixels across both current and pre-Pillow-14 runtimes."""
    flattened = getattr(image, "get_flattened_data", None)
    values = flattened() if callable(flattened) else image.getdata()
    return [int(value) for value in values]


def normalize_trinary_unknown_metadata(
    metadata: dict[str, Any], pixels: Iterable[int]
) -> tuple[dict[str, Any], bool]:
    """Keep slam_toolbox's canonical gray unknown cells unknown.

    Some deployed slam_toolbox builds save unknown cells as gray 205 while
    writing ``free_thresh: 0.25``.  Nav2 then interprets their occupancy
    probability (50/255) as free.  Normalize the threshold below that exact
    value so Nav2 and this loader both preserve the cells as unknown.
    """
    normalized = dict(metadata)
    if (
        str(normalized.get("mode", "trinary")).lower() != "trinary"
        or int(normalized.get("negate", 0)) != 0
        or TRINARY_UNKNOWN_GRAYSCALE not in {int(value) for value in pixels}
    ):
        return normalized, False
    unknown_probability = (
        255 - TRINARY_UNKNOWN_GRAYSCALE
    ) / 255.0
    free_threshold = float(normalized.get("free_thresh", 0.196))
    if free_threshold < unknown_probability:
        return normalized, False
    # 0.196 is Nav2's conventional trinary threshold and is strictly below
    # 50/255 (~0.196078), so 205 falls between free and occupied thresholds.
    normalized["free_thresh"] = 0.196
    return normalized, True


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
    reliable_structure_cache: dict[tuple[int, int, int], bool] = field(
        default_factory=dict, init=False, repr=False
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
        pixels = image_grayscale_values(image)
        metadata, _ = normalize_trinary_unknown_metadata(
            metadata, pixels
        )
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
        hit = self.raycast_static_hit(
            x,
            y,
            angle,
            minimum_range=minimum_range,
            maximum_range=maximum_range,
            unknown_is_blocked=unknown_is_blocked,
        )
        return None if hit is None else hit[0]

    def raycast_static_hit(
        self,
        x: float,
        y: float,
        angle: float,
        *,
        minimum_range: float,
        maximum_range: float,
        unknown_is_blocked: bool = False,
    ) -> tuple[float, int | None, int | None] | None:
        """Return first-hit range and its occupied cell when conclusive.

        The cell coordinates let localization distinguish a trustworthy wall
        from an isolated or short-lived mapped object. ``None`` cell values
        are reserved for callers that explicitly treat unknown as blocked.
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
                return (
                    (entered_at, None, None) if unknown_is_blocked else None
                )
            value = self.value_at(column, row)
            if value < 0:
                return (
                    (entered_at, None, None) if unknown_is_blocked else None
                )
            if value >= 65:
                return max(lower, entered_at), column, row
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

    def is_reliable_static_structure(
        self,
        column: int,
        row: int,
        *,
        minimum_span: float,
    ) -> bool:
        """Whether an occupied hit belongs to a continuous wall-like line.

        A contradiction is strong negative pose evidence only when the map hit
        is structural. Sampling multiple line orientations supports angled
        walls while requiring uninterrupted occupied cells rejects speckles,
        chair legs, and short furniture edges. Results are cached per map cell
        because the saved map is immutable while localization is running.
        """
        if (
            not (0 <= column < self.width and 0 <= row < self.height)
            or self.value_at(column, row) < 65
        ):
            return False
        required_cells = max(
            1,
            math.ceil(
                max(0.0, float(minimum_span)) / self.resolution - 1e-9
            ),
        )
        cache_key = (column, row, required_cells)
        cached = self.reliable_structure_cache.get(cache_key)
        if cached is not None:
            return cached
        if required_cells <= 1:
            self.reliable_structure_cache[cache_key] = True
            return True

        # Half a circle is sufficient because each candidate line is sampled
        # in both directions. Five-degree spacing limits perpendicular drift
        # to less than one 5 cm cell across the configured 0.75 m span.
        for angle_index in range(36):
            line_angle = angle_index * math.pi / 36.0
            direction_x = math.cos(line_angle)
            direction_y = math.sin(line_angle)
            extents: list[int] = []
            for sign in (-1, 1):
                extent = 0
                previous_cell = (column, row)
                for step in range(1, required_cells + 1):
                    check_cell = (
                        round(column + sign * step * direction_x),
                        round(row + sign * step * direction_y),
                    )
                    if check_cell == previous_cell:
                        continue
                    previous_cell = check_cell
                    check_column, check_row = check_cell
                    if (
                        not (0 <= check_column < self.width)
                        or not (0 <= check_row < self.height)
                        or self.value_at(check_column, check_row) < 65
                    ):
                        break
                    extent = step
                extents.append(extent)
            span_cells = extents[0] + 1 + extents[1]
            if span_cells >= required_cells:
                self.reliable_structure_cache[cache_key] = True
                return True
        self.reliable_structure_cache[cache_key] = False
        return False

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

    def footprint_overlap_cells(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        half_length: float,
        half_width: float,
        padding: float = 0.0,
        include_unknown: bool = True,
    ) -> tuple[tuple[int, int, int], ...]:
        """Return exact static raster cells intersecting an oriented body.

        This is intentionally diagnostic/escape geometry.  It does not grant
        a global collision exemption and it does not inspect dynamic inputs.
        """
        length = max(0.0, float(half_length) + float(padding))
        width = max(0.0, float(half_width) + float(padding))
        if length <= 0.0 or width <= 0.0:
            raise ValueError("footprint half dimensions must be positive")
        center = self.world_to_cell(float(x), float(y))
        if center is None:
            return ()
        robot_x_axis = (math.cos(yaw), math.sin(yaw))
        robot_y_axis = (-math.sin(yaw), math.cos(yaw))
        map_x_axis = (math.cos(self.origin_yaw), math.sin(self.origin_yaw))
        map_y_axis = (-math.sin(self.origin_yaw), math.cos(self.origin_yaw))
        half_cell = self.resolution / 2.0

        def intersects(cell_x: float, cell_y: float) -> bool:
            delta = (cell_x - float(x), cell_y - float(y))
            for axis in (robot_x_axis, robot_y_axis, map_x_axis, map_y_axis):
                separation = abs(delta[0] * axis[0] + delta[1] * axis[1])
                robot_projection = (
                    length * abs(
                        robot_x_axis[0] * axis[0]
                        + robot_x_axis[1] * axis[1]
                    )
                    + width * abs(
                        robot_y_axis[0] * axis[0]
                        + robot_y_axis[1] * axis[1]
                    )
                )
                cell_projection = half_cell * (
                    abs(map_x_axis[0] * axis[0] + map_x_axis[1] * axis[1])
                    + abs(map_y_axis[0] * axis[0] + map_y_axis[1] * axis[1])
                )
                if separation > robot_projection + cell_projection:
                    return False
            return True

        center_column, center_row = center
        radius = math.ceil(
            (math.hypot(length, width) + math.sqrt(2.0) * half_cell)
            / self.resolution
        ) + 1
        overlaps: list[tuple[int, int, int]] = []
        for row in range(center_row - radius, center_row + radius + 1):
            for column in range(center_column - radius, center_column + radius + 1):
                if not (0 <= column < self.width and 0 <= row < self.height):
                    continue
                value = self.value_at(column, row)
                if value < 0 and not include_unknown:
                    continue
                if value < 65 and value >= 0:
                    continue
                cell_x, cell_y = self.cell_center(column, row)
                if intersects(cell_x, cell_y):
                    overlaps.append((column, row, value))
        return tuple(overlaps)

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


def find_start_escape(
    saved_map: SavedOccupancyMap,
    start: dict[str, float],
    *,
    half_length: float,
    half_width: float,
    padding: float = 0.0,
    maximum_distance: float = 0.60,
    probe_step: float | None = None,
    live_blocked_points: Iterable[tuple[float, float]] = (),
    live_inflation: float = 0.0,
    directions: Iterable[int] = (1,),
) -> StartEscape | None:
    """Find the nearest exact-valid pose in the requested straight directions.

    Every intermediate footprint may retain only cells already overlapped at
    the initial pose.  Encountering any new occupied/unknown cell terminates
    the search, so this cannot become permission to cross a wall.
    """
    start_x = float(start["x"])
    start_y = float(start["y"])
    yaw = float(start.get("yaw", 0.0))
    initial_validation = saved_map.validate_footprint(
        start_x,
        start_y,
        yaw,
        half_length=half_length,
        half_width=half_width,
        padding=padding,
        allow_unknown=False,
        code_prefix="START",
    )
    if initial_validation.valid:
        return None
    if initial_validation.code == "START_FOOTPRINT_OUTSIDE_MAP":
        return None
    initial_cells = saved_map.footprint_overlap_cells(
        start_x,
        start_y,
        yaw,
        half_length=half_length,
        half_width=half_width,
        padding=padding,
    )
    initial_set = {(column, row) for column, row, _ in initial_cells}
    if not initial_set:
        return None
    step = (
        max(saved_map.resolution * 0.5, 0.005)
        if probe_step is None
        else max(0.005, min(float(probe_step), saved_map.resolution))
    )
    limit = max(0.0, float(maximum_distance))
    dynamic = tuple((float(x), float(y)) for x, y in live_blocked_points)
    samples = max(0, math.floor(limit / step))
    normalized_directions = tuple(dict.fromkeys(
        -1 if int(direction) < 0 else 1 for direction in directions
    ))
    for direction in normalized_directions:
        previous_count = len(initial_set)
        for index in range(1, samples + 1):
            distance = min(limit, index * step)
            x = start_x + direction * distance * math.cos(yaw)
            y = start_y + direction * distance * math.sin(yaw)
            overlap = saved_map.footprint_overlap_cells(
                x,
                y,
                yaw,
                half_length=half_length,
                half_width=half_width,
                padding=padding,
            )
            overlap_set = {(column, row) for column, row, _ in overlap}
            if (
                not overlap_set.issubset(initial_set)
                or len(overlap_set) > previous_count
            ):
                break
            previous_count = len(overlap_set)
            lateral = half_width + padding + max(
                0.0, float(live_inflation)
            )
            longitudinal = half_length + padding
            live_collision = False
            for obstacle_x, obstacle_y in dynamic:
                delta_x = obstacle_x - x
                delta_y = obstacle_y - y
                local_x = (
                    math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
                )
                local_y = (
                    -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
                )
                if abs(local_x) <= longitudinal and abs(local_y) <= lateral:
                    live_collision = True
                    break
            if live_collision:
                break
            validation = saved_map.validate_footprint(
                x,
                y,
                yaw,
                half_length=half_length,
                half_width=half_width,
                padding=padding,
                allow_unknown=False,
                code_prefix="START",
            )
            if validation.valid:
                return StartEscape(
                    start={"x": start_x, "y": start_y},
                    end={"x": x, "y": y},
                    yaw=yaw,
                    distance=distance,
                    initial_overlap_cells=tuple(sorted(initial_set)),
                    samples_checked=index,
                    motion_direction=direction,
                )
    return None


def localization_evidence_consensus(
    frames: Iterable[LocalizationEvidenceFrame],
    *,
    window_size: int,
    required_frames: int,
    candidate_position_tolerance: float,
    candidate_yaw_tolerance: float,
    minimum_scan_beams: int,
    required_scan_score: float,
    minimum_residual_beams: int,
    maximum_median_residual: float,
    maximum_p90_residual: float,
    minimum_raycast_beams: int,
    minimum_raycast_static_matches: int,
    maximum_raycast_contradiction_ratio: float,
) -> LocalizationConsensus:
    """Require a configurable majority of scans around one pose hypothesis.

    Every frame still passes all strict geometry gates independently. Robust
    medians are then computed only from the agreeing, passing candidate frames,
    so one person crossing the laser cannot invalidate an otherwise consistent
    acquisition and several contradictory frames still prevent acceptance.
    """
    size = max(1, int(window_size))
    required = max(1, min(size, int(required_frames)))
    values = tuple(frames)[-size:]
    if len(values) < size:
        return LocalizationConsensus(
            False,
            "CONSENSUS_WINDOW_INCOMPLETE",
            len(values),
            required,
            0,
            0,
        )

    def frame_passes(frame: LocalizationEvidenceFrame) -> bool:
        return bool(
            frame.valid_beams >= int(minimum_scan_beams)
            and frame.scan_score >= float(required_scan_score)
            and frame.residual_beams >= int(minimum_residual_beams)
            and frame.median_residual <= float(maximum_median_residual)
            and frame.p90_residual <= float(maximum_p90_residual)
            and frame.raycast_comparable_beams >= int(minimum_raycast_beams)
            and frame.raycast_static_matches
            >= int(minimum_raycast_static_matches)
            and frame.raycast_contradiction_ratio
            <= float(maximum_raycast_contradiction_ratio)
        )

    passing = tuple(frame for frame in values if frame_passes(frame))
    if len(passing) < required:
        return LocalizationConsensus(
            False,
            "CONSENSUS_K_OF_N_FAILED",
            len(values),
            required,
            len(passing),
            0,
        )

    position_tolerance = max(0.0, float(candidate_position_tolerance))
    yaw_tolerance = max(0.0, float(candidate_yaw_tolerance))

    def agrees(
        left: LocalizationEvidenceFrame,
        right: LocalizationEvidenceFrame,
    ) -> bool:
        return bool(
            math.hypot(left.pose_x - right.pose_x, left.pose_y - right.pose_y)
            <= position_tolerance
            and abs(_angle_delta(left.pose_yaw, right.pose_yaw)) <= yaw_tolerance
        )

    candidate_groups = [
        tuple(other for other in passing if agrees(seed, other))
        for seed in passing
    ]
    agreeing = max(candidate_groups, key=len, default=())
    if len(agreeing) < required:
        return LocalizationConsensus(
            False,
            "CANDIDATE_POSE_CONSENSUS_FAILED",
            len(values),
            required,
            len(passing),
            len(agreeing),
        )

    def median_number(attribute: str) -> float:
        return float(statistics.median(
            float(getattr(frame, attribute)) for frame in agreeing
        ))

    def median_count(attribute: str) -> int:
        return int(round(median_number(attribute)))

    return LocalizationConsensus(
        True,
        "ACCEPTED",
        len(values),
        required,
        len(passing),
        len(agreeing),
        scan_score=median_number("scan_score"),
        valid_beams=median_count("valid_beams"),
        residual_beams=median_count("residual_beams"),
        median_residual=median_number("median_residual"),
        p90_residual=median_number("p90_residual"),
        mean_residual=median_number("mean_residual"),
        raycast_comparable_beams=median_count("raycast_comparable_beams"),
        raycast_static_matches=median_count("raycast_static_matches"),
        raycast_dynamic_occlusions=median_count("raycast_dynamic_occlusions"),
        raycast_map_contradictions=median_count("raycast_map_contradictions"),
        raycast_inconclusive_map_hits=median_count(
            "raycast_inconclusive_map_hits"
        ),
        raycast_static_match_ratio=median_number(
            "raycast_static_match_ratio"
        ),
        raycast_dynamic_occlusion_ratio=median_number(
            "raycast_dynamic_occlusion_ratio"
        ),
        raycast_contradiction_ratio=median_number(
            "raycast_contradiction_ratio"
        ),
        raycast_median_error=median_number("raycast_median_error"),
        raycast_p90_error=median_number("raycast_p90_error"),
    )


def particle_cloud_uniqueness(
    particles: Iterable[tuple[float, float, float]],
    *,
    cluster_radius: float,
    alternative_separation: float,
    minimum_best_weight: float,
    minimum_dominance_ratio: float,
) -> ParticleCloudUniqueness:
    """Compare spatially separated weighted AMCL particle regions.

    Nearby coarse clusters are counted as the same pose region. A second
    region beyond ``alternative_separation`` must carry sufficiently less
    probability than the best region; otherwise repeated-layout aliases remain
    ambiguous and READY is rejected.
    """
    values = tuple(
        (float(x), float(y), max(0.0, float(weight)))
        for x, y, weight in particles
        if math.isfinite(float(x))
        and math.isfinite(float(y))
        and math.isfinite(float(weight))
        and float(weight) > 0.0
    )
    if not values:
        return ParticleCloudUniqueness(
            False, "PARTICLE_CLOUD_UNAVAILABLE", 0, 0, 0.0, 0.0, 0.0
        )
    total_weight = sum(weight for _, _, weight in values)
    if total_weight <= 0.0:
        return ParticleCloudUniqueness(
            False, "PARTICLE_CLOUD_UNAVAILABLE", len(values), 0, 0.0, 0.0, 0.0
        )

    radius = max(0.01, float(cluster_radius))
    # Deterministic weighted leader clustering avoids the single-link bridge
    # that can merge two real hypotheses through a sparse trail of particles.
    clusters: list[list[float]] = []
    for x, y, weight in sorted(values, key=lambda item: (item[0], item[1])):
        nearest_index = -1
        nearest_distance = math.inf
        for index, (center_x, center_y, cluster_weight) in enumerate(clusters):
            distance = math.hypot(x - center_x, y - center_y)
            if distance <= radius and distance < nearest_distance:
                nearest_index = index
                nearest_distance = distance
        if nearest_index < 0:
            clusters.append([x, y, weight])
            continue
        center_x, center_y, cluster_weight = clusters[nearest_index]
        updated_weight = cluster_weight + weight
        clusters[nearest_index] = [
            (center_x * cluster_weight + x * weight) / updated_weight,
            (center_y * cluster_weight + y * weight) / updated_weight,
            updated_weight,
        ]

    strongest = max(clusters, key=lambda cluster: cluster[2])
    separation = max(radius, float(alternative_separation))

    def region_weight(seed: list[float]) -> float:
        return sum(
            cluster[2]
            for cluster in clusters
            if math.hypot(cluster[0] - seed[0], cluster[1] - seed[1])
            < separation
        ) / total_weight

    best_weight = region_weight(strongest)
    alternatives = [
        region_weight(cluster)
        for cluster in clusters
        if math.hypot(cluster[0] - strongest[0], cluster[1] - strongest[1])
        >= separation
    ]
    alternative_weight = max(alternatives, default=0.0)
    dominance = (
        math.inf
        if alternative_weight <= 1e-12
        else best_weight / alternative_weight
    )
    accepted = bool(
        best_weight >= max(0.0, min(1.0, float(minimum_best_weight)))
        and dominance >= max(1.0, float(minimum_dominance_ratio))
    )
    reason = (
        "ACCEPTED"
        if accepted
        else "BEST_PARTICLE_CLUSTER_TOO_WEAK"
        if best_weight < float(minimum_best_weight)
        else "ALTERNATIVE_PARTICLE_CLUSTER_COMPETITIVE"
    )
    return ParticleCloudUniqueness(
        accepted,
        reason,
        len(values),
        len(clusters),
        round(best_weight, 6),
        round(alternative_weight, 6),
        math.inf if math.isinf(dominance) else round(dominance, 6),
        strongest[0],
        strongest[1],
    )


def global_scan_alternative_is_competitive(
    best_score: float,
    alternative_score: float,
    *,
    minimum_margin: float,
    minimum_ratio: float,
) -> bool:
    """Return true only when neither absolute nor relative evidence separates modes."""
    if alternative_score <= 0.0:
        return False
    margin = best_score - alternative_score
    ratio = best_score / alternative_score
    return bool(
        margin < max(0.0, float(minimum_margin))
        and ratio < max(1.0, float(minimum_ratio))
    )


def global_scan_candidate_uniqueness(
    saved_map: SavedOccupancyMap,
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    candidate_pose: tuple[float, float, float],
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
    maximum_beams: int = 45,
    minimum_usable_range: float = 0.20,
    maximum_usable_range: float = 6.0,
    endpoint_tolerance: float = 0.15,
    position_step: float = 0.20,
    heading_step: float = math.radians(15.0),
    alternative_separation: float = 0.75,
    minimum_best_score: float = 0.45,
    minimum_score_margin: float = 0.12,
    minimum_score_ratio: float = 1.15,
    candidate_position_tolerance: float = 0.45,
    candidate_yaw_tolerance: float = math.radians(25.0),
    candidate_score_tolerance: float = 0.04,
    search_center: tuple[float, float] | None = None,
    search_radius: float | None = None,
) -> GlobalScanUniqueness:
    """Compare a scan with coarse hypotheses independent from AMCL.

    Only known-free candidate cells and endpoints close to Saved-Map occupied
    cells participate.  The search is deliberately coarse and bounded: its
    purpose is to find a spatially separated competitive alias, not to replace
    AMCL's continuous pose estimate.  It is suitable for an occasional
    background acquisition check and must not run on every scan callback.
    """
    measurements = tuple(float(value) for value in ranges)
    if not measurements or saved_map.width <= 0 or saved_map.height <= 0:
        return GlobalScanUniqueness(
            False, "GLOBAL_SCAN_UNAVAILABLE", 0, 0, 0.0, 0.0, 0.0, 0.0
        )
    beam_stride = max(
        1, math.ceil(len(measurements) / max(1, int(maximum_beams)))
    )
    lower = max(float(range_min), float(minimum_usable_range))
    upper = min(float(range_max), float(maximum_usable_range))
    beams = tuple(
        (index, distance)
        for index, distance in enumerate(measurements)
        if index % beam_stride == 0
        and math.isfinite(distance)
        and lower <= distance <= upper
    )
    if len(beams) < 12:
        return GlobalScanUniqueness(
            False,
            "GLOBAL_SCAN_INSUFFICIENT_BEAMS",
            0,
            len(beams),
            0.0,
            0.0,
            0.0,
            0.0,
        )

    endpoint_radius_cells = max(
        1, math.ceil(max(0.0, float(endpoint_tolerance)) / saved_map.resolution)
    )
    nearby_static: set[int] = set()
    for row in range(saved_map.height):
        for column in range(saved_map.width):
            if saved_map.value_at(column, row) < 65:
                continue
            for offset_y in range(-endpoint_radius_cells, endpoint_radius_cells + 1):
                check_row = row + offset_y
                if not 0 <= check_row < saved_map.height:
                    continue
                for offset_x in range(
                    -endpoint_radius_cells, endpoint_radius_cells + 1
                ):
                    if (
                        offset_x * offset_x + offset_y * offset_y
                        > endpoint_radius_cells * endpoint_radius_cells
                    ):
                        continue
                    check_column = column + offset_x
                    if 0 <= check_column < saved_map.width:
                        nearby_static.add(
                            check_row * saved_map.width + check_column
                        )

    cell_stride = max(
        1, round(max(saved_map.resolution, float(position_step)) / saved_map.resolution)
    )
    constrained_radius = (
        None if search_center is None or search_radius is None
        else max(0.0, float(search_radius))
    )
    candidate_cells: list[tuple[float, float]] = []
    for row in range(0, saved_map.height, cell_stride):
        for column in range(0, saved_map.width, cell_stride):
            if saved_map.value_at(column, row) != 0:
                continue
            x, y = saved_map.cell_center(column, row)
            if (
                constrained_radius is not None
                and math.hypot(x - search_center[0], y - search_center[1])
                > constrained_radius
            ):
                continue
            candidate_cells.append((x, y))
    if not candidate_cells:
        return GlobalScanUniqueness(
            False,
            "GLOBAL_SCAN_NO_FREE_CANDIDATES",
            0,
            len(beams),
            0.0,
            0.0,
            0.0,
            0.0,
        )

    map_cosine = math.cos(saved_map.origin_yaw)
    map_sine = math.sin(saved_map.origin_yaw)
    yaw_step = max(math.radians(5.0), float(heading_step))
    heading_count = max(1, math.ceil(2.0 * math.pi / yaw_step))
    scored: list[tuple[float, float, float, float]] = []
    evaluated = 0
    for heading_index in range(heading_count):
        yaw = _angle_delta(-math.pi + heading_index * 2.0 * math.pi / heading_count, 0.0)
        heading_cosine = math.cos(yaw)
        heading_sine = math.sin(yaw)
        laser_offset_x = heading_cosine * float(laser_x) - heading_sine * float(laser_y)
        laser_offset_y = heading_sine * float(laser_x) + heading_cosine * float(laser_y)
        offsets = tuple(
            (
                laser_offset_x + distance * math.cos(
                    yaw + float(laser_yaw) + float(angle_min)
                    + index * float(angle_increment)
                ),
                laser_offset_y + distance * math.sin(
                    yaw + float(laser_yaw) + float(angle_min)
                    + index * float(angle_increment)
                ),
            )
            for index, distance in beams
        )
        for x, y in candidate_cells:
            matches = 0
            for offset_x, offset_y in offsets:
                delta_x = x + offset_x - saved_map.origin_x
                delta_y = y + offset_y - saved_map.origin_y
                local_x = map_cosine * delta_x + map_sine * delta_y
                local_y = -map_sine * delta_x + map_cosine * delta_y
                column = math.floor(local_x / saved_map.resolution)
                row = math.floor(local_y / saved_map.resolution)
                if (
                    0 <= column < saved_map.width
                    and 0 <= row < saved_map.height
                    and row * saved_map.width + column in nearby_static
                ):
                    matches += 1
            evaluated += 1
            scored.append((matches / len(beams), x, y, yaw))

    scored.sort(reverse=True, key=lambda item: item[0])
    separation = max(0.0, float(alternative_separation))

    # Endpoint-only scoring creates broad plateaus: several adjacent cells in
    # one room all receive the same score and an arbitrary grid cell can look
    # like an independent alias.  First perform spatial non-maximum
    # suppression, then compare a bounded set of actual pose modes using the
    # full expected range to the first saved-map wall on every beam.
    coarse_modes: list[tuple[float, float, float, float]] = []
    for hypothesis in scored:
        if any(
            math.hypot(hypothesis[1] - mode[1], hypothesis[2] - mode[2])
            < separation
            for mode in coarse_modes
        ):
            continue
        coarse_modes.append(hypothesis)
        if len(coarse_modes) >= 12:
            break

    candidate_coarse = max(
        (
            item for item in scored
            if math.hypot(
                float(candidate_pose[0]) - item[1],
                float(candidate_pose[1]) - item[2],
            ) <= max(0.0, float(candidate_position_tolerance))
            and abs(_angle_delta(float(candidate_pose[2]), item[3]))
            <= max(0.0, float(candidate_yaw_tolerance))
        ),
        key=lambda item: item[0],
        default=None,
    )
    if candidate_coarse is not None:
        candidate_mode_index = next(
            (
                index for index, mode in enumerate(coarse_modes)
                if math.hypot(
                    candidate_coarse[1] - mode[1],
                    candidate_coarse[2] - mode[2],
                ) <= max(saved_map.resolution, float(candidate_position_tolerance))
            ),
            None,
        )
        if candidate_mode_index is None:
            coarse_modes.append(candidate_coarse)
        else:
            # Orientation aliases at one physical position are not an
            # independent *spatial* mode.  Retain AMCL's orientation for that
            # basin and leave yaw corroboration to its dedicated gates.
            coarse_modes[candidate_mode_index] = candidate_coarse

    def range_score(hypothesis: tuple[float, float, float, float]) -> float:
        endpoint_score, x, y, yaw = hypothesis
        agreements: list[float] = []
        tolerance = max(saved_map.resolution, float(endpoint_tolerance))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        scan_x = x + cosine * float(laser_x) - sine * float(laser_y)
        scan_y = y + sine * float(laser_x) + cosine * float(laser_y)
        scan_yaw = yaw + float(laser_yaw)
        for index, measured in beams:
            expected = saved_map.raycast_static_range(
                scan_x,
                scan_y,
                scan_yaw + float(angle_min) + index * float(angle_increment),
                minimum_range=lower,
                maximum_range=upper,
            )
            if expected is None:
                continue
            error = float(measured) - float(expected)
            if error > tolerance:
                # A return beyond the first saved wall means this hypothesis
                # sees through static structure and is strongly contradictory.
                agreements.append(0.0)
            else:
                # Shorter returns may be furniture/people, but must not be
                # silently discarded: doing so lets a wrong pose win with only
                # a handful of convenient beams.
                agreements.append(math.exp(-abs(error) / tolerance))
        if len(agreements) < max(12, len(beams) // 3):
            return 0.0
        residual_score = statistics.fmean(agreements)
        return 0.25 * endpoint_score + 0.75 * residual_score

    refined_modes = sorted(
        (
            (range_score(mode), mode[1], mode[2], mode[3])
            for mode in coarse_modes
        ),
        reverse=True,
        key=lambda item: item[0],
    )
    best = refined_modes[0]
    alternative = (
        refined_modes[1]
        if len(refined_modes) > 1
        else (0.0, math.nan, math.nan, math.nan)
    )
    margin = best[0] - alternative[0]
    ratio = math.inf if alternative[0] <= 1e-12 else best[0] / alternative[0]
    matching_hypothesis = max(
        (
            item for item in refined_modes
            if math.hypot(
                float(candidate_pose[0]) - item[1],
                float(candidate_pose[1]) - item[2],
            ) <= max(0.0, float(candidate_position_tolerance))
            and abs(_angle_delta(float(candidate_pose[2]), item[3]))
            <= max(0.0, float(candidate_yaw_tolerance))
        ),
        key=lambda item: item[0],
        default=None,
    )
    candidate_position_error = (
        math.inf if matching_hypothesis is None else math.hypot(
            float(candidate_pose[0]) - matching_hypothesis[1],
            float(candidate_pose[1]) - matching_hypothesis[2],
        )
    )
    candidate_yaw_error = (
        math.inf if matching_hypothesis is None else abs(
            _angle_delta(float(candidate_pose[2]), matching_hypothesis[3])
        )
    )
    candidate_matches_best = bool(
        matching_hypothesis is not None
        and best[0] - matching_hypothesis[0]
        <= max(0.0, float(candidate_score_tolerance))
    )
    competitive_alternative = global_scan_alternative_is_competitive(
        best[0],
        alternative[0],
        minimum_margin=minimum_score_margin,
        minimum_ratio=minimum_score_ratio,
    )
    accepted = bool(
        best[0] >= max(0.0, min(1.0, float(minimum_best_score)))
        and candidate_matches_best
        and not competitive_alternative
    )
    reason = (
        "ACCEPTED"
        if accepted
        else "GLOBAL_SCAN_BEST_SCORE_TOO_WEAK"
        if best[0] < float(minimum_best_score)
        else "GLOBAL_SCAN_ALTERNATIVE_COMPETITIVE"
        if competitive_alternative
        else "GLOBAL_SCAN_CANDIDATE_NOT_BEST"
    )
    return GlobalScanUniqueness(
        accepted=accepted,
        reason=reason,
        evaluated_candidates=evaluated,
        usable_beams=len(beams),
        best_score=round(best[0], 4),
        alternative_score=round(alternative[0], 4),
        score_margin=round(margin, 4),
        score_ratio=math.inf if math.isinf(ratio) else round(ratio, 4),
        best_x=best[1],
        best_y=best[2],
        best_yaw=best[3],
        alternative_x=None if math.isnan(alternative[1]) else alternative[1],
        alternative_y=None if math.isnan(alternative[2]) else alternative[2],
        alternative_yaw=None if math.isnan(alternative[3]) else alternative[3],
        candidate_position_error=round(candidate_position_error, 4),
        candidate_yaw_error=round(candidate_yaw_error, 4),
    )


def localization_verification(
    *,
    confidence: float,
    confidence_threshold: float,
    pose_stable: bool,
    covariance_xy: float | None,
    covariance_yaw: float | None,
    maximum_covariance_xy: float,
    maximum_covariance_yaw: float,
    scan_valid_beams: int,
    minimum_scan_beams: int,
    scan_score: float,
    required_scan_score: float,
    residual_beams: int,
    minimum_residual_beams: int,
    median_residual: float,
    maximum_median_residual: float,
    p90_residual: float,
    maximum_p90_residual: float,
    raycast_comparable_beams: int,
    minimum_raycast_beams: int,
    raycast_static_matches: int,
    minimum_raycast_static_matches: int,
    raycast_contradiction_ratio: float,
    maximum_raycast_contradiction_ratio: float,
    heading_required: bool,
    heading_ready: bool,
    amcl_fresh: bool,
    scan_map_fresh: bool,
    scan_fresh: bool,
    tf_valid: bool,
    sensor_time_valid: bool,
) -> LocalizationVerification:
    """Evaluate every mandatory acquisition gate without score compensation."""
    if not sensor_time_valid:
        return LocalizationVerification(False, "SENSOR_TIME_INVALID")
    if not tf_valid:
        return LocalizationVerification(False, "TF_UNAVAILABLE")
    if not amcl_fresh:
        return LocalizationVerification(False, "AMCL_STALE")
    if not scan_fresh:
        return LocalizationVerification(False, "SCAN_STALE")
    if not scan_map_fresh:
        return LocalizationVerification(False, "SCAN_MAP_EVIDENCE_STALE")
    if not pose_stable:
        return LocalizationVerification(False, "POSE_UNSTABLE")
    if covariance_xy is None or covariance_yaw is None:
        return LocalizationVerification(False, "COVARIANCE_UNAVAILABLE")
    if (
        covariance_xy > maximum_covariance_xy
        or covariance_yaw > maximum_covariance_yaw
    ):
        return LocalizationVerification(False, "COVARIANCE_TOO_HIGH")
    if scan_valid_beams < minimum_scan_beams:
        return LocalizationVerification(False, "SCAN_INSUFFICIENT_BEAMS")
    if scan_score < required_scan_score:
        return LocalizationVerification(False, "SCAN_SCORE_TOO_LOW")
    if residual_beams < minimum_residual_beams:
        return LocalizationVerification(False, "SCAN_RESIDUALS_INSUFFICIENT")
    if median_residual > maximum_median_residual:
        return LocalizationVerification(False, "SCAN_MEDIAN_RESIDUAL_TOO_HIGH")
    if p90_residual > maximum_p90_residual:
        return LocalizationVerification(False, "SCAN_P90_RESIDUAL_TOO_HIGH")
    if raycast_comparable_beams < minimum_raycast_beams:
        return LocalizationVerification(
            False, "RAYCAST_INSUFFICIENT_COMPARABLE_BEAMS"
        )
    if raycast_contradiction_ratio > maximum_raycast_contradiction_ratio:
        return LocalizationVerification(False, "TOO_MANY_MAP_CONTRADICTIONS")
    if raycast_static_matches < minimum_raycast_static_matches:
        return LocalizationVerification(False, "INSUFFICIENT_STATIC_EVIDENCE")
    if heading_required and not heading_ready:
        return LocalizationVerification(False, "INSUFFICIENT_HEADING_DIVERSITY")
    if confidence < confidence_threshold:
        return LocalizationVerification(False, "LOW_CONFIDENCE")
    return LocalizationVerification(True, "ACCEPTED")


def localization_confidence(
    covariance: list[float] | tuple[float, ...],
    *,
    stability_score: float,
    scan_map_score: float,
    scan_map_threshold: float,
    raycast_match_ratio: float | None = None,
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
    # The acceptance threshold is a gate, not the top of the quality scale.
    # A sublinear curve preserves useful separation across the full 0..1
    # scan range instead of making every score above 0.35 indistinguishable.
    endpoint_quality = max(
        0.0, min(1.0, float(scan_map_score))
    ) ** 0.7
    raycast_source = (
        float(scan_map_score)
        if raycast_match_ratio is None
        else float(raycast_match_ratio)
    )
    raycast_quality = max(0.0, min(1.0, raycast_source)) ** 0.7
    geometry_score = 0.65 * endpoint_quality + 0.35 * raycast_quality
    return round(
        max(0.0, min(1.0, covariance_score * stability * geometry_score)), 4
    )


def compact_lethal_cells(
    message: Any,
    *,
    threshold: int = 100,
    max_cells: int | None = 600,
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
    limit = None if max_cells is None else max(0, int(max_cells))
    if limit == 0:
        return output
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
        if limit is not None and len(output) >= limit:
            break
    return output
