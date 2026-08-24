import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / "navigation-stack"))

import navigation_core  # noqa: E402
from navigation_core import (  # noqa: E402
    ActiveSegment,
    DynamicObstacleOverlay,
    LocalizationEvidenceFrame,
    MapNavigationGeometry,
    NavigationDebugLog,
    RouteMetadata,
    SavedOccupancyMap,
    SensorClockEstimator,
    StopTurnStateLatticePlanner,
    StopTurnRoute,
    TurnBlockTracker,
    UnwrappedYawProgress,
    bounded_heading_evidence,
    canonicalize_stop_turn_path,
    choose_turn_direction,
    classify_planning_failure,
    compact_lethal_cells,
    controller_abort_is_live_blockage,
    densify_straight_segment,
    dynamic_block_requires_alternative,
    dynamic_exclusions_intersect_route,
    dynamic_trajectory_conflict_ttc,
    endpoint_braking_speed_limit,
    evaluate_corridor,
    exact_euclidean_distance_transform,
    execution_pose_continuity,
    filter_static_map_scan,
    find_start_escape,
    global_scan_candidate_uniqueness,
    global_scan_alternative_is_competitive,
    heading_diversity,
    heading_position_spread,
    image_grayscale_values,
    deskew_scan_points,
    localization_confidence,
    localization_evidence_consensus,
    localization_verification,
    mapping_pose_match_quality,
    mask_scan_self_returns,
    normalize_trinary_unknown_metadata,
    path_overlap_ratio,
    particle_cloud_uniqueness,
    position_within_tolerance,
    preferred_turn_bay_directions,
    post_turn_reanchor_requires_turn,
    pose_stability,
    rotation_swept_clearance,
    route_geometry_metadata,
    scan_raycast_consistency,
    scan_to_map_match,
    segment_travel_watchdog,
    straight_heading_lock,
    straight_segment_progress,
    turn_braking_speed_limit,
    turn_hysteresis_transition,
    validate_executable_grid_path,
    validate_rotation_sweep,
    validate_rotation_sweep_neighborhood,
    validate_stop_turn_route,
)
from speed_profiles import (  # noqa: E402
    AutoNavigationProfiles,
    ProfileVelocityLimiter,
    SpeedModeStore,
    SpeedProfileError,
)


def _saved_map(tmp_path: Path) -> SavedOccupancyMap:
    image = Image.new("L", (4, 3), 254)
    image.putpixel((1, 2), 0)    # Occupied: ROS cell (1, 0).
    image.putpixel((2, 1), 128)  # Unknown: ROS cell (2, 1).
    image.save(tmp_path / "map.png")
    (tmp_path / "map.yaml").write_text(
        "image: map.png\nresolution: 0.2\norigin: [-2.0, -3.0, 1.5707963267948966]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    return SavedOccupancyMap.load(tmp_path / "map.yaml")


def test_trinary_gray_205_remains_unknown_with_unsafe_slam_threshold(
    tmp_path: Path,
) -> None:
    image = Image.new("L", (3, 1), 254)
    image.putpixel((1, 0), 205)
    image.putpixel((2, 0), 0)
    image.save(tmp_path / "map.pgm")
    metadata = {
        "image": "map.pgm",
        "resolution": 0.05,
        "origin": [0, 0, 0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "mode": "trinary",
    }
    (tmp_path / "map.yaml").write_text(yaml.safe_dump(metadata))

    normalized, changed = normalize_trinary_unknown_metadata(
        metadata, image_grayscale_values(image)
    )
    saved = SavedOccupancyMap.load(tmp_path / "map.yaml")

    assert changed
    assert normalized["free_thresh"] == pytest.approx(0.196)
    assert saved.occupancy == [0, -1, 100]


def _verification_result(**overrides: object):
    values = {
        "confidence": 0.90,
        "confidence_threshold": 0.72,
        "pose_stable": True,
        "covariance_xy": 0.02,
        "covariance_yaw": 0.01,
        "maximum_covariance_xy": 0.50,
        "maximum_covariance_yaw": 0.50,
        "scan_valid_beams": 40,
        "minimum_scan_beams": 25,
        "scan_score": 0.85,
        "required_scan_score": 0.70,
        "residual_beams": 35,
        "minimum_residual_beams": 20,
        "median_residual": 0.03,
        "maximum_median_residual": 0.075,
        "p90_residual": 0.07,
        "maximum_p90_residual": 0.115,
        "raycast_comparable_beams": 40,
        "minimum_raycast_beams": 25,
        "raycast_static_matches": 30,
        "minimum_raycast_static_matches": 20,
        "raycast_contradiction_ratio": 0.05,
        "maximum_raycast_contradiction_ratio": 0.20,
        "heading_required": True,
        "heading_ready": True,
        "amcl_fresh": True,
        "scan_map_fresh": True,
        "scan_fresh": True,
        "tf_valid": True,
        "sensor_time_valid": True,
    }
    values.update(overrides)
    return localization_verification(**values)


def _boxed_raycast_fixture(
    beam_count: int = 40,
) -> tuple[SavedOccupancyMap, float, float, float, list[float]]:
    size = 120
    occupancy = [0] * (size * size)
    for cell in range(10, 111):
        occupancy[10 * size + cell] = 100
        occupancy[110 * size + cell] = 100
        occupancy[cell * size + 10] = 100
        occupancy[cell * size + 110] = 100
    saved = SavedOccupancyMap(size, size, 0.1, 0.0, 0.0, 0.0, occupancy)
    laser_x = laser_y = 6.05
    angle_min = -math.pi
    increment = 2.0 * math.pi / beam_count
    ranges = [
        saved.raycast_static_range(
            laser_x,
            laser_y,
            angle_min + index * increment,
            minimum_range=0.1,
            maximum_range=8.0,
        )
        for index in range(beam_count)
    ]
    assert all(distance is not None for distance in ranges)
    return saved, laser_x, laser_y, increment, [
        float(distance) for distance in ranges if distance is not None
    ]


def _classified_raycast_fixture(
    *,
    static_matches: int,
    dynamic_occlusions: int,
    map_contradictions: int,
):
    beam_count = static_matches + dynamic_occlusions + map_contradictions
    saved, laser_x, laser_y, increment, expected = _boxed_raycast_fixture(
        beam_count
    )
    measured = list(expected)
    dynamic_end = static_matches + dynamic_occlusions
    for index in range(static_matches, dynamic_end):
        measured[index] = max(0.2, measured[index] - 0.50)
    for index in range(dynamic_end, beam_count):
        measured[index] += 0.50
    return scan_raycast_consistency(
        saved,
        measured,
        angle_min=-math.pi,
        angle_increment=increment,
        range_min=0.1,
        range_max=8.0,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=0.0,
        maximum_beams=beam_count,
        maximum_usable_range=8.0,
        match_tolerance=0.15,
    )


def _cell_center(saved: SavedOccupancyMap, column: int, row: int) -> tuple[float, float]:
    return saved.cell_center(column, row)


def _manual_map(
    width: int,
    height: int,
    free_cells: set[tuple[int, int]],
    *,
    resolution: float = 0.05,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
) -> SavedOccupancyMap:
    occupancy = [
        0 if (column, row) in free_cells else 100
        for row in range(height)
        for column in range(width)
    ]
    saved = SavedOccupancyMap(
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        0.0,
        occupancy,
    )
    saved.navigation_geometry = MapNavigationGeometry.build(saved)
    return saved


def _free_rectangle(
    minimum_column: int,
    maximum_column: int,
    minimum_row: int,
    maximum_row: int,
) -> set[tuple[int, int]]:
    return {
        (column, row)
        for row in range(minimum_row, maximum_row + 1)
        for column in range(minimum_column, maximum_column + 1)
    }


def test_executable_path_rejects_free_centerline_when_physical_footprint_hits_wall() -> None:
    width, height, resolution = 50, 30, 0.05
    costs = [0] * (width * height)
    # Horizontal wall whose cells do not contain the path centerline. The
    # robot's 0.10 m half-width still overlaps it between sparse waypoints.
    for column in range(20, 23):
        costs[10 * width + column] = 100
    result = validate_executable_grid_path(
        [{"x": 0.25, "y": 0.43}, {"x": 2.20, "y": 0.43}],
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_unknown=False,
    )
    assert result.valid is False
    assert result.code == "PATH_FOOTPRINT_COLLISION"
    assert result.cell_cost == 100
    assert result.collision_x == pytest.approx(1.025)
    assert result.collision_y == pytest.approx(0.525)


def test_executable_path_accepts_full_footprint_through_wide_free_corridor() -> None:
    width, height, resolution = 50, 30, 0.05
    costs = [0] * (width * height)
    for column in range(width):
        costs[4 * width + column] = 100
        costs[20 * width + column] = 100
    result = validate_executable_grid_path(
        [
            {"x": 0.25, "y": 0.60},
            {"x": 1.10, "y": 0.72},
            {"x": 2.20, "y": 0.60},
        ],
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_unknown=False,
    )
    assert result.valid is True
    assert result.samples_checked > 70


def test_executable_path_does_not_expand_footprint_twice_over_inscribed_costs() -> None:
    width, height, resolution = 50, 30, 0.05
    costs = [0] * (width * height)
    costs[10 * width + 20] = 99
    beside_inscribed = validate_executable_grid_path(
        [{"x": 0.25, "y": 0.43}, {"x": 2.20, "y": 0.43}],
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_unknown=False,
        lethal_threshold=100,
        inscribed_threshold=99,
    )
    assert beside_inscribed.valid is True

    through_inscribed = validate_executable_grid_path(
        [{"x": 0.25, "y": 0.525}, {"x": 2.20, "y": 0.525}],
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_unknown=False,
        lethal_threshold=100,
        inscribed_threshold=99,
    )
    assert through_inscribed.valid is False
    assert through_inscribed.cell_cost == 99


def test_start_escape_live_validation_allows_only_shrinking_initial_overlap() -> None:
    width, height, resolution = 50, 30, 0.05
    costs = [0] * (width * height)
    for column in range(20, 23):
        costs[10 * width + column] = 100
    escape = [{"x": 1.025, "y": 0.43}, {"x": 1.50, "y": 0.43}]

    ordinary = validate_executable_grid_path(
        escape,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
    )
    bounded_escape = validate_executable_grid_path(
        escape,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_monotonic_initial_overlap=True,
    )

    assert not ordinary.valid
    assert bounded_escape.valid

    # A different cell encountered ahead is not part of the initial overlap
    # and must remain a hard collision.
    costs[10 * width + 27] = 100
    new_collision = validate_executable_grid_path(
        escape,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=costs,
        half_length=0.15,
        half_width=0.10,
        allow_monotonic_initial_overlap=True,
    )
    assert not new_collision.valid
    assert new_collision.code == "PATH_FOOTPRINT_COLLISION"


def test_exact_saved_grid_negative_rotated_origin_and_y_axis(tmp_path: Path) -> None:
    saved = _saved_map(tmp_path)
    assert (saved.width, saved.height, len(saved.occupancy)) == (4, 3, 12)
    for column in range(saved.width):
        for row in range(saved.height):
            assert saved.world_to_cell(*_cell_center(saved, column, row)) == (column, row)
    assert saved.value_at(1, 0) == 100
    assert saved.value_at(2, 1) == -1
    assert saved.value_at(0, 2) == 0


def test_goal_validation_preserves_occupied_unknown_clearance_and_dynamic_cells(tmp_path: Path) -> None:
    saved = _saved_map(tmp_path)
    occupied = _cell_center(saved, 1, 0)
    unknown = _cell_center(saved, 2, 1)
    free = _cell_center(saved, 0, 2)
    assert saved.validate_goal(*occupied, clearance_m=0).code == "GOAL_OCCUPIED"
    assert saved.validate_goal(*unknown, clearance_m=0).code == "GOAL_UNKNOWN"
    assert saved.validate_goal(50, 50, clearance_m=0).code == "GOAL_OUTSIDE_MAP"
    assert saved.validate_goal(*free, clearance_m=0.25).code == "GOAL_CLEARANCE"
    assert saved.validate_goal(
        *free, clearance_m=0.05, lethal_world_cells=[free]
    ).code == "GOAL_LETHAL"
    assert saved.validate_goal(*free, clearance_m=0.05).valid


def test_global_planning_scan_filters_saved_corridor_walls_but_keeps_new_object() -> None:
    occupancy = [0] * (40 * 20)
    for row in range(20):
        occupancy[row * 40 + 24] = 100
    saved = SavedOccupancyMap(40, 20, 0.05, 0, 0, 0, occupancy)

    static = filter_static_map_scan(
        saved,
        [1.18],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=8.0,
        laser_x=0.025,
        laser_y=0.525,
        laser_yaw=0.0,
        expected_range_tolerance=0.08,
    )
    dynamic = filter_static_map_scan(
        saved,
        [0.95],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=8.0,
        laser_x=0.025,
        laser_y=0.525,
        laser_yaw=0.0,
        expected_range_tolerance=0.08,
    )

    assert static.static_map_matches == 1
    assert math.isinf(static.ranges[0])
    assert dynamic.static_map_matches == 0
    assert dynamic.dynamic_points_kept == 1
    assert dynamic.ranges[0] == 0.95  # New object protrudes before saved wall.


def test_static_filter_keeps_beam_when_raycast_is_unknown() -> None:
    saved = SavedOccupancyMap(10, 10, 0.1, 0, 0, 0, [0] * 100)
    filtered = filter_static_map_scan(
        saved,
        [0.5],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=8.0,
        laser_x=0.55,
        laser_y=0.55,
        laser_yaw=0.0,
        expected_range_tolerance=0.08,
    )
    assert filtered.ranges == [0.5]
    assert filtered.raycast_unavailable == 1


def test_physical_footprint_still_rejects_a_genuinely_too_narrow_corridor() -> None:
    occupancy = [0] * (30 * 30)
    for column in range(30):
        occupancy[9 * 30 + column] = 100
        occupancy[11 * 30 + column] = 100
    saved = SavedOccupancyMap(30, 30, 0.1, 0, 0, 0, occupancy)

    validation = saved.validate_footprint(
        1.05,
        1.05,
        0.0,
        half_length=0.15,
        half_width=0.10,
    )

    assert validation.code == "GOAL_FOOTPRINT_BLOCKED"


@pytest.mark.parametrize(
    ("inputs", "expected"),
    [
        ({"tf_ready": False, "costmap_ready": True, "start_cost": 0, "goal_cost": 0, "route_crosses_unknown": False}, "TF_ERROR"),
        ({"tf_ready": True, "costmap_ready": False, "start_cost": None, "goal_cost": None, "route_crosses_unknown": False}, "COSTMAP_NOT_READY"),
        ({"tf_ready": True, "costmap_ready": True, "start_cost": 100, "goal_cost": 0, "route_crosses_unknown": False}, "START_BLOCKED"),
        ({"tf_ready": True, "costmap_ready": True, "start_cost": 0, "goal_cost": 100, "route_crosses_unknown": False}, "GOAL_BLOCKED"),
        ({"tf_ready": True, "costmap_ready": True, "start_cost": 0, "goal_cost": 0, "route_crosses_unknown": True}, "UNKNOWN_SPACE"),
        ({"tf_ready": True, "costmap_ready": True, "start_cost": 0, "goal_cost": 0, "route_crosses_unknown": False}, "NO_VALID_PATH"),
    ],
)
def test_humble_planner_failure_diagnostics_are_specific(inputs: dict, expected: str) -> None:
    assert classify_planning_failure(**inputs) == expected


def test_navigation_debug_file_is_rotating_and_fully_disabled_by_config(tmp_path: Path) -> None:
    disabled_path = tmp_path / "disabled.log"
    disabled = NavigationDebugLog(enabled=False, path=disabled_path)
    assert disabled.event("PLAN_RESULT", status="SUCCESS") == ""
    assert not disabled_path.exists()

    enabled_path = tmp_path / "enabled.log"
    enabled = NavigationDebugLog(
        enabled=True,
        path=enabled_path,
        max_bytes=1024,
        backup_count=2,
    )
    message = enabled.event("STATE", **{"from": "READY", "to": "PLANNING", "reason": "test"})
    enabled.close()
    assert "[NAV][STATE]" in message
    assert "reason=\"test\"" in enabled_path.read_text()


def test_oriented_footprint_catches_collision_missed_by_center_clearance() -> None:
    width = height = 25
    occupancy = [0] * (width * height)
    occupancy[10 * width + 12] = 100
    saved = SavedOccupancyMap(width, height, 0.1, 0, 0, 0, occupancy)
    center = saved.cell_center(10, 10)

    # The legacy 15 cm center circle passes, but the 25 cm half-length body
    # reaches the occupied cell when its long axis points toward it.
    assert saved.validate_goal(*center, clearance_m=0.15).valid
    blocked = saved.validate_footprint(
        *center,
        0.0,
        half_length=0.25,
        half_width=0.08,
    )
    assert blocked.code == "GOAL_FOOTPRINT_BLOCKED"

    rotated = saved.validate_footprint(
        *center,
        math.pi / 2,
        half_length=0.25,
        half_width=0.08,
    )
    assert rotated.valid


def test_goal_snap_validates_complete_oriented_footprint() -> None:
    width = height = 25
    occupancy = [0] * (width * height)
    occupancy[10 * width + 12] = 100
    saved = SavedOccupancyMap(width, height, 0.1, 0, 0, 0, occupancy)
    requested = saved.cell_center(10, 10)

    snapped = saved.nearest_valid_goal(
        *requested,
        clearance_m=0.15,
        max_distance_m=0.6,
        yaw=0.0,
        footprint_half_length=0.25,
        footprint_half_width=0.08,
    )

    assert snapped is not None
    assert saved.validate_footprint(
        *snapped,
        0.0,
        half_length=0.25,
        half_width=0.08,
    ).valid


def test_nearest_valid_goal_snaps_unsafe_click_within_a_strict_bound() -> None:
    occupancy = [0] * (15 * 15)
    occupancy[7 * 15 + 7] = 100
    saved = SavedOccupancyMap(15, 15, 0.1, -1.0, -2.0, math.pi / 6, occupancy)
    requested = saved.cell_center(7, 7)

    snapped = saved.nearest_valid_goal(
        *requested,
        clearance_m=0.15,
        max_distance_m=0.45,
    )

    assert snapped is not None
    assert math.dist(requested, snapped) <= 0.45
    assert saved.validate_goal(*snapped, clearance_m=0.15).valid


def test_nearest_valid_goal_never_moves_an_outside_or_unresolvable_click() -> None:
    saved = SavedOccupancyMap(5, 5, 0.1, 0, 0, 0, [100] * 25)
    assert saved.nearest_valid_goal(
        -1, -1, clearance_m=0.1, max_distance_m=0.45,
    ) is None
    assert saved.nearest_valid_goal(
        0.25, 0.25, clearance_m=0.1, max_distance_m=0.45,
    ) is None


def test_localization_requires_fresh_scan_tf_low_covariance_and_stability() -> None:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.01
    covariance[35] = 0.02
    confidence = localization_confidence(
        covariance,
        stability_score=1.0,
        scan_map_score=0.6,
        scan_map_threshold=0.35,
        scan_fresh=True,
        tf_stable=True,
        odometry_healthy=True,
        sensor_time_valid=True,
    )
    assert 0.60 < confidence < 0.75
    assert localization_confidence(
        covariance,
        stability_score=0.2,
        scan_map_score=0.6,
        scan_map_threshold=0.35,
        scan_fresh=True,
        tf_stable=True,
        odometry_healthy=True,
        sensor_time_valid=True,
    ) < 0.25
    assert localization_confidence(
        covariance,
        stability_score=1.0,
        scan_map_score=0.6,
        scan_map_threshold=0.35,
        scan_fresh=False,
        tf_stable=True,
        odometry_healthy=True,
        sensor_time_valid=True,
    ) == 0


def test_untrusted_global_candidate_keeps_final_global_scan_threshold() -> None:
    verdict = _verification_result(
        scan_score=0.55,
        required_scan_score=0.70,
        confidence=0.99,
        covariance_xy=0.001,
        covariance_yaw=0.001,
        heading_required=True,
        heading_ready=True,
    )
    assert not verdict.accepted
    assert verdict.reason == "SCAN_SCORE_TOO_LOW"


def test_localization_confidence_preserves_scan_quality_separation() -> None:
    covariance = [0.0] * 36
    scores = [0.40, 0.55, 0.75, 0.90]
    confidences = [
        localization_confidence(
            covariance,
            stability_score=1.0,
            scan_map_score=score,
            scan_map_threshold=0.35,
            raycast_match_ratio=0.80,
            scan_fresh=True,
            tf_stable=True,
            odometry_healthy=True,
            sensor_time_valid=True,
        )
        for score in scores
    ]

    assert confidences == sorted(confidences)
    assert len(set(confidences)) == len(scores)
    assert confidences[-1] < 0.99
    assert confidences[1] < confidences[2] - 0.05


def test_localization_confidence_rejects_stable_wrong_pose_and_bad_clock() -> None:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = 0.001
    common = {
        "stability_score": 1.0,
        "scan_map_threshold": 0.35,
        "scan_fresh": True,
        "tf_stable": True,
        "odometry_healthy": True,
        "sensor_time_valid": True,
    }
    assert localization_confidence(
        covariance, scan_map_score=0.05, **common
    ) < 0.2
    assert localization_confidence(
        covariance,
        scan_map_score=0.8,
        **{**common, "sensor_time_valid": False},
    ) == 0


def test_mapping_pose_hint_is_corrected_by_slam_not_treated_as_ground_truth() -> None:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.04
    covariance[35] = math.radians(8) ** 2
    result = mapping_pose_match_quality(
        (1.0, 2.0, 0.0),
        (1.45, 1.8, math.radians(30)),
        covariance,
        maximum_position_correction=1.5,
        maximum_yaw_correction=math.radians(75),
        maximum_xy_stddev=0.75,
        maximum_yaw_stddev=math.radians(45),
    )

    assert result["accepted"]
    assert result["reason"] == "SLAM_POSE_CONFIRMED"
    assert result["position_correction_m"] > 0.4


@pytest.mark.parametrize(
    ("corrected", "variance", "reason"),
    [
        ((3.0, 2.0, 0.0), 0.01, "POSITION_CORRECTION_TOO_LARGE"),
        ((1.0, 2.0, math.pi), 0.01, "YAW_CORRECTION_TOO_LARGE"),
        ((1.1, 2.0, 0.1), 1.0, "POSITION_UNCERTAIN"),
    ],
)
def test_mapping_pose_match_rejects_distant_or_uncertain_correction(
    corrected: tuple[float, float, float],
    variance: float,
    reason: str,
) -> None:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = variance
    covariance[35] = 0.01
    result = mapping_pose_match_quality(
        (1.0, 2.0, 0.0),
        corrected,
        covariance,
        maximum_position_correction=1.5,
        maximum_yaw_correction=math.radians(75),
        maximum_xy_stddev=0.75,
        maximum_yaw_stddev=math.radians(45),
    )

    assert not result["accepted"]
    assert result["reason"] == reason


def _localization_frame(
    index: int,
    *,
    good: bool = True,
    x: float = 1.0,
    dynamic_occlusions: int = 4,
) -> LocalizationEvidenceFrame:
    return LocalizationEvidenceFrame(
        timestamp=float(index),
        pose_x=x,
        pose_y=2.0,
        pose_yaw=0.2,
        scan_score=0.84 if good else 0.18,
        valid_beams=80,
        residual_beams=60 if good else 8,
        median_residual=0.035 if good else 0.30,
        p90_residual=0.075 if good else 0.50,
        mean_residual=0.045 if good else 0.35,
        raycast_comparable_beams=60,
        raycast_static_matches=32 if good else 3,
        raycast_dynamic_occlusions=dynamic_occlusions,
        raycast_map_contradictions=3 if good else 35,
        raycast_inconclusive_map_hits=2,
        raycast_static_match_ratio=0.53 if good else 0.05,
        raycast_dynamic_occlusion_ratio=dynamic_occlusions / 60.0,
        raycast_contradiction_ratio=0.05 if good else 0.58,
        raycast_median_error=0.03 if good else 0.30,
        raycast_p90_error=0.07 if good else 0.50,
    )


def _localization_consensus(frames: list[LocalizationEvidenceFrame]):
    return localization_evidence_consensus(
        frames,
        window_size=7,
        required_frames=5,
        candidate_position_tolerance=0.10,
        candidate_yaw_tolerance=math.radians(10),
        minimum_scan_beams=25,
        required_scan_score=0.70,
        minimum_residual_beams=20,
        maximum_median_residual=0.075,
        maximum_p90_residual=0.115,
        minimum_raycast_beams=25,
        minimum_raycast_static_matches=20,
        maximum_raycast_contradiction_ratio=0.20,
    )


def test_localization_k_of_n_survives_two_transient_bad_scans_including_latest() -> None:
    frames = [_localization_frame(index) for index in range(5)]
    frames.extend([
        _localization_frame(5, good=False),
        _localization_frame(6, good=False),
    ])

    consensus = _localization_consensus(frames)

    assert consensus.accepted
    assert consensus.agreeing_frames == 5
    assert consensus.scan_score == pytest.approx(0.84)


def test_localization_accepts_dynamic_occlusion_only_with_static_structure() -> None:
    frames = [
        _localization_frame(index, dynamic_occlusions=25)
        for index in range(7)
    ]

    consensus = _localization_consensus(frames)

    assert consensus.accepted
    assert consensus.raycast_static_matches == 32
    assert consensus.raycast_dynamic_occlusions == 25


def test_localization_rejects_persistent_contradiction_and_pose_alias_split() -> None:
    contradictory = [
        *[_localization_frame(index) for index in range(4)],
        *[_localization_frame(index, good=False) for index in range(4, 7)],
    ]
    aliases = [
        *[_localization_frame(index, x=1.0) for index in range(4)],
        *[_localization_frame(index, x=2.0) for index in range(4, 7)],
    ]

    assert _localization_consensus(contradictory).reason == "CONSENSUS_K_OF_N_FAILED"
    assert _localization_consensus(aliases).reason == "CANDIDATE_POSE_CONSENSUS_FAILED"


def test_particle_cloud_requires_one_dominant_spatial_hypothesis() -> None:
    unique = particle_cloud_uniqueness(
        [(1.0, 1.0, 0.35), (1.05, 1.02, 0.35), (3.0, 1.0, 0.30)],
        cluster_radius=0.30,
        alternative_separation=0.75,
        minimum_best_weight=0.55,
        minimum_dominance_ratio=2.0,
    )
    ambiguous = particle_cloud_uniqueness(
        [(1.0, 1.0, 0.52), (3.0, 1.0, 0.48)],
        cluster_radius=0.30,
        alternative_separation=0.75,
        minimum_best_weight=0.55,
        minimum_dominance_ratio=2.0,
    )

    assert unique.accepted
    assert unique.dominance_ratio > 2.0
    assert not ambiguous.accepted
    assert ambiguous.reason == "BEST_PARTICLE_CLUSTER_TOO_WEAK"


def test_independent_global_scan_rejects_collapsed_amcl_alias() -> None:
    width, height = 180, 100
    occupancy = [-1] * (width * height)

    def add_room(left: int, right: int) -> None:
        for row in range(10, 91):
            for column in range(left, right + 1):
                occupancy[row * width + column] = 0
        for column in range(left, right + 1):
            occupancy[10 * width + column] = 100
            occupancy[90 * width + column] = 100
        for row in range(10, 91):
            occupancy[row * width + left] = 100
            occupancy[row * width + right] = 100

    add_room(10, 70)
    add_room(100, 160)
    saved = SavedOccupancyMap(width, height, 0.1, 0.0, 0.0, 0.0, occupancy)
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / 72
    source_pose = (4.05, 5.05, 0.0)
    ranges = [
        saved.raycast_static_range(
            source_pose[0],
            source_pose[1],
            source_pose[2] + angle_min + index * angle_increment,
            minimum_range=0.1,
            maximum_range=8.0,
        )
        for index in range(72)
    ]
    assert all(distance is not None for distance in ranges)

    result = global_scan_candidate_uniqueness(
        saved,
        [float(distance) for distance in ranges if distance is not None],
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=8.0,
        candidate_pose=source_pose,
        position_step=0.2,
        heading_step=math.radians(15.0),
        alternative_separation=1.5,
        minimum_best_score=0.65,
        minimum_score_margin=0.12,
        minimum_score_ratio=1.15,
    )

    assert not result.accepted
    assert result.reason == "GLOBAL_SCAN_ALTERNATIVE_COMPETITIVE"
    assert result.best_score == pytest.approx(result.alternative_score)
    assert math.hypot(
        float(result.best_x) - float(result.alternative_x),
        float(result.best_y) - float(result.alternative_y),
    ) >= 1.5


def test_global_scan_modes_accept_either_absolute_or_relative_separation() -> None:
    assert not global_scan_alternative_is_competitive(
        0.5133,
        0.4394,
        minimum_margin=0.12,
        minimum_ratio=1.15,
    )
    assert not global_scan_alternative_is_competitive(
        0.80,
        0.67,
        minimum_margin=0.12,
        minimum_ratio=1.25,
    )
    assert global_scan_alternative_is_competitive(
        0.50,
        0.47,
        minimum_margin=0.12,
        minimum_ratio=1.15,
    )


def test_independent_global_scan_accepts_one_unique_room_center() -> None:
    saved, laser_x, laser_y, increment, ranges = _boxed_raycast_fixture(72)
    result = global_scan_candidate_uniqueness(
        saved,
        ranges,
        angle_min=-math.pi,
        angle_increment=increment,
        range_min=0.1,
        range_max=8.0,
        candidate_pose=(laser_x, laser_y, 0.0),
        position_step=0.2,
        heading_step=math.radians(15.0),
        alternative_separation=1.5,
        minimum_best_score=0.65,
        minimum_score_margin=0.10,
        minimum_score_ratio=1.10,
    )

    assert result.accepted, result
    assert result.reason == "ACCEPTED"
    assert result.candidate_position_error <= 0.45


def test_mapping_pose_search_discovers_heading_instead_of_trusting_wrong_hint() -> None:
    width, height = 110, 80
    occupancy = [-1] * (width * height)
    for row in range(5, 76):
        for column in range(5, 106):
            occupancy[row * width + column] = 0
    for column in range(5, 106):
        occupancy[5 * width + column] = 100
        occupancy[75 * width + column] = 100
    for row in range(5, 76):
        occupancy[row * width + 5] = 100
        occupancy[row * width + 105] = 100
    # Two asymmetric internal walls make the correct yaw distinguishable.
    for row in range(15, 42):
        occupancy[row * width + 70] = 100
    for column in range(25, 48):
        occupancy[55 * width + column] = 100
    saved = SavedOccupancyMap(
        width, height, 0.1, 0.0, 0.0, 0.0, occupancy
    )
    actual = (4.15, 3.25, math.radians(70.0))
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / 72
    ranges = [
        saved.raycast_static_range(
            actual[0],
            actual[1],
            actual[2] + angle_min + index * angle_increment,
            minimum_range=0.1,
            maximum_range=8.0,
        )
        for index in range(72)
    ]

    result = global_scan_candidate_uniqueness(
        saved,
        [float(value) if value is not None else math.inf for value in ranges],
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=8.0,
        candidate_pose=(actual[0], actual[1], math.radians(-80.0)),
        position_step=0.2,
        heading_step=math.radians(15.0),
        alternative_separation=0.75,
        minimum_best_score=0.42,
        minimum_score_margin=0.08,
        minimum_score_ratio=1.12,
        search_center=(actual[0], actual[1]),
        search_radius=1.0,
        require_candidate_match=False,
        alternative_yaw_separation=math.radians(45.0),
    )

    assert result.accepted, result
    assert math.hypot(
        float(result.best_x) - actual[0], float(result.best_y) - actual[1]
    ) <= 0.2
    assert abs(math.atan2(
        math.sin(float(result.best_yaw) - actual[2]),
        math.cos(float(result.best_yaw) - actual[2]),
    )) <= math.radians(10.0)


def test_mapping_pose_search_rejects_competing_headings_at_same_position() -> None:
    saved, laser_x, laser_y, increment, ranges = _boxed_raycast_fixture(72)
    result = global_scan_candidate_uniqueness(
        saved,
        ranges,
        angle_min=-math.pi,
        angle_increment=increment,
        range_min=0.1,
        range_max=8.0,
        candidate_pose=(laser_x, laser_y, 0.3),
        position_step=0.2,
        heading_step=math.radians(15.0),
        alternative_separation=0.75,
        minimum_best_score=0.42,
        minimum_score_margin=0.08,
        minimum_score_ratio=1.12,
        search_center=(laser_x, laser_y),
        search_radius=0.3,
        require_candidate_match=False,
        alternative_yaw_separation=math.radians(45.0),
    )

    assert not result.accepted
    assert result.reason == "GLOBAL_SCAN_ALTERNATIVE_COMPETITIVE"
    assert result.best_x == pytest.approx(result.alternative_x)
    assert result.best_y == pytest.approx(result.alternative_y)
    assert abs(math.atan2(
        math.sin(float(result.best_yaw) - float(result.alternative_yaw)),
        math.cos(float(result.best_yaw) - float(result.alternative_yaw)),
    )) >= math.radians(45.0)


def test_execution_pose_continuity_accepts_the_same_motion_in_map_and_odom() -> None:
    local_x, local_y = 0.10, 0.02

    def moved(pose: dict[str, float]) -> dict[str, float]:
        yaw = pose["yaw"]
        return {
            "x": pose["x"] + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            "y": pose["y"] + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
            "yaw": yaw + 0.05,
        }

    previous_map = {"x": 1.8, "y": 1.2, "yaw": 1.0}
    previous_odom = {"x": 0.3, "y": -0.2, "yaw": 0.2}
    result = execution_pose_continuity(
        previous_map,
        moved(previous_map),
        previous_odom,
        moved(previous_odom),
        maximum_translation_residual=0.12,
        maximum_yaw_residual=math.radians(20.0),
    )

    assert result.consistent
    assert result.translation_residual == pytest.approx(0.0, abs=1e-9)
    assert result.yaw_residual == pytest.approx(0.0, abs=1e-9)


def test_execution_pose_continuity_rejects_latest_amcl_jump_without_odom_motion() -> None:
    result = execution_pose_continuity(
        {"x": 1.8127, "y": 1.2561, "yaw": 2.7711},
        {"x": 0.2887, "y": 0.7899, "yaw": 0.0034},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.03, "y": 0.0, "yaw": 0.01},
        maximum_translation_residual=0.12,
        maximum_yaw_residual=math.radians(20.0),
    )

    assert not result.consistent
    assert result.translation_residual > 1.4
    assert result.yaw_residual > math.radians(150.0)


def test_shared_sensor_clock_preserves_capture_intervals_with_large_offset() -> None:
    clock = SensorClockEstimator(minimum_sync_samples=5)
    offset = 1_019_650_000_000
    accepted = []
    for index, sensor in enumerate(("scan", "odom", "imu", "imu", "scan", "odom")):
        source = 10_000_000_000 + index * 50_000_000
        delay = (index % 3 + 1) * 2_000_000
        result = clock.observe(
            sensor,
            source_nanoseconds=source,
            arrival_nanoseconds=source + offset + delay,
        )
        if result.accepted:
            accepted.append((source, result.corrected_nanoseconds))
    assert clock.state == "SYNCED"
    assert len(accepted) >= 2
    assert accepted[-1][1] - accepted[-2][1] == accepted[-1][0] - accepted[-2][0]
    assert abs(clock.offset_nanoseconds / 1e9 - 1019.652) < 0.001


def test_one_bad_timestamp_is_dropped_without_invalidating_the_clock() -> None:
    clock = SensorClockEstimator(minimum_sync_samples=3, invalid_debounce_samples=3)
    for index in range(3):
        source = 1_000_000_000 + index * 100_000_000
        clock.observe(
            "scan",
            source_nanoseconds=source,
            arrival_nanoseconds=source + 5_000_000_000,
        )
    bad = clock.observe(
        "scan", source_nanoseconds=0, arrival_nanoseconds=7_000_000_000
    )
    recovered = clock.observe(
        "scan",
        source_nanoseconds=1_400_000_000,
        arrival_nanoseconds=6_400_000_000,
    )
    assert not bad.accepted
    assert bad.state == "SYNCED"
    assert recovered.accepted
    assert clock.state == "SYNCED"


def test_persistent_clock_jump_becomes_invalid_and_relearns() -> None:
    clock = SensorClockEstimator(minimum_sync_samples=3, invalid_debounce_samples=3)
    source = 1_000_000_000
    for index in range(3):
        value = source + index * 100_000_000
        clock.observe("odom", source_nanoseconds=value, arrival_nanoseconds=value + 5_000_000_000)
    for index in range(3, 6):
        value = source + index * 100_000_000
        result = clock.observe("odom", source_nanoseconds=value, arrival_nanoseconds=value + 15_000_000_000)
    assert result.state == "SENSOR_TIME_INVALID"
    for index in range(6, 9):
        value = source + index * 100_000_000
        result = clock.observe("odom", source_nanoseconds=value, arrival_nanoseconds=value + 15_000_000_000)
    assert result.accepted
    assert clock.state == "SYNCED"


def test_common_mcu_clock_reset_clears_monotonic_history_and_relearns() -> None:
    clock = SensorClockEstimator(minimum_sync_samples=3, invalid_debounce_samples=3)
    for index in range(6):
        source = 50_000_000_000 + index * 20_000_000
        clock.observe(
            ("scan", "odom", "imu")[index % 3],
            source_nanoseconds=source,
            arrival_nanoseconds=source + 5_000_000_000,
        )

    for index in range(3):
        result = clock.observe(
            ("scan", "odom", "imu")[index],
            source_nanoseconds=1_000_000_000 + index * 20_000_000,
            arrival_nanoseconds=60_000_000_000 + index * 20_000_000,
        )
    assert not result.accepted
    assert clock.state == "SENSOR_TIME_INVALID"

    for index in range(4):
        result = clock.observe(
            ("scan", "odom", "imu")[index % 3],
            source_nanoseconds=2_000_000_000 + index * 20_000_000,
            arrival_nanoseconds=61_000_000_000 + index * 20_000_000,
        )
    assert result.accepted
    assert clock.state == "SYNCED"


def test_pose_stability_uses_window_median_and_circular_yaw() -> None:
    stable = pose_stability([
        (0.0, 1.00, 2.00, math.pi - 0.02),
        (0.3, 1.01, 2.01, -math.pi + 0.01),
        (0.6, 0.99, 2.00, math.pi - 0.01),
        (0.9, 1.00, 1.99, -math.pi + 0.02),
        (1.2, 1.01, 2.00, math.pi),
    ])
    assert stable.passes(
        minimum_samples=5,
        minimum_duration_seconds=1.0,
        maximum_xy_spread=0.05,
        maximum_median_deviation=0.03,
        maximum_yaw_variance=0.01,
        maximum_yaw_spread=0.05,
    )
    unstable = pose_stability([
        (0.0, 1.0, 2.0, 0.0),
        (0.3, 1.0, 2.0, 0.0),
        (0.6, 2.0, 2.0, 1.0),
        (0.9, 1.0, 2.0, 0.0),
        (1.2, 1.0, 2.0, 0.0),
    ])
    assert unstable.xy_spread > 0.5
    assert unstable.yaw_spread > 0.5


def test_scan_map_match_distinguishes_correct_and_wrong_pose() -> None:
    occupancy = [0] * (100 * 100)
    saved = SavedOccupancyMap(100, 100, 0.1, 0.0, 0.0, 0.0, occupancy)
    endpoints = [(7.0, 5.0), (5.0, 7.0), (3.0, 5.0), (5.0, 3.0)]
    for x, y in endpoints:
        column, row = saved.world_to_cell(x, y) or (-1, -1)
        saved.occupancy[row * saved.width + column] = 100
    correct = scan_to_map_match(
        saved,
        [2.0, 2.0, 2.0, 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=8.0,
        laser_x=5.0,
        laser_y=5.0,
        laser_yaw=0.0,
        maximum_beams=4,
        endpoint_tolerance=0.11,
    )
    wrong = scan_to_map_match(
        saved,
        [2.0, 2.0, 2.0, 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=8.0,
        laser_x=5.8,
        laser_y=5.0,
        laser_yaw=0.0,
        maximum_beams=4,
        endpoint_tolerance=0.11,
    )
    assert correct.score == 1.0
    assert wrong.score == 0.0


def test_scan_map_residual_rejects_offset_hidden_by_coarse_score() -> None:
    occupancy = [0] * (300 * 30)
    for row in range(30):
        occupancy[row * 300 + 200] = 100
    saved = SavedOccupancyMap(300, 30, 0.01, 0.0, 0.0, 0.0, occupancy)
    common = {
        "angle_min": 0.0,
        "angle_increment": 0.0,
        "range_min": 0.1,
        "range_max": 8.0,
        "laser_x": 1.0,
        "laser_y": 0.155,
        "laser_yaw": 0.0,
        "maximum_beams": 4,
        "endpoint_tolerance": 0.12,
    }
    good = scan_to_map_match(saved, [0.98, 0.97, 0.96, 0.95], **common)
    offset = scan_to_map_match(saved, [0.90, 0.91, 0.92, 0.93], **common)

    assert good.score == offset.score == 1.0
    assert good.median_residual < 0.05
    assert good.p90_residual < 0.07
    assert offset.median_residual > 0.07
    assert offset.p90_residual > 0.07


def test_endpoint_match_cannot_hide_wrong_first_hit_raycast() -> None:
    size = 120
    saved = SavedOccupancyMap(
        size, size, 0.1, 0.0, 0.0, 0.0, [0] * (size * size)
    )
    laser_x = laser_y = 5.05
    # Each false first hit belongs to a substantial mapped wall, so seeing
    # through it remains a conclusive contradiction rather than map speckle.
    for offset in range(-4, 5):
        for column, row in (
            (60, 50 + offset),
            (50 + offset, 60),
            (40, 50 + offset),
            (50 + offset, 40),
        ):
            saved.occupancy[row * size + column] = 100
    for column, row in ((70, 50), (50, 70), (30, 50), (50, 30)):
        saved.occupancy[row * size + column] = 100
    common = {
        "angle_min": 0.0,
        "angle_increment": math.pi / 2,
        "range_min": 0.1,
        "range_max": 8.0,
        "laser_x": laser_x,
        "laser_y": laser_y,
        "laser_yaw": 0.0,
        "maximum_beams": 4,
    }
    endpoint = scan_to_map_match(
        saved, [2.0] * 4, endpoint_tolerance=0.12, **common
    )
    raycast = scan_raycast_consistency(
        saved, [2.0] * 4, match_tolerance=0.15, **common
    )

    assert endpoint.score == 1.0
    assert raycast.comparable_beams == 4
    assert raycast.static_matches == 0
    assert raycast.dynamic_occlusions == 0
    assert raycast.map_contradictions == 4
    assert raycast.contradiction_ratio == 1.0
    verdict = _verification_result(
        scan_valid_beams=endpoint.valid_beams,
        minimum_scan_beams=4,
        scan_score=endpoint.score,
        residual_beams=endpoint.residual_beams,
        minimum_residual_beams=4,
        median_residual=endpoint.median_residual,
        p90_residual=endpoint.p90_residual,
        raycast_comparable_beams=raycast.comparable_beams,
        minimum_raycast_beams=4,
        raycast_static_matches=raycast.static_matches,
        minimum_raycast_static_matches=4,
        raycast_contradiction_ratio=raycast.contradiction_ratio,
    )
    assert not verdict.accepted
    assert verdict.reason == "TOO_MANY_MAP_CONTRADICTIONS"


def test_missing_short_map_object_is_inconclusive_not_pose_contradiction() -> None:
    size = 100
    saved = SavedOccupancyMap(
        size, size, 0.1, 0.0, 0.0, 0.0, [0] * (size * size)
    )
    # The scan sees a mapped endpoint at 2 m, beyond an isolated stale pixel
    # at 1 m. The short object is not trustworthy enough to disprove the pose.
    saved.occupancy[50 * size + 60] = 100
    saved.occupancy[50 * size + 70] = 100
    raycast = scan_raycast_consistency(
        saved,
        [2.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=8.0,
        laser_x=5.05,
        laser_y=5.05,
        laser_yaw=0.0,
        maximum_beams=1,
        match_tolerance=0.15,
        minimum_reliable_structure_span=0.75,
    )

    assert raycast.comparable_beams == 0
    assert raycast.map_contradictions == 0
    assert raycast.inconclusive_map_hits == 1


def test_angled_continuous_wall_is_reliable_static_structure() -> None:
    size = 100
    saved = SavedOccupancyMap(
        size, size, 0.05, 0.0, 0.0, 0.0, [0] * (size * size)
    )
    center_column = center_row = 50
    angle = math.radians(30.0)
    for step in range(-10, 11):
        column = round(center_column + step * math.cos(angle))
        row = round(center_row + step * math.sin(angle))
        saved.occupancy[row * size + column] = 100

    assert saved.is_reliable_static_structure(
        center_column,
        center_row,
        minimum_span=0.75,
    )


def test_correct_pose_passes_endpoint_and_raycast_verification() -> None:
    saved, laser_x, laser_y, increment, ranges = _boxed_raycast_fixture()
    common = {
        "angle_min": -math.pi,
        "angle_increment": increment,
        "range_min": 0.1,
        "range_max": 8.0,
        "laser_x": laser_x,
        "laser_y": laser_y,
        "laser_yaw": 0.0,
        "maximum_beams": 40,
    }
    endpoint = scan_to_map_match(
        saved, ranges, endpoint_tolerance=0.12, **common
    )
    raycast = scan_raycast_consistency(
        saved, ranges, match_tolerance=0.15, **common
    )
    verdict = _verification_result(
        scan_valid_beams=endpoint.valid_beams,
        scan_score=endpoint.score,
        residual_beams=endpoint.residual_beams,
        median_residual=endpoint.median_residual,
        p90_residual=endpoint.p90_residual,
        raycast_comparable_beams=raycast.comparable_beams,
        raycast_static_matches=raycast.static_matches,
        raycast_contradiction_ratio=raycast.contradiction_ratio,
    )

    assert endpoint.score >= 0.9
    assert raycast.static_match_ratio == 1.0
    assert raycast.dynamic_occlusions == 0
    assert raycast.map_contradictions == 0
    assert verdict.accepted
    assert verdict.reason == "ACCEPTED"


def test_raycast_verification_tolerates_partial_dynamic_occlusion() -> None:
    saved, laser_x, laser_y, increment, ranges = _boxed_raycast_fixture()
    occluded = list(ranges)
    for index in range(0, 40, 5):
        occluded[index] = max(0.2, occluded[index] - 0.75)
    raycast = scan_raycast_consistency(
        saved,
        occluded,
        angle_min=-math.pi,
        angle_increment=increment,
        range_min=0.1,
        range_max=8.0,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=0.0,
        maximum_beams=40,
        maximum_usable_range=8.0,
        match_tolerance=0.15,
    )
    verdict = _verification_result(
        raycast_comparable_beams=raycast.comparable_beams,
        raycast_static_matches=raycast.static_matches,
        raycast_contradiction_ratio=raycast.contradiction_ratio,
    )

    assert raycast.static_matches == 32
    assert raycast.dynamic_occlusions == 8
    assert raycast.map_contradictions == 0
    assert verdict.accepted


def test_insufficient_raycast_evidence_never_defaults_to_pass() -> None:
    verdict = _verification_result(
        raycast_comparable_beams=8,
        raycast_static_matches=8,
        raycast_contradiction_ratio=0.0,
    )
    assert not verdict.accepted
    assert verdict.reason == "RAYCAST_INSUFFICIENT_COMPARABLE_BEAMS"


def test_dynamic_occlusion_is_not_equivalent_to_map_contradiction() -> None:
    occluded = _classified_raycast_fixture(
        static_matches=30,
        dynamic_occlusions=40,
        map_contradictions=5,
    )
    contradictory = _classified_raycast_fixture(
        static_matches=30,
        dynamic_occlusions=5,
        map_contradictions=40,
    )

    occluded_verdict = _verification_result(
        raycast_comparable_beams=occluded.comparable_beams,
        raycast_static_matches=occluded.static_matches,
        raycast_contradiction_ratio=occluded.contradiction_ratio,
    )
    contradictory_verdict = _verification_result(
        raycast_comparable_beams=contradictory.comparable_beams,
        raycast_static_matches=contradictory.static_matches,
        raycast_contradiction_ratio=contradictory.contradiction_ratio,
    )

    assert occluded.dynamic_occlusions == 40
    assert occluded.map_contradictions == 5
    assert occluded_verdict.accepted
    assert contradictory.dynamic_occlusions == 5
    assert contradictory.map_contradictions == 40
    assert not contradictory_verdict.accepted
    assert contradictory_verdict.reason == "TOO_MANY_MAP_CONTRADICTIONS"


def test_extreme_dynamic_occlusion_fails_without_static_evidence() -> None:
    raycast = _classified_raycast_fixture(
        static_matches=8,
        dynamic_occlusions=67,
        map_contradictions=5,
    )
    verdict = _verification_result(
        raycast_comparable_beams=raycast.comparable_beams,
        raycast_static_matches=raycast.static_matches,
        raycast_contradiction_ratio=raycast.contradiction_ratio,
    )

    assert not verdict.accepted
    assert verdict.reason == "INSUFFICIENT_STATIC_EVIDENCE"


def test_force_rescan_heading_diversity_rejects_a_thirty_degree_cluster() -> None:
    clustered = heading_diversity(
        map(math.radians, [0, 10, 20, 30]), bin_count=8
    )
    diverse = heading_diversity(
        map(math.radians, [0, 60, 120, 180]), bin_count=8
    )
    assert len(clustered.observed_bins) < 4
    assert math.degrees(clustered.span_radians) == pytest.approx(30)
    assert len(diverse.observed_bins) >= 4
    assert math.degrees(diverse.span_radians) == pytest.approx(180)


def test_force_rescan_accepts_real_non_cardinal_heading_coverage() -> None:
    observed = heading_diversity(
        map(math.radians, [3, 47, 92, 139, 176]), bin_count=8
    )
    assert len(observed.observed_bins) >= 4
    assert math.degrees(observed.span_radians) >= 150


def test_force_rescan_heading_bins_progress_during_rotation() -> None:
    headings = [0, 48, 96, 144, 192, 240]
    observed_counts = [
        len(heading_diversity(
            map(math.radians, headings[:count]), bin_count=8
        ).observed_bins)
        for count in range(1, len(headings) + 1)
    ]

    assert observed_counts[0] == 1
    assert observed_counts == sorted(observed_counts)
    assert observed_counts[-1] >= 5


def test_heading_evidence_is_bounded_by_angular_bins() -> None:
    evidence: list[float] = []
    for index in range(10_000):
        evidence = bounded_heading_evidence(
            evidence,
            math.radians((index % 360) + 0.1),
            bin_count=8,
        )

    observed = heading_diversity(evidence, bin_count=8)

    assert len(evidence) == 8
    assert observed.observed_bins == tuple(range(8))
    assert math.degrees(observed.span_radians) >= 170


def test_heading_diversity_keeps_exact_wraparound_span_for_large_input() -> None:
    headings = [
        math.radians(value)
        for _ in range(1_000)
        for value in (350, 5, 170)
    ]

    observed = heading_diversity(headings, bin_count=8)

    assert math.degrees(observed.span_radians) == pytest.approx(180)


def test_repeated_layout_candidate_must_stay_consistent_across_headings() -> None:
    consistent_candidate = [
        (1.00, 2.00),
        (1.03, 1.98),
        (0.98, 2.02),
        (1.01, 2.01),
    ]
    aliased_candidate = [
        *consistent_candidate[:2],
        (2.10, 2.05),
    ]

    assert heading_position_spread(consistent_candidate) < 0.12
    assert heading_position_spread(aliased_candidate) > 1.0


def test_40cm_corridor_allows_straight_but_not_unsafe_rotation() -> None:
    walls_40cm = [
        (x, side)
        for x in (-0.15, 0.0, 0.15, 0.5)
        for side in (-0.20, 0.20)
    ]
    assessment = evaluate_corridor(
        walls_40cm,
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        localization_uncertainty=0.02,
        rotation_margin=0.03,
        front_clearance_required=0.14,
    )
    assert assessment.available_width == pytest.approx(0.40)
    assert assessment.hard_required_width == pytest.approx(0.24)
    assert assessment.auto_required_width == pytest.approx(0.32)
    assert assessment.classification == "CLEAR"
    assert assessment.can_go_straight
    assert not assessment.can_rotate


@pytest.mark.parametrize("offset", [0.0, 0.01, 0.02])
def test_40cm_corridor_tolerates_realistic_offset_and_scan_noise(offset: float) -> None:
    points = [
        (x, side - offset + noise)
        for x, noise in [(-0.15, 0.002), (0.0, -0.003), (0.3, 0.001), (0.6, 0.0)]
        for side in (-0.20, 0.20)
    ]
    assessment = evaluate_corridor(
        points,
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        front_clearance_required=0.14,
    )
    assert assessment.can_go_straight


def test_corridor_geometry_reports_all_three_decision_states() -> None:
    uncertain = evaluate_corridor(
        [(0.0, -0.15), (0.0, 0.15)],
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        localization_uncertainty=0.02,
        front_clearance_required=0.14,
    )
    physically_blocked = evaluate_corridor(
        [(0.0, -0.11), (0.0, 0.11)],
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        front_clearance_required=0.14,
    )
    front_blocked = evaluate_corridor(
        [(0.0, -0.20), (0.0, 0.20), (0.28, 0.0)],
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        front_clearance_required=0.14,
    )
    assert uncertain.physically_passable
    assert uncertain.classification == "NARROW_OR_UNCERTAIN"
    assert uncertain.can_go_straight
    assert not physically_blocked.physically_passable
    assert physically_blocked.classification == "PHYSICALLY_BLOCKED"
    assert front_blocked.front_clearance == pytest.approx(0.13)
    assert front_blocked.classification == "PHYSICALLY_BLOCKED"
    assert front_blocked.reason == "FRONT_CLEARANCE"


def test_corridor_comfort_margin_does_not_expand_hard_front_envelope() -> None:
    assessment = evaluate_corridor(
        [(0.0, -0.20), (0.0, 0.20), (0.24, 0.13)],
        half_length=0.15,
        half_width=0.10,
        side_margin=0.05,
        hard_side_margin=0.02,
        translation_lateral_margin=0.01,
        front_clearance_required=0.14,
    )
    assert assessment.front_clearance == math.inf
    assert assessment.reason != "FRONT_CLEARANCE"


def test_route_overlap_rejects_near_duplicates_but_keeps_distinct_corridors() -> None:
    original = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]
    near_duplicate = [{"x": 0.0, "y": 0.05}, {"x": 2.0, "y": 0.05}]
    distinct = [
        {"x": 0.0, "y": 0.0},
        {"x": 0.6, "y": 0.8},
        {"x": 1.4, "y": 0.8},
        {"x": 2.0, "y": 0.0},
    ]
    assert path_overlap_ratio(original, near_duplicate) > 0.85
    assert path_overlap_ratio(original, distinct) < 0.85


def test_dynamic_obstacle_payload_is_metric_and_bounded() -> None:
    message = SimpleNamespace(
        info=SimpleNamespace(
            width=3,
            resolution=0.1,
            origin=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=2.0)),
        ),
        data=[0, 100, 0, 99, 100, 0],
    )
    assert compact_lethal_cells(message, max_cells=2) == [
        {"x": -0.85, "y": 2.05},
        {"x": -0.85, "y": 2.15},
    ]
    assert compact_lethal_cells(message, max_cells=None) == [
        {"x": -0.85, "y": 2.05},
        {"x": -0.85, "y": 2.15},
    ]


def test_saved_static_hits_can_be_removed_from_dynamic_overlay(tmp_path: Path) -> None:
    saved = _saved_map(tmp_path)
    occupied = saved.cell_center(1, 0)
    free = saved.cell_center(0, 2)

    assert saved.occupied_within(*occupied, 0.05)
    assert saved.occupied_within(occupied[0] + 0.08, occupied[1], 0.10)
    assert not saved.occupied_within(*free, 0.05)


def test_rotation_clearance_uses_the_complete_rectangular_body_sweep() -> None:
    corner_radius = math.hypot(0.15, 0.05)
    assert rotation_swept_clearance(
        0.15, 0.05, half_length=0.15, half_width=0.05
    ) == pytest.approx(0.0)
    assert rotation_swept_clearance(
        0.0, 0.10, half_length=0.15, half_width=0.05
    ) == pytest.approx(0.10 - corner_radius)
    assert rotation_swept_clearance(
        0.30, 0.0, half_length=0.15, half_width=0.05
    ) == pytest.approx(0.30 - corner_radius)


def test_exact_edt_matches_brute_force_reference_on_small_grid() -> None:
    width, height = 7, 5
    blocked_cells = {(1, 1), (5, 3), (0, 4)}
    blocked = [
        (column, row) in blocked_cells
        for row in range(height)
        for column in range(width)
    ]
    actual = exact_euclidean_distance_transform(
        blocked, width=width, height=height
    )
    expected = [
        min(math.hypot(column - bx, row - by) for bx, by in blocked_cells)
        for row in range(height)
        for column in range(width)
    ]
    assert actual == pytest.approx(expected)


def test_sixty_degree_heading_is_an_exact_lattice_bin_and_stays_straight() -> None:
    saved = _manual_map(50, 50, _free_rectangle(1, 48, 1, 48))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {"x": 0.50, "y": 0.50, "yaw": 0.0}
    goal = {
        "x": start["x"] + 0.80 * math.cos(math.radians(60)),
        "y": start["y"] + 0.80 * math.sin(math.radians(60)),
    }
    route = planner.plan(start, goal)
    assert route is not None
    assert route.heading_bins == (4,)
    assert len(route.points) == 2
    assert validate_rotation_sweep(
        saved,
        start["x"],
        start["y"],
        0.0,
        math.radians(60),
        half_length=0.15,
        half_width=0.10,
    ).valid


def test_dense_straight_segment_never_leaves_only_a_pose_behind_robot() -> None:
    points = densify_straight_segment(
        {"x": -0.24, "y": 0.65},
        {"x": -0.24, "y": 2.08},
        spacing=0.05,
    )
    assert points[0] == {"x": -0.24, "y": 0.65}
    assert points[-1] == {"x": -0.24, "y": 2.08}
    assert len(points) > 20
    assert all(point["x"] == pytest.approx(-0.24) for point in points)
    assert all(
        0.0 < right["y"] - left["y"] <= 0.05 + 1e-9
        for left, right in zip(points, points[1:])
    )


def test_active_vertical_segment_heading_never_chases_stale_start() -> None:
    start = {"x": -0.400119, "y": 1.470302}
    end = {"x": -0.400119, "y": 2.095102}
    segment = ActiveSegment.create(
        planned_start=start,
        effective_start=start,
        endpoint=end,
        segment_index=0,
        route_id="vertical-regression",
        segment_token=7,
    )
    pose = {"x": -0.400119, "y": 1.70, "yaw": 1.5305}

    decision = straight_heading_lock(
        segment,
        pose,
        heading_kp=1.2,
        cross_track_kp=1.0,
        maximum_angular=0.18,
        heading_deadband=math.radians(1.0),
        cross_track_deadband=0.01,
        hard_heading_error=math.radians(12.0),
        hard_cross_track=0.08,
    )

    assert segment.fixed_heading == pytest.approx(math.pi / 2)
    assert decision.heading_error == pytest.approx(math.pi / 2 - 1.5305)
    assert decision.forward_allowed is True


def test_reanchored_segment_uses_actual_pose_and_new_fixed_heading() -> None:
    segment = ActiveSegment.create(
        planned_start={"x": 0.0, "y": 0.0},
        effective_start={"x": 0.05, "y": 0.02},
        endpoint={"x": 1.0, "y": 0.0},
        segment_index=1,
        route_id="route",
        segment_token=8,
    )

    assert segment.planned_start == {"x": 0.0, "y": 0.0}
    assert segment.effective_start == {"x": 0.05, "y": 0.02}
    assert segment.fixed_heading == pytest.approx(math.atan2(-0.02, 0.95))
    assert segment.segment_length == pytest.approx(math.hypot(0.95, 0.02))


def test_reverse_segment_keeps_chassis_heading_opposite_travel() -> None:
    segment = ActiveSegment.create(
        planned_start={"x": 1.0, "y": 0.0},
        effective_start={"x": 1.0, "y": 0.0},
        endpoint={"x": 0.5, "y": 0.0},
        segment_index=0,
        route_id="reverse-turn-bay",
        segment_token=9,
        motion_direction=-1,
    )

    assert segment.motion_direction == -1
    assert segment.fixed_heading == pytest.approx(0.0)
    decision = straight_heading_lock(
        segment,
        {"x": 0.9, "y": 0.0, "yaw": 0.0},
        heading_kp=1.2,
        cross_track_kp=1.0,
        maximum_angular=0.18,
        heading_deadband=math.radians(1.0),
        cross_track_deadband=0.01,
        hard_heading_error=math.radians(12.0),
        hard_cross_track=0.08,
    )
    assert decision.forward_allowed is True
    assert decision.heading_error == pytest.approx(0.0)


def test_straight_large_heading_or_cross_track_error_blocks_forward() -> None:
    segment = ActiveSegment.create(
        planned_start={"x": 0.0, "y": 0.0},
        effective_start={"x": 0.0, "y": 0.0},
        endpoint={"x": 1.0, "y": 0.0},
        segment_index=0,
        route_id="route",
        segment_token=1,
    )
    parameters = {
        "heading_kp": 1.2,
        "cross_track_kp": 1.0,
        "maximum_angular": 0.18,
        "heading_deadband": math.radians(1.0),
        "cross_track_deadband": 0.01,
        "hard_heading_error": math.radians(12.0),
        "hard_cross_track": 0.08,
    }

    heading = straight_heading_lock(
        segment, {"x": 0.1, "y": 0.0, "yaw": 0.5}, **parameters
    )
    cross = straight_heading_lock(
        segment, {"x": 0.1, "y": 0.10, "yaw": 0.0}, **parameters
    )

    assert heading.forward_allowed is False
    assert heading.reason == "HEADING_ERROR_HARD_LIMIT"
    assert cross.forward_allowed is False
    assert cross.reason == "CROSS_TRACK_HARD_LIMIT"


def test_straight_small_error_ignores_rpp_curvature_direction() -> None:
    segment = ActiveSegment.create(
        planned_start={"x": 0.0, "y": 0.0},
        effective_start={"x": 0.0, "y": 0.0},
        endpoint={"x": 1.0, "y": 0.0},
        segment_index=0,
        route_id="route",
        segment_token=2,
    )
    decision = straight_heading_lock(
        segment,
        {"x": 0.2, "y": 0.015, "yaw": 0.02},
        heading_kp=1.2,
        cross_track_kp=1.0,
        maximum_angular=0.18,
        heading_deadband=math.radians(1.0),
        cross_track_deadband=0.005,
        hard_heading_error=math.radians(12.0),
        hard_cross_track=0.08,
    )

    assert decision.forward_allowed is True
    assert -0.18 <= decision.angular < 0.0


@pytest.mark.parametrize(
    ("y", "remaining", "passed"),
    [
        (2.00, 0.095102, False),
        (2.095102, 0.0, False),
        (2.105, -0.009898, True),
    ],
)
def test_vertical_segment_endpoint_progress(y: float, remaining: float, passed: bool) -> None:
    progress = straight_segment_progress(
        {"x": -0.400119, "y": 1.470302},
        {"x": -0.400119, "y": 2.095102},
        {"x": -0.400119, "y": y},
    )

    assert progress.remaining_longitudinal == pytest.approx(remaining)
    assert progress.passed_endpoint is passed
    assert progress.signed_cross_track == pytest.approx(0.0)


def test_endpoint_braking_limit_scales_with_profile_deceleration() -> None:
    remaining = 0.10
    slow = endpoint_braking_speed_limit(
        remaining, deceleration=0.45, reaction_time=0.15
    )
    normal = endpoint_braking_speed_limit(
        remaining, deceleration=0.55, reaction_time=0.15
    )
    fast = endpoint_braking_speed_limit(
        remaining, deceleration=0.60, reaction_time=0.15
    )

    assert 0.0 < slow < normal < fast
    assert endpoint_braking_speed_limit(
        0.0, deceleration=0.60, reaction_time=0.15
    ) == 0.0


def test_segment_watchdog_rejects_multi_meter_travel_for_short_segment() -> None:
    decision = segment_travel_watchdog(
        segment_length=0.625,
        elapsed=15.0,
        positive_travel=2.1,
        expected_speed=0.17,
        settle_allowance=2.0,
        travel_factor=2.0,
        minimum_travel_slack=0.30,
        time_factor=3.0,
    )

    assert decision.exceeded is True
    assert decision.reason == "POSITIVE_TRAVEL_LIMIT"
    assert decision.travel_limit < 2.1


def test_turn_hysteresis_does_not_chatter_inside_reentry_band() -> None:
    completion = math.radians(3.0)
    reentry = math.radians(6.0)
    assert turn_hysteresis_transition(
        "TURN",
        math.radians(2.9),
        completion_tolerance=completion,
        reentry_tolerance=reentry,
        stable_elapsed=0.0,
        stable_dwell=0.4,
    ) == "TURN_SETTLING"
    assert turn_hysteresis_transition(
        "TURN_SETTLING",
        math.radians(4.5),
        completion_tolerance=completion,
        reentry_tolerance=reentry,
        stable_elapsed=0.2,
        stable_dwell=0.4,
    ) == "TURN_SETTLING"
    # Runtime regression: after entering settling at <=3 degrees, passive
    # chassis drift repeatedly stopped near 5.8 degrees.  That is still inside
    # the 6 degree Schmitt band and must finish after the zero-command dwell,
    # not hang until an operator pauses/resumes the mission.
    assert turn_hysteresis_transition(
        "TURN_SETTLING",
        math.radians(5.8),
        completion_tolerance=completion,
        reentry_tolerance=reentry,
        stable_elapsed=0.4,
        stable_dwell=0.4,
    ) == "STRAIGHT_PREPARE"
    assert turn_hysteresis_transition(
        "TURN_SETTLING",
        math.radians(6.1),
        completion_tolerance=completion,
        reentry_tolerance=reentry,
        stable_elapsed=0.0,
        stable_dwell=0.4,
    ) == "TURN"


def test_turn_direction_tracks_overshoot_error_sign_on_reentry() -> None:
    safety = {
        "left_static_safe": True,
        "right_static_safe": True,
        "left_live_safe": True,
        "right_live_safe": True,
    }

    assert choose_turn_direction(math.radians(-6.2), **safety) == -1
    assert choose_turn_direction(math.radians(6.2), **safety) == 1


def test_rotation_sweep_rejects_collision_missed_at_both_endpoint_headings() -> None:
    resolution = 0.01
    width = height = 80
    occupancy = [0] * (width * height)
    obstacle_column = round((0.035 + 0.40) / resolution - 0.5)
    obstacle_row = round((0.175 + 0.40) / resolution - 0.5)
    occupancy[obstacle_row * width + obstacle_column] = 100
    saved = SavedOccupancyMap(
        width, height, resolution, -0.40, -0.40, 0.0, occupancy
    )
    assert saved.validate_footprint(
        0.0, 0.0, 0.0, half_length=0.15, half_width=0.10
    ).valid
    assert saved.validate_footprint(
        0.0, 0.0, math.pi / 2, half_length=0.15, half_width=0.10
    ).valid
    assert not validate_rotation_sweep(
        saved,
        0.0,
        0.0,
        0.0,
        math.pi / 2,
        half_length=0.15,
        half_width=0.10,
    ).valid


def test_rotation_neighborhood_rejects_a_zero_tolerance_corner() -> None:
    resolution = 0.01
    width = height = 80
    occupancy = [0] * (width * height)
    obstacle_column = round((0.17 + 0.40) / resolution - 0.5)
    obstacle_row = round((0.06 + 0.40) / resolution - 0.5)
    occupancy[obstacle_row * width + obstacle_column] = 100
    saved = SavedOccupancyMap(
        width, height, resolution, -0.40, -0.40, 0.0, occupancy
    )

    assert validate_rotation_sweep(
        saved,
        0.0,
        0.0,
        0.0,
        math.pi / 2,
        half_length=0.15,
        half_width=0.10,
    ).valid
    robust = validate_rotation_sweep_neighborhood(
        saved,
        0.0,
        0.0,
        0.0,
        math.pi / 2,
        half_length=0.15,
        half_width=0.10,
        robustness_radius=0.01,
    )
    assert not robust.valid
    assert robust.code == "TURN_SWEEP_NOT_ROBUST"


def test_moving_scan_deskew_reduces_static_wall_geometry_error() -> None:
    beam_count = 61
    angle_min = -0.60
    angle_increment = 1.20 / (beam_count - 1)
    scan_time = 0.20
    linear_velocity = 0.30
    angular_velocity = 0.40
    time_increment = scan_time / (beam_count - 1)
    ranges = []
    for index in range(beam_count):
        timestamp = index * time_increment
        yaw = angular_velocity * timestamp
        radius = linear_velocity / angular_velocity
        pose_x = radius * math.sin(yaw)
        beam_world = yaw + angle_min + index * angle_increment
        ranges.append((2.0 - pose_x) / math.cos(beam_world))
    corrected = deskew_scan_points(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=8.0,
        time_increment=time_increment,
        scan_time=scan_time,
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
    )
    final_yaw = angular_velocity * scan_time
    radius = linear_velocity / angular_velocity
    final_x = radius * math.sin(final_yaw)
    final_y = radius * (1.0 - math.cos(final_yaw))

    def world_x(point_x: float, point_y: float) -> float:
        return final_x + math.cos(final_yaw) * point_x - math.sin(final_yaw) * point_y

    deskew_error = sum(abs(world_x(x, y) - 2.0) for _, x, y in corrected) / len(corrected)
    no_deskew_error = sum(
        abs(world_x(
            distance * math.cos(angle_min + index * angle_increment),
            distance * math.sin(angle_min + index * angle_increment),
        ) - 2.0)
        for index, distance in enumerate(ranges)
    ) / len(ranges)
    assert deskew_error < 1e-6
    assert deskew_error < no_deskew_error * 0.05


def test_global_rotation_progress_uses_unwrapped_measured_yaw() -> None:
    progress = UnwrappedYawProgress()
    progress.reset(math.radians(170))
    assert progress.update(math.radians(179)) == pytest.approx(math.radians(9))
    assert progress.update(math.radians(-170)) == pytest.approx(math.radians(20))
    # No odometry change means no progress even if a turn command persists.
    assert progress.update(math.radians(-170)) == pytest.approx(math.radians(20))


def test_route_execution_cost_includes_large_initial_turn() -> None:
    saved = _manual_map(100, 100, _free_rectangle(1, 98, 1, 98))
    start = {"x": 2.5, "y": 2.5}
    heading = math.radians(109.0)
    goal = {
        "x": start["x"] + 2.5 * math.cos(heading),
        "y": start["y"] + 2.5 * math.sin(heading),
    }

    metadata = route_geometry_metadata(
        saved,
        saved.navigation_geometry,
        (start, goal),
        half_length=0.15,
        half_width=0.10,
        linear_speed=0.20,
        angular_speed=0.60,
        start_yaw=math.radians(-50.0),
    )

    assert metadata.initial_turn_angle == pytest.approx(math.radians(159.0))
    assert metadata.internal_turn_angle == 0.0
    assert metadata.execution_total_turn_angle == pytest.approx(
        metadata.initial_turn_angle
    )
    assert metadata.turn_count == 1
    assert metadata.estimated_time > metadata.total_length / 0.20


def test_route_metadata_rejects_total_width_as_proof_of_side_clearance() -> None:
    saved = _manual_map(80, 40, _free_rectangle(1, 78, 1, 38))
    near_wall = route_geometry_metadata(
        saved,
        saved.navigation_geometry,
        ({"x": 0.50, "y": 0.16}, {"x": 3.00, "y": 0.16}),
        half_length=0.15,
        half_width=0.10,
    )
    centered = route_geometry_metadata(
        saved,
        saved.navigation_geometry,
        ({"x": 0.50, "y": 1.00}, {"x": 3.00, "y": 1.00}),
        half_length=0.15,
        half_width=0.10,
    )

    # Both centerlines see the same room width. Only the per-side body gap
    # exposes that the first route is practically touching one wall.
    assert near_wall.minimum_passage_width == pytest.approx(
        centered.minimum_passage_width
    )
    assert near_wall.minimum_side_clearance == pytest.approx(0.01)
    assert centered.minimum_side_clearance > 0.50


def test_clearance_bands_prefer_centerline_over_shorter_wall_hugging_path() -> None:
    free = _free_rectangle(2, 77, 2, 57) - _free_rectangle(30, 40, 10, 38)
    saved = _manual_map(80, 60, free)
    start = {"x": 0.50, "y": 1.50, "yaw": 0.0}
    goal = {"x": 3.50, "y": 1.50}
    shortest = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        turn_robustness_radius=0.0,
        hard_side_margin=0.0,
        preferred_side_margin=0.0,
    ).plan(start, goal)
    clearance_aware = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        turn_robustness_radius=0.0,
        hard_side_margin=0.01,
        preferred_side_margin=0.05,
    ).plan(start, goal)

    assert shortest is not None
    assert clearance_aware is not None
    assert shortest.metadata.minimum_side_clearance < 0.03
    assert clearance_aware.metadata.minimum_side_clearance >= 0.05
    assert clearance_aware.metadata.total_length > shortest.metadata.total_length


def test_shallow_shortcut_is_widened_before_fewer_turns_are_preferred() -> None:
    # Two rooms joined by a 35 cm passage. A shallow diagonal is executable,
    # but it leaves less than 7 cm on one side of a 20 cm chassis. Retaining
    # one nearly-collinear waypoint centers the translation through the choke.
    free = (
        _free_rectangle(2, 30, 3, 56)
        | _free_rectangle(25, 75, 25, 31)
        | _free_rectangle(70, 97, 3, 56)
    )
    saved = _manual_map(100, 60, free)
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        hard_side_margin=0.01,
        preferred_side_margin=0.07,
        turn_robustness_radius=0.0,
    )
    start = {"x": 0.70, "y": 1.60, "yaw": math.pi}
    goal = {"x": 4.30, "y": 1.40}
    shortcut = planner._route_result(
        [start, {"x": 1.30, "y": 1.425}, goal],
        start_yaw=start["yaw"],
    )

    assert shortcut is not None
    assert shortcut.metadata.minimum_side_clearance < 0.07
    widened = planner._widen_route_with_one_waypoint(
        shortcut,
        (),
        required_side_clearance=0.07,
        start_yaw=start["yaw"],
        goal_yaw=None,
    )

    assert widened is not None
    assert widened.metadata.minimum_side_clearance >= 0.07
    assert len(widened.points) == len(shortcut.points) + 1
    assert widened.metadata.total_length <= shortcut.metadata.total_length + 0.30


def test_minimum_turn_seed_removes_extra_stop_without_losing_width() -> None:
    free = (
        _free_rectangle(3, 13, 3, 45)
        | _free_rectangle(3, 70, 30, 40)
    )
    saved = _manual_map(75, 55, free)
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        hard_side_margin=0.01,
        preferred_side_margin=0.05,
        turn_robustness_radius=0.0,
    )
    start = {"x": 0.40, "y": 0.35, "yaw": math.pi}
    goal = {"x": 2.50, "y": 1.65}
    start_cell = saved.world_to_cell(start["x"], start["y"])
    goal_cell = saved.world_to_cell(goal["x"], goal["y"])
    assert start_cell is not None and goal_cell is not None

    shortest_seed = planner._grid_seed(
        start_cell,
        goal_cell,
        (),
        minimum_center_clearance=0.15,
    )
    minimum_turn_seed = planner._minimum_turn_grid_seed(
        start_cell,
        goal_cell,
        (),
        minimum_center_clearance=0.15,
    )
    shortest = planner._route_result(
        planner._canonical_route_from_seed(
            shortest_seed,
            start,
            goal,
            minimum_center_clearance=0.15,
        ),
        start_yaw=start["yaw"],
    )
    minimum_turn = planner._route_result(
        planner._canonical_route_from_seed(
            minimum_turn_seed,
            start,
            goal,
            minimum_center_clearance=0.15,
        ),
        start_yaw=start["yaw"],
    )

    assert shortest is not None and minimum_turn is not None
    assert shortest.metadata.turn_count == 3
    assert minimum_turn.metadata.turn_count == 2
    assert shortest.metadata.minimum_side_clearance >= 0.05
    assert minimum_turn.metadata.minimum_side_clearance >= 0.05
    assert minimum_turn.metadata.total_length > shortest.metadata.total_length

    selected = planner.plan(start, goal)
    assert selected is not None
    assert selected.points == minimum_turn.points
    assert selected.metadata.turn_count == 2


def test_visibility_waypoint_removes_grid_seed_turn_with_small_detour() -> None:
    free = _free_rectangle(2, 77, 2, 57) - _free_rectangle(32, 42, 6, 32)
    saved = _manual_map(80, 60, free)
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        hard_side_margin=0.01,
        preferred_side_margin=0.05,
        turn_robustness_radius=0.0,
    )
    start = {"x": 1.20, "y": 0.40, "yaw": -0.70}
    goal = {"x": 3.00, "y": 1.60}
    start_cell = saved.world_to_cell(start["x"], start["y"])
    goal_cell = saved.world_to_cell(goal["x"], goal["y"])
    assert start_cell is not None and goal_cell is not None

    seed = planner._minimum_turn_grid_seed(
        start_cell,
        goal_cell,
        (),
        minimum_center_clearance=0.15,
    )
    seeded = planner._route_result(
        planner._canonical_route_from_seed(
            seed,
            start,
            goal,
            minimum_center_clearance=0.15,
        ),
        start_yaw=start["yaw"],
    )
    selected = planner.plan(start, goal)

    assert seeded is not None and selected is not None
    assert len(seeded.points) == 4
    assert seeded.metadata.turn_count == 3
    assert len(selected.points) == 3
    assert selected.metadata.turn_count == 2
    assert selected.metadata.minimum_side_clearance >= 0.05
    assert selected.metadata.total_length > seeded.metadata.total_length
    assert selected.metadata.total_length <= seeded.metadata.total_length + 0.30
    assert selected.metadata.estimated_time < seeded.metadata.estimated_time


def test_adjacent_shallow_corners_can_move_to_one_safe_waypoint() -> None:
    saved = _manual_map(45, 45, _free_rectangle(2, 42, 2, 42))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    points = [
        {"x": 0.40, "y": 0.40},
        {"x": 0.75, "y": 0.65},
        {"x": 1.05, "y": 0.55},
        {"x": 1.35, "y": 0.70},
        {"x": 1.70, "y": 0.45},
    ]
    baseline = planner._route_result(points, start_yaw=-1.0)

    assert baseline is not None
    reduced = planner._reduce_one_route_corner(
        baseline,
        (),
        maximum_total_length=baseline.metadata.total_length + 0.30,
        minimum_center_clearance=0.15,
        start_yaw=-1.0,
        goal_yaw=None,
    )

    assert reduced is not None
    assert len(reduced.points) == len(baseline.points) - 1
    assert reduced.metadata.turn_count < baseline.metadata.turn_count
    assert reduced.metadata.total_length <= baseline.metadata.total_length + 0.30


def test_equally_safe_direct_candidate_wins_by_using_fewer_turns(
    monkeypatch,
) -> None:
    saved = _manual_map(80, 80, _free_rectangle(1, 78, 1, 78))
    planner = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry, max_expansions=1
    )
    start = {"x": 1.0, "y": 1.0, "yaw": math.radians(-50.0)}
    goal = {"x": 2.0, "y": 3.0}

    def candidate(
        route,
        *,
        start_yaw=None,
        goal_yaw=None,
        segment_directions=None,
    ):
        del start_yaw, goal_yaw, segment_directions
        direct = len(route) == 2
        angle = math.radians(159.0 if direct else 55.0)
        return StopTurnRoute(
            points=tuple(route),
            metadata=RouteMetadata(
                total_length=2.5 if direct else 2.8,
                minimum_passage_width=1.0,
                minimum_static_clearance=0.5,
                minimum_turn_clearance=0.5,
                turn_count=1 if direct else 2,
                total_turn_angle=angle,
                initial_turn_angle=(angle if direct else math.radians(15.0)),
                internal_turn_angle=(0.0 if direct else math.radians(40.0)),
                final_turn_angle=0.0,
                execution_total_turn_angle=angle,
                narrow_segments=(),
                estimated_time=18.0 if direct else 16.0,
                turn_safe=True,
            ),
            heading_bins=(0,) if direct else (0, 1),
        )

    monkeypatch.setattr(planner, "_route_result", candidate)
    monkeypatch.setattr(planner, "_grid_seed", lambda *args, **kwargs: [])

    route = planner.plan(start, goal)

    assert route is not None
    assert len(route.points) == 2
    assert route.metadata.turn_count == 1
    assert route.metadata.estimated_time == 18.0


def test_small_initial_turn_keeps_short_direct_candidate() -> None:
    saved = _manual_map(80, 80, _free_rectangle(1, 78, 1, 78))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {"x": 1.0, "y": 1.0, "yaw": math.radians(5.0)}
    goal = {"x": 3.0, "y": 1.2}

    route = planner.plan(start, goal)

    assert route is not None
    assert len(route.points) == 2
    assert route.metadata.initial_turn_angle < math.radians(2.0)


def test_post_turn_reanchor_uses_straight_band_for_small_drift() -> None:
    assert not post_turn_reanchor_requires_turn(
        math.radians(5.0),
        0.04,
        straight_entry_heading_limit=math.radians(6.0),
        straight_entry_cross_track_limit=0.08,
    )
    assert post_turn_reanchor_requires_turn(
        math.radians(8.0),
        0.04,
        straight_entry_heading_limit=math.radians(6.0),
        straight_entry_cross_track_limit=0.08,
    )
    assert post_turn_reanchor_requires_turn(
        math.radians(2.0),
        0.10,
        straight_entry_heading_limit=math.radians(6.0),
        straight_entry_cross_track_limit=0.08,
    )
def test_actual_project_map_rejects_direct_line_and_finds_exact_detour() -> None:
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    start = {"x": 1.685, "y": 1.415, "yaw": 0.0}
    goal = {"x": -0.065, "y": -3.135}
    direct = validate_stop_turn_route(
        saved,
        (start, goal),
        half_length=0.15,
        half_width=0.10,
    )
    assert not direct.valid
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    route = planner.plan(start, goal)
    assert route is not None
    assert len(route.points) > 2
    assert validate_stop_turn_route(
        saved,
        route.points,
        half_length=0.15,
        half_width=0.10,
    ).valid


def test_recorded_project_map_planner_case_reaches_lattice_before_deadline() -> None:
    """Regression for the 2026-08-18 zero-expansion 12-second timeout."""
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {
        "x": 0.2599872457,
        "y": 0.7900053497,
        "yaw": 2.7517760728,
    }
    goal = {"x": 2.4008163265, "y": 0.1951020408}

    result = planner.plan_result(start, goal, planning_time_budget=12.0)

    assert result.success
    assert result.expansions > 0
    assert result.route is not None
    assert validate_stop_turn_route(
        saved,
        result.route.points,
        half_length=0.15,
        half_width=0.10,
        segment_directions=result.route.segment_directions,
    ).valid


def test_actual_project_map_never_routes_through_preserved_unknown_space() -> None:
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    start = {"x": -3.265, "y": 4.415, "yaw": 0.0}
    goal = {"x": -1.765, "y": 3.315}
    direct = validate_stop_turn_route(
        saved,
        (start, goal),
        half_length=0.15,
        half_width=0.10,
    )
    route = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry
    ).plan(start, goal)

    assert not direct.valid
    assert direct.code == "PATH_UNKNOWN_COLLISION"
    assert route is None


def test_actual_runtime_start_overlap_is_classified_and_escapes_forward() -> None:
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {
        "x": 0.7174204546654707,
        "y": 1.4057562960543704,
        "yaw": 2.8682432380132292,
    }
    goal = {"x": -1.903265306122449, "y": 2.1532653061224494}

    start_cell = saved.world_to_cell(start["x"], start["y"])
    goal_cell = saved.world_to_cell(goal["x"], goal["y"])
    assert start_cell is not None and goal_cell is not None
    assert saved.navigation_geometry.same_component(start_cell, goal_cell)
    assert planner._grid_seed(start_cell, goal_cell, ())
    assert not saved.validate_footprint(
        **start, half_length=0.15, half_width=0.10, code_prefix="START"
    ).valid

    classified = planner.plan_result(start, goal, planning_time_budget=12.0)
    assert classified.status == "START_STATIC_OVERLAP"
    assert classified.status != "SEARCH_TIME_BUDGET_EXCEEDED"
    assert classified.start_escape is not None

    recovered = planner.plan_result(
        start,
        goal,
        planning_time_budget=12.0,
        allow_start_escape=True,
    )
    assert recovered.success
    assert recovered.start_escape is not None
    assert recovered.start_escape.distance <= 0.60
    assert recovered.start_escape.yaw == pytest.approx(start["yaw"])
    assert saved.validate_footprint(
        recovered.start_escape.end["x"],
        recovered.start_escape.end["y"],
        start["yaw"],
        half_length=0.15,
        half_width=0.10,
        code_prefix="START",
    ).valid
    assert recovered.route is not None
    assert recovered.route.points[-1] == goal


def test_actual_runtime_pose_prefers_goal_aligned_turn_bay_direction() -> None:
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        linear_speed=0.27,
        angular_speed=0.60,
        turn_bay_max_distance=0.80,
    )
    start = {
        "x": 1.94006334733655,
        "y": 1.1641913867574625,
        "yaw": -1.4428476156244743,
    }

    for goal, expected_direction in (
        ({"x": 0.585, "y": 1.565}, -1),
        ({"x": -0.003265306122449, "y": 0.330816326530613}, 1),
    ):
        result = planner.plan_result(
            start,
            goal,
            planning_time_budget=6.0,
            allow_start_escape=True,
        )
        assert result.success
        assert result.status != "SEARCH_TIME_BUDGET_EXCEEDED"
        assert result.route is not None
        assert result.route.points[0] == {"x": start["x"], "y": start["y"]}
        assert result.route.points[-1] == goal
        # Relocate without changing chassis yaw, preferring whichever of
        # forward/reverse lies in the destination half-plane, then turn.
        assert result.route.segment_directions[0] == expected_direction
        if expected_direction < 0:
            assert result.route.points[1]["y"] > start["y"] + 0.05
        else:
            assert result.route.points[1]["y"] < start["y"] - 0.20
        assert validate_stop_turn_route(
            saved,
            result.route.points,
            half_length=0.15,
            half_width=0.10,
            segment_directions=result.route.segment_directions,
        ).valid


def test_near_wall_runtime_start_does_not_apply_translation_margin_to_turn() -> None:
    """Regression for the post-arrival 12-second search-budget failure."""
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(project / "sample-data/maps/map-bundle/map.yaml")
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        turn_bay_max_distance=0.80,
        hard_side_margin=0.02,
        preferred_side_margin=0.07,
    )
    start = {
        "x": 1.9242407874035217,
        "y": 1.10615200144539,
        "yaw": -1.1734686632699747,
    }
    goal = {"x": -1.069591836734694, "y": 1.9012244897959194}

    result = planner.plan_result(
        start,
        goal,
        planning_time_budget=3.0,
        allow_start_escape=True,
    )

    assert result.success
    assert result.status != "SEARCH_TIME_BUDGET_EXCEEDED"
    assert result.route is not None
    assert result.route.points[0] == {"x": start["x"], "y": start["y"]}
    assert result.route.points[-1] == goal


def test_turn_bay_direction_order_tracks_goal_projection() -> None:
    start = {"x": 1.0, "y": 1.0, "yaw": 0.0}

    assert preferred_turn_bay_directions(
        start, {"x": 2.0, "y": 1.2}
    ) == (1, -1)
    assert preferred_turn_bay_directions(
        start, {"x": 0.0, "y": 0.8}
    ) == (-1, 1)


def test_turn_bay_does_not_reverse_when_the_required_turn_is_already_clear() -> None:
    saved = _manual_map(80, 80, _free_rectangle(1, 78, 1, 78))
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        turn_bay_max_distance=0.60,
    )
    start = {"x": 2.0, "y": 2.0, "yaw": 0.0}
    goal = {"x": 1.0, "y": 2.0}

    candidate = planner._turn_bay_candidate(start, goal, (), None)

    assert candidate is not None
    assert candidate.segment_directions[0] == 1


def test_start_escape_rejects_a_new_static_overlap_cell() -> None:
    free = _free_rectangle(1, 38, 1, 18)
    free.remove((10, 8))   # Initial side overlap that persists while moving.
    free.remove((14, 10))  # A new wall cell immediately ahead.
    saved = _manual_map(40, 20, free)
    start = {"x": 0.525, "y": 0.525, "yaw": 0.0}

    assert saved.footprint_overlap_cells(
        **start, half_length=0.15, half_width=0.10
    )
    assert find_start_escape(
        saved,
        start,
        half_length=0.15,
        half_width=0.10,
        maximum_distance=0.50,
    ) is None

    reverse_escape = find_start_escape(
        saved,
        start,
        half_length=0.15,
        half_width=0.10,
        maximum_distance=0.50,
        directions=(1, -1),
    )
    assert reverse_escape is not None
    assert reverse_escape.motion_direction == -1
    assert reverse_escape.end["x"] < start["x"]

    result = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry
    ).plan_result(
        start,
        {"x": 1.50, "y": 0.525},
        allow_start_escape=True,
        maximum_start_escape_distance=0.50,
    )
    assert result.status == "START_ESCAPE_UNAVAILABLE"
    assert result.start_escape is None


def test_latest_runtime_overlap_does_not_reverse_as_start_escape() -> None:
    project = Path(__file__).parents[3]
    saved = SavedOccupancyMap.load(
        project / "sample-data/maps/map-bundle/map.yaml"
    )
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        linear_speed=0.27,
        angular_speed=0.60,
        turn_bay_max_distance=0.80,
    )
    start = {
        "x": 0.38273931698398644,
        "y": 1.3962081589759003,
        "yaw": 0.14514972477459964,
    }
    goal = {
        "x": 1.8579591836734695,
        "y": 1.242040816326531,
    }

    result = planner.plan_result(
        start,
        goal,
        planning_time_budget=6.0,
        allow_start_escape=True,
    )

    assert not result.success
    assert result.status == "START_ESCAPE_UNAVAILABLE"
    assert result.start_escape is None


def test_dynamic_overlay_clusters_points_filters_static_wall_and_expires() -> None:
    free = _free_rectangle(1, 38, 1, 28)
    for row in range(1, 29):
        free.discard((20, row))
    saved = _manual_map(40, 30, free)
    overlay = DynamicObstacleOverlay(
        ttl_seconds=1.0, cluster_distance=0.10, association_distance=0.20
    )
    snapshot = overlay.observe(
        [
            (1.04, 0.50),  # Saved wall plus small localization/raster offset.
            (0.76, 0.48), (0.78, 0.50), (0.80, 0.52),  # One person.
        ],
        now=10.0,
        saved_map=saved,
        static_tolerance=0.08,
    )
    assert len(snapshot) == 1
    assert snapshot[0].observation_count == 1
    moved = overlay.observe(
        [(0.82, 0.50), (0.84, 0.52)],
        now=10.2,
        saved_map=saved,
        static_tolerance=0.08,
    )
    assert len(moved) == 1
    assert moved[0].id == snapshot[0].id
    assert moved[0].observation_count == 2
    assert overlay.snapshot(11.3) == ()


def test_dynamic_overlay_counts_at_most_one_observation_per_track_per_frame() -> None:
    overlay = DynamicObstacleOverlay(
        ttl_seconds=2.0,
        cluster_distance=0.05,
        association_distance=0.20,
    )

    # Separate cell clusters can both fit the association radius of the first
    # tentative track. One costmap callback still represents only one frame.
    first_frame = overlay.observe(((0.00, 0.00), (0.10, 0.00)), now=10.0)

    assert len(first_frame) == 2
    assert {item.observation_count for item in first_frame} == {1}
    second_frame = overlay.observe(((0.01, 0.00), (0.11, 0.00)), now=10.2)
    assert len(second_frame) == 2
    assert {item.observation_count for item in second_frame} == {2}


def test_dynamic_overlay_classifies_fixed_chair_after_confirmation_window() -> None:
    overlay = DynamicObstacleOverlay(
        ttl_seconds=2.0,
        motion_threshold=0.12,
        stationary_confirmation_seconds=1.0,
    )

    overlay.observe(((0.80, 0.02),), now=10.0)
    overlay.observe(((0.81, 0.01),), now=10.5)
    snapshot = overlay.observe(((0.80, 0.02),), now=11.1)

    assert len(snapshot) == 1
    assert snapshot[0].observation_count == 3
    assert snapshot[0].speed < 0.12
    assert snapshot[0].motion_state == "STATIONARY"


def test_dynamic_overlay_tracks_person_without_growing_historical_trail() -> None:
    overlay = DynamicObstacleOverlay(
        ttl_seconds=2.0,
        association_distance=0.35,
        motion_threshold=0.12,
    )

    first = overlay.observe(((0.00, 0.00),), now=20.0)[0]
    overlay.observe(((0.12, 0.00),), now=20.3)
    moving = overlay.observe(((0.24, 0.00),), now=20.6)[0]

    assert moving.id == first.id
    assert moving.motion_state == "MOVING"
    assert moving.speed == pytest.approx(0.40)
    assert moving.bounds == pytest.approx((0.24, 0.0, 0.24, 0.0))
    assert moving.radius == pytest.approx(overlay.observation_radius)


def test_dynamic_overlay_spatial_clustering_keeps_transitive_components() -> None:
    overlay = DynamicObstacleOverlay(cluster_distance=0.10)

    clusters = overlay._clusters(
        (
            (0.00, 0.00),
            (0.09, 0.00),
            (0.18, 0.00),  # Connected transitively through the middle point.
            (1.00, 1.00),
        ),
        0.10,
    )

    assert sorted(len(cluster) for cluster in clusters) == [1, 3]


def test_controller_confirmed_blocker_bypasses_only_static_point_filter() -> None:
    free = _free_rectangle(1, 38, 1, 28)
    for row in range(1, 29):
        free.discard((20, row))
    saved = _manual_map(40, 30, free)
    overlay = DynamicObstacleOverlay(ttl_seconds=2.0)
    wall_point = (1.04, 0.50)

    assert overlay.observe(
        (wall_point,),
        now=10.0,
        saved_map=saved,
        static_tolerance=0.08,
    ) == ()
    confirmed = overlay.observe_confirmed_blocker(
        (wall_point,), now=10.1
    )

    assert len(confirmed) == 1
    assert confirmed[0].center_x == pytest.approx(wall_point[0])
    assert confirmed[0].center_y == pytest.approx(wall_point[1])
    # This affects only the TTL overlay; Saved Map remains authoritative and
    # immutable.
    assert saved.occupied_within(*wall_point, 0.08)


def test_dynamic_exclusion_detours_without_mutating_saved_map() -> None:
    free = _free_rectangle(2, 67, 2, 57) - _free_rectangle(28, 35, 15, 45)
    saved = _manual_map(70, 60, free)
    before = tuple(saved.occupancy)
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        turn_robustness_radius=0.0,
        hard_side_margin=0.01,
        preferred_side_margin=0.05,
    )
    start = {"x": 0.50, "y": 1.50, "yaw": 0.0}
    goal = {"x": 3.00, "y": 1.50}
    direct = planner.plan(start, goal)
    assert direct is not None

    result = planner.plan_result(
        start,
        goal,
        exclusions=((1.20, 0.70, 0.35),),
        planning_time_budget=5.0,
    )
    assert result.success
    assert result.route is not None
    assert result.route.points != direct.points
    assert any(point["y"] > 2.0 for point in result.route.points[1:-1])
    assert result.route.metadata.minimum_side_clearance >= 0.05
    assert tuple(saved.occupancy) == before


def test_all_routes_temporarily_blocked_is_not_static_unreachable() -> None:
    free = _free_rectangle(2, 57, 8, 15)
    saved = _manual_map(60, 24, free)
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {"x": 0.30, "y": 0.575, "yaw": 0.0}
    goal = {"x": 2.70, "y": 0.575}
    assert planner.plan(start, goal) is not None

    result = planner.plan_result(
        start,
        goal,
        exclusions=((1.50, 0.575, 0.45),),
        planning_time_budget=5.0,
    )
    assert result.status == "DYNAMICALLY_BLOCKED"
    assert result.status != "GOAL_DISCONNECTED"


def test_dynamic_obstacles_only_affect_the_upcoming_route_horizon() -> None:
    route = [
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 4.0, "y": 0.0},
    ]
    assert dynamic_exclusions_intersect_route(
        route, ((1.0, 0.0, 0.20),), horizon=2.0
    )
    assert not dynamic_exclusions_intersect_route(
        route, ((-0.40, 0.0, 0.20),), horizon=2.0
    )
    assert not dynamic_exclusions_intersect_route(
        route, ((1.0, 0.80, 0.20),), horizon=2.0
    )
    assert not dynamic_exclusions_intersect_route(
        route, ((3.0, 0.0, 0.20),), horizon=2.0
    )


def test_dynamic_trajectory_conflict_requires_moving_ttc_not_static_intersection() -> None:
    route = [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]
    stationary = navigation_core.DynamicObstacle(
        1, 0.45, 0.0, 0.05, (0.4, -0.05, 0.5, 0.05),
        1.0, 2.0, 10, 1.0, motion_state="STATIONARY",
    )
    crossing = navigation_core.DynamicObstacle(
        2, 0.45, 0.35, 0.05, (0.4, 0.3, 0.5, 0.4),
        1.0, 2.0, 10, 1.0,
        velocity_x=0.0, velocity_y=-0.15, speed=0.15,
        motion_state="MOVING",
    )
    nearby_away = navigation_core.DynamicObstacle(
        3, 0.45, 0.35, 0.05, (0.4, 0.3, 0.5, 0.4),
        1.0, 2.0, 10, 1.0,
        velocity_x=0.0, velocity_y=0.15, speed=0.15,
        motion_state="MOVING",
    )

    assert dynamic_trajectory_conflict_ttc(
        route, stationary, robot_speed=0.17, footprint_inflation=0.12
    ) is None
    assert dynamic_trajectory_conflict_ttc(
        route, crossing, robot_speed=0.17, footprint_inflation=0.12
    ) == pytest.approx(1.8, abs=0.2)
    assert dynamic_trajectory_conflict_ttc(
        route, nearby_away, robot_speed=0.17, footprint_inflation=0.12
    ) is None


def test_turn_braking_speed_limit_reduces_before_completion_band() -> None:
    tolerance = math.radians(3.0)
    far = turn_braking_speed_limit(
        math.radians(45.0),
        completion_tolerance=tolerance,
        angular_deceleration=2.0,
        reaction_time=0.12,
    )
    near = turn_braking_speed_limit(
        math.radians(8.0),
        completion_tolerance=tolerance,
        angular_deceleration=2.0,
        reaction_time=0.12,
    )

    assert far > 0.60
    assert 0.0 < near < 0.40
    assert turn_braking_speed_limit(
        math.radians(2.9),
        completion_tolerance=tolerance,
        angular_deceleration=2.0,
        reaction_time=0.12,
    ) == 0.0


def test_controller_zero_abort_with_fresh_near_front_evidence_is_live_blockage() -> None:
    # 08:53:23: Humble FollowPath has no result diagnostics, Motion Safety did
    # not hard-stop, but RPP requested zero with the obstacle only 16.8 cm in
    # front of the footprint. This is a live wait/replan condition, not proof
    # that the destination is unreachable.
    assert controller_abort_is_live_blockage(
        error_code=None,
        error_msg="",
        atomic_motion_safety_block=False,
        dynamic_route_intersection=False,
        controller_zero_linear=True,
        repeated_zero_linear_abort=False,
        corridor_sample_fresh=True,
        corridor_front_clearance=0.168,
        corridor_blockage_limit=0.17,
    )
    assert not controller_abort_is_live_blockage(
        error_code=None,
        error_msg="",
        atomic_motion_safety_block=False,
        dynamic_route_intersection=False,
        controller_zero_linear=True,
        repeated_zero_linear_abort=False,
        corridor_sample_fresh=True,
        corridor_front_clearance=0.346,
        corridor_blockage_limit=0.17,
    )


def test_controller_diagnostics_but_not_repeated_zero_alone_are_live_blockage() -> None:
    assert controller_abort_is_live_blockage(
        error_code=106,
        error_msg="No valid control: predicted collision ahead",
        atomic_motion_safety_block=False,
        dynamic_route_intersection=False,
        controller_zero_linear=False,
        repeated_zero_linear_abort=False,
        corridor_sample_fresh=False,
        corridor_front_clearance=math.inf,
        corridor_blockage_limit=0.17,
    )
    assert not controller_abort_is_live_blockage(
        error_code=None,
        error_msg="",
        atomic_motion_safety_block=False,
        dynamic_route_intersection=False,
        controller_zero_linear=True,
        repeated_zero_linear_abort=True,
        corridor_sample_fresh=False,
        corridor_front_clearance=math.inf,
        corridor_blockage_limit=0.17,
    )


def test_hard_controller_block_requires_a_distinct_route() -> None:
    for reason in (
        "CONTROLLER_ABORT_LIVE_BLOCKAGE",
        "CONTROLLER_ABORT:UNCONFIRMED",
        "MOTION_SAFETY_DYNAMIC_BLOCK",
        "CONFIRMED_DYNAMIC_ROUTE_BLOCK",
        "LIVE_ROUTE_CLEARANCE_INSUFFICIENT",
    ):
        assert dynamic_block_requires_alternative(reason)

    assert not dynamic_block_requires_alternative(
        "PREDICTED_DYNAMIC_ROUTE_BLOCK"
    )


def test_replanned_copy_of_blocked_remaining_route_is_not_distinct() -> None:
    blocked_remaining = [
        {"x": 0.05, "y": 1.71},
        {"x": 0.435, "y": 1.565},
        {"x": 1.20, "y": 1.40},
    ]
    same_route_from_new_pose = [
        {"x": 0.06, "y": 1.713},
        {"x": 0.435, "y": 1.565},
        {"x": 1.20, "y": 1.40},
    ]
    detour = [
        {"x": 0.06, "y": 1.713},
        {"x": 0.10, "y": 0.90},
        {"x": 1.20, "y": 1.40},
    ]
    assert path_overlap_ratio(blocked_remaining, same_route_from_new_pose) >= 0.85
    assert path_overlap_ratio(blocked_remaining, detour) < 0.85


def test_near_final_goal_is_position_complete_but_internal_corner_is_not() -> None:
    current = {
        "x": -1.6786696503710226,
        "y": 1.7188354687906582,
        "yaw": -1.1047,
    }
    destination = {
        "x": -1.7093877551020404,
        "y": 1.6879591836734704,
    }
    assert position_within_tolerance(current, destination, 0.12)
    assert not position_within_tolerance(current, destination, 0.03)
    assert math.hypot(
        current["x"] - destination["x"], current["y"] - destination["y"]
    ) == pytest.approx(0.043554, abs=1e-6)
    target_heading = math.atan2(
        destination["y"] - current["y"], destination["x"] - current["x"]
    )
    heading_error = math.atan2(
        math.sin(target_heading - current["yaw"]),
        math.cos(target_heading - current["yaw"]),
    )
    assert target_heading == pytest.approx(-2.3536, abs=1e-4)
    assert abs(math.degrees(heading_error)) == pytest.approx(71.55, abs=0.1)


def test_real_expansion_bound_has_a_search_limit_reason(monkeypatch) -> None:
    free = _free_rectangle(2, 47, 2, 37) - _free_rectangle(22, 27, 10, 30)
    saved = _manual_map(50, 40, free)
    planner = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry, max_expansions=1
    )
    monkeypatch.setattr(planner, "_grid_seed", lambda *args, **kwargs: [])
    monkeypatch.setattr(
        planner, "_minimum_turn_grid_seed", lambda *args, **kwargs: []
    )
    result = planner.plan_result(
        {"x": 0.30, "y": 1.00, "yaw": 0.0},
        {"x": 2.20, "y": 1.00},
        planning_time_budget=5.0,
    )
    assert result.status == "SEARCH_EXPANSION_LIMIT"
    assert result.expansions == 1


def test_plan_result_can_retry_with_a_larger_per_request_expansion_bound(
    monkeypatch,
) -> None:
    free = _free_rectangle(2, 47, 2, 37) - _free_rectangle(22, 27, 10, 30)
    saved = _manual_map(50, 40, free)
    planner = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry, max_expansions=1
    )
    monkeypatch.setattr(planner, "_grid_seed", lambda *args, **kwargs: [])
    monkeypatch.setattr(
        planner, "_minimum_turn_grid_seed", lambda *args, **kwargs: []
    )

    result = planner.plan_result(
        {"x": 0.30, "y": 1.00, "yaw": 0.0},
        {"x": 2.20, "y": 1.00},
        planning_time_budget=5.0,
        search_expansion_limit=2,
    )

    assert result.status == "SEARCH_EXPANSION_LIMIT"
    assert result.expansions == 2
    assert planner.max_expansions == 1


def test_measured_start_turn_uses_physical_width_not_corridor_reserve() -> None:
    free = _free_rectangle(1, 38, 1, 38)
    free.remove((16, 17))
    saved = _manual_map(40, 40, free)
    planner = StopTurnStateLatticePlanner(
        saved,
        saved.navigation_geometry,
        hard_side_margin=0.02,
    )
    yaw = math.radians(55.0)

    assert saved.validate_footprint(
        1.0,
        1.0,
        yaw,
        half_length=0.15,
        half_width=0.10,
        code_prefix="START",
    ).valid
    assert planner._turn_valid(
        1.0,
        1.0,
        yaw,
        yaw + planner.heading_step,
        robust=False,
        measured_start=True,
    )
    assert not planner._turn_valid(
        1.0,
        1.0,
        yaw,
        yaw + planner.heading_step,
        robust=False,
    )


def test_reverse_clear_start_is_not_misclassified_as_statically_trapped(
    monkeypatch,
) -> None:
    saved = _manual_map(40, 40, _free_rectangle(1, 38, 1, 38))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {"x": 1.0, "y": 1.0, "yaw": 0.0}
    monkeypatch.setattr(planner, "plan", lambda *args, **kwargs: None)
    monkeypatch.setattr(planner, "_turn_valid", lambda *args, **kwargs: False)
    monkeypatch.setattr(
        planner,
        "_translation_valid",
        lambda left, right: float(right["x"]) < float(left["x"]),
    )

    result = planner.plan_result(start, {"x": 1.8, "y": 1.0})

    assert result.status == "NO_EXACT_STOP_TURN_ROUTE"


def test_turn_block_tracker_requires_atomic_clear_dwell() -> None:
    tracker = TurnBlockTracker(clear_dwell_seconds=0.30)
    assert tracker.update(sequence=10, blocked=True, now=1.0)
    original = tracker.blocked_since
    assert tracker.update(sequence=11, blocked=False, now=1.1)
    assert tracker.blocked_since == original
    assert tracker.update(sequence=11, blocked=False, now=1.5)
    assert tracker.blocked_since == original
    assert tracker.update(sequence=12, blocked=True, now=1.6)
    assert tracker.blocked_since == original
    assert tracker.update(sequence=13, blocked=False, now=1.7)
    assert not tracker.update(sequence=14, blocked=False, now=2.01)


def test_equally_safe_oblique_detour_beats_longer_right_angle_route() -> None:
    free = _free_rectangle(2, 97, 2, 77) - _free_rectangle(30, 32, 20, 22)
    saved = _manual_map(100, 80, free)
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    start = {"x": 0.50, "y": 0.50, "yaw": 0.0}
    goal = {"x": 4.00, "y": 3.00}

    route = planner.plan(start, goal)

    assert route is not None
    assert len(route.points) == 3
    assert route.metadata.total_length < 5.0
    headings = [
        math.atan2(
            right["y"] - left["y"],
            right["x"] - left["x"],
        )
        for left, right in zip(route.points, route.points[1:])
    ]
    assert all(
        not math.isclose(abs(heading), math.pi / 2, abs_tol=math.radians(1.0))
        and not math.isclose(heading, 0.0, abs_tol=math.radians(1.0))
        for heading in headings
    )
    assert validate_stop_turn_route(
        saved,
        route.points,
        half_length=0.15,
        half_width=0.10,
    ).valid


def test_adequate_clearance_ranking_does_not_reward_many_extra_turns() -> None:
    saved = _manual_map(20, 20, _free_rectangle(1, 18, 1, 18))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)

    def route(length: float, turns: int, estimated_time: float) -> StopTurnRoute:
        return StopTurnRoute(
            points=({"x": 0.0, "y": 0.0}, {"x": length, "y": 0.0}),
            metadata=RouteMetadata(
                total_length=length,
                minimum_passage_width=0.50,
                minimum_static_clearance=0.25,
                minimum_turn_clearance=0.05,
                turn_count=turns,
                total_turn_angle=turns * math.pi / 4,
                initial_turn_angle=0.0,
                internal_turn_angle=turns * math.pi / 4,
                final_turn_angle=0.0,
                execution_total_turn_angle=turns * math.pi / 4,
                narrow_segments=(),
                estimated_time=estimated_time,
                turn_safe=True,
            ),
            heading_bins=(0,),
        )

    short = route(2.6, 2, 20.0)
    long = route(5.4, 7, 45.0)

    assert planner.ranking_key(short) < planner.ranking_key(long)


def test_equal_routes_use_dominant_map_axis_as_a_bounded_tie_breaker() -> None:
    saved = _manual_map(20, 20, _free_rectangle(1, 18, 1, 18))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    metadata = RouteMetadata(
        total_length=2.0,
        minimum_passage_width=0.50,
        minimum_static_clearance=0.25,
        minimum_turn_clearance=0.05,
        turn_count=0,
        total_turn_angle=0.0,
        initial_turn_angle=0.0,
        internal_turn_angle=0.0,
        final_turn_angle=0.0,
        execution_total_turn_angle=0.0,
        narrow_segments=(),
        estimated_time=10.0,
        turn_safe=True,
        minimum_side_clearance=0.08,
    )
    axis = StopTurnRoute(
        ({"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}),
        metadata,
        (0,),
    )
    diagonal = StopTurnRoute(
        ({"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}),
        metadata,
        (3,),
    )

    assert planner.ranking_key(axis) < planner.ranking_key(diagonal)


def test_stop_turn_planner_handles_a_true_90_degree_l_route() -> None:
    free = (
        _free_rectangle(5, 45, 5, 14)
        | _free_rectangle(36, 45, 5, 50)
        | _free_rectangle(30, 46, 5, 23)
    )
    saved = _manual_map(60, 60, free)
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    route = planner.plan(
        {"x": 0.50, "y": 0.625, "yaw": 0.0},
        {"x": 1.925, "y": 2.30},
    )
    assert route is not None
    assert route.heading_bins == (0, 6)
    assert len(route.points) == 3
    assert route.metadata.turn_count == 1
    assert route.metadata.total_turn_angle == pytest.approx(math.pi / 2)
    assert route.metadata.turn_safe is True


def test_narrow_straight_corridor_turns_in_room_then_drives_straight() -> None:
    free = (
        _free_rectangle(4, 18, 4, 28)
        | _free_rectangle(16, 54, 14, 19)
    )
    saved = _manual_map(60, 40, free)
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    route = planner.plan(
        {"x": 0.50, "y": 0.50, "yaw": math.pi / 2},
        {"x": 2.50, "y": 0.825},
    )
    assert route is not None
    assert route.heading_bins == (6, 0)
    assert route.metadata.turn_safe is True
    assert route.metadata.narrow_segments == ({
        "segment_index": 1,
        "passage_width": pytest.approx(0.30),
        "length": pytest.approx(2.0),
    },)


def test_route_candidates_find_distinct_sides_without_persistent_exclusions() -> None:
    free = _free_rectangle(2, 67, 2, 57) - _free_rectangle(28, 35, 15, 45)
    saved = _manual_map(70, 60, free)
    planner = StopTurnStateLatticePlanner(
        saved, saved.navigation_geometry, turn_robustness_radius=0.0
    )
    routes = planner.plan_candidates(
        {"x": 0.50, "y": 1.50, "yaw": 0.0},
        {"x": 3.00, "y": 1.50},
        maximum_candidates=3,
    )
    assert len(routes) == 2
    middle_y = [
        sum(point["y"] for point in route.points[1:-1])
        / len(route.points[1:-1])
        for route in routes
    ]
    assert min(middle_y) < 1.0
    assert max(middle_y) > 2.0
    assert path_overlap_ratio(list(routes[0].points), list(routes[1].points)) < 0.80


def test_route_candidate_search_uses_one_shared_wall_clock_budget(monkeypatch) -> None:
    saved = _manual_map(70, 30, _free_rectangle(2, 67, 2, 27))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    primary = StopTurnRoute(
        points=({"x": 0.50, "y": 0.75}, {"x": 3.00, "y": 0.75}),
        metadata=RouteMetadata(
            total_length=2.5,
            minimum_passage_width=1.0,
            minimum_static_clearance=0.5,
            minimum_turn_clearance=0.5,
            turn_count=0,
            total_turn_angle=0.0,
            initial_turn_angle=0.0,
            internal_turn_angle=0.0,
            final_turn_angle=0.0,
            execution_total_turn_angle=0.0,
            narrow_segments=(),
            estimated_time=12.5,
            turn_safe=True,
        ),
        heading_bins=(0,),
    )
    calls: list[float | None] = []

    def plan(_start, _goal, *, exclusions=(), deadline_monotonic=None):
        del exclusions
        calls.append(deadline_monotonic)
        return primary

    clock = iter((100.0, 113.0))
    monkeypatch.setattr(navigation_core.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(planner, "plan", plan)

    routes = planner.plan_candidates(
        {"x": 0.50, "y": 0.75, "yaw": 0.0},
        {"x": 3.00, "y": 0.75},
        maximum_candidates=3,
        planning_time_budget=12.0,
    )

    assert routes == [primary]
    assert calls == [112.0]


def test_primary_only_candidate_request_never_searches_exclusions(monkeypatch) -> None:
    saved = _manual_map(70, 30, _free_rectangle(2, 67, 2, 27))
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    primary = StopTurnRoute(
        points=({"x": 0.50, "y": 0.75}, {"x": 3.00, "y": 0.75}),
        metadata=RouteMetadata(
            total_length=2.5,
            minimum_passage_width=1.0,
            minimum_static_clearance=0.5,
            minimum_turn_clearance=0.5,
            turn_count=0,
            total_turn_angle=0.0,
            initial_turn_angle=0.0,
            internal_turn_angle=0.0,
            final_turn_angle=0.0,
            execution_total_turn_angle=0.0,
            narrow_segments=(),
            estimated_time=12.5,
            turn_safe=True,
        ),
        heading_bins=(0,),
    )
    calls: list[tuple[tuple[float, float, float], ...]] = []

    def plan(_start, _goal, *, exclusions=(), deadline_monotonic=None):
        del deadline_monotonic
        calls.append(tuple(exclusions))
        return primary

    monkeypatch.setattr(planner, "plan", plan)
    routes = planner.plan_candidates(
        {"x": 0.50, "y": 0.75, "yaw": 0.0},
        {"x": 3.00, "y": 0.75},
        maximum_candidates=1,
        planning_time_budget=12.0,
    )

    assert routes == [primary]
    assert calls == [()]


def test_planner_rejects_free_goal_in_a_physically_disconnected_component() -> None:
    free = (
        _free_rectangle(2, 22, 2, 22)
        | _free_rectangle(37, 57, 2, 22)
    )
    saved = _manual_map(60, 30, free)
    planner = StopTurnStateLatticePlanner(saved, saved.navigation_geometry)
    assert saved.occupancy[12 * saved.width + 47] == 0
    assert planner.plan(
        {"x": 0.625, "y": 0.625, "yaw": 0.0},
        {"x": 2.375, "y": 0.625},
    ) is None


def test_committed_humble_control_set_has_only_straight_and_rotation_primitives() -> None:
    project = Path(__file__).parents[1]
    control_set = json.loads((
        project
        / "navigation-stack/control_sets/rovera_5cm_24_heading_stop_turn.json"
    ).read_text())
    metadata = control_set["lattice_metadata"]
    assert metadata["grid_resolution"] == 0.05
    assert metadata["num_of_headings"] == 24
    assert metadata["heading_angles"][2] == pytest.approx(math.radians(30))
    assert metadata["heading_angles"][3] == pytest.approx(math.radians(45))
    assert metadata["heading_angles"][4] == pytest.approx(math.radians(60))
    assert metadata["heading_angles"][6] == pytest.approx(math.radians(90))
    assert len(control_set["primitives"]) == metadata["number_of_trajectories"]
    for primitive in control_set["primitives"]:
        translation = primitive["trajectory_length"] > 0.0
        if translation:
            assert primitive["start_angle_index"] == primitive["end_angle_index"]
            assert all(
                pose[2] == pytest.approx(
                    metadata["heading_angles"][primitive["start_angle_index"]]
                )
                for pose in primitive["poses"]
            )
        else:
            assert all(pose[0] == pose[1] == 0.0 for pose in primitive["poses"])


def test_scan_self_filter_masks_only_points_inside_calibrated_body() -> None:
    ranges = [0.09, 0.30, 1.0, math.inf]
    filtered, masked = mask_scan_self_returns(
        ranges,
        angle_min=-math.pi / 2,
        angle_increment=math.pi / 2,
        range_min=0.05,
        range_max=8.0,
        laser_x=-0.0046412,
        laser_y=0.0,
        laser_yaw=0.0,
        half_length=0.20,
        half_width=0.18,
    )
    assert masked == 1
    assert math.isnan(filtered[0])
    assert filtered[1:] == ranges[1:]


def test_scan_self_filter_removes_live_right_body_returns_but_keeps_obstacle() -> None:
    laser_x = -0.0046412

    def filter_point(point_x: float, point_y: float) -> tuple[list[float], int]:
        sensor_x = point_x - laser_x
        distance = math.hypot(sensor_x, point_y)
        angle = math.atan2(point_y, sensor_x)
        return mask_scan_self_returns(
            [distance],
            angle_min=angle,
            angle_increment=0.0,
            range_min=0.05,
            range_max=8.0,
            laser_x=laser_x,
            laser_y=0.0,
            laser_yaw=0.0,
            half_length=0.20,
            half_width=0.18,
        )

    # Exact blocker coordinates observed on robot 170 while Mapping was active.
    for body_point in ((-0.1416, -0.1632), (0.1307, -0.1671)):
        filtered, masked = filter_point(*body_point)
        assert masked == 1
        assert math.isnan(filtered[0])

    outside_distance = math.hypot(0.0 - laser_x, -0.20)
    filtered, masked = filter_point(0.0, -0.20)
    assert masked == 0
    assert filtered == pytest.approx([outside_distance])


def test_motion_safety_startup_recovers_inactive_velocity_smoother() -> None:
    project = Path(__file__).parents[1]
    launch = (
        project / "motion-safety/launch/motion_safety.launch.py"
    ).read_text()
    recovery = (
        project
        / "motion-safety/scripts/ensure_velocity_smoother_active.py"
    ).read_text()
    compose = yaml.safe_load(
        (project / "compose.navigation.yml").read_text()
    )
    healthcheck = compose["services"]["motion-safety"]["healthcheck"]["test"][1]

    assert "TimerAction(" in launch
    assert "ensure_velocity_smoother_active.py" in launch
    assert "MAX_ATTEMPTS = 20" in recovery
    assert "GetState" in recovery
    assert "Transition.TRANSITION_CONFIGURE" in recovery
    assert "Transition.TRANSITION_ACTIVATE" in recovery
    assert "velocity-smoother-active" in healthcheck


def test_navigation_motion_tuning_stays_within_final_smoother_limits() -> None:
    project = Path(__file__).parents[1]
    navigation = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )
    smoother = yaml.safe_load(
        (project / "motion-safety/config/velocity_smoother.yaml").read_text()
    )
    safety = yaml.safe_load(
        (project / "motion-safety/config/safety.yaml").read_text()
    )["rovera_motion_safety"]["ros__parameters"]
    sensor_time = yaml.safe_load(
        (project / "navigation-stack/config/sensor_time.yaml").read_text()
    )["rovera_sensor_normalizer"]["ros__parameters"]
    controller = navigation["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    planner_server = navigation["planner_server"]["ros__parameters"]
    planner = planner_server["GridBased"]
    global_costmap = navigation["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = navigation["local_costmap"]["local_costmap"]["ros__parameters"]
    behavior = navigation["behavior_server"]["ros__parameters"]
    limits = smoother["velocity_smoother"]["ros__parameters"]

    assert controller["odom_topic"] == "/odometry/filtered"
    assert follow["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert follow["use_rotate_to_heading"] is False
    assert follow["use_collision_detection"] is False
    assert follow["allow_reversing"] is True
    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert planner["allow_unknown"] is False
    assert planner["smooth_path"] is False
    assert planner["allow_reverse_expansion"] is False
    assert planner["analytic_expansion_ratio"] < 0
    assert planner["lattice_filepath"].startswith("/opt/rovera/control_sets/")
    assert planner_server["planner_plugins"] == ["GridBased", "ThetaDiagnostic"]
    assert 0.15 < follow["desired_linear_vel"] <= limits["max_velocity"][0]
    assert follow["rotate_to_heading_angular_vel"] == limits["max_velocity"][2]
    assert follow["max_angular_accel"] == limits["max_accel"][2]
    assert follow["lookahead_dist"] >= 0.35
    assert follow["regulated_linear_scaling_min_speed"] >= 0.07
    assert follow["max_allowed_time_to_collision_up_to_carrot"] >= 0.5
    assert 10.0 <= controller["progress_checker"]["movement_time_allowance"] <= 15.0
    assert controller["failure_tolerance"] <= 1.0
    assert follow["rotate_to_heading_min_angle"] <= math.radians(6)
    assert global_costmap["update_frequency"] >= 5
    assert local_costmap["update_frequency"] > global_costmap["update_frequency"]
    assert global_costmap["obstacle_layer"]["combination_method"] == 1
    assert global_costmap["footprint_padding"] == 0.0
    assert local_costmap["footprint_padding"] == 0.0
    assert global_costmap["static_layer"]["footprint_clearing_enabled"] is True
    assert 0.15 <= global_costmap["inflation_layer"]["inflation_radius"] <= 0.20
    assert local_costmap["inflation_layer"]["inflation_radius"] == global_costmap[
        "inflation_layer"
    ]["inflation_radius"]
    assert local_costmap["update_frequency"] >= 10
    assert local_costmap["resolution"] <= 0.025
    assert navigation["rovera_navigation_adapter"]["ros__parameters"][
        "translation_lateral_margin"
    ] == safety["translation_lateral_margin"]
    assert local_costmap["obstacle_layer"]["scan"]["observation_persistence"] == 0.0
    assert local_costmap["obstacle_layer"]["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    for costmap in (global_costmap, local_costmap):
        scan = costmap["obstacle_layer"]["scan"]
        assert scan["marking"] is True
        assert scan["clearing"] is True
        assert scan["obstacle_max_range"] >= 3.0
        assert scan["raytrace_max_range"] >= scan["obstacle_max_range"]
        assert scan["expected_update_rate"] <= 0.30
    assert global_costmap["obstacle_layer"]["scan"]["observation_persistence"] == 0.0
    assert global_costmap["obstacle_layer"]["scan"]["topic"] == "/scan_planning"
    assert global_costmap["obstacle_layer"]["scan"]["inf_is_valid"] is True
    assert global_costmap["obstacle_layer"]["observation_sources"] == "scan"
    assert "failed_segment_layer" in global_costmap["plugins"]
    failed_layer = global_costmap["failed_segment_layer"]
    assert failed_layer["plugin"] == "nav2_costmap_2d::StaticLayer"
    assert failed_layer["map_topic"] == "/navigation/failed_segment_mask"
    assert failed_layer["map_subscribe_transient_local"] is True
    assert failed_layer["use_maximum"] is False
    assert failed_layer["footprint_clearing_enabled"] is False
    assert global_costmap["always_send_full_costmap"] is True
    assert local_costmap["always_send_full_costmap"] is False
    assert local_costmap["obstacle_layer"]["scan"]["topic"] == "/scan_navigation"
    assert local_costmap["obstacle_layer"]["observation_sources"] == "scan"
    assert 10.0 <= controller["controller_frequency"] <= local_costmap["update_frequency"]
    assert limits["smoothing_frequency"] >= controller["controller_frequency"]
    assert abs(limits["max_decel"][0]) >= 0.5
    localization = navigation["rovera_navigation_adapter"]["ros__parameters"]
    assert localization["footprint_half_length"] == 0.15
    assert localization["footprint_half_width"] == 0.10
    assert localization["localization_rotation_minimum_obstacle_distance"] == safety[
        "rotation_margin"
    ]
    assert localization["planning_footprint_padding"] == global_costmap["footprint_padding"]
    assert safety["lidar_obstacle_avoidance_enabled"] is True
    assert safety["half_length"] == localization["footprint_half_length"]
    assert safety["half_width"] == localization["footprint_half_width"]
    # The self-return mask may be wider than the physical collision body; it
    # removes LiDAR hits from wheels/accessories and is not planning padding.
    assert sensor_time["scan_self_filter_half_length"] >= safety["half_length"]
    assert sensor_time["scan_self_filter_half_width"] >= safety["half_width"]
    assert safety["clearance"] == 0.04
    assert safety["clear_hysteresis"] == 0.20
    assert localization["corridor_hard_side_margin"] <= safety["side_margin"]
    assert safety["side_margin"] <= localization["corridor_side_margin"]
    # Collision planning uses the physical body without reusing scan-mask size.
    expected_collision_footprint = (
        "[[0.15, 0.10], [0.15, -0.10], [-0.15, -0.10], [-0.15, 0.10]]"
    )
    assert local_costmap["footprint"] == expected_collision_footprint
    assert global_costmap["footprint"] == expected_collision_footprint
    assert behavior["max_rotational_vel"] == limits["max_velocity"][2]
    assert behavior["rotational_acc_lim"] == limits["max_accel"][2]
    assert localization["scan_map_maximum_beams"] <= 120
    assert localization["scan_map_minimum_score"] > 0
    assert localization["localization_global_scan_map_minimum_score"] >= localization[
        "scan_map_minimum_score"
    ]
    assert localization["localization_global_final_scan_map_minimum_score"] > localization[
        "scan_map_minimum_score"
    ]
    assert localization["localization_global_scan_map_minimum_score"] >= localization[
        "localization_global_final_scan_map_minimum_score"
    ]
    assert localization["localization_final_minimum_residual_beams"] >= 20
    assert localization["localization_final_max_median_residual"] < 0.08
    assert localization["localization_final_max_p90_residual"] < localization[
        "localization_coarse_match_tolerance"
    ]
    assert localization["localization_coarse_match_tolerance"] > localization[
        "localization_final_max_median_residual"
    ]
    assert localization["localization_final_max_p90_residual"] < localization[
        "localization_coarse_match_tolerance"
    ]
    assert localization["planning_static_match_tolerance"] != localization[
        "localization_coarse_match_tolerance"
    ]
    assert localization["localization_raycast_minimum_comparable_beams"] >= 20
    assert localization["localization_raycast_minimum_static_matches"] >= 20
    assert localization["localization_raycast_minimum_static_matches"] <= localization[
        "localization_raycast_minimum_comparable_beams"
    ]
    assert 12 <= localization[
        "localization_operator_hint_minimum_comparable_beams"
    ] < localization["localization_raycast_minimum_comparable_beams"]
    assert 12 <= localization[
        "localization_operator_hint_minimum_static_matches"
    ] <= localization["localization_operator_hint_minimum_comparable_beams"]
    assert 0.65 <= localization[
        "localization_operator_hint_minimum_static_match_ratio"
    ] <= 0.80
    assert localization["dynamic_unconfirmed_blocker_timeout_seconds"] >= (
        localization["dynamic_obstacle_persistence_seconds"]
    )
    assert localization["dynamic_unconfirmed_blocker_log_interval_seconds"] >= 1.0
    assert 0 < localization[
        "localization_raycast_maximum_contradiction_ratio"
    ] <= 0.20
    assert localization["localization_raycast_match_tolerance"] <= 0.15
    assert localization[
        "localization_raycast_minimum_reliable_structure_span"
    ] >= 0.75
    assert localization["localization_global_min_heading_bins"] >= 4
    assert 135 <= localization["localization_global_min_heading_span_degrees"] < 180
    assert 0 < localization["scan_tf_wait_seconds"] <= 0.05
    assert 0 < localization["scan_tf_fallback_max_age_seconds"] <= 0.15
    assert 180.0 <= localization["auto_localization_max_angle_degrees"] < 360.0
    assert localization["localization_global_strong_min_heading_bins"] >= 2
    assert 60.0 <= localization[
        "localization_global_strong_min_heading_span_degrees"
    ] <= 90.0
    assert navigation["amcl"]["ros__parameters"]["max_particles"] >= 3000
    assert navigation["amcl"]["ros__parameters"]["max_beams"] >= 90
    assert 4.0 <= localization["localization_verify_timeout_seconds"] <= 6.0
    # This stale block was never launched; motion-safety owns the one smoother source of truth.
    assert "velocity_smoother" not in navigation


def test_micro_ros_agent_uses_stable_info_verbosity_by_default() -> None:
    project = Path(__file__).parents[1]

    for filename in ("compose.yaml", "compose.legacy-hardware.yml"):
        compose = (project / filename).read_text()
        assert compose.count("${MICRO_ROS_AGENT_VERBOSITY:-4}") == 2
        assert "${MICRO_ROS_AGENT_VERBOSITY:-2}" not in compose
    assert "MICRO_ROS_AGENT_VERBOSITY=4" in (
        project / "edge.env.example"
    ).read_text()


def test_ekf_uses_one_wheel_measurement_path_and_keeps_imu_unfused() -> None:
    project = Path(__file__).parents[1]
    ekf = yaml.safe_load(
        (project / "navigation-stack/config/ekf.yaml").read_text()
    )["ekf_filter_node"]["ros__parameters"]
    assert 0.05 <= ekf["transform_time_offset"] <= 0.20

    enabled = [index for index, value in enumerate(ekf["odom0_config"]) if value]
    assert enabled == [6, 11]  # forward velocity and yaw rate only
    assert "imu0" not in ekf


def test_all_auto_speed_profiles_are_centralized_and_within_manual_fast() -> None:
    project = Path(__file__).parents[1]
    profiles = AutoNavigationProfiles.load(
        project / "navigation-stack/config/auto_navigation_speed_profiles.yaml"
    )

    assert profiles.default_mode == "NORMAL"
    assert profiles.get("slow").linear_max == 0.17
    assert profiles.get("slow").collision_horizon == 0.50
    assert profiles.get("slow").min_lookahead_dist >= 0.50
    assert profiles.get("slow").lookahead_time >= 3.0
    assert profiles.get("normal").linear_max == 0.27
    assert profiles.get("fast").linear_max == profiles.hardware.linear_max == 0.33
    assert profiles.get("fast").angular_max == profiles.hardware.angular_max == 0.8
    assert profiles.get("fast").regulated_min_radius >= 1.0
    assert profiles.get("fast").regulated_min_speed <= 0.10
    assert profiles.get("fast").min_lookahead_dist >= 0.55
    assert profiles.behavior_parameters("SLOW") == {
        "max_rotational_vel": profiles.hardware.angular_max,
        "rotational_acc_lim": profiles.hardware.angular_accel_max,
    }
    assert profiles.smoother_parameters()["max_velocity"] == [0.33, 0.0, 0.8]
    with pytest.raises(SpeedProfileError):
        profiles.get("TURBO")


def test_speed_mode_persists_and_invalid_saved_state_falls_back(tmp_path: Path) -> None:
    store = SpeedModeStore(tmp_path / "mode.json")
    assert store.load() == "NORMAL"
    assert store.save("fast") == "FAST"
    assert SpeedModeStore(tmp_path / "mode.json").load() == "FAST"
    (tmp_path / "mode.json").write_text('{"mode":"TURBO"}')
    assert SpeedModeStore(tmp_path / "mode.json").load() == "NORMAL"


def test_auto_velocity_limiter_switches_profiles_without_touching_manual() -> None:
    project = Path(__file__).parents[1]
    profiles = AutoNavigationProfiles.load(
        project / "navigation-stack/config/auto_navigation_speed_profiles.yaml"
    )
    limiter = ProfileVelocityLimiter()
    linear, angular, reasons = limiter.apply(
        0.33, 0.8, profiles.get("SLOW"), now=1.0
    )
    assert linear == pytest.approx(0.015)
    assert angular == pytest.approx(0.06)
    assert "PROFILE_LINEAR_MAX" in reasons
    assert "PROFILE_ANGULAR_MAX" in reasons
    # A runtime switch retains the current command and ramps with FAST's own
    # Auto acceleration. The shared manual path is not an input to this class.
    fast_linear, fast_angular, _ = limiter.apply(
        0.33, 0.8, profiles.get("FAST"), now=1.1
    )
    assert fast_linear == pytest.approx(0.075)
    assert fast_angular == pytest.approx(0.26)


def test_pure_rotation_uses_one_shared_manual_fast_acceleration_stage() -> None:
    project = Path(__file__).parents[1]
    profiles = AutoNavigationProfiles.load(
        project / "navigation-stack/config/auto_navigation_speed_profiles.yaml"
    )
    limiter = ProfileVelocityLimiter(
        pure_rotation_angular_max=profiles.hardware.angular_max,
        pure_rotation_angular_decel=profiles.hardware.angular_decel_max,
    )

    linear, angular, reasons = limiter.apply(
        0.0, 0.8, profiles.get("SLOW"), now=1.0
    )

    assert linear == 0.0
    assert angular == profiles.hardware.angular_max
    assert "PROFILE_ANGULAR_ACCEL" not in reasons

    settling_linear, settling_angular, settling_reasons = limiter.apply(
        0.2, 0.0, profiles.get("SLOW"), now=1.1
    )
    assert (settling_linear, settling_angular) == (0.0, 0.0)
    assert "PURE_ROTATION_SETTLE" in settling_reasons

    resumed_linear, resumed_angular, _ = limiter.apply(
        0.2, 0.0, profiles.get("SLOW"), now=1.8
    )
    assert resumed_linear > 0.0
    assert resumed_angular == 0.0


def test_custom_behavior_trees_keep_path_until_bounded_recovery(tmp_path: Path) -> None:
    project = Path(__file__).parents[1]
    profiles = AutoNavigationProfiles.load(
        project / "navigation-stack/config/auto_navigation_speed_profiles.yaml"
    )
    paths = profiles.write_behavior_trees(tmp_path)

    fast_root = ElementTree.parse(paths["FAST"]).getroot()
    stable_sequence = fast_root.find(".//Sequence[@name='NavigateWithStablePath']")
    rate = fast_root.find(".//RateController")
    planners = fast_root.findall(".//ComputePathToPose")
    wait = fast_root.find(".//Wait")
    backups = fast_root.findall(".//BackUp")
    spins = fast_root.findall(".//Spin")
    follow_recovery = fast_root.find(".//RecoveryNode[@name='FollowPath']")
    recovery = fast_root.find(".//RecoveryNode[@name='NavigateRecovery']")
    clears = fast_root.findall(".//ClearEntireCostmap")
    assert stable_sequence is not None
    assert rate is None
    assert len(planners) == 1
    assert wait is not None and wait.attrib["wait_duration"] == "1"
    assert [backup.attrib for backup in backups] == [
        {"name": "BackUp-First", "backup_dist": "0.15", "backup_speed": "0.12"},
        {"name": "BackUp-Second", "backup_dist": "0.15", "backup_speed": "0.12"},
    ]
    assert spins == []
    assert follow_recovery is None
    assert clears == []
    assert recovery is not None and recovery.attrib["number_of_retries"] == "4"


def test_navigation_image_installs_the_configured_planner() -> None:
    project = Path(__file__).parents[1]
    dockerfile = (project / "navigation-stack/Dockerfile").read_text()

    assert "ros-humble-nav2-smac-planner" in dockerfile
    assert "ros-humble-nav2-rotation-shim-controller" in dockerfile
    assert "ros-humble-domain-bridge" in dockerfile


def test_navigation_debug_logging_defaults_on_and_uses_persistent_state_bind() -> None:
    project = Path(__file__).parents[1]
    compose = yaml.safe_load((project / "compose.navigation.yml").read_text())
    navigation = compose["services"]["navigation-stack"]

    assert navigation["environment"]["NAVIGATION_DEBUG_LOG"] == (
        "${NAVIGATION_DEBUG_LOG:-true}"
    )
    assert navigation["environment"]["NAVIGATION_DEBUG_LOG_PATH"].startswith(
        "${NAVIGATION_DEBUG_LOG_PATH:-/var/lib/rovera/navigation/logs/"
    )
    assert any(
        volume.get("target") == "/var/lib/rovera"
        for volume in navigation["volumes"]
        if isinstance(volume, dict)
    )


def test_pi_dds_profile_rejects_incompatible_remote_participants() -> None:
    project = Path(__file__).parents[1]
    profile = ElementTree.parse(project / "micro_ros_fastdds.xml").getroot()
    namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}

    assert profile.findtext(
        ".//f:participant/f:rtps/f:builtin/f:discovery_config/"
        "f:ignoreParticipantFlags",
        namespaces=namespace,
    ) == "FILTER_DIFFERENT_HOST"


def test_rviz_bridge_is_one_way_and_uses_a_lan_observation_profile() -> None:
    project = Path(__file__).parents[1]
    config = yaml.safe_load(
        (project / "navigation-stack/config/rviz_domain_bridge.yaml").read_text()
    )
    assert config["from_domain"] == 20
    assert config["to_domain"] == 21
    assert set(config["topics"]) == {
        "/map",
        "/scan_mapping",
        "/tf",
        "/tf_static",
        "/odometry/filtered",
        "/slam_toolbox/graph_visualization",
        "/robot_description",
    }
    serialized = (project / "navigation-stack/config/rviz_domain_bridge.yaml").read_text()
    assert "/cmd_vel" not in serialized
    assert "/odom_raw" not in serialized
    assert "bidirectional" not in serialized

    profile = ElementTree.parse(
        project / "rviz_lan_fastdds.xml"
    ).getroot()
    namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    assert profile.find(
        ".//f:participant/f:rtps/f:builtin/f:discovery_config/"
        "f:ignoreParticipantFlags",
        namespaces=namespace,
    ) is None
    assert (project / "rviz_lan_fastdds.xml").read_text() == (
        project.parents[1] / "config/rviz/rviz_lan_fastdds.xml"
    ).read_text()

    compose = yaml.safe_load((project / "compose.coexistence.yml").read_text())
    bridge = compose["services"]["rviz-bridge"]
    assert bridge["command"][-1] == "/opt/rovera/config/rviz_domain_bridge.yaml"
    assert bridge["environment"]["FASTRTPS_DEFAULT_PROFILES_FILE"] == (
        "/etc/rovera/rviz_lan_fastdds.xml"
    )


def test_mapping_profile_rejects_ambiguous_corridor_loop_closures() -> None:
    project = Path(__file__).parents[1]
    parameters = yaml.safe_load(
        (project / "navigation-stack/config/slam_toolbox.yaml").read_text()
    )["slam_toolbox"]["ros__parameters"]

    assert parameters["ceres_loss_function"] == "HuberLoss"
    assert parameters["minimum_distance_penalty"] >= 0.5
    assert parameters["minimum_travel_distance"] >= 0.1
    assert parameters["minimum_travel_heading"] >= 0.1
    assert parameters["loop_match_minimum_chain_size"] >= 15
    assert parameters["loop_match_maximum_variance_coarse"] <= 1.0
    assert parameters["loop_match_minimum_response_coarse"] >= 0.45
    assert parameters["loop_match_minimum_response_fine"] >= 0.55
    assert parameters["loop_search_space_dimension"] <= 4.0

    compose = yaml.safe_load((project / "compose.coexistence.yml").read_text())
    mounts = compose["services"]["mapping-stack"]["volumes"]
    assert any(
        mount.get("source") == "./navigation-stack/config/slam_toolbox.yaml"
        and mount.get("target") == "/opt/rovera/config/slam_toolbox.yaml"
        and mount.get("read_only") is True
        for mount in mounts
    )
