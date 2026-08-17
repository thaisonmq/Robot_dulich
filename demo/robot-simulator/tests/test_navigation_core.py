import math
import sys
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / "navigation-stack"))

from navigation_core import (  # noqa: E402
    NavigationDebugLog,
    SavedOccupancyMap,
    SensorClockEstimator,
    classify_planning_failure,
    compact_lethal_cells,
    evaluate_corridor,
    filter_static_map_scan,
    heading_diversity,
    localization_confidence,
    mask_scan_self_returns,
    path_overlap_ratio,
    pose_stability,
    rotation_swept_clearance,
    scan_to_map_match,
    validate_executable_grid_path,
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


def _cell_center(saved: SavedOccupancyMap, column: int, row: int) -> tuple[float, float]:
    return saved.cell_center(column, row)


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
    assert localization_confidence(
        covariance,
        stability_score=1.0,
        scan_map_score=0.6,
        scan_map_threshold=0.35,
        scan_fresh=True,
        tf_stable=True,
        odometry_healthy=True,
        sensor_time_valid=True,
    ) > 0.9
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
    assert not uncertain.can_go_straight
    assert not physically_blocked.physically_passable
    assert physically_blocked.classification == "PHYSICALLY_BLOCKED"
    assert front_blocked.front_clearance == pytest.approx(0.13)
    assert front_blocked.classification == "PHYSICALLY_BLOCKED"
    assert front_blocked.reason == "FRONT_CLEARANCE"


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
        half_length=0.15,
        half_width=0.10,
    )
    assert masked == 1
    assert math.isnan(filtered[0])
    assert filtered[1:] == ranges[1:]


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
    controller = navigation["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    planner_server = navigation["planner_server"]["ros__parameters"]
    planner = planner_server["GridBased"]
    global_costmap = navigation["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = navigation["local_costmap"]["local_costmap"]["ros__parameters"]
    behavior = navigation["behavior_server"]["ros__parameters"]
    limits = smoother["velocity_smoother"]["ros__parameters"]

    assert controller["odom_topic"] == "/odometry/filtered"
    assert follow["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow["primary_controller"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    # Small 5-10 degree errors remain with RPP; large initial reversals still
    # select pure rotation.
    assert math.radians(25) <= follow["angular_dist_threshold"] <= math.radians(35)
    assert 0.0 < follow["angular_disengage_threshold"] < follow["angular_dist_threshold"]
    assert follow["closed_loop"] is True
    assert follow["rotate_to_goal_heading"] is False
    assert planner["plugin"] == "nav2_theta_star_planner/ThetaStarPlanner"
    assert planner["allow_unknown"] is False
    assert planner["how_many_corners"] == 8
    assert planner["w_euc_cost"] == 1.0
    # Euclidean distance dominates small inflation changes, preventing an
    # otherwise open straight route from becoming an S-shaped preview. The
    # footprint check, not a large soft cost, remains the collision authority.
    assert 0.0 < planner["w_traversal_cost"] <= 0.05
    assert planner_server["planner_plugins"] == ["GridBased", "FootprintGrid"]
    footprint_planner = planner_server["FootprintGrid"]
    assert footprint_planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert footprint_planner["allow_unknown"] is False
    assert footprint_planner["downsample_costmap"] is False
    assert 0.20 <= follow["forward_sampling_distance"] <= 0.30
    assert 0.15 < follow["desired_linear_vel"] <= limits["max_velocity"][0]
    assert follow["rotate_to_heading_angular_vel"] == limits["max_velocity"][2]
    assert follow["max_angular_accel"] == limits["max_accel"][2]
    assert follow["lookahead_dist"] >= 0.35
    assert follow["regulated_linear_scaling_min_speed"] >= 0.07
    assert follow["max_allowed_time_to_collision_up_to_carrot"] >= 0.5
    assert 10.0 <= controller["progress_checker"]["movement_time_allowance"] <= 15.0
    assert controller["failure_tolerance"] <= 1.0
    # Wide/right-angle corners must remain forward arcs, while an almost
    # opposite initial heading still selects the fast in-place turn.
    assert math.pi / 2 < follow["rotate_to_heading_min_angle"] < math.pi
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
    assert localization["corridor_hard_side_margin"] <= safety["side_margin"]
    assert safety["side_margin"] <= localization["corridor_side_margin"]
    # The complete measured footprint is the single source of truth.
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
    assert localization["localization_global_min_heading_bins"] >= 4
    assert 135 <= localization["localization_global_min_heading_span_degrees"] < 180
    assert 0 < localization["scan_tf_wait_seconds"] <= 0.05
    assert 0 < localization["scan_tf_fallback_max_age_seconds"] <= 0.15
    assert localization["auto_localization_max_angle_degrees"] >= 360.0
    assert navigation["amcl"]["ros__parameters"]["max_particles"] >= 3000
    assert navigation["amcl"]["ros__parameters"]["max_beams"] >= 90
    assert localization["localization_verify_timeout_seconds"] <= 3.0
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
