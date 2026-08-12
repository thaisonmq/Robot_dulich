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
    SavedOccupancyMap,
    SensorClockEstimator,
    compact_lethal_cells,
    localization_confidence,
    mask_scan_self_returns,
    navigation_abort_state,
    pose_stability,
    rotation_swept_clearance,
    scan_to_map_match,
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


def test_navigation_abort_is_blocked_only_after_bounded_recovery() -> None:
    assert navigation_abort_state(0) == "FAILED"
    assert navigation_abort_state(1) == "BLOCKED"
    assert navigation_abort_state(6) == "BLOCKED"


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
    ranges = [0.123, 0.30, 1.0, math.inf]
    filtered, masked = mask_scan_self_returns(
        ranges,
        angle_min=-math.pi / 2,
        angle_increment=math.pi / 2,
        range_min=0.12,
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


def test_navigation_motion_tuning_stays_within_final_smoother_limits() -> None:
    project = Path(__file__).parents[1]
    navigation = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )
    smoother = yaml.safe_load(
        (project / "motion-safety/config/velocity_smoother.yaml").read_text()
    )
    controller = navigation["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    planner = navigation["planner_server"]["ros__parameters"]["GridBased"]
    global_costmap = navigation["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = navigation["local_costmap"]["local_costmap"]["ros__parameters"]
    limits = smoother["velocity_smoother"]["ros__parameters"]

    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert planner["smooth_path"] is True
    assert planner["allow_unknown"] is False
    assert planner["lattice_filepath"].endswith("/diff/output.json")
    assert planner["cost_penalty"] >= 2.0
    assert planner["rotation_penalty"] <= 2.0
    assert 0.15 < follow["desired_linear_vel"] <= limits["max_velocity"][0]
    assert 0.55 < follow["rotate_to_heading_angular_vel"] <= limits["max_velocity"][2]
    assert follow["lookahead_dist"] >= 0.35
    assert follow["regulated_linear_scaling_min_speed"] >= 0.07
    assert follow["max_allowed_time_to_collision_up_to_carrot"] >= 0.5
    assert controller["progress_checker"]["movement_time_allowance"] >= 10.0
    assert controller["failure_tolerance"] >= 1.5
    assert follow["rotate_to_heading_min_angle"] <= 0.4
    assert global_costmap["update_frequency"] >= 5
    assert local_costmap["update_frequency"] > global_costmap["update_frequency"]
    assert global_costmap["obstacle_layer"]["combination_method"] == 1
    assert global_costmap["footprint_padding"] > local_costmap["footprint_padding"]
    assert global_costmap["inflation_layer"]["inflation_radius"] >= 0.4
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
    assert global_costmap["obstacle_layer"]["scan"]["observation_persistence"] <= 0.5
    assert controller["controller_frequency"] >= 20.0
    assert limits["smoothing_frequency"] >= controller["controller_frequency"]
    assert abs(limits["max_decel"][0]) >= 0.5
    localization = navigation["rovera_navigation_adapter"]["ros__parameters"]
    assert localization["footprint_half_length"] == 0.20
    assert localization["footprint_half_width"] == 0.18
    expected_footprint = "[[0.20, 0.18], [0.20, -0.18], [-0.20, -0.18], [-0.20, 0.18]]"
    assert local_costmap["footprint"] == expected_footprint
    assert global_costmap["footprint"] == expected_footprint
    assert localization["scan_map_maximum_beams"] <= 120
    assert localization["scan_map_minimum_score"] > 0
    assert localization["localization_verify_timeout_seconds"] <= 3.0
    # This stale block was never launched; motion-safety owns the one smoother source of truth.
    assert "velocity_smoother" not in navigation


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
    assert profiles.get("normal").linear_max == 0.27
    assert profiles.get("fast").linear_max == profiles.hardware.linear_max == 0.33
    assert profiles.get("fast").angular_max == profiles.hardware.angular_max == 0.8
    assert profiles.get("FAST").controller_parameters()[
        "FollowPath.rotate_to_heading_angular_vel"
    ] == 0.8
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


def test_custom_behavior_trees_use_profile_replan_and_bounded_recovery(tmp_path: Path) -> None:
    project = Path(__file__).parents[1]
    profiles = AutoNavigationProfiles.load(
        project / "navigation-stack/config/auto_navigation_speed_profiles.yaml"
    )
    paths = profiles.write_behavior_trees(tmp_path)

    fast_root = ElementTree.parse(paths["FAST"]).getroot()
    rate = fast_root.find(".//RateController")
    wait = fast_root.find(".//Wait")
    backups = fast_root.findall(".//BackUp")
    recovery = fast_root.find(".//RecoveryNode[@name='NavigateRecovery']")
    assert rate is not None and rate.attrib["hz"] == "2.00"
    assert wait is not None and wait.attrib["wait_duration"] == "1"
    assert [backup.attrib for backup in backups] == [
        {"name": "BackUp-First", "backup_dist": "0.15", "backup_speed": "0.12"},
        {"name": "BackUp-Second", "backup_dist": "0.15", "backup_speed": "0.12"},
    ]
    assert recovery is not None and recovery.attrib["number_of_retries"] == "5"


def test_navigation_image_installs_the_configured_planner() -> None:
    project = Path(__file__).parents[1]
    dockerfile = (project / "navigation-stack/Dockerfile").read_text()

    assert "ros-humble-nav2-smac-planner" in dockerfile
