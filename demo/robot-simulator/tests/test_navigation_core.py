import math
import sys
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / "navigation-stack"))

from navigation_core import (  # noqa: E402
    SavedOccupancyMap,
    compact_lethal_cells,
    localization_confidence,
    navigation_abort_state,
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
        covariance, stable_samples=5, scan_fresh=True, tf_stable=True
    ) > 0.9
    assert localization_confidence(
        covariance, stable_samples=1, scan_fresh=True, tf_stable=True
    ) < 0.25
    assert localization_confidence(
        covariance, stable_samples=5, scan_fresh=False, tf_stable=True
    ) == 0


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
        {"x": -0.95, "y": 2.15},
    ]


def test_navigation_abort_is_blocked_only_after_bounded_recovery() -> None:
    assert navigation_abort_state(0) == "FAILED"
    assert navigation_abort_state(1) == "BLOCKED"
    assert navigation_abort_state(6) == "BLOCKED"


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
    limits = smoother["velocity_smoother"]["ros__parameters"]

    assert planner["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert planner["smooth_path"] is True
    assert planner["allow_unknown"] is False
    assert planner["cost_travel_multiplier"] >= 2.0
    assert 0.15 < follow["desired_linear_vel"] <= limits["max_velocity"][0]
    assert 0.55 < follow["rotate_to_heading_angular_vel"] <= limits["max_velocity"][2]
    assert follow["lookahead_dist"] >= 0.35
    assert follow["regulated_linear_scaling_min_speed"] >= 0.07
    assert follow["max_allowed_time_to_collision_up_to_carrot"] >= 0.5
    assert controller["progress_checker"]["movement_time_allowance"] >= 10.0
    assert global_costmap["update_frequency"] >= 3
    assert global_costmap["inflation_layer"]["inflation_radius"] >= 0.4


def test_navigation_image_installs_the_configured_planner() -> None:
    project = Path(__file__).parents[1]
    dockerfile = (project / "navigation-stack/Dockerfile").read_text()

    assert "ros-humble-nav2-smac-planner" in dockerfile
