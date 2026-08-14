import math

from safety_core import (
    Direction,
    SafetyConfig,
    ScanSample,
    StopHysteresis,
    clip_motion_by_mask,
    evaluate_scan,
    motion_blocked_by_mask,
    stopping_clearance,
)


CONFIG = SafetyConfig()


def scan_with_points(points: list[tuple[float, float]]) -> ScanSample:
    ranges = [math.inf] * 360
    for x, y in points:
        angle = math.atan2(y, x)
        index = round((angle + math.pi) / (2 * math.pi / 360)) % 360
        ranges[index] = math.hypot(x, y)
    return ScanSample(-math.pi, 2 * math.pi / 360, 0.12, 8.0, tuple(ranges))


def test_front_and_rear_obstacles_only_block_matching_translation() -> None:
    front = scan_with_points([(0.24, 0.0)])
    assert evaluate_scan(front, linear_x=0.1, angular_z=0, config=CONFIG).stop
    assert not evaluate_scan(front, linear_x=-0.1, angular_z=0, config=CONFIG).stop
    rear = scan_with_points([(-0.24, 0.0)])
    assert evaluate_scan(rear, linear_x=-0.1, angular_z=0, config=CONFIG).stop


def test_side_and_corner_set_directional_polygon_mask() -> None:
    side = evaluate_scan(
        scan_with_points([(0.0, 0.24)]),
        linear_x=0,
        angular_z=0.4,
        config=CONFIG,
    )
    assert side.stop and side.blocked & Direction.LEFT
    corner = evaluate_scan(
        scan_with_points([(0.22, 0.19)]),
        linear_x=0.1,
        angular_z=0,
        config=CONFIG,
    )
    assert corner.blocked & Direction.FRONT
    assert corner.blocked & Direction.LEFT


def test_two_side_walls_do_not_become_a_front_stop() -> None:
    corridor = scan_with_points([
        (-0.1, -0.36), (0.0, -0.36), (0.3, -0.36),
        (-0.1, 0.36), (0.0, 0.36), (0.3, 0.36),
        (1.0, 0.0),
    ])
    decision = evaluate_scan(
        corridor,
        linear_x=0.17,
        angular_z=0.02,
        config=CONFIG,
    )
    assert not decision.stop
    assert not decision.blocked & Direction.FRONT


def test_front_object_in_swept_footprint_still_stops() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.32, 0.0), (1.0, 0.5)]),
        linear_x=0.17,
        angular_z=0.0,
        config=CONFIG,
    )
    assert decision.stop
    assert decision.blocked & Direction.FRONT


def test_corridor_can_allow_forward_but_block_left_rotation() -> None:
    corridor = scan_with_points([(0.0, -0.275), (0.0, 0.275), (1.0, 0.0)])
    assert not evaluate_scan(
        corridor, linear_x=0.1, angular_z=0.0, config=CONFIG
    ).stop
    assert not evaluate_scan(
        corridor, linear_x=0.1, angular_z=0.04, config=CONFIG
    ).stop
    rotating = evaluate_scan(
        corridor, linear_x=0.0, angular_z=0.3, config=CONFIG
    )
    assert rotating.stop
    assert rotating.blocked & Direction.LEFT


def test_forward_arc_clips_only_unsafe_turn_component() -> None:
    decision = evaluate_scan(
        # Seven centimetres beside the 0.18 m half-width clears the 0.06 m
        # straight margin, but lies inside the rectangular corner's turn sweep.
        scan_with_points([(0.0, 0.25), (1.0, 0.0)]),
        linear_x=0.15,
        angular_z=0.10,
        config=CONFIG,
    )
    assert not decision.stop
    assert decision.speed_scale > 0
    assert decision.angular_scale == 0
    assert decision.reason == "left_turn_clearance"


def test_external_direction_mask_clips_components_independently() -> None:
    assert clip_motion_by_mask(0.15, 0.10, Direction.LEFT) == (0.15, 0.0)
    assert clip_motion_by_mask(0.15, 0.10, Direction.FRONT) == (0.0, 0.10)


def test_dynamic_braking_distance_grows_with_velocity() -> None:
    assert stopping_clearance(0.15, CONFIG) > stopping_clearance(0.05, CONFIG) >= 0.10


def test_nan_inf_and_out_of_range_are_ignored_fail_closed_when_empty() -> None:
    scan = ScanSample(-1, 0.1, 0.12, 8.0, (math.nan, math.inf, 0.0, 9.0))
    result = evaluate_scan(scan, linear_x=0.1, angular_z=0, config=CONFIG)
    assert result.stop and result.reason == "empty_scan"


def test_slow_zone_scales_without_stopping() -> None:
    result = evaluate_scan(scan_with_points([(0.38, 0.0)]), linear_x=0.05, angular_z=0, config=CONFIG)
    assert not result.stop
    assert 0 < result.speed_scale < 1


def test_slow_zone_only_applies_to_commanded_direction() -> None:
    rear_close = scan_with_points([(-0.38, 0.0)])
    forward = evaluate_scan(
        rear_close,
        linear_x=0.1,
        angular_z=0,
        config=CONFIG,
    )
    reverse = evaluate_scan(
        rear_close,
        linear_x=-0.1,
        angular_z=0,
        config=CONFIG,
    )
    assert forward.speed_scale == 1.0
    assert 0 < reverse.speed_scale < 1.0


def test_lidar_self_return_inside_footprint_is_ignored() -> None:
    # The real robot intermittently reports its rear cover at about 0.12 m.
    # A clear external return keeps the scan valid while the self-hit is
    # removed from obstacle evaluation.
    scan = scan_with_points([(-0.12, -0.02), (1.0, 0.0)])
    result = evaluate_scan(scan, linear_x=0.1, angular_z=0, config=CONFIG)
    assert not result.stop
    assert result.blocked == Direction.NONE
    assert result.nearest_clearance > 0.5


def test_scan_with_only_self_returns_still_fails_closed() -> None:
    result = evaluate_scan(
        scan_with_points([(-0.12, -0.02)]),
        linear_x=0.1,
        angular_z=0,
        config=CONFIG,
    )
    assert result.stop
    assert result.reason == "empty_scan"


def test_clear_hysteresis_rejects_chatter() -> None:
    gate = StopHysteresis(0.4)
    assert gate.update(True, 1.0)
    assert gate.update(False, 1.1)
    assert gate.update(True, 1.2)
    assert gate.update(False, 1.3)
    assert gate.update(False, 1.69)
    assert not gate.update(False, 1.71)


def test_external_direction_mask_blocks_only_matching_motion() -> None:
    assert motion_blocked_by_mask(0.1, 0.0, Direction.FRONT)
    assert not motion_blocked_by_mask(-0.1, 0.0, Direction.FRONT)
    assert motion_blocked_by_mask(0.0, 0.2, Direction.LEFT)
    assert not motion_blocked_by_mask(0.0, -0.2, Direction.LEFT)
    assert motion_blocked_by_mask(
        -0.1,
        -0.2,
        Direction.REAR | Direction.RIGHT,
    )
