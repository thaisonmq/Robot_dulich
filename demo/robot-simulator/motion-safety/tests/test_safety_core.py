import math

from safety_core import (
    Direction,
    SafetyConfig,
    ScanSample,
    StopHysteresis,
    clip_motion_by_mask,
    evaluate_scan,
    motion_blocked_by_mask,
    maximum_safe_speed,
    protective_input_timeout_reason,
    safety_snapshot_payload,
    stopping_clearance,
)


CONFIG = SafetyConfig()


def test_required_protective_heartbeat_is_fail_closed() -> None:
    sources = (
        ("estop", True, 0.0, 1.2),
        ("cliff", False, 0.0, 0.6),
    )
    assert protective_input_timeout_reason(10.0, sources) == "estop_timeout"
    assert protective_input_timeout_reason(
        10.0,
        (("estop", True, 9.0, 1.2),),
    ) == ""
    assert protective_input_timeout_reason(
        10.3,
        (("estop", True, 9.0, 1.2),),
    ) == "estop_timeout"
    assert protective_input_timeout_reason(
        9.0,
        (("estop", True, 10.0, 1.2),),
    ) == "estop_timeout"


def scan_with_points(points: list[tuple[float, float]]) -> ScanSample:
    ranges = [math.inf] * 360
    for x, y in points:
        angle = math.atan2(y, x)
        index = round((angle + math.pi) / (2 * math.pi / 360)) % 360
        ranges[index] = math.hypot(x, y)
    return ScanSample(-math.pi, 2 * math.pi / 360, 0.12, 8.0, tuple(ranges))


def test_front_and_rear_obstacles_only_block_matching_translation() -> None:
    front = scan_with_points([(0.18, 0.0)])
    assert evaluate_scan(front, linear_x=0.1, angular_z=0, config=CONFIG).stop
    assert not evaluate_scan(front, linear_x=-0.1, angular_z=0, config=CONFIG).stop
    rear = scan_with_points([(-0.18, 0.0)])
    assert evaluate_scan(rear, linear_x=-0.1, angular_z=0, config=CONFIG).stop


def test_side_and_corner_set_directional_polygon_mask() -> None:
    side = evaluate_scan(
        scan_with_points([(0.14, 0.14)]),
        linear_x=0,
        angular_z=0.4,
        config=CONFIG,
    )
    assert side.stop and side.blocked & Direction.LEFT
    corner = evaluate_scan(
        scan_with_points([(0.18, 0.11)]),
        linear_x=0.1,
        angular_z=0,
        config=CONFIG,
    )
    assert corner.blocked & Direction.FRONT


def test_two_side_walls_do_not_become_a_front_stop() -> None:
    corridor = scan_with_points([
        (-0.1, -0.20), (0.0, -0.20), (0.3, -0.20),
        (-0.1, 0.20), (0.0, 0.20), (0.3, 0.20),
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
        scan_with_points([(0.18, 0.0), (1.0, 0.5)]),
        linear_x=0.17,
        angular_z=0.0,
        config=CONFIG,
    )
    assert decision.stop
    assert decision.blocked & Direction.FRONT


def test_corridor_can_allow_forward_but_block_left_rotation() -> None:
    corridor = scan_with_points([
        (-0.14, -0.168), (0.0, -0.168), (0.14, -0.168),
        (-0.14, 0.168), (0.0, 0.168), (0.14, 0.168), (1.0, 0.0),
    ])
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
        # Five centimetres beside the 0.10 m half-width clears the 0.04 m
        # straight margin, but lies inside the rectangular corner's turn sweep.
        scan_with_points([(0.13, 0.145), (1.0, 0.0)]),
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
    assert stopping_clearance(0.15, CONFIG) > stopping_clearance(0.05, CONFIG) >= 0.04


def test_nan_inf_and_out_of_range_are_ignored_fail_closed_when_empty() -> None:
    scan = ScanSample(-1, 0.1, 0.12, 8.0, (math.nan, math.inf, 0.0, 9.0))
    result = evaluate_scan(scan, linear_x=0.1, angular_z=0, config=CONFIG)
    assert result.stop and result.reason == "empty_scan"


def test_slow_zone_scales_without_stopping() -> None:
    result = evaluate_scan(
        scan_with_points([(0.195, 0.0)]),
        linear_x=0.05,
        angular_z=0,
        measured_linear_x=0.0,
        config=CONFIG,
    )
    assert not result.stop
    assert 0 < result.speed_scale < 1


def test_slow_zone_only_applies_to_commanded_direction() -> None:
    rear_close = scan_with_points([(-0.20, 0.0)])
    forward = evaluate_scan(
        rear_close,
        linear_x=0.1,
        angular_z=0,
        measured_linear_x=0.0,
        config=CONFIG,
    )
    reverse = evaluate_scan(
        rear_close,
        linear_x=-0.1,
        angular_z=0,
        measured_linear_x=0.0,
        config=CONFIG,
    )
    assert forward.speed_scale == 1.0
    assert 0 < reverse.speed_scale < 1.0


def test_side_obstacle_allows_translation_but_clips_unsafe_arc() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.18, 0.13), (1.0, 0.0)]),
        linear_x=0.15,
        angular_z=-0.20,
        config=CONFIG,
    )
    assert decision.speed_scale == 1.0
    assert decision.angular_scale == 0.0
    assert not decision.blocked & Direction.FRONT
    assert decision.blocked & Direction.RIGHT
    assert not decision.stop


def test_close_parallel_wall_is_not_in_straight_braking_envelope() -> None:
    decision = evaluate_scan(
        scan_with_points([
            (0.18, 0.12), (0.30, 0.12), (0.50, 0.12), (1.0, 0.0),
        ]),
        linear_x=0.12,
        angular_z=0.0,
        measured_linear_x=0.12,
        config=CONFIG,
    )
    assert not decision.stop
    assert not decision.blocked & Direction.FRONT
    assert decision.speed_scale == 1.0


def test_translation_block_reports_the_actual_front_beam() -> None:
    scan = scan_with_points([(0.18, 0.0), (1.0, 0.5)])
    decision = evaluate_scan(
        scan,
        linear_x=0.12,
        angular_z=0.0,
        measured_linear_x=0.12,
        config=CONFIG,
    )
    assert decision.stop
    assert decision.blocking_beam_index >= 0
    assert decision.blocking_point_x is not None
    assert decision.blocking_point_y is not None
    assert decision.blocking_range == scan.ranges[decision.blocking_beam_index]
    assert decision.predicted_swept_clearance == decision.front_clearance
    assert decision.required_swept_clearance == decision.required_stop_distance


def test_scan_points_are_transformed_from_laser_to_chassis_frame() -> None:
    config = SafetyConfig(laser_x=-0.005)
    decision = evaluate_scan(
        scan_with_points([(0.30, 0.0)]),
        linear_x=0.05,
        angular_z=0.0,
        measured_linear_x=0.0,
        config=config,
    )
    assert math.isclose(decision.front_clearance, 0.14, abs_tol=1e-6)


def test_obstacle_beyond_slow_envelope_does_not_create_far_stop() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.50, 0.0)]),
        linear_x=0.24,
        angular_z=0.0,
        config=CONFIG,
    )
    assert not decision.stop
    assert decision.speed_scale == 1.0
    assert not decision.blocked & Direction.FRONT


def test_stopping_distance_and_slowdown_are_progressive_by_speed() -> None:
    clearances = [stopping_clearance(speed, CONFIG) for speed in (0.10, 0.18, 0.24)]
    assert clearances[0] < clearances[1] < clearances[2]
    obstacle = scan_with_points([(0.21, 0.0)])
    scales = [
        evaluate_scan(
            obstacle,
            linear_x=speed,
            angular_z=0.0,
            measured_linear_x=0.0,
            config=CONFIG,
        ).speed_scale
        for speed in (0.10, 0.18, 0.24)
    ]
    assert scales[0] > scales[1] > scales[2]


def test_stationary_robot_gets_a_usable_speed_cap_instead_of_command_latch() -> None:
    # The bumper has 4.5 cm free while the requested 16 cm/s command would need
    # more room.  Measured odometry is stationary, so safety must issue the
    # physically admissible speed rather than treating the request as current
    # momentum and latching the direction at zero forever.
    decision = evaluate_scan(
        scan_with_points([(0.20, 0.0), (1.0, 0.5)]),
        linear_x=0.16,
        angular_z=0.0,
        measured_linear_x=0.0,
        config=CONFIG,
    )
    output_speed = 0.16 * decision.speed_scale
    assert not decision.stop
    assert 0.03 < output_speed < 0.08
    assert math.isclose(
        stopping_clearance(output_speed, CONFIG),
        0.045,
        abs_tol=1e-6,
    )


def test_measured_motion_inside_stop_envelope_still_hard_stops() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.20, 0.0), (1.0, 0.5)]),
        linear_x=0.16,
        angular_z=0.0,
        measured_linear_x=0.16,
        config=CONFIG,
    )
    assert decision.stop
    assert decision.speed_scale == 0.0
    assert decision.blocked & Direction.FRONT


def test_maximum_safe_speed_is_inverse_of_stopping_clearance() -> None:
    for speed in (0.02, 0.08, 0.16, 0.24):
        clearance = stopping_clearance(speed, CONFIG)
        assert math.isclose(maximum_safe_speed(clearance, CONFIG), speed)


def test_lidar_self_return_inside_footprint_is_ignored() -> None:
    # The real robot intermittently reports its rear cover at about 0.12 m.
    # A clear external return keeps the scan valid while the self-hit is
    # removed from obstacle evaluation.
    scan = scan_with_points([(-0.12, -0.02), (1.0, 0.0)])
    result = evaluate_scan(scan, linear_x=0.1, angular_z=0, config=CONFIG)
    assert not result.stop
    assert result.blocked == Direction.NONE
    assert result.nearest_clearance > 0.5


def test_last_run_rear_right_body_return_is_ignored() -> None:
    # Replay the fixed 160 mm return that followed the chassis during the
    # 2026-09-04 run. It is inside the shared 0.31 x 0.20 m envelope.
    scan = scan_with_points([(-0.1530087617, -0.0467794580), (1.0, 0.0)])
    result = evaluate_scan(scan, linear_x=-0.08, angular_z=0.0, config=CONFIG)

    assert not result.stop
    assert not result.blocked & Direction.REAR


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


def test_measured_angular_velocity_protects_sweep_after_command_slows() -> None:
    scan = scan_with_points([(0.115, 0.14), (2.0, 0.0)])
    commanded_only = evaluate_scan(
        scan,
        linear_x=0.0,
        angular_z=0.04,
        measured_angular_z=0.04,
        config=CONFIG,
    )
    still_rotating = evaluate_scan(
        scan,
        linear_x=0.0,
        angular_z=0.04,
        measured_angular_z=0.8,
        config=CONFIG,
    )
    assert not commanded_only.stop
    assert still_rotating.stop
    assert still_rotating.reason == "rotation_sweep_collision"
    assert still_rotating.blocked & Direction.LEFT
    assert still_rotating.blocking_beam_index >= 0
    assert still_rotating.blocking_point_x is not None
    assert still_rotating.blocking_point_y is not None
    assert still_rotating.blocking_range == scan.ranges[
        still_rotating.blocking_beam_index
    ]
    assert still_rotating.requested_rotation_direction == "LEFT"
    assert still_rotating.measured_angular_velocity == 0.8
    assert still_rotating.predicted_swept_clearance is not None
    assert still_rotating.required_swept_clearance == CONFIG.rotation_margin


def test_zero_command_still_reports_prospective_turn_blockage() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.115, 0.14), (2.0, 0.0)]),
        linear_x=0.0,
        angular_z=0.0,
        measured_linear_x=0.0,
        measured_angular_z=0.0,
        config=CONFIG,
    )
    assert not decision.stop
    assert decision.rotation_left_blocked
    assert not decision.rotation_right_blocked


def test_atomic_snapshot_reuses_one_decision_sequence_and_diagnostics() -> None:
    decision = evaluate_scan(
        scan_with_points([(0.18, 0.0), (1.0, 0.5)]),
        linear_x=0.12,
        angular_z=0.0,
        measured_linear_x=0.12,
        measured_angular_z=0.0,
        config=CONFIG,
    )
    payload = safety_snapshot_payload(
        sequence=123,
        stamp=10.5,
        health="BLOCKED:front_sweep_collision",
        stop=True,
        reason=decision.reason,
        source="MOTION_SAFETY",
        direction_mask=int(decision.blocked),
        input_linear=0.12,
        input_angular=0.0,
        output_linear=0.0,
        output_angular=0.0,
        measured_linear=0.12,
        measured_angular=0.0,
        decision=decision,
    )
    assert payload["seq"] == 123
    assert payload["reason"] == "FRONT_SWEEP_COLLISION"
    assert payload["direction_mask"] == int(decision.blocked)
    assert payload["output_v"] == payload["output_w"] == 0.0
    assert payload["blocking_beam_index"] == decision.blocking_beam_index
