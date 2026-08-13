from pathlib import Path


ADAPTER_SOURCE = (
    Path(__file__).parents[1] / "navigation-stack" / "adapter_node.py"
).read_text()
EDGE_CLIENT_SOURCE = (
    Path(__file__).parents[1] / "simulator" / "client.py"
).read_text()
MAP_CACHE_SOURCE = (
    Path(__file__).parents[1] / "simulator" / "map_cache.py"
).read_text()


def _method_source(name: str, next_name: str) -> str:
    start = ADAPTER_SOURCE.index(f"    def {name}(")
    end = ADAPTER_SOURCE.index(f"    def {next_name}(", start)
    return ADAPTER_SOURCE[start:end]


def test_only_recent_sustained_navigation_pose_gets_fast_local_verification() -> None:
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    operator = _method_source("_set_initial_pose", "_deactivate_map")

    assert 'last_pose.get("source", "")' in automatic
    assert '"recent_navigation_pose"' in automatic
    assert "self.localization_seed_approximate = not recent_verified_pose" in automatic
    assert "self.global_search_requires_rotation = False" in automatic
    assert "approximate=True" in operator
    assert "approximate=False" not in operator


def test_approximate_pose_searches_near_position_over_every_heading() -> None:
    publish = _method_source("_publish_initial_pose", "_reset_localization_evidence")

    assert "position_variance = 0.36" in publish
    assert "yaw_variance = math.pi ** 2 / 3.0" in publish
    assert '"odom", "base_footprint", Time()' in publish
    assert "message.header.stamp = latest_odom.header.stamp" in publish
    assert "self.initial_pose_requested = False" in publish
    assert "Duration(" not in publish


def test_recent_verified_pose_is_not_broadened_into_an_adjacent_wall_cell() -> None:
    publish = _method_source("_publish_initial_pose", "_reset_localization_evidence")

    exact = publish.split("else:", 1)[1]
    assert 'position_variance = max(0.01, float(pose.get("covariance", 0.25)))' in exact
    assert "yaw_variance = 0.02" in exact


def test_each_localization_phase_discards_old_evidence_and_uses_full_threshold() -> None:
    global_localization = _method_source("_start_global_localization", "_safe_to_rotate")
    tick = _method_source("_localization_tick", "_load_map")
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")

    assert "self._reset_localization_evidence()" in global_localization
    assert "self.localization_confidence_threshold" in evidence
    assert "required_confidence" not in tick
    assert '"LOCALIZING_APPROXIMATE_POSE"' in tick
    assert "self.approximate_pose_timeout" in tick
    assert "self._start_global_localization()" in tick


def test_localization_never_rotates_without_explicit_authorization() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    operator = _method_source("_set_initial_pose", "_deactivate_map")
    tick = _method_source("_localization_tick", "_load_map")

    assert 'payload.get("allow_rotation", False)' in dispatch
    assert "self.localization_rotation_authorized = False" in automatic
    assert "self.localization_rotation_authorized = False" in operator
    assert "if not self.localization_rotation_authorized" in tick


def test_localization_rotation_keeps_every_live_safety_gate() -> None:
    safe = _method_source("_safe_to_rotate", "_stop_localization_rotation")
    tick = _method_source("_localization_tick", "_load_map")

    for required in (
        "self._critical_sensor_time_healthy()",
        'self.safety_health.startswith("HEALTHY")',
        "not self.estop_active",
        "self.safety_direction_mask == 0",
        "self.last_manual_takeover_monotonic",
        "self.current_goal_handle is None",
        "self.nearest_rotation_obstacle",
        "self.rotation_minimum_obstacle_distance",
    ):
        assert required in safe
    assert "self.last_amcl_pose" not in safe
    assert "self.scan_map_valid_beams" not in safe
    assert "self.rotation_angle >= self.rotation_max_angle" in tick
    assert "self._stop_localization_rotation()" in tick
    assert 'self.localization_state = "LOCALIZING_GLOBAL"' in tick


def test_global_search_requires_rotation_only_after_explicit_authorization() -> None:
    global_search = _method_source("_start_global_localization", "_safe_to_rotate")
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")

    guard = global_search.index("if not self.localization_rotation_authorized")
    requirement = global_search.index("self.global_search_requires_rotation = True")
    assert guard < requirement
    assert "self.global_search_requires_rotation = True" in global_search
    assert "self.rotation_angle >= self.global_observation_minimum_rotation" in evidence
    assert 'self.localization_state = "LOCALIZATION_REQUIRED"' in global_search


def test_ready_session_reuses_continuously_verified_pose_without_reset() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")

    assert 'self.localized and self.localization_state == "READY"' in dispatch
    ready_branch = dispatch.split(
        'if self.localized and self.localization_state == "READY":', 1
    )[1].split("elif self.localization_state in", 1)[0]
    assert '"status": "completed"' in ready_branch
    assert "self._begin_localization_verification" not in ready_branch
    assert "self._start_global_localization" not in ready_branch


def test_auto_go_force_global_bypasses_ready_and_old_amcl_pose() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")

    force_branch = dispatch.split("if force_global:", 1)[1].split(
        'elif self.localized and self.localization_state == "READY":', 1
    )[0]
    assert 'payload.get("force_global", False)' in dispatch
    assert "if not allow_rotation" in force_branch
    assert "self.localization_rotation_authorized = True" in force_branch
    assert "self._start_global_localization()" in force_branch


def test_repeated_force_global_does_not_reset_an_active_scan() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    force_branch = dispatch.split("if force_global:", 1)[1].split(
        'elif self.localized and self.localization_state == "READY":', 1
    )[0]

    active_guard = force_branch.index('"LOCALIZING_GLOBAL", "LOCALIZING_ROTATING"')
    accepted = force_branch.index('"status": "accepted"', active_guard)
    restart = force_branch.index("self._start_global_localization()", accepted)
    assert active_guard < accepted < restart
    assert '"LOCALIZING_SETTLING"' in force_branch[active_guard:accepted]


def test_completed_rotation_settles_then_uses_fresh_stationary_evidence() -> None:
    begin = _method_source(
        "_begin_localization_settling", "_start_localization_settling_evidence"
    )
    evidence = _method_source(
        "_start_localization_settling_evidence", "_nomotion_update_due"
    )
    tick = _method_source("_localization_tick", "_load_map")

    assert "self._stop_localization_rotation()" in begin
    assert 'self.localization_state = "LOCALIZING_SETTLING"' in begin
    assert "self.last_amcl_pose = None" not in evidence
    assert "self.last_amcl_covariance = []" not in evidence
    assert "self.pose_window.clear()" in evidence
    assert "self.scan_map_scores.clear()" in evidence
    max_rotation = tick.split(
        "if self.rotation_angle >= self.rotation_max_angle:", 1
    )[1].split("if not self._safe_to_rotate():", 1)[0]
    assert "self._begin_localization_settling(now)" in max_rotation
    assert '"LOCALIZATION_FAILED"' not in max_rotation
    assert "self.localization_rotation_settle" in tick


def test_map_load_does_not_inject_a_persisted_robot_pose() -> None:
    load_branch = EDGE_CLIENT_SOURCE.split(
        'if command == "map.load" and self.config.navigation_backend == "ros2":', 1
    )[1].split('if (\n            command == "mapping.start"', 1)[0]

    assert 'command_payload["map_path"]' in load_branch
    assert 'command_payload["last_known_pose"]' not in load_branch
    assert "self.map_cache.activation_pose" not in load_branch

    restore_payload = MAP_CACHE_SOURCE.split(
        "    def active_load_payload(", 1
    )[1].split("    def delete_local(", 1)[0]
    assert '"last_known_pose"' not in restore_payload
    assert "self.activation_pose" not in restore_payload


def test_existing_amcl_pose_is_passively_verified_before_authorized_global_search() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    verify = _method_source(
        "_begin_localization_verification", "_begin_auto_localization"
    )

    existing_pose = dispatch.index("elif self.last_amcl_pose is not None")
    global_search = dispatch.index("self._start_global_localization()", existing_pose)
    assert existing_pose < global_search
    assert "allow_rotation=allow_rotation" in dispatch[existing_pose:global_search]
    assert "self.global_search_requires_rotation = False" in verify


def test_ready_requires_scan_map_pose_window_and_synchronized_time() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    evidence = _method_source("_localization_evidence_ready", "_localization_tick")

    assert "self._pose_is_stable()" in evidence
    assert "self.amcl_pose_freshness" in evidence
    assert "self.scan_map_score >= self.scan_map_threshold" in evidence
    assert "self._critical_sensor_time_healthy()" in evidence
    assert "self.global_observation_minimum_rotation" in evidence
    assert 'self.localization_state == "VERIFYING"' in tick
    assert "self.localization_ready_hold" in tick
    assert 'self.localized and self.localization_state == "READY"' in tick
    assert "self.ready_evidence_invalid_since" in tick


def test_ready_tracking_uses_low_threshold_without_stationary_reacquisition() -> None:
    confidence = _method_source(
        "_refresh_localization_confidence", "_path_callback"
    )
    tracking = _method_source(
        "_localization_tracking_evidence_ready", "_nomotion_update_due"
    )
    tick = _method_source("_localization_tick", "_load_map")

    assert 'self.localized and self.localization_state == "READY"' in confidence
    assert "self.localization_low_threshold" in tracking
    assert "self.amcl_pose_freshness" in tracking
    assert "self._critical_sensor_time_healthy()" in tracking
    assert "self._pose_is_stable()" not in tracking
    assert "self.scan_map_score >= self.scan_map_threshold" not in tracking
    ready_branch = tick.split(
        'if self.localized and self.localization_state == "READY":', 1
    )[1].split("localizing_states =", 1)[0]
    assert "self._localization_tracking_evidence_ready(now)" in ready_branch


def test_tracked_pose_does_not_apply_stationary_acquisition_gate() -> None:
    confidence = _method_source(
        "_refresh_localization_confidence", "_path_callback"
    )
    tracking = _method_source(
        "_localization_tracking_evidence_ready", "_nomotion_update_due"
    )
    tick = _method_source("_localization_tick", "_load_map")

    assert "navigation_in_progress = self._navigation_in_progress()" in confidence
    assert "tracking_ready_pose" in confidence
    assert "stability_score=stability_score" in confidence
    assert "navigation_in_progress = self._navigation_in_progress()" in tick
    assert "self._pose_is_stable()" not in tracking
    assert "self._localization_tracking_evidence_ready(now)" in tick
    assert "max(fresh_scan_map_score, self.scan_map_threshold)" in confidence
    assert "self._nomotion_update_due(" in tick


def test_active_but_stationary_navigation_refreshes_amcl_before_pose_expires() -> None:
    refresh = _method_source("_nomotion_update_due", "_localization_tick")

    assert "self.amcl_pose_freshness * 0.5" in refresh
    assert "now - self.last_amcl_monotonic >= refresh_age" in refresh
    assert "self.nomotion_update_client.service_is_ready()" in refresh


def test_scan_callback_never_replaces_capture_stamp_with_now() -> None:
    scan = _method_source("_scan_callback", "_sensor_time_callback")
    normalizer = (
        Path(__file__).parents[1] / "navigation-stack" / "sensor_normalizer.py"
    ).read_text()

    assert "header.stamp" not in scan
    assert "SensorClockEstimator" in normalizer
    assert "corrected_nanoseconds" in normalizer


def test_cancel_revokes_nav_velocity_ownership_before_waiting_for_ack() -> None:
    cancel = _method_source("_cancel_navigation", "_pause_navigation")

    revoke = cancel.index('self.motion_owner = "NONE"')
    zero = cancel.index("self.navigation_velocity.publish(Twist())")
    wait = cancel.index("self._wait(handle.cancel_goal_async()")
    assert revoke < zero < wait


def test_persistent_clock_failure_revokes_navigation_but_has_a_grace_period() -> None:
    tick = _method_source("_localization_tick", "_load_map")
    degrade = _method_source("_degrade_localization", "_localization_lost")

    assert "self.sensor_time_invalid_grace" in tick
    assert 'self.localization_state = "SENSOR_TIME_INVALID"' in degrade
    assert "handle.cancel_goal_async()" in degrade
    assert 'self.motion_owner = "NONE"' in degrade
    assert "self.localization_velocity.publish(Twist())" in degrade


def test_sensor_recovery_replays_saved_pose_before_global_localization() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    recovery = tick.split(
        'if self.localization_state == "SENSOR_TIME_INVALID":', 1
    )[1].split("self._refresh_localization_confidence()", 1)[0]
    saved_pose = recovery.index("elif self.localization_seed_pose is not None")
    replay = recovery.index("self._begin_auto_localization(seed_pose)", saved_pose)
    global_search = recovery.index("self._start_global_localization()", replay)
    assert saved_pose < replay < global_search


def test_compute_path_lets_live_nav2_costmap_validate_the_start_pose() -> None:
    compute_path = _method_source("_compute_path", "_navigate")

    # Saved maps are quantized to 5 cm and can overlap the already occupied
    # robot pose after a small SLAM/localization shift. Goal safety remains an
    # adapter preflight, but the live Nav2 costmap must decide whether the
    # robot has a valid path out of its current footprint.
    assert "_validate_start_footprint" not in compute_path
    assert "resolved_goal, goal_adjusted = self._resolve_planning_goal" in compute_path
    assert "goal.use_start = False" in compute_path
    assert "self.compute_path_client.send_goal_async(goal)" in compute_path


def test_compute_path_rebuilds_current_footprint_before_planning() -> None:
    refresh = _method_source(
        "_refresh_global_costmap_for_planning",
        "_compute_path",
    )
    compute_path = _method_source("_compute_path", "_navigate")

    assert "ClearEntireCostmap.Request()" in refresh
    assert "self.clear_global_costmap_client.call_async" in refresh
    assert "time.sleep(0.45)" in refresh
    reset = compute_path.index("self._refresh_global_costmap_for_planning()")
    plan = compute_path.index("self.compute_path_client.send_goal_async(goal)")
    assert reset < plan
