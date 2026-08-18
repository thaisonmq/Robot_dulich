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
        'self.safety_health.startswith("BLOCKED")',
        "not self.estop_active",
        "self.safety_direction_mask & commanded_direction",
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
    assert "self.localization_rotation_blocked_timeout" in tick
    assert '"rotation_clearance_blocked"' in tick


def test_global_search_is_stationary_first_then_requires_authorized_rotation() -> None:
    global_search = _method_source("_start_global_localization", "_safe_to_rotate")
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")
    tick = _method_source("_localization_tick", "_load_map")

    guard = global_search.index("if not self.localization_rotation_authorized")
    stationary = global_search.index("self.global_search_requires_rotation = False", guard)
    assert guard < stationary
    assert "self.global_search_rotation_pending = True" in global_search
    assert "self.global_rotate_delay" in tick
    assert "if not self.localization_rotation_authorized" in tick
    assert "if self.global_search_rotation_pending" in tick
    assert "self.global_search_requires_rotation = True" in tick
    assert "len(self.localization_heading_bins)" in evidence
    assert "self.global_minimum_heading_bins" in evidence
    assert "self.localization_heading_span" in evidence
    assert "self.global_minimum_heading_span" in evidence
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


def test_stop_turn_arrival_is_position_only_and_reanchor_is_bounded() -> None:
    compute = _method_source("_compute_path", "_navigate")
    navigate = _method_source("_navigate", "_segment_execution_tick")
    begin_turn = _method_source("_begin_turn_or_settling", "_dispatch_prepared_segment")
    complete = _method_source("_complete_active_segment", "_finish_execution_success")

    assert "maximum_candidates=1" in compute
    assert "self._execution_destination(goal_payload)" in navigate
    assert "self.stop_turn_max_reanchors_per_segment" in begin_turn
    assert "self.stop_turn_require_final_yaw" in complete


def test_cancel_discards_transaction_local_route_and_destination_state() -> None:
    cancel = _method_source("_cancel_navigation", "_pause_navigation")

    for required in (
        "self.paused_goal = None",
        "self.execution_goal = None",
        'self.execution_route_id = ""',
        "self.route_candidates = {}",
        'self.selected_route_id = ""',
        'self.current_mission_id = ""',
        "self.latest_global_path = []",
    ):
        assert required in cancel


def test_ready_requires_scan_map_pose_window_and_synchronized_time() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    evidence = _method_source("_localization_evidence_ready", "_localization_tick")

    assert "self._pose_is_stable()" in evidence
    assert "self.amcl_pose_freshness" in evidence
    assert "self.global_scan_map_threshold" in evidence
    assert "self.scan_map_score >= required_scan_score" in evidence
    assert "self._critical_sensor_time_healthy()" in evidence
    assert "self.localization_final_max_median_residual" in evidence
    assert "self.localization_final_max_p90_residual" in evidence
    assert "self.localization_final_minimum_residual_beams" in evidence
    assert "self.global_minimum_heading_bins" in evidence
    assert 'self.localization_state == "VERIFYING"' in tick
    assert "self.localization_ready_hold" in tick
    assert 'self.localized and self.localization_state == "READY"' in tick
    assert "self.ready_evidence_invalid_since" in tick


def test_verify_timeout_does_not_restart_an_accepted_candidate_during_ready_hold() -> None:
    tick = _method_source("_localization_tick", "_load_map")
    verify_timeout = tick.split(
        'self.localization_state == "VERIFYING"', 1
    )[1].split("if now - self.localization_started_monotonic", 1)[0]

    assert "and not localization_ready" in verify_timeout
    assert "and self.ready_evidence_since is None" in verify_timeout
    assert "self._start_global_localization()" in verify_timeout


def test_stationary_global_search_rejects_weak_alias_and_resamples_when_turn_blocked() -> None:
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")
    tick = _method_source("_localization_tick", "_load_map")

    assert 'self.localization_state == "LOCALIZING_GLOBAL"' in evidence
    assert "self.global_search_rotation_pending" in evidence
    assert "self.global_scan_map_threshold" in evidence
    assert "self.stationary_global_candidate_ambiguous" in tick
    assert "self.stationary_global_retry_delay" in tick
    assert 'action="GLOBAL_RESAMPLE"' in tick
    assert "self._start_global_localization()" in tick


def test_map_initialization_cannot_timeout_before_session_clock_is_started() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    initializing_guard = tick.index(
        'if self.localization_state == "LOCALIZATION_INITIALIZING":'
    )
    timeout = tick.index("self.localization_started_monotonic >= self.localization_timeout")
    assert initializing_guard < timeout


def test_heading_observation_is_independent_from_candidate_quality() -> None:
    observation = _method_source(
        "_record_heading_observation", "_update_scan_map_match"
    )
    callback = _method_source("_scan_callback", "_sensor_time_callback")
    assert "valid_beams < self.scan_map_minimum_beams" in observation
    assert "self._scan_heading_in_odom(message)" in observation
    assert "match.score" not in observation
    assert "residual" not in observation
    assert callback.index("self._record_heading_observation(message)") < callback.index(
        "self._update_scan_map_match(message)"
    )


def test_scan_tf_uses_bounded_wait_and_age_checked_fallback() -> None:
    transform = _method_source("_scan_transform", "_tf_debug")
    assert "timeout=Duration(" in transform
    assert 'lookup_transform(\n                    target_frame, source_frame, Time()' in transform
    assert "self.scan_tf_fallback_max_age" in transform
    assert '"TF_AT_SCAN_MISS"' in transform
    assert '"TF_FALLBACK"' in transform
    assert '"SCAN_REJECTED_TF"' in transform


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
    assert "self.tracking_scan_map_sanity_threshold" in confidence
    assert "max(fresh_scan_map_score, self.scan_map_threshold)" not in confidence
    assert "self._nomotion_update_due(" in tick


def test_active_but_stationary_navigation_refreshes_amcl_before_pose_expires() -> None:
    refresh = _method_source("_nomotion_update_due", "_localization_tick")

    assert "self.amcl_pose_freshness * 0.5" in refresh
    assert "now - self.last_amcl_monotonic >= refresh_age" in refresh
    assert "self.nomotion_update_client.service_is_ready()" in refresh


def test_scan_callback_never_replaces_capture_stamp_with_now() -> None:
    scan = _method_source("_scan_callback", "_sensor_time_callback")
    transform = _method_source("_scan_transform", "_update_scan_map_match")
    normalizer = (
        Path(__file__).parents[1] / "navigation-stack" / "sensor_normalizer.py"
    ).read_text()

    assert "header.stamp" not in scan
    assert "Time.from_msg(message.header.stamp)" in transform
    assert 'target_frame' in transform
    assert "SensorClockEstimator" in normalizer
    assert "corrected_nanoseconds" in normalizer


def test_imu_clock_failure_cannot_invalidate_scan_and_odom_timing() -> None:
    normalizer = (
        Path(__file__).parents[1] / "navigation-stack" / "sensor_normalizer.py"
    ).read_text()

    assert "self.clocks =" in normalizer
    assert "navigation_clock = SensorClockEstimator" in normalizer
    assert '"scan": navigation_clock' in normalizer
    assert '"odom": navigation_clock' in normalizer
    assert '"imu": SensorClockEstimator' in normalizer
    critical = normalizer.split("critical_clock_state =", 1)[1].split(
        "status =", 1
    )[0]
    assert 'for name in ("scan", "odom")' in critical
    assert 'self.clocks["imu"]' not in critical
    assert '"imu_diagnostic_health": sensors["imu"]' in normalizer


def test_runtime_deskews_only_with_fresh_measured_odometry() -> None:
    normalizer = (
        Path(__file__).parents[1] / "navigation-stack" / "sensor_normalizer.py"
    ).read_text()
    assert "deskew_laser_scan_ranges" in normalizer
    assert "message.time_increment" in normalizer
    assert "message.scan_time" in normalizer
    assert "self.measured_linear_velocity" in normalizer
    assert "self.measured_angular_velocity" in normalizer
    assert "self.last_odometry_monotonic" in normalizer


def test_localization_rotation_progress_comes_from_actual_unwrapped_yaw() -> None:
    tick = _method_source("_localization_tick", "_load_map")
    assert "self.rotation_yaw_progress.update" in tick
    assert "self.rotation_angle += abs(self.rotation_speed)" not in tick


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
    assert "self._critical_sensor_time_status()" in tick
    assert "sensor_time_reason" in tick
    assert 'self.localization_state = "SENSOR_TIME_INVALID"' in degrade
    assert "handle.cancel_goal_async()" in degrade
    assert 'self.motion_owner = "NONE"' in degrade
    assert "self.navigation_velocity.publish(Twist())" in degrade
    assert "self.localization_velocity.publish(Twist())" in degrade
    assert "self.latest_global_path = []" not in degrade
    assert "self.sensor_time_resume_context" in degrade


def test_transient_sensor_fault_keeps_context_then_replans_same_destination() -> None:
    restore = _method_source(
        "_restore_after_sensor_time_pause", "_fail_sustained_sensor_time_pause"
    )
    resume = _method_source(
        "_resume_sensor_time_navigation_if_ready", "_localization_lost"
    )

    assert '"VERIFYING"' in restore
    assert "self._begin_localization_verification(allow_rotation=False)" in restore
    assert '"goal": dict(self.paused_goal)' in ADAPTER_SOURCE
    assert "self._plan_stop_turn_from_current(goal)" in resume
    assert "self._navigate(" in resume
    assert '"mission_id": mission_id' in resume
    assert '"route_id": route_id' in resume


def test_sensor_time_diagnostics_identify_the_failing_critical_stream() -> None:
    timing = _method_source(
        "_critical_sensor_time_status", "_critical_sensor_time_healthy"
    )

    for required in (
        '"status_age_ms"', '"timestamp_age_ms"', '"arrival_fresh"',
        '"timestamp_valid"', '"frame_valid"', '"clock_state"',
        '"invalid_streak"', '"rejected_packets"', '"last_rejection"',
        'f"{prefix}_ARRIVAL_STALE"', 'f"{prefix}_TIMESTAMP_INVALID"',
    ):
        assert required in timing


def test_compute_path_uses_cached_stop_turn_geometry_and_live_validation() -> None:
    compute_path = _method_source("_compute_path", "_navigate")
    serialize = _method_source(
        "_serialize_stop_turn_candidates", "_compute_alternative_routes"
    )

    assert "resolved_goal, goal_adjusted = self._resolve_planning_goal" in compute_path
    assert "self.stop_turn_planner.plan_candidates" in compute_path
    assert "self._serialize_stop_turn_candidates(planned)" in compute_path
    assert "self._route_metadata(points, original=points)" in serialize
    assert 'planner="StopTurnStateLattice24"' in compute_path
    assert "_request_path_once" not in compute_path


def test_raw_plan_never_replaces_validated_visualization_or_follow_path() -> None:
    callback = _method_source("_path_callback", "_path_length")
    compute = _method_source("_compute_path", "_navigate")
    serialize = _method_source(
        "_serialize_stop_turn_candidates", "_compute_alternative_routes"
    )
    navigate = _method_source("_navigate", "_navigation_feedback")

    assert "self.latest_planner_raw_path = path" in callback
    assert "self.latest_global_path = path" not in callback
    assert "self.stop_turn_planner.plan_candidates" in compute
    assert '"PRE_FOLLOW_PATH"' in navigate
    assert "metadata = self._route_metadata" in serialize
    assert "points = self._ensure_executable_path" in navigate


def test_executable_validator_combines_live_master_and_saved_static_walls() -> None:
    validate = _method_source(
        "_validate_executable_path", "_path_after_initial_distance"
    )

    assert 'validation_source = "GLOBAL_MASTER_COSTMAP"' in validate
    assert "self.saved_map.occupancy" in validate
    assert "lethal_threshold=100" in validate
    assert "inscribed_threshold=99" in validate
    assert "lethal_threshold=65" in validate
    assert 'validation_source = "SAVED_STATIC_MAP"' in validate
    assert "self._path_after_initial_distance" in validate


def test_compute_path_never_fabricates_or_mutates_a_failed_candidate() -> None:
    compute_path = _method_source("_compute_path", "_navigate")

    assert '"GOAL_PHYSICALLY_UNREACHABLE"' in compute_path
    assert "self.failed_segments =" not in compute_path
    assert "_request_path_once" not in compute_path
    assert "_refresh_global_costmap_for_planning" not in compute_path
    assert 'self._set_state("BLOCKED"' not in compute_path


def test_invalid_preview_never_falls_back_to_a_curved_planner() -> None:
    ensure = _method_source("_ensure_executable_path", "_global_cost_at")

    assert "validate_stop_turn_route" in ensure
    assert "canonicalize_stop_turn_path" in ensure
    assert "_request_path_once" not in ensure
    assert "FOOTPRINT_AWARE_PLANNER" not in ensure


def test_scan_filter_is_used_only_for_global_planning_topic() -> None:
    scan = _method_source("_planning_scan_message", "_scan_callback")
    callback = _method_source("_scan_callback", "_sensor_time_callback")

    assert "filter_static_map_scan" in scan
    assert 'self._scan_transform("map", message)' in scan
    assert "self.last_amcl_pose" not in scan
    assert "self.planning_static_match_tolerance" in scan
    assert "self.localization_coarse_match_tolerance" not in scan
    assert "self.navigation_scan.publish(message)" in callback
    assert "self.planning_scan.publish(self._planning_scan_message(message))" in callback


def test_navigation_abort_uses_confirmed_corridor_then_preserves_user_choice() -> None:
    result = _method_source("_navigation_result", "_set_recovery_terminal")
    recover = _method_source("_recover_navigation", "_cancel_navigation")
    evidence = _method_source("_corridor_failure_evidence", "_mark_failed_segment")

    assert "self._corridor_failure_evidence()" in result
    assert '"CORRIDOR_CLEAR", "NARROW_OR_UNCERTAIN"' in result
    assert '"PHYSICALLY_BLOCKED"' in result
    assert 'self._enter_dynamic_wait("CONFIRMED_DYNAMIC_ROUTE_BLOCK")' in result
    assert '"CONFIRMED_STATIC_PHYSICAL_BLOCKAGE", goal_generation' in result
    assert 'self._set_state("NAVIGATING", "automatic_segment_retry")' in result
    assert "self.corridor_confirmation_samples" in evidence
    assert "self.corridor_confirmation_duration" in evidence
    assert "localization_reliable" in result
    assert '"LOCALIZATION_UNRELIABLE"' in result


def test_manual_handoff_preserves_goal_and_resume_replans_from_current_pose() -> None:
    handoff = _method_source("_manual_handoff", "_resume_auto_from_current_pose")
    resume = _method_source("_resume_auto_from_current_pose", "_start_selected_route")

    assert 'self._set_state("MANUAL_BYPASS"' in handoff
    assert "self.paused_goal = None" not in handoff
    assert '"destination_preserved": True' in handoff
    assert 'self.localization_state != "READY"' in resume
    assert "points = self._plan_stop_turn_from_current(goal)" in resume
    assert 'self._set_state("PLANNING", "resume_from_current_pose")' in resume


def test_selected_preview_is_executed_as_isolated_straight_segments() -> None:
    selected = _method_source("_start_selected_route", "_mapping_command")
    navigate = _method_source("_navigate", "_navigation_feedback")

    assert 'points = list(candidate["points"])' in selected
    assert '"points": points' in selected
    assert "self._request_path_once" not in selected
    assert "canonicalize_stop_turn_path" in navigate
    assert 'executor="StopTurnSegmentExecutor"' in navigate
    sender = _method_source("_send_current_straight_segment", "_navigation_feedback")
    assert "densify_straight_segment" in sender
    assert "for point in segment_points" in sender
    assert 'action_goal.goal_checker_id = "segment_goal_checker"' in sender
    assert "self.follow_path_client.send_goal_async" in sender
    assert 'phase="TURN_BEGIN"' in navigate
    assert "command.linear.x = 0.0" in navigate
    assert "NavigateToPose" not in navigate
    assert "actual_execution_path_route_id=route_id" in navigate


def test_active_segment_is_the_execution_geometry_authority() -> None:
    heading = _method_source("_current_path_heading", "_speed_profile_state")
    prepare = _method_source("_prepare_active_segment", "_begin_turn_or_settling")
    sender = _method_source(
        "_send_current_straight_segment", "_segment_callback_current"
    )

    assert "self.active_segment.fixed_heading" in heading
    assert 'self.execution_phase in {"TURN", "TURN_SETTLING"}' in heading
    assert "ActiveSegment.create" in prepare
    assert 'context="STRAIGHT_REANCHOR"' in prepare
    assert "validate_stop_turn_route" in prepare
    assert '"REANCHOR_TRANSLATION_INVALID"' in prepare
    assert "self._schedule_execution_replan" in prepare
    assert "active.effective_start" in sender
    assert "active.endpoint" in sender
    assert "active.fixed_heading" in sender


def test_straight_controller_and_endpoint_guards_are_independent_of_rpp_curvature() -> None:
    velocity = _method_source("_auto_velocity_callback", "_update_motion_metrics")
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")

    assert "straight_heading_lock" in velocity
    assert "endpoint_braking_speed_limit" in velocity
    assert "0.0 if straight_phase else message.angular.z" in velocity
    assert 'linear = 0.0' in velocity
    assert '"GEOMETRIC_ENDPOINT_STOP"' in velocity
    assert "straight_segment_progress" in tick
    assert "progress.passed_endpoint" in tick
    assert "segment_travel_watchdog" in tick
    assert '"ENDPOINT_OVERSHOOT_CROSS_TRACK"' in tick


def test_segment_result_callbacks_require_generation_token_and_index() -> None:
    sender = _method_source(
        "_send_current_straight_segment", "_segment_callback_current"
    )
    guard = _method_source("_segment_callback_current", "_navigation_feedback")
    result = _method_source("_navigation_result", "_set_recovery_terminal")

    assert "token=segment_token" in sender
    assert "index=segment_index" in sender
    assert "active.segment_token == segment_token" in guard
    assert "active.segment_index == segment_index" in guard
    assert "self._segment_callback_current" in result
    assert 'evidence_reason == "UNCONFIRMED"' in result
    assert 'self._set_state("NAVIGATING", "automatic_segment_retry")' in result


def test_turn_hysteresis_and_persistent_safety_block_are_bounded() -> None:
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")

    assert "turn_hysteresis_transition" in tick
    assert '"TURN_SETTLING"' in tick
    assert "self.execution_turn_reentry_tolerance" in tick
    assert "self.execution_turn_stable_dwell" in tick
    assert "self.execution_turn_safety_block_timeout" in tick
    assert '"PERSISTENT_SAFETY_BLOCK"' in tick
    assert "self._schedule_execution_replan" in tick
    assert '"TURN_CMD"' in tick


def test_navigation_debug_events_cover_new_geometry_and_recovery_sources() -> None:
    for event in (
        '"LOCALIZATION_VERIFY"',
        '"CORRIDOR"',
        '"STOP"',
        '"FAILED_SEGMENT"',
        '"REPLAN"',
    ):
        assert event in ADAPTER_SOURCE
    assert '"/safety/stop_source"' in ADAPTER_SOURCE
    assert '"/navigation/failed_segment_mask"' in ADAPTER_SOURCE


def test_failed_segment_ttl_clears_only_the_dedicated_mask() -> None:
    tick = _method_source("_failed_segment_tick", "_corridor_failure_evidence")
    publish = _method_source("_publish_failed_segments", "_failed_segment_tick")

    assert "ClearEntireCostmap" not in tick
    assert "ClearEntireCostmap" not in publish
    assert "self._publish_failed_segments()" in tick
    assert "message.data = [0] * (width * height)" in publish
    assert '"/map", self._map_callback, transient_map_qos' in ADAPTER_SOURCE


def test_initial_preview_produces_candidates_without_persistent_scratch_state() -> None:
    compute = _method_source("_compute_path", "_navigate")
    alternatives = _method_source(
        "_compute_alternative_routes", "_wait_for_global_costmap_after"
    )
    assert "plan_candidates" in compute
    assert '"route_candidates": candidates' in compute
    assert "self.route_candidates =" in compute
    assert "self.failed_segments =" not in alternatives
    assert "_publish_failed_segments" not in alternatives
    assert 'source="EXPLICIT_ALTERNATIVE_SEARCH"' in alternatives
    assert "self.stop_turn_planner.plan_candidates" in alternatives


def test_live_narrow_uncertainty_does_not_cancel_prevalidated_auto_route() -> None:
    callback = _method_source("_scan_callback", "_sensor_time_callback")
    decision = callback.split("self._corridor_failure_evidence()", 1)[1]
    assert 'if evidence_reason == "PHYSICALLY_BLOCKED"' in decision
    assert 'evidence_reason in {"NARROW_OR_UNCERTAIN"' not in decision
