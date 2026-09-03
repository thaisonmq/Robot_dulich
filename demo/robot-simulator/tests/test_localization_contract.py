import ast
from pathlib import Path

import yaml


ADAPTER_SOURCE = (
    Path(__file__).parents[1] / "navigation-stack" / "adapter_node.py"
).read_text()
NAVIGATION_CORE_SOURCE = (
    Path(__file__).parents[1] / "navigation-stack" / "navigation_core.py"
).read_text()
EDGE_CLIENT_SOURCE = (
    Path(__file__).parents[1] / "simulator" / "client.py"
).read_text()
MAP_CACHE_SOURCE = (
    Path(__file__).parents[1] / "simulator" / "map_cache.py"
).read_text()
LAUNCH_SOURCE = (
    Path(__file__).parents[1]
    / "navigation-stack"
    / "launch"
    / "navigation_stack.launch.py"
).read_text()


def _method_source(name: str, next_name: str) -> str:
    start = ADAPTER_SOURCE.index(f"    def {name}(")
    end = ADAPTER_SOURCE.index(f"    def {next_name}(", start)
    return ADAPTER_SOURCE[start:end]


def _navigate_calls(source: str) -> list[ast.Call]:
    tree = ast.parse(source)
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_navigate"
    ]


def _keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def test_stale_safety_subscription_accepts_a_restarted_sequence_epoch() -> None:
    watchdog = _method_source(
        "_safety_subscription_watchdog_tick", "_safety_status_callback"
    )

    reset = watchdog.index("self.safety_snapshot_sequence = -1")
    recreate = watchdog.index("self.create_subscription(", reset)
    assert "self.destroy_subscription(old_subscription)" in watchdog
    assert reset < recreate
    assert 'String, "/safety/status"' in watchdog
    assert '"SAFETY_SUBSCRIPTION_REBIND"' in watchdog


def test_only_trusted_pose_sources_get_fast_local_verification() -> None:
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    operator = _method_source("_set_initial_pose", "_deactivate_map")
    odometry = _method_source("_begin_odometry_prior", "_begin_operator_pose_hint")
    fallback = _method_source("_begin_operator_pose_hint", "_set_initial_pose")

    assert 'last_pose.get("source", "")' in automatic
    assert '"recent_navigation_pose"' in automatic
    assert "self.localization_seed_approximate = not recent_verified_pose" in automatic
    assert "self.global_search_requires_rotation = False" in automatic
    assert "self._odometry_predicted_map_pose()" not in operator
    assert "self._begin_odometry_prior(" not in operator
    assert "self._begin_operator_pose_hint(" in operator
    assert "approximate=False" in odometry
    assert "approximate=True" in fallback


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


def test_trusted_continuity_pose_tolerates_live_occlusion_after_restart() -> None:
    verdict = _method_source(
        "_localization_verdict", "_localization_quality_ready"
    )

    assert "trusted_continuity_candidate = bool(" in verdict
    assert 'in {"LOCALIZING_LAST_POSE", "VERIFYING"}' in verdict
    assert "self.localization_odometry_prior_active" in verdict
    assert "not self.global_search_untrusted" in verdict
    assert "not self.localization_seed_approximate" in verdict
    assert "self.localization_tracking_maximum_contradiction_ratio" in verdict
    assert verdict.count("maximum_raycast_contradiction_ratio=(") == 2


def test_continue_mapping_gates_scans_until_slam_confirms_the_pose_hint() -> None:
    scan = _method_source("_scan_callback", "_sensor_time_callback")
    pose = _method_source("_mapping_pose_callback", "_amcl_pose_callback")
    verify = _method_source(
        "_verify_mapping_continuation_pose", "_load_mapping_posegraph"
    )

    assert 'self.current_state == "MAPPING_LOCALIZING"' in scan
    assert "self.mapping_relocalization_max_probes" in scan
    assert "mapping_pose_match_quality(" in pose
    assert "self.mapping_relocalization_event.set()" in pose
    assert "self.mapping_relocalization_event.wait(" in verify
    assert "self._load_mapping_posegraph(filename, seed)" in verify
    assert 'self.current_state = "MAPPING_ERROR"' in verify


def test_continue_mapping_searches_full_heading_and_confirms_scan_geometry() -> None:
    verify = _method_source(
        "_verify_mapping_continuation_pose", "_load_mapping_posegraph"
    )
    callback = _method_source(
        "_mapping_pose_callback", "_amcl_pose_callback"
    )

    assert "global_scan_candidate_uniqueness(" in verify
    assert "require_candidate_match=False" in verify
    assert "alternative_yaw_separation=math.radians(45.0)" in verify
    assert 'source_yaml = Path(filename).parent / "map.yaml"' in verify
    assert "self._load_mapping_posegraph(filename, seed)" in verify
    assert "scan_to_map_match(" in callback
    assert "self.mapping_relocalization_required_confirmations" in callback
    assert '"SCAN_MAP_GEOMETRY_UNSTABLE"' in callback


def test_continue_mapping_checks_deserialize_result_and_never_trusts_hint_directly() -> None:
    load = _method_source("_load_mapping_posegraph", "_slam_process_ids")
    command = _method_source("_mapping_command", "_verify_mapping_continuation_pose")

    assert "START_AT_GIVEN_POSE" in load
    assert 'hasattr(response, "result")' in load
    assert "not bool(response.result)" in load
    assert "_verify_mapping_continuation_pose(" in command
    assert 'self.current_state = "MAPPING_RUNNING"' in command


def test_continued_mapping_clears_only_repeatedly_disproven_old_obstacles() -> None:
    record = _method_source(
        "_record_mapping_change_evidence", "_apply_mapping_change_evidence"
    )
    apply = _method_source(
        "_apply_mapping_change_evidence", "_amcl_pose_callback"
    )
    save = _method_source("_save_mapping_bundle", "_schedule_autosave")

    assert 'self._scan_transform("map", message)' in record
    assert "source_map.value_at(*cell) >= 65" in record
    assert "free_cells: set[tuple[int, int]]" in record
    assert "hit_cells: set[tuple[int, int]]" in record
    assert "self.mapping_change_minimum_free_observations" in apply
    assert "free_count < (3 * hit_count + minimum)" in apply
    assert "pixels[column, image_row] = 254" in apply
    assert "self._apply_mapping_change_evidence(" in save
    assert '"MAPPING_STALE_CELLS_CLEARED"' in save


def test_each_localization_phase_discards_old_evidence_and_uses_full_threshold() -> None:
    global_localization = _method_source("_start_global_localization", "_safe_to_rotate")
    tick = _method_source("_localization_tick", "_load_map")
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")

    assert "self._reset_localization_evidence()" in global_localization
    assert "self.localization_confidence_threshold" in verdict
    assert "self._required_localization_scan_score()" in evidence
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
    assert "rotation_was_authorized = self.localization_rotation_authorized" in operator
    assert "self.localization_rotation_authorized = rotation_was_authorized" in operator
    assert "self.localization_rotation_authorized = False" not in operator
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
    assert 'self.localization_state = "AMBIGUOUS"' in tick
    assert "self.approximate_hint_allowed = True" in tick
    assert "self.localization_rotation_blocked_timeout" in tick
    assert '"rotation_clearance_blocked"' in tick


def test_global_search_is_passive_first_and_only_velocity_requires_authorization() -> None:
    global_search = _method_source("_start_global_localization", "_safe_to_rotate")
    evidence = _method_source("_localization_evidence_ready", "_localization_tick")
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")
    tick = _method_source("_localization_tick", "_load_map")

    assert "if not self.localization_rotation_authorized" not in global_search
    assert 'self.localization_state = "PASSIVE_LOCALIZING"' in global_search
    assert "self.global_search_rotation_pending = True" in global_search
    assert "self.global_rotate_delay" in tick
    assert "if not self.localization_rotation_authorized" in tick
    assert "if self.global_search_rotation_pending" in tick
    assert "self.global_search_requires_rotation = not strong_candidate" in tick
    assert "self.global_search_untrusted" in verdict
    assert "require_heading=False" in evidence
    assert "self.particle_uniqueness.accepted" in verdict
    assert "self._start_next_localization_rotation(now)" in tick
    assert 'mode="PASSIVE_GLOBAL"' in global_search


def test_operator_hint_resolves_without_unsafe_rotation_only_with_strict_local_evidence() -> None:
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")
    operator = _method_source("_begin_operator_pose_hint", "_set_initial_pose")
    required_score = _method_source(
        "_required_localization_scan_score", "_localization_verdict"
    )

    operator_branch = verdict.split(
        "if self.localization_operator_hint_active:", 1
    )[1].split("self._request_global_scan_uniqueness()", 1)[0]
    assert "self.localization_operator_hint_active = True" in operator
    assert "self.localization_seed_pose" in operator_branch
    assert "self.operator_hint_search_radius" in operator_branch
    assert "self.localization_pending_operator_hint" in operator_branch
    assert "self._global_heading_diversity_ready()" not in operator_branch
    assert '"INSUFFICIENT_HEADING_DIVERSITY"' not in operator_branch
    assert "return verdict" in operator_branch
    assert "hinted_candidate" in verdict
    assert "self.localization_operator_hint_minimum_raycast_beams" in verdict
    assert "self.localization_operator_hint_minimum_static_matches" in verdict
    assert "self.localization_operator_hint_minimum_explained_ratio" in verdict
    assert "consensus.raycast_dynamic_occlusion_ratio" in verdict
    assert '"OPERATOR_HINT_EXPLAINED_RATIO_TOO_LOW"' in verdict
    assert "self.localization_operator_hint_minimum_scan_score" in required_score
    assert "self.particle_uniqueness.accepted" in verdict.split(
        "if self.localization_operator_hint_active:", 1
    )[0]


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
    assert "self._odometry_predicted_map_pose()" in force_branch
    assert "self._begin_odometry_prior(" in force_branch
    assert "self._start_global_localization()" in force_branch


def test_repeated_force_global_does_not_reset_an_active_scan() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    force_branch = dispatch.split("if force_global:", 1)[1].split(
        'elif self.localized and self.localization_state == "READY":', 1
    )[0]

    active_guard = force_branch.index('"PASSIVE_LOCALIZING", "CANDIDATE", "VERIFYING"')
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
    observation_rotation = tick.split(
        "if self.rotation_angle >= self.localization_next_observation_angle:", 1
    )[1].split("if not self._safe_to_rotate():", 1)[0]
    assert "self._begin_localization_settling(now)" in observation_rotation
    assert '"LOCALIZATION_FAILED"' not in observation_rotation
    assert "self.localization_rotation_settle" in tick


def test_map_load_only_injects_a_recent_verified_navigation_pose() -> None:
    load_branch = EDGE_CLIENT_SOURCE.split(
        'if command == "map.load" and self.config.navigation_backend == "ros2":', 1
    )[1].split('if (\n            command == "mapping.start"', 1)[0]

    assert 'command_payload["map_path"]' in load_branch
    assert 'command_payload["last_known_pose"] = recent_pose' in load_branch
    assert 'recent_pose["source"] = "recent_navigation_pose"' in load_branch
    assert "max_age_seconds=3600" in load_branch
    assert 'recent_pose.get("verification_version", 0)' in load_branch

    restore_payload = MAP_CACHE_SOURCE.split(
        "    def active_load_payload(", 1
    )[1].split("    def delete_local(", 1)[0]
    assert 'payload["last_known_pose"] = recent_pose' in restore_payload
    assert 'recent_pose["source"] = "recent_navigation_pose"' in restore_payload
    assert 'recent_pose.get("verification_version", 0)' in restore_payload


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


def test_success_discards_transaction_local_route_and_destination_state() -> None:
    finish = _method_source("_finish_execution_success", "_restart_segment_from_current")

    assert finish.index(
        "completed_route_id = self.execution_route_id"
    ) < finish.index('self.execution_route_id = ""')
    assert "completed_goal" in finish
    assert "route_id=completed_route_id" in finish

    for required in (
        "self.paused_goal = None",
        "self.execution_goal = None",
        'self.execution_route_id = ""',
        "self.execution_points = []",
        "self.execution_segment_directions = []",
        "self.route_candidates = {}",
        'self.selected_route_id = ""',
        'self.current_mission_id = ""',
        "self.latest_global_path = []",
    ):
        assert required in finish

    # Completing a route is not a localization boundary. Keep the verified
    # map/odometry anchor so the next destination can start immediately.
    for forbidden in (
        "self.localized = False",
        "self.localization_state =",
        "self.trajectory_map_from_odom = None",
        "self._reset_localization_evidence()",
    ):
        assert forbidden not in finish


def test_ready_requires_scan_map_pose_window_and_synchronized_time() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    evidence = _method_source("_localization_evidence_ready", "_localization_tick")
    quality = _method_source("_localization_quality_ready", "_actual_odom_yaw")
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")

    assert "self._pose_is_stable()" in verdict
    assert "self.amcl_pose_freshness" in verdict
    assert "self._required_localization_scan_score()" in evidence
    assert "required_scan_score=required_scan_score" in quality
    assert "self._critical_sensor_time_healthy()" in verdict
    assert "self.localization_final_max_median_residual" in verdict
    assert "self.localization_final_max_p90_residual" in verdict
    assert "self.localization_final_minimum_residual_beams" in verdict
    assert "self.localization_raycast_minimum_beams" in verdict
    assert "self.localization_raycast_minimum_static_matches" in verdict
    assert "self.localization_raycast_maximum_contradiction_ratio" in verdict
    assert "self._global_heading_diversity_ready()" in verdict
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


def test_stationary_global_search_rejects_weak_alias_without_reset_loop() -> None:
    threshold = _method_source(
        "_required_localization_scan_score", "_localization_verdict"
    )
    tick = _method_source("_localization_tick", "_load_map")

    assert '"PASSIVE_LOCALIZING", "LOCALIZING_GLOBAL"' in threshold
    assert "self.global_search_rotation_pending" in threshold
    assert "self.global_scan_map_threshold" in threshold
    assert "self.global_final_scan_map_threshold" in threshold
    assert "self.stationary_global_candidate_ambiguous" in tick
    assert 'self.localization_state = "AMBIGUOUS"' in tick
    assert "self.passive_global_retry_delay" not in tick
    assert "self.last_passive_global_retry_monotonic" not in tick
    hint_timeout = tick.split("if broad_seed:", 1)[1].split(
        "try:\n                self._start_global_localization()", 1
    )[0]
    assert 'self.localization_state = "AMBIGUOUS"' in hint_timeout
    assert "evidence_preserved=True" in hint_timeout
    assert "self._reset_localization_evidence()" not in hint_timeout


def test_map_initialization_cannot_timeout_before_session_clock_is_started() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    initializing_guard = tick.index(
        'if self.localization_state == "LOCALIZATION_INITIALIZING":'
    )
    timeout = tick.index("self.localization_started_monotonic >= self.localization_timeout")
    assert initializing_guard < timeout


def test_heading_observation_uses_basic_quality_not_final_raycast_gate() -> None:
    observation = _method_source(
        "_record_heading_observation", "_update_scan_map_match"
    )
    callback = _method_source("_scan_callback", "_sensor_time_callback")
    update = _method_source("_update_scan_map_match", "_planning_scan_message")
    assert "valid_beams < self.scan_map_minimum_beams" in observation
    assert "self._scan_heading_in_odom(message)" in observation
    assert "heading_observation_valid" in observation
    assert "self._pose_is_stable()" in observation
    assert "not self.rotation_active" in observation
    assert "self.localization_confidence_threshold" not in observation
    assert "self._required_localization_scan_score()" in update
    assert "self.localization_raycast_minimum_reliable_structure_span" in update
    heading_gate = update.split("return bool(", 1)[1]
    assert "match.score" in heading_gate
    assert "match.median_residual" in heading_gate
    assert "match.p90_residual" in heading_gate
    assert "raycast." not in heading_gate
    assert callback.index("self._update_scan_map_match(message)") < callback.index(
        "self._record_heading_observation("
    )


def test_localization_callbacks_are_serialized_and_pose_unpack_uses_snapshot() -> None:
    observation = _method_source(
        "_record_heading_observation", "_update_scan_map_match"
    )

    assert "self.localization_lock = threading.RLock()" in ADAPTER_SOURCE
    for callback in (
        "_amcl_pose_callback",
        "_particle_cloud_callback",
        "_scan_callback",
        "_localization_tick",
    ):
        assert f"@localization_callback\n    def {callback}(" in ADAPTER_SOURCE
    assert "pose_snapshot = self.last_amcl_pose" in observation
    assert "candidate_x, candidate_y, _ = pose_snapshot" in observation
    assert "candidate_x, candidate_y, _ = self.last_amcl_pose" not in observation


def test_adapter_process_is_respawned_if_an_unhandled_failure_escapes() -> None:
    assert LAUNCH_SOURCE.count('executable="adapter_node"') == 2
    assert LAUNCH_SOURCE.count("respawn=True") >= 3  # two adapters plus SLAM


def test_localization_verify_log_contains_every_mandatory_gate_metric() -> None:
    tick = _method_source("_localization_tick", "_load_map")
    for field in (
        "state=self.localization_state",
        "candidate_pose=self.last_amcl_pose",
        "pose_stability={",
        "confidence=self.localization_confidence",
        "scan_score=self.scan_map_score",
        "scan_score_required=self._required_localization_scan_score()",
        "covariance_xy=",
        "covariance_yaw=",
        "median_residual_m=",
        "p90_residual_m=",
        "raycast_comparable_beams=",
        "raycast_static_matches=",
        "raycast_dynamic_occlusions=",
        "raycast_map_contradictions=",
        "raycast_inconclusive_map_hits=",
        "raycast_static_match_ratio=",
        "raycast_dynamic_occlusion_ratio=",
        "raycast_contradiction_ratio=",
        "raycast_matches=",
        "raycast_match_ratio=",
        "raycast_median_error_m=",
        "raycast_p90_error_m=",
        "heading_bins=",
        "heading_bin_ids=",
        "heading_span_deg=",
        "rotation_degrees=",
        "global_search_untrusted=",
        "accepted=",
        "reason=",
    ):
        assert field in tick


def test_new_navigation_start_rechecks_fresh_raycast_without_canceling_route() -> None:
    start_gate = _method_source(
        "_localization_start_evidence_ready", "_begin_localization_settling"
    )
    wait_gate = _method_source(
        "_wait_for_localization_start_evidence", "_begin_localization_settling"
    )
    navigate = _method_source("_navigate", "_segment_execution_tick")

    assert "self._localization_tracking_evidence_ready(now)" in start_gate
    assert "self.tracking_scan_map_sanity_threshold" in start_gate
    assert "self.localization_final_minimum_residual_beams" in start_gate
    assert "self.localization_final_max_median_residual" in start_gate
    assert "self.localization_final_max_p90_residual" in start_gate
    assert "self.localization_raycast_minimum_beams" not in start_gate
    assert "self.localization_raycast_minimum_static_matches" not in start_gate
    assert "self.localization_tracking_maximum_contradiction_ratio" in start_gate
    assert "self.localization_raycast_maximum_contradiction_ratio" not in start_gate
    assert "self._localization_start_evidence_ready(now)" in wait_gate
    assert "self.localization_start_evidence_wait" in wait_gate
    assert "time.sleep(min(0.025, remaining))" in wait_gate
    assert "not recovery_attempt" in navigate
    assert "self._wait_for_localization_start_evidence()" in navigate
    assert '"NAVIGATION_START_REJECTED"' in navigate
    assert "cancel_goal_async" not in start_gate


def test_failed_global_checkpoint_continues_rotation_without_counting_heading() -> None:
    checkpoint = _method_source(
        "_localization_checkpoint_observed", "_localization_tick"
    )
    tick = _method_source("_localization_tick", "_load_map")
    settling = tick.split(
        "and self._localization_checkpoint_observed(now)", 1
    )[1].split(
        'self.localization_state == "VERIFYING"', 1
    )[0]

    assert "self._pose_is_stable()" in checkpoint
    assert "self.last_scan_map_monotonic" in checkpoint
    assert "self.scan_map_valid_beams >= self.scan_map_minimum_beams" in checkpoint
    assert "self._localization_checkpoint_observed(now)" in tick
    assert "and not localization_ready" in settling
    assert "self._start_next_localization_rotation(now)" in settling
    assert "self._localization_quality_ready(" not in settling
    assert "self._record_heading_observation" not in settling


def test_scan_tf_uses_bounded_wait_and_age_checked_fallback() -> None:
    transform = _method_source("_scan_transform", "_tf_debug")
    assert "timeout=Duration(" in transform
    assert 'lookup_transform(\n                    target_frame, source_frame, Time()' in transform
    assert "self.scan_tf_fallback_max_age" in transform
    assert '"TF_AT_SCAN_MISS"' in transform
    assert '"TF_FALLBACK"' in transform
    assert '"SCAN_REJECTED_TF"' in transform


def test_untrusted_global_requires_unique_particle_region_not_heading_count() -> None:
    evidence = _method_source(
        "_localization_evidence_ready", "_localization_rejection_reason"
    )
    global_search = _method_source(
        "_start_global_localization", "_safe_to_rotate"
    )

    assert "self.global_search_untrusted = True" in global_search
    assert "require_heading=False" in evidence
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")
    assert "self.particle_uniqueness.accepted" in verdict
    assert '"PARTICLE_CLOUD_STALE"' in verdict


def test_adaptive_global_heading_requirements_allow_strong_early_exit() -> None:
    requirement = _method_source(
        "_global_heading_requirement", "_global_heading_diversity_ready"
    )
    rotation = _method_source(
        "_start_next_localization_rotation", "_localization_evidence_ready"
    )
    tick = _method_source("_localization_tick", "_resolve_runtime_map_yaml")

    assert "self.global_strong_minimum_heading_bins" in requirement
    assert "self.global_strong_minimum_heading_span" in requirement
    assert "self.global_minimum_heading_bins" in requirement
    assert "self.global_minimum_heading_span" in requirement
    assert "math.radians(15.0)" in rotation
    assert "math.radians(30.0)" in rotation
    assert "self.localization_next_observation_angle" in rotation
    assert "localization_ready = self._localization_evidence_ready(now)" in tick


def test_hypothesis_jump_invalidates_old_heading_corroboration() -> None:
    callback = _method_source("_amcl_pose_callback", "_pose_is_stable")

    assert "self.global_search_untrusted" in callback
    assert "self.localization_evidence_headings.clear()" in callback
    assert "self.localization_heading_positions.clear()" in callback
    assert "self.localization_heading_bins = ()" in callback
    assert 'reason="SPATIAL_HYPOTHESIS_JUMP"' in callback


def test_cross_heading_position_inconsistency_resets_corroboration() -> None:
    observation = _method_source(
        "_record_heading_observation", "_update_scan_map_match"
    )

    assert "heading_position_spread" in observation
    assert "self.pose_maximum_xy_spread * 2.0" in observation
    assert "self.localization_heading_positions.clear()" in observation
    assert 'reason="SPATIAL_HYPOTHESIS_INCONSISTENT_ACROSS_HEADINGS"' in observation


def test_tf_chain_diagnostic_names_each_link_and_bounded_failure() -> None:
    diagnostic = _method_source(
        "_tf_link_diagnostic", "_scan_heading_in_odom"
    )

    for link in (
        '"map_to_odom"',
        '"odom_to_base"',
        '"base_to_laser"',
        '"map_to_laser"',
    ):
        assert link in diagnostic
    assert '"amcl_pose_age_ms"' in diagnostic
    assert '"scan_navigation_age_ms"' in diagnostic
    assert '"global_localization_service_ready"' in diagnostic
    assert '"nomotion_update_service_ready"' in diagnostic
    assert "self.localization_tf_chain_timeout" in diagnostic
    assert 'reason="LOCALIZATION_TF_CHAIN_UNAVAILABLE"' in diagnostic


def test_command_rejection_is_logged_before_expected_state_guard() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")

    command_log = dispatch.index('"COMMAND"')
    expected_state_guard = dispatch.index("expected_state != self.current_state")
    rejected_log = dispatch.index('"COMMAND_REJECTED"', expected_state_guard)
    state_conflict = dispatch.index('"STATE_CONFLICT"', rejected_log)
    assert command_log < expected_state_guard < rejected_log < state_conflict


def test_runtime_map_loader_and_active_log_use_canonical_runtime_bundle() -> None:
    resolve = _method_source("_resolve_runtime_map_yaml", "_log_active_map")
    active_log = _method_source("_log_active_map", "_load_map")
    load = _method_source("_load_map", "_set_initial_pose")

    assert "self.map_root.resolve(strict=True)" in resolve
    assert 'payload["map_path"]' in resolve
    assert "relative_to(runtime_root)" in resolve
    assert "sample-data" not in resolve + load
    assert '"MAP_ACTIVE"' in active_log
    for field in (
        "canonical_map_yaml_path",
        "image_path",
        "resolution",
        "width",
        "height",
        "origin",
    ):
        assert field in active_log


def test_localization_rotation_has_a_dedicated_diagnostic_event() -> None:
    tick = _method_source("_localization_tick", "_resolve_runtime_map_yaml")

    assert '"LOCALIZATION_ROTATE"' in tick
    assert "current_actual_yaw" in tick
    assert "accumulated_yaw_span" in tick
    assert "requested_angular" in tick
    assert "final_safety_output" in tick


def test_post_turn_reanchor_enters_straight_inside_bounded_band() -> None:
    prepare = _method_source(
        "_prepare_active_segment", "_begin_turn_or_settling"
    )

    assert "post_turn_reanchor_requires_turn" in prepare
    assert "self.straight_hard_heading_error" in prepare
    assert "self.straight_hard_cross_track" in prepare
    assert 'phase="STRAIGHT_ENTRY_CORRECTION"' in prepare
    correction = prepare.split('phase="STRAIGHT_ENTRY_CORRECTION"', 1)[1]
    assert "self._dispatch_prepared_segment(goal_generation)" in correction


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
    degrade = _method_source(
        "_degrade_localization", "_restore_after_sensor_time_pause"
    )
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
    for active_recovery_state in (
        '"WAIT_FOR_DYNAMIC_CLEAR"',
        '"WAITING_FOR_DYNAMIC_CLEAR"',
        '"DYNAMIC_REPLAN"',
    ):
        assert active_recovery_state in degrade


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


def test_sensor_and_safety_heartbeats_have_a_dedicated_callback_group() -> None:
    init = _method_source("__init__", "_nav_debug")

    assert "self.critical_status_callback_group = MutuallyExclusiveCallbackGroup()" in init
    assert (
        'String, "/sensors/time_status", self._sensor_time_callback, 1,\n'
        "            callback_group=self.critical_status_callback_group"
    ) in init
    assert (
        'String, "/safety/status", self._safety_status_callback, 1,\n'
        "            callback_group=self.critical_status_callback_group"
    ) in init
    assert (
        "self._safety_subscription_watchdog_tick,\n"
        "            callback_group=self.critical_status_callback_group"
    ) in init


def test_compute_path_uses_cached_stop_turn_geometry_and_live_validation() -> None:
    compute_path = _method_source("_compute_path", "_navigate")
    serialize = _method_source(
        "_serialize_stop_turn_candidates", "_compute_alternative_routes"
    )

    assert "resolved_goal, goal_adjusted = self._resolve_planning_goal" in compute_path
    assert "request_planner.plan_candidates" in compute_path
    assert "self._serialize_stop_turn_candidates(planned)" in compute_path
    assert "self._route_metadata(" in serialize
    assert "segment_directions=directions" in serialize
    assert 'planner="StopTurnStateLattice24"' in compute_path
    assert "_request_path_once" not in compute_path


def test_compute_path_retries_only_bounded_stop_turn_search_failures() -> None:
    compute_path = _method_source("_compute_path", "_navigate")

    assert 'planner_result.status == "SEARCH_EXPANSION_LIMIT"' in compute_path
    assert (
        'planner_result.status == "SEARCH_TIME_BUDGET_EXCEEDED"'
        not in compute_path
    )
    assert '"PLAN_SEARCH_RETRY"' in compute_path
    assert "search_expansion_limit=retry_expansion_limit" in compute_path
    assert "self.stop_turn_retry_planning_budget" in compute_path
    assert "and not live_obstacle_search" in compute_path
    assert "self.stop_turn_live_obstacle_planning_budget" in compute_path


def test_measured_start_turn_matches_executor_physical_footprint() -> None:
    assert "measured_start: bool = False" in NAVIGATION_CORE_SOURCE
    assert (
        "0.0 if measured_start else self.hard_side_margin"
        in NAVIGATION_CORE_SOURCE
    )
    assert "measured_start=at_measured_start" in NAVIGATION_CORE_SOURCE


def test_raw_plan_never_replaces_validated_visualization_or_follow_path() -> None:
    callback = _method_source("_path_callback", "_path_length")
    compute = _method_source("_compute_path", "_navigate")
    serialize = _method_source(
        "_serialize_stop_turn_candidates", "_compute_alternative_routes"
    )
    navigate = _method_source("_navigate", "_navigation_feedback")

    assert "self.latest_planner_raw_path = path" in callback
    assert "self.latest_global_path = path" not in callback
    assert "request_planner.plan_candidates" in compute
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
    assert "None if allow_monotonic_initial_overlap else 99" in validate
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


def test_started_compute_failure_becomes_nonterminal_automatic_recovery() -> None:
    compute = _method_source("_compute_path", "_navigate")

    assert "def begin_automatic_plan_recovery(" in compute
    assert 'payload.get("auto_recover")' in compute
    assert '"recovery_pending": True' in compute
    assert 'status="RECOVERY_PENDING"' in compute
    assert "self.execution_goal = dict(resolved_goal)" in compute
    assert "self.paused_goal = dict(resolved_goal)" in compute
    assert "self._enter_dynamic_wait(" in compute
    assert "resume_automatically=True" in compute
    assert 'self.latest_feedback.pop("terminal_reason", None)' in compute


def test_invalid_preview_never_falls_back_to_a_curved_planner() -> None:
    ensure = _method_source("_ensure_executable_path", "_global_cost_at")

    assert "self._validate_static_stop_turn_route(" in ensure
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
    assert 'self._localization_lost(' in result
    assert '"SEGMENT_CONTROLLER_UNRELIABLE_POSE"' in result
    assert '"SEGMENT_CONTROLLER_FAILURE", goal_generation' in result


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
    assert "motion_direction=motion_direction" in prepare
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
    assert "active.motion_direction < 0" in velocity
    assert "profile.backup_speed" in velocity
    assert '"DIRECTIONAL_SAFETY_BLOCK"' in velocity
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


def test_segment_completion_is_atomic_idempotent_and_cannot_skip_final_leg() -> None:
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")
    complete = _method_source(
        "_complete_active_segment", "_finish_execution_success"
    )
    finish = _method_source(
        "_finish_execution_success", "_restart_segment_from_current"
    )
    result = _method_source("_navigation_result", "_set_recovery_terminal")

    assert "with self.state_lock:" in complete
    for authority in (
        "goal_generation == self.navigation_goal_generation",
        "self.execution_segment_token == segment_token",
        "self.execution_segment_index == segment_index",
        'self.current_state == "NAVIGATING"',
    ):
        assert authority in complete
    assert '"SEGMENT_COMPLETION_IGNORED"' in complete
    assert "segment_token=active.segment_token" in tick
    assert "segment_index=active.segment_index" in tick
    assert "segment_token=segment_token" in result
    assert "segment_index=segment_index" in result

    # Even if a future regression corrupts the segment index, terminal success
    # remains impossible while the measured chassis is outside goal tolerance.
    assert "authoritative_distance" in finish
    assert "self.stop_turn_final_position_tolerance" in finish
    assert 'rejection_reason = "FINAL_POSITION_NOT_REACHED"' in finish
    assert '"GOAL_COMPLETION_REJECTED"' in finish
    assert "self.navigation_goal_generation += 1" in finish


def test_turn_hysteresis_and_persistent_safety_block_are_bounded() -> None:
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")
    turn_start = _method_source(
        "_begin_turn_or_settling", "_dispatch_prepared_segment"
    )
    sweep = _method_source(
        "_live_rotation_sweep_clear", "_manual_takeover_callback"
    )

    assert "turn_hysteresis_transition" in tick
    assert '"TURN_SETTLING"' in tick
    assert "self.execution_turn_reentry_tolerance" in tick
    assert "self.execution_turn_stable_dwell" in tick
    assert "self.execution_turn_safety_block_timeout" in tick
    assert '"PERSISTENT_SAFETY_BLOCK"' in tick
    assert 'action="RELOCATE_TO_TURN_BAY"' in tick
    assert "self._start_turn_bay_recovery(pose, generation)" in tick
    assert 'action="ALTERNATIVE_DIRECTION"' not in tick
    assert "self._schedule_execution_replan" in tick
    assert '"TURN_CMD"' in tick
    assert "self.latest_corridor.can_rotate" in sweep
    assert "self._live_rotation_sweep_clear(now)" in tick
    assert "self._live_rotation_sweep_clear(now)" in turn_start

    project = Path(__file__).parents[1]
    parameters = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )["rovera_navigation_adapter"]["ros__parameters"]
    assert parameters["stop_turn_safety_block_timeout_seconds"] == 1.0


def test_turn_reentry_reselects_direction_after_target_overshoot() -> None:
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")
    settling = tick.split(
        'if self.execution_phase == "TURN_SETTLING":', 1
    )[1].split(
        "\n        direction = self.execution_turn_direction", 1
    )[0]
    reentry = settling.split('if transition == "TURN":', 1)[1]

    assert "previous_direction = self.execution_turn_direction" in reentry
    assert "direction = choose_turn_direction(" in reentry
    assert "self.execution_turn_direction =" in reentry
    assert '"TURN" if direction else "WAIT_FOR_TURN_CLEAR"' in reentry
    assert "previous_direction=previous_direction" in reentry
    assert "direction=self.execution_turn_direction" in reentry
    assert "self.execution_turn_reentry_since" in settling
    assert "self.execution_turn_reentry_dwell" in reentry


def test_segment_transitions_wait_for_measured_zero_and_turn_momentum_brakes() -> None:
    tick = _method_source("_segment_execution_tick", "_fresh_execution_pose")

    prepare = tick.split('if self.execution_phase == "STRAIGHT_PREPARE":', 1)[1]
    assert 'self.pipeline_samples.get("motion_safety")' in prepare
    assert "final_velocity_settled" in prepare
    assert "self.execution_velocity_settle_timeout" in prepare
    assert "self._actual_odom_yaw()" in tick
    assert "self.turn_motion_tracker.observe(" in tick
    assert "current_angular_speed=turn_motion.angular_speed" in tick
    assert "self.execution_turn_min_effective_speed" in tick
    assert '"TURN_STALL_BOOST"' in tick


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
    assert "request_planner.plan_candidates" in alternatives


def test_preview_and_alternative_search_preserve_center_mission_identity() -> None:
    compute = _method_source("_compute_path", "_navigate")
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")

    assert 'payload.get("mission_id")' in compute
    alternatives = dispatch.split('if command == "navigation.alternatives":', 1)[1]
    alternatives = alternatives.split('if command == "navigation.select_route":', 1)[0]
    assert "self.current_mission_id" in alternatives
    assert 'payload.get("mission_id")' in alternatives


def test_live_narrow_uncertainty_does_not_cancel_prevalidated_auto_route() -> None:
    callback = _method_source("_scan_callback", "_sensor_time_callback")
    decision = callback.split("self._corridor_failure_evidence()", 1)[1]
    assert 'if evidence_reason == "PHYSICALLY_BLOCKED"' in decision
    assert 'evidence_reason in {"NARROW_OR_UNCERTAIN"' not in decision


def test_confirmed_physical_corridor_block_runs_bounded_goal_recovery() -> None:
    callback = _method_source("_scan_callback", "_sensor_time_callback")
    recover = _method_source(
        "_recover_confirmed_corridor_blockage", "_enter_narrow_path_decision"
    )
    velocity = _method_source("_auto_velocity_callback", "_update_motion_metrics")
    turn_bay = _method_source(
        "_find_and_start_turn_bay", "_odom_pose"
    )
    completion = _method_source(
        "_complete_active_segment", "_finish_execution_success"
    )

    assert "self._recover_confirmed_corridor_blockage(" in callback
    assert "self._enter_narrow_path_decision(" not in callback
    assert '"CORRIDOR_PHYSICAL_BLOCK"' in velocity
    assert "self._fresh_corridor_blockage(now)" in velocity
    assert "self.execution_segment_token += 1" in recover
    assert "handle.cancel_goal_async()" in recover
    assert "self._mark_failed_segment(reason, corridor)" in recover
    assert "preferred_translation_direction=-1" in recover
    assert 'recovery_reason="CORRIDOR_PHYSICAL_BLOCK"' in recover
    assert '"CORRIDOR_BLOCKED_NO_SAFE_RELOCATION"' in recover
    assert "self.execution_narrow_segments = {0}" in turn_bay
    assert 'action="REPLAN_ORIGINAL_DESTINATION"' in completion
    assert "target=self._replan_execution_from_current" in completion


def test_follow_path_abort_diagnostics_precede_retry_budget_and_preserve_goal() -> None:
    result = _method_source("_navigation_result", "_set_recovery_terminal")
    policy = _method_source(
        "_controller_abort_live_blockage", "_runtime_live_blockage_reason"
    )

    assert 'getattr(result, "error_code"' in result
    assert 'getattr(result, "error_msg"' in result
    assert '"FOLLOW_PATH_RESULT"' in result
    assert "controller_abort_is_live_blockage" in policy
    assert "_controller_abort_live_blockage" in result
    assert 'self._enter_dynamic_wait("CONTROLLER_ABORT_LIVE_BLOCKAGE")' in result
    assert result.index("_controller_abort_live_blockage") < result.index(
        "self.navigation_recovery_attempts < self.failed_segment_max_replans"
    )


def test_rotation_sweep_stop_and_start_escape_remain_runtime_recoverable() -> None:
    safety = _method_source("_safety_status_callback", "_safety_source_callback")
    atomic = _method_source("_atomic_dynamic_blockage", "_remaining_execution_route")
    prepare = _method_source("_prepare_active_segment", "_begin_turn_or_settling")
    escape = _method_source(
        "_start_escape_execution", "_final_position_distance"
    )
    validation = _method_source(
        "_validate_executable_path", "_path_after_initial_distance"
    )

    assert '"ROTATION_SWEEP_COLLISION"' in safety
    assert '"ROTATION_SWEEP_COLLISION"' in atomic
    assert '"START_ESCAPE", "TURN_BAY"' in prepare
    assert "allow_monotonic_initial_overlap=active_relocation" in prepare
    assert "endpoint = dict(refreshed_escape.end)" in prepare
    assert 'status="ALREADY_CLEAR"' in prepare
    assert "START_ESCAPE_ALREADY_CLEAR" in prepare
    assert "self._live_start_escape_directions(" in escape
    assert "directions=escape_directions" in escape
    assert "escape.motion_direction" in escape
    assert "None if allow_monotonic_initial_overlap else 99" in validation
    direction_gate = _method_source(
        "_live_start_escape_directions", "_compute_path"
    )
    assert "preferred_turn_bay_directions(pose, goal)" in direction_gate
    assert "((1, 1), (-1, 2))" in direction_gate


def test_navigation_pose_jump_stops_before_replan_and_preserves_destination() -> None:
    update = _method_source("_update_pose", "_publish_initial_pose")
    guard = _method_source(
        "_execution_pose_candidate_accepted",
        "_pause_for_execution_pose_discontinuity",
    )
    pause = _method_source(
        "_pause_for_execution_pose_discontinuity",
        "_release_execution_pose_hold",
    )
    release = _method_source(
        "_release_execution_pose_hold", "_fresh_execution_pose"
    )
    fresh = _method_source("_fresh_execution_pose", "_prepare_active_segment")

    assert "_execution_pose_candidate_accepted(pose)" in update
    assert "_execution_pose_candidate_accepted(pose)" in fresh
    assert "execution_pose_continuity(" in guard
    assert "self.navigation_velocity.publish(Twist())" in guard
    assert guard.index("self.navigation_velocity.publish(Twist())") < guard.index(
        "_pause_for_execution_pose_discontinuity"
    )
    assert '"goal": goal' in pause
    assert '"path": list(self.latest_global_path)' in pause
    assert "self.latest_global_path = []" not in pause
    assert "self._begin_localization_verification(allow_rotation=True)" in pause
    assert "self.execution_pose_hold = False" in release
    assert "self.pose =" in release


def test_live_blockage_replan_exhaustion_enters_bounded_evidence_wait() -> None:
    schedule = _method_source(
        "_schedule_execution_replan", "_enter_dynamic_wait"
    )
    live_branch = schedule.split("if live_blockage_reason:", 1)[1].split(
        "if self.execution_replan_attempts", 1
    )[0]

    assert "self._enter_dynamic_wait" in live_branch
    assert "_set_recovery_terminal" not in live_branch
    assert schedule.index("if live_blockage_reason:") < schedule.index(
        "if self.execution_replan_attempts"
    )
    assert 'self._enter_dynamic_wait("REPLAN_BUDGET_COOLDOWN")' in schedule
    assert "_set_recovery_terminal" not in schedule


def test_dynamic_wait_periodically_replans_and_success_resumes_same_goal() -> None:
    tick = _method_source("_dynamic_recovery_tick", "_attempt_dynamic_replan")
    attempt = _method_source("_attempt_dynamic_replan", "_replan_execution_from_current")
    blocker = _method_source(
        "_observe_controller_blocker", "_schedule_execution_replan"
    )

    assert "self.dynamic_obstacle_wait" in tick
    assert "self.dynamic_replan_retry" in tick
    assert 'self._set_state("DYNAMIC_REPLAN"' in tick
    assert "target=self._attempt_dynamic_replan" in tick
    assert "goal = dict(self.execution_goal or self.paused_goal or {})" in attempt
    assert "self._navigate(" in attempt
    assert 'self._set_state("WAITING_FOR_DYNAMIC_CLEAR"' in attempt
    assert "destination_preserved=True" in attempt
    assert "minimum_observations=self.dynamic_planning_minimum_observations" in blocker
    assert 'result="POSITION_UNCONFIRMED"' in blocker
    assert "observe_confirmed_blocker" not in blocker
    # Missing scan classification must not renew another wait window. Every
    # generated route is still constrained by the live costmap and footprint.
    assert "self.dynamic_unconfirmed_blocker_timeout" not in tick
    assert "self._stop_unconfirmed_dynamic_recovery()" not in tick
    assert "self._observe_controller_blocker()" in tick
    assert "corridor_blocked" in tick
    assert 'self.dynamic_block_reason.startswith("CONTROLLER_ABORT")' not in tick
    enter = _method_source("_enter_dynamic_wait", "_dynamic_recovery_tick")
    navigate = _method_source("_navigate", "_start_escape_execution")
    assert 'self.current_state == "NAVIGATING"' not in enter.split(
        "new_physical_encounter = bool(", 1
    )[1].split(")", 1)[0]
    assert "if not recovery_attempt:" in navigate
    assert '.split("-clear-resume", 1)[0]' in attempt


def test_live_costmap_filters_static_cells_before_bounding_dynamic_overlay() -> None:
    callback = _method_source("_costmap_callback", "_refresh_dynamic_obstacle_view")

    complete = callback.index("max_cells=None")
    static_filter = callback.index("if self.saved_map is not None:")
    distance_priority = callback.index("obstacles.sort(")
    bounded = callback.index("obstacles = obstacles[: self.dynamic_overlay_max_cells]")
    observed = callback.index("self.dynamic_overlay.observe(")

    assert complete < static_filter < distance_priority < bounded < observed
    assert "saved_map=None" in callback


def test_dynamic_recovery_quickly_replans_around_any_persistent_obstacle() -> None:
    tick = _method_source("_dynamic_recovery_tick", "_attempt_dynamic_replan")
    attempt = _method_source("_attempt_dynamic_replan", "_replan_execution_from_current")

    assert "self.dynamic_planning_minimum_observations" in tick
    assert 'action="PROACTIVE_TRAJECTORY_CONFLICT"' in tick
    assert "dynamic_trajectory_conflict_ttc" in tick
    assert 'self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}' in tick
    assert "self.active_segment is not None" in tick
    assert "self.current_goal_handle is not None" in tick
    assert "self._dynamic_tracking_evidence_trusted(now)" in tick
    assert "ttc > 0.0 or immediate_physical_block" in tick
    assert 'item.motion_state in {"MOVING", "STATIONARY"}' not in tick
    assert 'item.motion_state == "MOVING"' in tick
    assert 'item.motion_state == "STATIONARY"' in tick
    assert '"MOVING_OBSTACLE_HAS_PRIORITY"' in tick
    assert "corridor_evidence_fresh" in tick
    assert "moving_route_blocked" in tick
    assert "stationary_route_blocked" in tick
    assert "controller_corridor_blocked" in tick
    assert "bool(self.dynamic_block_reason)" in tick
    for state in ("CLASSIFYING", "REPLAN_PENDING", "REPLAN_RUNNING", "WAITING"):
        assert f'self.dynamic_recovery_state = "{state}"' in tick
    assert "not requires_alternative" in attempt
    assert "requires_alternative" in attempt
    assert "unconfirmed_replan_due" not in tick
    assert 'result="RESUME_ORIGINAL_ROUTE"' in attempt
    assert '"segment_directions": resume_directions' in attempt
    assert '"SEARCHING_ALTERNATIVE_ROUTES"' in tick
    assert "self.dynamic_moving_obstacle_max_wait" not in tick
    assert "self._stop_persistent_moving_dynamic_recovery()" not in tick


def test_dynamic_wait_evaluates_clearance_along_the_blocked_route() -> None:
    heading = _method_source("_current_path_heading", "_speed_profile_state")
    fresh = _method_source(
        "_corridor_sample_fresh_for_path", "_record_controller_abort"
    )

    assert '"WAIT_FOR_DYNAMIC_CLEAR"' in heading
    assert "self.dynamic_blocked_route[1:]" in heading
    assert "self.dynamic_blocked_segment_directions[index]" in heading
    assert "corridor.classification == \"PHYSICALLY_BLOCKED\"" in fresh
    assert "corridor.physically_passable" in fresh


def test_dynamic_clear_requires_sustained_route_aligned_samples() -> None:
    project = Path(__file__).parents[1]
    navigation = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )
    parameters = navigation["rovera_navigation_adapter"]["ros__parameters"]

    assert parameters["dynamic_obstacle_clear_dwell_seconds"] >= 1.0


def test_auto_route_uses_two_centimetre_hard_and_seven_centimetre_preferred_margin() -> None:
    project = Path(__file__).parents[1]
    navigation = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )
    parameters = navigation["rovera_navigation_adapter"]["ros__parameters"]
    compute = _method_source("_compute_path", "_navigate")
    recovery = _method_source(
        "_plan_stop_turn_from_current", "_resume_auto_from_current_pose"
    )
    static_validation = _method_source(
        "_validate_static_stop_turn_route", "_decision_keepout"
    )

    assert parameters["corridor_side_margin"] == 0.07
    assert parameters["stop_turn_minimum_route_side_clearance"] == 0.02
    assert "self._hard_route_side_clearance()" in compute
    assert "self._stop_turn_planner_for_clearance(" in compute
    assert "request_planner.plan_result(" in compute
    assert "self.corridor_side_margin" in compute
    assert '"ROUTE_CLEARANCE_INSUFFICIENT"' in compute
    assert '"ROUTE_CLEARANCE_INSUFFICIENT"' in recovery
    assert "initial_reverse_clearance_recovery" in compute
    assert "initial_reverse_clearance_recovery" in recovery
    assert "directions[0] >= 0" in static_validation
    assert "any(direction < 0 for direction in directions[1:])" in static_validation
    assert "self.turn_bay_max_distance" in static_validation
    assert "half_width=self.footprint_half_width" in static_validation
    assert "half_width=reserved_half_width" in static_validation


def test_planning_and_recovery_project_route_aligned_front_lidar_keepout() -> None:
    helper = _method_source(
        "_live_front_keepout_for_route", "_route_signature"
    )
    compute = _method_source("_compute_path", "_navigate")
    blocker = _method_source(
        "_observe_controller_blocker", "_schedule_execution_replan"
    )

    assert "self.corridor_samples" in helper
    assert "recent_corridors" in helper
    assert "key=lambda item: float(item[1].front_clearance)" in helper
    assert "math.radians(20.0)" in helper
    assert "self.footprint_half_length + front_clearance" in helper
    assert "segment_length" in helper
    assert '"PLAN_LIVE_FRONT_KEEPOUT"' in compute
    assert 'action="APPLY_BEFORE_SEARCH"' in compute
    assert 'action="DEFER_UNTIL_FIRST_SEGMENT"' in compute
    assert "front_keepout_can_precede_route_search(" in compute
    assert 'action="REPLAN_BEFORE_MOTION"' in compute
    assert "exclusions=(live_front_keepout,)" in compute
    assert "planning_exclusions = (live_front_keepout,)" in compute
    assert "exclusions=planning_exclusions" in compute
    assert 'source="ROUTE_ALIGNED_FRONT_LIDAR"' in blocker
    assert 'result="KEEP_OUT_CONFIRMED"' in blocker

    presearch = compute.split("observed_front_keepout =", 1)[1].split(
        "planner_result =", 1
    )[0]
    assert "blocked_only=True" in presearch


def test_controller_blocker_is_a_persistent_recovery_planning_keepout() -> None:
    exclusions = _method_source(
        "_dynamic_exclusions", "_dynamic_affects_remaining_route"
    )
    blocker = _method_source(
        "_observe_controller_blocker", "_schedule_execution_replan"
    )
    wait = _method_source("_enter_dynamic_wait", "_dynamic_recovery_tick")
    recovery = _method_source(
        "_plan_stop_turn_from_current", "_resume_auto_from_current_pose"
    )

    assert "self.dynamic_blocked_keepout" in exclusions
    assert "return (self.dynamic_blocked_keepout,)" in exclusions
    assert "return self._dynamic_exclusions()" in exclusions
    assert "self.alternative_route_keepout_radius" in blocker
    assert 'result="KEEP_OUT_REUSED"' in blocker
    assert "obstacle.radius + footprint_inflation" in blocker
    assert "force: bool = False" in blocker
    assert "self._observe_controller_blocker(force=True)" in wait
    assert "self._dynamic_planning_exclusions()" in recovery


def test_map_changes_clear_latched_dynamic_recovery_state() -> None:
    load_map = _method_source("_load_map", "_deactivate_map")
    deactivate_map = _method_source("_deactivate_map", "_goal_pose")

    for source in (load_map, deactivate_map):
        assert "self._reset_dynamic_recovery()" in source


def test_planner_keeps_local_reducer_but_uses_global_visibility_topology() -> None:
    planner = NAVIGATION_CORE_SOURCE

    assert "def _reduce_one_route_corner(" in planner
    assert "replacement_local_length > old_local_length + 0.30" in planner
    assert "result.metadata.turn_count" in planner
    assert "def _minimum_turn_visibility_route(" in planner
    assert "start_state = (0, -1)" in planner
    assert "visibility_result, minimum_turn_proven" in planner


def test_dynamic_replan_pause_invalidates_inflight_route_restart() -> None:
    attempt = _method_source("_attempt_dynamic_replan", "_replan_execution_from_current")
    pause = _method_source("_pause_navigation", "_manual_handoff")

    generation_checks = [
        index
        for index in range(len(attempt))
        if attempt.startswith(
            "if expected_generation != self.navigation_goal_generation:",
            index,
        )
    ]
    assert len(generation_checks) >= 2
    assert generation_checks[1] < attempt.index("same_blocked_route = bool(")
    for state in (
        '"WAIT_FOR_DYNAMIC_CLEAR"',
        '"WAITING_FOR_DYNAMIC_CLEAR"',
        '"DYNAMIC_REPLAN"',
    ):
        assert state in pause


def test_dynamic_recovery_rejects_same_blocked_geometry_without_retry_burn() -> None:
    attempt = _method_source("_attempt_dynamic_replan", "_replan_execution_from_current")

    confirmation_gate = attempt.index("if requires_alternative:")
    ordinary_replan = attempt.index("self._plan_stop_turn_from_current(goal)")
    assert confirmation_gate < ordinary_replan
    assert "self._attempt_dynamic_local_bypass_or_selection(" in attempt[
        confirmation_gate:ordinary_replan
    ]
    assert "return" in attempt[confirmation_gate:ordinary_replan]

    assert "path_overlap_ratio" in attempt
    assert "self.dynamic_blocked_route" in attempt
    assert 'self._set_state("WAITING_FOR_DYNAMIC_CLEAR"' in attempt
    assert 'result="ALTERNATIVE_REJECTED"' in attempt
    assert '"STILL_INTERSECTS_BLOCKER"' in attempt
    assert "self.dynamic_failed_route_signatures" in attempt
    assert "self.execution_replan_attempts += 1" not in attempt


def test_dynamic_blockage_only_auto_starts_a_same_corridor_bypass() -> None:
    classify = _method_source(
        "_dynamic_route_relation", "_present_dynamic_route_selection"
    )
    recover = _method_source(
        "_attempt_dynamic_local_bypass_or_selection", "_attempt_dynamic_replan"
    )

    assert "path_overlap_ratio(self.dynamic_blocked_route, points)" in classify
    assert "path_maximum_deviation(" in classify
    assert "self.dynamic_local_bypass_minimum_overlap" in classify
    assert "self.dynamic_local_bypass_maximum_deviation" in classify
    assert "plan_candidates(" in recover
    assert "exclusions=self._dynamic_planning_exclusions()" in recover
    assert "self._serialize_stop_turn_candidates(planned)" in recover
    local_branch = recover.split("if local_candidates:", 1)[1].split(
        "if global_candidates:", 1
    )[0]
    global_branch = recover.split("if global_candidates:", 1)[1].split(
        "self._set_dynamic_blocked_no_route(", 1
    )[0]
    assert "self._navigate(" in local_branch
    assert 'result="AUTO_RESUME_IN_OLD_CORRIDOR"' in local_branch
    assert "global_route_started=False" in local_branch
    assert "self._present_dynamic_route_selection(" in global_branch
    assert "self._navigate(" not in global_branch


def test_global_dynamic_alternative_auto_selects_shortest_after_countdown() -> None:
    present = _method_source(
        "_present_dynamic_route_selection",
        "_attempt_dynamic_local_bypass_or_selection",
    )
    dispatch = _method_source("_dispatch", "_mapping_command")
    tick = _method_source("_dynamic_recovery_tick", "_dynamic_route_relation")

    assert 'candidate["requires_user_confirmation"] = True' in present
    assert 'candidate["recovery_route_kind"] = "GLOBAL_ALTERNATIVE"' in present
    auto_select = _method_source(
        "_auto_start_shortest_dynamic_route",
        "_attempt_dynamic_local_bypass_or_selection",
    )

    assert '"ALTERNATIVE_ROUTE_SELECTION_COUNTDOWN"' in present
    assert "self.dynamic_route_selection_timeout" in present
    assert 'self.latest_feedback["route_selection_deadline_unix_ms"]' in present
    assert 'self._set_state(\n                "ROUTE_SELECTION"' in present
    assert "self.navigation_velocity.publish(Twist())" in present
    assert "resume_automatically=False" in present
    assert "autonomous_global_route_started=False" in present
    assert "self._navigate(" not in present
    assert 'command == "navigation.select_route"' in dispatch
    assert "self._start_selected_route(" in dispatch
    assert 'command == "navigation.route_selection_back"' in dispatch
    assert 'return self._cancel_navigation("CANCELED")' in dispatch
    assert 'self.dynamic_recovery_state = "WAITING_OLD_ROUTE_ONLY"' not in dispatch
    assert 'self.current_state == "ROUTE_SELECTION"' in tick
    assert '"AWAITING_USER_ROUTE_CONFIRMATION"' in tick
    assert 'self.dynamic_recovery_state = "AUTO_SELECT_RUNNING"' in tick
    assert '"AUTO_SELECTING_SHORTEST_ROUTE"' in tick
    assert "self._auto_start_shortest_dynamic_route" in tick
    assert "self.route_candidates.values()" in auto_select
    assert 'choice="AUTO_TIMEOUT"' in auto_select
    assert "self._set_dynamic_blocked_no_route(" in auto_select


def test_exhausted_dynamic_search_relocates_before_becoming_blocked() -> None:
    recover = _method_source(
        "_attempt_dynamic_local_bypass_or_selection", "_attempt_dynamic_replan"
    )
    blocked = _method_source(
        "_set_dynamic_blocked_no_route", "_auto_start_shortest_dynamic_route"
    )

    assert 'result="EXHAUSTED"' in recover
    assert 'result="NO_SAFE_ROUTE"' in recover
    assert 'self.dynamic_recovery_state = "ACTIVE_RELOCATION"' in recover
    assert 'result="ACTIVE_RELOCATION_REQUESTED"' in recover
    assert "self._start_turn_bay_recovery(" in recover
    assert (
        recover.index("self._start_turn_bay_recovery(")
        < recover.rindex("self._set_dynamic_blocked_no_route(")
    )
    assert recover.count("self._set_dynamic_blocked_no_route(") >= 2
    assert 'self.dynamic_recovery_state = "EXHAUSTED"' in blocked
    assert 'self._set_state("BLOCKED"' in blocked
    assert "WAITING_FOR_DYNAMIC_CLEAR" not in blocked


def test_dynamic_local_bypass_thresholds_are_explicit_configuration() -> None:
    project = Path(__file__).parents[1]
    parameters = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )["rovera_navigation_adapter"]["ros__parameters"]

    assert parameters["dynamic_local_bypass_minimum_overlap"] == 0.35
    assert parameters["dynamic_local_bypass_maximum_deviation_m"] == 0.50
    assert parameters["dynamic_obstacle_persistence_seconds"] == 0.75
    assert parameters["dynamic_route_selection_timeout_seconds"] == 10.0


def test_clearance_planners_share_stable_topology_in_conservative_bands() -> None:
    factory = _method_source(
        "_stop_turn_planner_for_clearance", "_live_start_escape_directions"
    )

    assert "requested * 500.0" in factory
    assert "planner._stable_primary_routes = base._stable_primary_routes" in factory


def test_static_disconnect_remains_retryable_and_preserves_runtime_mission() -> None:
    dynamic = _method_source("_attempt_dynamic_replan", "_replan_execution_from_current")
    ordinary = _method_source("_replan_execution_from_current", "_send_current_straight_segment")
    recovery = _method_source("_set_recovery_terminal", "_recover_navigation")

    assert 'exc.code == "GOAL_PHYSICALLY_UNREACHABLE"' in dynamic
    assert "self._set_recovery_terminal" in dynamic
    assert "self._enter_dynamic_wait(" in ordinary
    assert 'self.latest_feedback["destination_preserved"] = True' in ordinary
    assert 'self.latest_feedback.pop("terminal_reason", None)' in recovery
    assert "self._enter_dynamic_wait(" in recovery
    assert "resume_automatically=True" in recovery


def test_runtime_detects_lost_steering_and_turn_hard_stop_as_recovery() -> None:
    safety = _method_source("_safety_status_callback", "_safety_source_callback")
    turn = _method_source("_segment_execution_tick", "_start_turn_bay_recovery")

    assert 'reason="STEERING_AUTHORITY_LOST"' in safety
    assert '"STEERING_AUTHORITY_LOST", self.navigation_goal_generation' in safety
    assert 'action="RELOCATE_AFTER_HARD_STOP"' in safety
    assert "self._start_turn_bay_recovery(" in safety
    assert 'action="REJECT_ERROR_INCREASING_DIRECTION"' in turn
    assert 'self.execution_phase = "WAIT_FOR_TURN_CLEAR"' in turn


def test_turn_block_recovery_moves_one_safe_step_before_global_replanning() -> None:
    relocate = _method_source(
        "_find_and_start_turn_bay", "_odom_pose"
    )
    wait = _method_source("_enter_dynamic_wait", "_dynamic_recovery_tick")
    tick = _method_source("_dynamic_recovery_tick", "_dynamic_route_relation")
    relation = _method_source(
        "_dynamic_route_relation", "_present_dynamic_route_selection"
    )
    candidates = _method_source(
        "_attempt_dynamic_local_bypass_or_selection", "_attempt_dynamic_replan"
    )
    prepare = _method_source(
        "_prepare_active_segment", "_begin_turn_or_settling"
    )
    complete = _method_source(
        "_complete_active_segment", "_finish_execution_success"
    )

    assert "self._dynamic_planning_exclusions()" in relocate
    assert "segment_departs_exclusions(" in relocate
    assert "endpoint_clearance=self.planning_footprint_padding" in relocate
    assert 'self._enter_dynamic_wait("TURN_BLOCKED_NO_SAFE_RELOCATION")' in relocate
    assert "dynamic_block_requires_alternative(reason)" in wait
    assert '== "TURN_BLOCKED_NO_SAFE_RELOCATION"' in tick
    assert "self._safety_blocks_turn(" in tick
    assert "self.dynamic_local_bypass_minimum_deviation" in relation
    assert "maximum_deviation" in candidates
    assert "self.dynamic_local_bypass_minimum_deviation" in candidates
    assert '"START_ESCAPE", "TURN_BAY"' in prepare
    assert "allow_monotonic_initial_overlap=active_relocation" in prepare
    assert "minimum_relocation_distance" in relocate
    assert "self.footprint_half_length" in relocate
    assert "planner.plan_result" not in relocate
    assert "selected = (candidate, direction)" in relocate
    assert "display = [dict(pose), dict(candidate)]" in relocate
    assert 'relocation_reason == "TURN_BAY"' in complete
    assert "self.latest_corridor.can_go_straight" in complete
    assert "not self.latest_corridor.can_rotate" in complete
    assert 'status="CONTINUE_NARROW_TRANSLATION"' in complete
    assert "preferred_translation_direction=active.motion_direction" in complete
    assert "preferred_translation_direction in live_directions" in relocate


def test_scan_map_mismatch_never_destroys_a_continuously_tracked_pose() -> None:
    tick = _method_source("_localization_tick", "_load_map")
    lost = _method_source("_localization_lost", "_global_heading_requirement")

    assert "SUSTAINED_SCAN_MAP_MISMATCH" not in tick
    assert "execution_scan_map_mismatch" not in tick
    assert "self._localization_tracking_evidence_ready(now)" in tick
    assert "now - self.ready_evidence_invalid_since >= self.localization_low_grace" in tick
    # Genuine evidence expiry still owns the destructive recovery path.
    assert "self.trajectory_map_from_odom = None" in lost
    assert "self._start_global_localization()" in lost


def test_sensor_and_localization_recovery_never_clear_goal_on_replan_failure() -> None:
    sensor = _method_source(
        "_resume_sensor_time_navigation_if_ready",
        "_resume_localization_navigation_if_ready",
    )
    localization = _method_source(
        "_resume_localization_navigation_if_ready", "_localization_lost"
    )
    wait = _method_source("_enter_dynamic_wait", "_dynamic_recovery_tick")

    assert 'self._set_state("PAUSED"' not in sensor
    assert "self.sensor_time_resume_context = None" in sensor
    sensor_failure = sensor.split("except AdapterError as exc:", 1)[1].split(
        "                return", 1
    )[0]
    assert "self.sensor_time_resume_context = None" not in sensor_failure
    assert 'action="REPLAN_WAIT_AND_RETRY"' in sensor
    assert 'self._set_state("PAUSED"' not in localization
    assert '"WAITING_FOR_DYNAMIC_CLEAR"' in localization
    assert 'self.latest_feedback.pop("terminal_reason", None)' in sensor
    assert 'self.latest_feedback.pop("terminal_reason", None)' in localization
    assert 'self.latest_feedback.pop("terminal_reason", None)' in wait


def test_no_last_pose_starts_passive_localization_without_velocity_ownership() -> None:
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    passive = _method_source("_start_global_localization", "_safe_to_rotate")

    assert "self._start_global_localization()" in automatic
    assert 'self.localization_state = "PASSIVE_LOCALIZING"' in passive
    assert "self.localization_velocity.publish" not in passive
    assert "self.motion_owner = \"LOCALIZATION\"" not in passive


def test_acquisition_uses_configurable_multi_frame_consensus_and_uniqueness() -> None:
    verdict = _method_source("_localization_verdict", "_localization_quality_ready")
    callback = _method_source("_particle_cloud_callback", "_pose_is_stable")
    project = Path(__file__).parents[1]
    parameters = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )["rovera_navigation_adapter"]["ros__parameters"]

    assert parameters["localization_consensus_window_size"] == 7
    assert parameters["localization_consensus_required_frames"] == 5
    assert "localization_evidence_consensus(" in verdict
    assert "self.localization_evidence_frames" in verdict
    assert "particle_cloud_uniqueness(" in callback
    assert "self.particle_uniqueness.accepted" in verdict
    assert "self._request_global_scan_uniqueness()" in verdict
    assert "self.global_scan_uniqueness.accepted" in verdict
    assert '"GLOBAL_SCAN_UNIQUENESS"' in ADAPTER_SOURCE
    assert (
        'ParticleCloud,\n            "/particle_cloud",\n'
        "            self._particle_cloud_callback,\n"
        "            qos_profile_sensor_data"
    ) in ADAPTER_SOURCE


def test_untrusted_pose_cannot_create_map_relative_dynamic_planning_authority() -> None:
    scan = _method_source("_planning_scan_message", "_scan_callback")
    costmap = _method_source("_costmap_callback", "_refresh_dynamic_obstacle_view")
    safety = _method_source("_safety_status_callback", "_safety_source_callback")
    ready_reset = _method_source(
        "_reset_pre_ready_planning_evidence", "_publish_failed_segments"
    )

    assert 'self.localization_state != "READY"' in scan
    assert "[math.inf for _ in message.ranges]" in scan
    assert 'self.localization_state != "READY"' in costmap
    assert "and self.localized" in safety
    assert 'and self.localization_state == "READY"' in safety
    assert "self.dynamic_overlay = DynamicObstacleOverlay(" in ready_reset
    assert "ClearEntireCostmap.Request()" in ready_reset


def test_map_disagreement_revokes_only_costmap_tracking_stop_authority() -> None:
    costmap = _method_source("_costmap_callback", "_refresh_dynamic_obstacle_view")
    trust = _method_source(
        "_dynamic_tracking_evidence_trusted", "_refresh_dynamic_obstacle_view"
    )
    exclusions = _method_source(
        "_dynamic_exclusions", "_dynamic_affects_remaining_route"
    )

    assert "if not self._dynamic_tracking_evidence_trusted():" in costmap
    assert '"DYNAMIC_TRACKING_SUPPRESSED"' in costmap
    assert 'stats.get("reason") == "EXPECTED_RANGE_MATCH"' in trust
    assert 'stats.get("dynamic_points_kept"' in trust
    assert "self.dynamic_tracking_maximum_point_ratio" in trust
    assert "self.dynamic_tracking_minimum_static_matches" in trust
    assert "if not self._dynamic_tracking_evidence_trusted():" in exclusions
    # Raw Motion Safety remains the independent physical stop authority.
    assert "self._atomic_dynamic_blockage" not in costmap


def test_costmap_transform_prefers_message_timestamp_and_bounds_latest_fallback() -> None:
    callback = _method_source("_costmap_callback", "_refresh_dynamic_obstacle_view")

    assert '"map", source_frame, Time.from_msg(message.header.stamp)' in callback
    assert '"map", source_frame, Time()' in callback
    assert "transform_age > self.scan_tf_fallback_max_age" in callback
    assert 'reason="LATEST_TF_STALE_FOR_MESSAGE"' in callback
    assert 'fallback="BOUNDED_FRESH_LATEST"' in callback


def test_approximate_hint_discards_yaw_and_never_sets_ready() -> None:
    operator = _method_source("_set_initial_pose", "_deactivate_map")
    fallback = _method_source("_begin_operator_pose_hint", "_set_initial_pose")

    assert '"yaw": 0.0' in operator
    assert "self.global_search_untrusted = True" in fallback
    assert "self.localization_operator_hint_active = True" in fallback
    assert 'self.localization_state = "LOCALIZING_APPROXIMATE_POSE"' in fallback
    assert '"localized": False' in operator
    assert 'self.localization_state = "READY"' not in operator


def test_explicit_operator_pose_overrides_the_old_odometry_prior() -> None:
    projected = _method_source("_odometry_predicted_map_pose", "_update_pose")
    operator = _method_source("_set_initial_pose", "_deactivate_map")
    prior = _method_source("_begin_odometry_prior", "_begin_operator_pose_hint")
    tick = _method_source("_localization_tick", "_load_map")

    assert "self.trajectory_map_from_odom is None" in projected
    assert "self._critical_sensor_time_healthy()" in projected
    assert "self.localization_odometry_prior_rejected_epoch" in projected
    assert '"odom", "base_footprint", Time()' in projected
    assert '"covariance": 0.01' in projected
    assert "self._odometry_predicted_map_pose()" not in operator
    assert "self._begin_odometry_prior(" not in operator
    assert "self._begin_operator_pose_hint(" in operator
    assert "self.localization_odometry_prior_active = True" in prior
    assert 'self.localization_state = "LOCALIZING_LAST_POSE"' in prior
    assert "self.odometry_prior_timeout" in tick
    assert '"ODOMETRY_PRIOR_REJECTED"' in tick
    assert "self._begin_operator_pose_hint(" in tick
    assert "self._start_global_localization()" in tick


def test_operator_hint_runs_one_independent_scan_seed_before_waiting_on_amcl() -> None:
    request = _method_source(
        "_request_global_scan_uniqueness", "_path_callback"
    )
    tick = _method_source("_localization_tick", "_load_map")

    assert "operator_seed: bool = False" in request
    assert "require_candidate_match=not operator_seed" in request
    assert "self.operator_hint_search_radius" in request
    assert "math.radians(45.0) if operator_seed else None" in request
    assert "self.localization_seed_approximate = False" in request
    assert "self.localization_operator_hint_active = True" in request
    assert "self._publish_initial_pose(scan_seed, approximate=False)" in request
    assert '"LOCALIZATION_OPERATOR_SCAN_SEED"' in request
    assert "strict_verification_required=True" in request
    assert "self._request_global_scan_uniqueness(operator_seed=True)" in tick


def test_repeated_operator_hint_preserves_a_converging_attempt() -> None:
    operator = _method_source("_set_initial_pose", "_deactivate_map")

    assert "previous_hint = self.localization_pending_operator_hint" in operator
    assert "<= 0.20" in operator
    assert '"CANDIDATE", "VERIFYING"' in operator
    assert '"LOCALIZATION_HINT_IGNORED"' in operator
    repeated_branch = operator.split(
        "if repeated_hint and self.localization_state in active_hint_states:", 1
    )[1].split("self._begin_operator_pose_hint(", 1)[0]
    assert "self._reset_localization_evidence()" not in repeated_branch
    assert "evidence_preserved=True" in repeated_branch


def test_failed_localization_is_terminal_until_new_evidence_or_command() -> None:
    tick = _method_source("_localization_tick", "_load_map")

    ready_acceptance = tick.index('self.localization_state = "READY"')
    failed_terminal = tick.index(
        'if self.localization_state == "LOCALIZATION_FAILED":',
        ready_acceptance,
    )
    generic_timeout = tick.index(
        "now - self.localization_started_monotonic >= self.localization_timeout",
        failed_terminal,
    )
    assert ready_acceptance < failed_terminal < generic_timeout


def test_odometry_trajectory_is_unanchored_until_first_trusted_localization() -> None:
    record = _method_source("_record_odometry_trajectory", "_anchor_odometry_trajectory")
    anchor = _method_source("_anchor_odometry_trajectory", "_update_pose")
    update = _method_source("_update_pose", "_publish_initial_pose")

    assert '"quality": "UNANCHORED"' in record
    assert '"frame": "odom"' in record
    assert '"quality": "TRUSTED"' in record
    assert '"quality": "RECONSTRUCTED"' in anchor
    assert "self._record_odometry_trajectory(odom_transform)" in update


def test_status_trajectory_is_bounded_before_transport() -> None:
    state = _method_source("_state", "_wait")

    assert '"trajectory": self._status_trajectory()' in state
    assert "self.odometry_trajectory[-200:]" in state
    assert "maximum_points: int = 40" in state
    assert '"trajectory": list(self.odometry_trajectory[-200:])' not in state


def test_localization_loss_preserves_revalidates_and_resumes_mission() -> None:
    lost = _method_source("_localization_lost", "_global_heading_requirement")
    resume = _method_source(
        "_resume_localization_navigation_if_ready", "_localization_lost"
    )

    assert "self.localization_resume_context" in lost
    assert "self.paused_goal" in lost
    assert "handle.cancel_goal_async()" in lost
    assert "original_points" in resume
    assert "self._navigate(" in resume
    assert "self._plan_stop_turn_from_current(goal)" in resume
    assert 'action="WAIT_AND_RETRY"' in resume
    assert "destination_preserved=True" in resume


def test_restart_recovery_persists_destination_but_never_publishes_stale_route() -> None:
    persist = _method_source(
        "_persist_active_navigation_mission", "_clear_active_navigation_mission"
    )
    load = _method_source("_load_map", "_begin_odometry_prior")
    navigate = _method_source("_navigate", "_start_escape_execution")
    finish = _method_source("_finish_execution_success", "_restart_segment_from_current")
    cancel = _method_source("_cancel_navigation", "_pause_navigation")

    assert '"/var/lib/rovera/navigation/active-mission.json"' in ADAPTER_SOURCE
    assert "output.flush()" in persist
    assert "os.fsync(output.fileno())" in persist
    assert "os.replace(temporary, target)" in persist
    assert '"resume_automatically": bool(resume_automatically)' in persist
    assert "self.localization_resume_context = {" in load
    assert '"path": []' in load
    assert 'stale_route_published=False' in load
    assert 'stored_mission.get("path")' not in load
    assert "self._persist_active_navigation_mission(resume_automatically=True)" in navigate
    assert 'self._clear_active_navigation_mission("goal_succeeded")' in finish
    assert "resume_automatically=False" in cancel
    assert "elif target != \"PAUSED\":" in cancel


def test_final_position_short_circuit_is_before_turn_and_endpoint_replan() -> None:
    tick = _method_source("_segment_execution_tick", "_start_turn_bay_recovery")
    prepare = _method_source("_prepare_active_segment", "_begin_turn_or_settling")
    complete = _method_source(
        "_complete_final_position_if_reached", "_segment_execution_tick"
    )
    finish = _method_source("_finish_execution_success", "_restart_segment_from_current")

    first_final_check = tick.index("_complete_final_position_if_reached")
    assert first_final_check < tick.index("_prepare_active_segment")
    straight = tick.split('if self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}:', 1)[1]
    assert straight.index("_complete_final_position_if_reached") < straight.index(
        "straight_segment_progress"
    )
    assert prepare.index("_complete_final_position_if_reached") < prepare.index(
        "planned_progress.passed_endpoint"
    )
    assert prepare.index("_complete_final_position_if_reached") < prepare.index(
        "_begin_turn_or_settling"
    )
    assert "straight_segment_progress" in complete
    assert "self.straight_endpoint_tolerance" in complete
    assert "self.saved_map.validate_footprint" in complete
    assert '"GOAL_COMPLETION_DEFERRED"' in complete
    assert '"FINAL_CROSS_TRACK_OUTSIDE_ENDPOINT_BAND"' in complete
    assert '"FINAL_FOOTPRINT_NOT_STATIC_VALID"' in complete
    assert '"GOAL_REACHED"' in finish
    assert 'mode="POSITION_ONLY"' in finish
    assert 'phase="FINAL_TURN_END"' in finish
    assert "physical_final_turn" in finish


def test_final_tolerance_is_separate_from_internal_segment_tolerance() -> None:
    project = Path(__file__).parents[1]
    navigation = yaml.safe_load(
        (project / "navigation-stack/config/nav2_params.yaml").read_text()
    )
    parameters = navigation["rovera_navigation_adapter"]["ros__parameters"]
    controller = navigation["controller_server"]["ros__parameters"]

    assert parameters["straight_endpoint_tolerance"] == 0.03
    assert parameters["stop_turn_final_position_tolerance"] == 0.12
    assert parameters["stop_turn_final_position_tolerance"] == controller[
        "goal_checker"
    ]["xy_goal_tolerance"]


def test_adapter_visualization_contract_always_identifies_route_authority() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")

    assert '"route_id": (' in dispatch
    assert "self.execution_route_id" in dispatch
    assert "self.selected_route_id" in dispatch


def test_edge_health_keeps_route_selection_bound_to_its_mission() -> None:
    assert '"mission_id": backend_state.get("mission_id", "")' in EDGE_CLIENT_SOURCE


def test_unexpected_localization_callback_fault_revokes_all_motion_authority() -> None:
    decorator = ADAPTER_SOURCE[
        ADAPTER_SOURCE.index("def localization_callback("):
        ADAPTER_SOURCE.index("\n\nclass NavigationAdapter")
    ]
    fail_closed = _method_source(
        "_fail_closed_localization_callback",
        "_prepare_motionless_localization_command",
    )

    assert "self._fail_closed_localization_callback(method.__name__, exc)" in decorator
    for required in (
        "expected_generation != self.navigation_goal_generation",
        "self.current_goal_handle = None",
        "self.navigation_goal_generation += 1",
        "self.execution_segment_token += 1",
        'self.execution_phase = "IDLE"',
        'self.motion_owner = "NONE"',
        "self.sensor_time_resume_context = None",
        "self.localization_resume_context = None",
        "for publisher in (self.navigation_velocity, self.localization_velocity)",
        "handle.cancel_goal_async()",
        'resume_automatically=False',
    ):
        assert required in fail_closed
    assert fail_closed.index("publisher.publish(Twist())") < fail_closed.index(
        'self._set_state(\n                "LOCALIZATION_FAILED"'
    )


def test_explicit_pose_changes_require_motionless_state_and_fence_recovery() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    set_pose = _method_source("_set_initial_pose", "_deactivate_map")
    fence = _method_source(
        "_prepare_motionless_localization_command",
        "_degrade_localization",
    )

    relocalize = dispatch.split('if command == "map.relocalize":', 1)[1]
    assert "self._prepare_motionless_localization_command()" in relocalize
    assert "self._prepare_motionless_localization_command()" in set_pose
    for required in (
        "self.current_goal_handle is not None",
        'self.motion_owner != "NONE"',
        "self.rotation_active",
        'self.execution_phase != "IDLE"',
        '"MOTION_ACTIVE"',
        "self.navigation_goal_generation += 1",
        "self.execution_segment_token += 1",
        "self.sensor_time_resume_context = None",
        "self.localization_resume_context = None",
        "self.navigation_velocity.publish(Twist())",
        "self.localization_velocity.publish(Twist())",
    ):
        assert required in fence


def test_all_automatic_recovery_navigation_commits_are_generation_fenced() -> None:
    recovery_calls = [
        call
        for call in _navigate_calls(ADAPTER_SOURCE)
        if isinstance(_keyword(call, "recovery_attempt"), ast.Constant)
        and _keyword(call, "recovery_attempt").value is True
    ]

    assert recovery_calls
    for call in recovery_calls:
        assert _keyword(call, "expected_generation") is not None, (
            f"recovery _navigate call at line {call.lineno} has no generation fence"
        )

    sensor_resume = _method_source(
        "_resume_sensor_time_navigation_if_ready",
        "_resume_localization_navigation_if_ready",
    )
    assert "expected_generation = self.navigation_goal_generation" in sensor_resume
    assert "expected_generation=expected_generation" in sensor_resume
    assert '"sensor_time_navigation_resume"' in sensor_resume


def test_localization_resume_does_not_rebind_outer_directions_in_worker() -> None:
    resume = _method_source(
        "_resume_localization_navigation_if_ready",
        "_localization_lost",
    )
    worker = resume.split("        def resume() -> None:", 1)[1]

    assert "directions = list(original_directions)" in worker
    assert "original_directions =" not in worker
    assert worker.count("expected_generation=expected_generation") == 3
    assert '"localization_navigation_resume"' in worker


def test_operator_motion_interrupts_clear_every_automatic_resume_context() -> None:
    estop = _method_source("_estop_callback", "_direction_mask_callback")
    manual = _method_source(
        "_manual_takeover_callback", "_record_odometry_trajectory"
    )
    methods = (
        estop,
        manual,
        _method_source("_cancel_navigation", "_pause_navigation"),
        _method_source("_manual_handoff", "_resume_auto_from_current_pose"),
    )

    for source in methods:
        assert "self.sensor_time_resume_context = None" in source
        assert "self.sensor_time_resume_in_progress = False" in source
        assert "self.localization_resume_context = None" in source
        assert "self.localization_resume_in_progress = False" in source
    assert '"TURN_BAY_RECOVERY"' in estop
    assert "self.localization_velocity.publish(Twist())" in estop
    assert '"TURN_BAY_RECOVERY"' in manual
    assert "self.localization_velocity.publish(Twist())" in manual


def test_turn_bay_worker_checks_generation_inside_atomic_state_commit() -> None:
    start = _method_source("_start_turn_bay_recovery", "_find_and_start_turn_bay")
    worker = _method_source("_find_and_start_turn_bay", "_odom_pose")
    commit = worker.split("        candidate, direction = selected", 1)[1]

    assert "with self.state_lock:" in start
    assert "goal_generation != self.navigation_goal_generation" in start
    assert "with self.state_lock:" in commit
    assert commit.index("goal_generation != self.navigation_goal_generation") < (
        commit.index('self.execution_phase = "STRAIGHT_PREPARE"')
    )
