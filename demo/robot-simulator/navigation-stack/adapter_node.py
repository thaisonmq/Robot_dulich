from __future__ import annotations

from collections import deque
from functools import wraps
import hashlib
import json
import math
import os
import signal
import shutil
import socket
import tarfile
import threading
import time
import traceback
import statistics
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
from PIL import Image
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from navigation_core import (
    ActiveSegment,
    DynamicObstacle,
    DynamicObstacleOverlay,
    ExecutablePathValidation,
    GlobalScanUniqueness,
    LocalizationConsensus,
    LocalizationEvidenceFrame,
    LocalizationVerification,
    MapNavigationGeometry,
    NavigationDebugLog,
    ParticleCloudUniqueness,
    PoseStability,
    SavedOccupancyMap,
    StopTurnStateLatticePlanner,
    TurnBlockTracker,
    UnwrappedYawProgress,
    bounded_heading_evidence,
    canonicalize_stop_turn_path,
    classify_planning_failure,
    compact_lethal_cells,
    controller_abort_is_live_blockage,
    choose_turn_direction,
    densify_straight_segment,
    dynamic_block_requires_alternative,
    dynamic_exclusions_intersect_route,
    dynamic_trajectory_conflict_ttc,
    endpoint_braking_speed_limit,
    environment_flag,
    evaluate_corridor,
    execution_pose_continuity,
    filter_static_map_scan,
    find_start_escape,
    global_scan_candidate_uniqueness,
    heading_diversity,
    heading_position_spread,
    image_grayscale_values,
    localization_confidence,
    localization_evidence_consensus,
    localization_verification,
    mapping_pose_match_quality,
    particle_cloud_uniqueness,
    normalize_trinary_unknown_metadata,
    path_maximum_deviation,
    path_overlap_ratio,
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
    validate_stop_turn_route,
)
from speed_profiles import (
    AutoNavigationProfiles,
    ProfileVelocityLimiter,
    SpeedModeStore,
    SpeedProfileError,
)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class AdapterError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def localization_callback(method: Any) -> Any:
    """Serialize localization callbacks and keep one bad frame fail-closed.

    Socket commands and background scan evaluation run outside ROS' default
    callback group, so a MultiThreadedExecutor alone cannot protect the shared
    AMCL/scan evidence.  A callback exception must stop localization, not tear
    down the adapter process and leave its container deceptively alive.
    """

    @wraps(method)
    def guarded(self: "NavigationAdapter", *args: Any, **kwargs: Any) -> Any:
        with self.localization_lock:
            try:
                return method(self, *args, **kwargs)
            except Exception as exc:  # ROS executors otherwise stop spinning.
                self.localized = False
                self.localization_state = "LOCALIZATION_FAILED"
                self.get_logger().error(
                    f"{method.__name__} failed: {type(exc).__name__}: {exc}"
                )
                self._nav_debug(
                    "LOCALIZATION_CALLBACK_ERROR",
                    callback=method.__name__,
                    error_type=type(exc).__name__,
                    error=str(exc),
                    traceback=traceback.format_exc(),
                )
                self._stop_localization_rotation()
                self._set_state("LOCALIZATION_FAILED", "localization_callback_error")
                return None

    return guarded


def localization_serialized(method: Any) -> Any:
    """Use the localization mutex for non-ROS command/state transitions."""

    @wraps(method)
    def synchronized(self: "NavigationAdapter", *args: Any, **kwargs: Any) -> Any:
        with self.localization_lock:
            return method(self, *args, **kwargs)

    return synchronized


class NavigationAdapter(Node):
    def __init__(self) -> None:
        super().__init__("rovera_navigation_adapter")
        self.mode = os.getenv("NAVIGATION_MODE", "NAVIGATION").upper()
        self.socket_path = Path(
            os.getenv(
                "NAVIGATION_SOCKET_PATH",
                "/var/lib/rovera/navigation/navigation.sock",
            )
        )
        self.map_root = Path(os.getenv("ROVERA_MAP_ROOT", "/var/lib/rovera/maps"))
        self.active_navigation_mission_path = Path(
            os.getenv(
                "ACTIVE_NAVIGATION_MISSION_PATH",
                "/var/lib/rovera/navigation/active-mission.json",
            )
        )
        self.active_navigation_mission_lock = threading.Lock()
        self.navigation_debug_enabled = environment_flag(
            "NAVIGATION_DEBUG_LOG", True
        )
        self.navigation_debug_log = NavigationDebugLog(
            enabled=self.navigation_debug_enabled,
            path=os.getenv(
                "NAVIGATION_DEBUG_LOG_PATH",
                "/var/lib/rovera/navigation/logs/navigation-debug.log",
            ),
        )
        self.state_lock = threading.Lock()
        # Localization state is also touched by the socket worker and the
        # independent global-scan worker, not just ROS callbacks.
        self.localization_lock = threading.RLock()
        self.current_state = "IDLE" if self.mode == "MAPPING" else "STARTING"
        self.localization_state = "IDLE"
        self.localized = False
        self.initial_pose_requested = False
        self.map_id = ""
        self.map_version = 0
        self.mapping_payload: dict[str, Any] = {}
        self.mapping_relocalization_event = threading.Event()
        self.mapping_relocalization_active = False
        self.mapping_relocalization_probe_count = 0
        self.mapping_relocalization_max_probes = max(
            1, int(os.getenv("MAPPING_RELOCALIZATION_MAX_PROBES", "1"))
        )
        self.mapping_relocalization_timeout = max(
            2.0, float(os.getenv("MAPPING_RELOCALIZATION_TIMEOUT_SECONDS", "12"))
        )
        self.mapping_relocalization_max_position_correction = max(
            0.1,
            float(os.getenv("MAPPING_RELOCALIZATION_MAX_POSITION_CORRECTION_M", "1.5")),
        )
        self.mapping_relocalization_max_yaw_correction = math.radians(max(
            5.0,
            float(os.getenv("MAPPING_RELOCALIZATION_MAX_YAW_CORRECTION_DEG", "75")),
        ))
        self.mapping_relocalization_max_xy_stddev = max(
            0.05, float(os.getenv("MAPPING_RELOCALIZATION_MAX_XY_STDDEV_M", "0.75"))
        )
        self.mapping_relocalization_max_yaw_stddev = math.radians(max(
            5.0, float(os.getenv("MAPPING_RELOCALIZATION_MAX_YAW_STDDEV_DEG", "45"))
        ))
        self.mapping_relocalization_hint: tuple[float, float, float] | None = None
        self.mapping_relocalization_result: dict[str, Any] | None = None
        self.mapping_pose_search_event = threading.Event()
        self.mapping_pose_search_active = False
        self.mapping_pose_search_snapshot: dict[str, Any] | None = None
        self.mapping_relocalization_latest_snapshot: dict[str, Any] | None = None
        self.mapping_relocalization_source_map: SavedOccupancyMap | None = None
        self.mapping_relocalization_corrected_pose: (
            tuple[float, float, float] | None
        ) = None
        self.mapping_relocalization_geometry_confirmations = 0
        self.mapping_relocalization_geometry_samples = 0
        self.mapping_relocalization_required_confirmations = max(
            2, int(os.getenv("MAPPING_RELOCALIZATION_CONFIRMATION_SCANS", "3"))
        )
        self.mapping_relocalization_max_validation_scans = max(
            self.mapping_relocalization_required_confirmations,
            int(os.getenv("MAPPING_RELOCALIZATION_MAX_VALIDATION_SCANS", "8")),
        )
        self.mapping_pose_search_minimum_score = float(
            os.getenv("MAPPING_POSE_SEARCH_MINIMUM_SCORE", "0.42")
        )
        self.mapping_pose_search_minimum_margin = float(
            os.getenv("MAPPING_POSE_SEARCH_MINIMUM_MARGIN", "0.08")
        )
        self.mapping_pose_search_minimum_ratio = float(
            os.getenv("MAPPING_POSE_SEARCH_MINIMUM_RATIO", "1.12")
        )
        self.mapping_relocalization_diagnostics: dict[str, Any] = {
            "state": "NOT_REQUIRED",
            "hint_is_approximate": True,
        }
        # A continued pose-graph contains every historical scan. SLAM Toolbox
        # can add new structure, but it does not forget an obstacle merely
        # because later rays pass through its old cells. Record only repeated
        # capture-time, map-frame free-ray evidence from the current session;
        # it is applied conservatively when the new immutable bundle is saved.
        self.mapping_free_cell_observations: dict[tuple[int, int], int] = {}
        self.mapping_hit_cell_observations: dict[tuple[int, int], int] = {}
        self.mapping_change_evidence_scans = 0
        self.mapping_change_maximum_beams = max(
            24, int(os.getenv("MAPPING_CHANGE_MAXIMUM_BEAMS", "90"))
        )
        self.mapping_change_minimum_free_observations = max(
            3, int(os.getenv("MAPPING_CHANGE_MINIMUM_FREE_OBSERVATIONS", "8"))
        )
        self.mapping_change_endpoint_protection = max(
            0.08,
            float(os.getenv("MAPPING_CHANGE_ENDPOINT_PROTECTION_M", "0.15")),
        )
        self.current_goal_handle: Any = None
        self.navigation_goal_generation = 0
        self.paused_goal: dict[str, float] | None = None
        self.current_mission_id = ""
        self.latest_feedback: dict[str, Any] = {}
        self.saved_map: SavedOccupancyMap | None = None
        self.map_navigation_geometry: MapNavigationGeometry | None = None
        self.stop_turn_planner: StopTurnStateLatticePlanner | None = None
        self.active_map_path: Path | None = None
        self.map_received_monotonic = 0.0
        self.last_scan_monotonic = 0.0
        self.scan_clock_skew_seconds = 0.0
        self.sensor_time_status: dict[str, Any] = {}
        self.last_sensor_time_status_monotonic = 0.0
        self.sensor_time_invalid_since: float | None = None
        # A source-clock fault suspends autonomous motion, but a short-lived
        # transport/clock hiccup must not discard the mission that was already
        # executing.  Keep the recovery data separate from normal pause/manual
        # handoff state so only this fault path can resume it automatically.
        self.sensor_time_failure_reason = ""
        self.sensor_time_failure_diagnostics: dict[str, Any] = {}
        self.sensor_time_pause_started_monotonic: float | None = None
        self.sensor_time_previous_localization_state = ""
        self.sensor_time_resume_context: dict[str, Any] | None = None
        self.sensor_time_resume_in_progress = False
        self.localization_resume_context: dict[str, Any] | None = None
        self.localization_resume_in_progress = False
        self.last_localization_resume_attempt_monotonic = 0.0
        self.sensor_time_hard_failed = False
        self.sensor_time_hard_failure_timeout = 15.0
        self.safety_health = "UNKNOWN"
        self.estop_active = False
        self.safety_direction_mask = 0
        self.safety_snapshot: dict[str, Any] = {}
        self.safety_snapshot_sequence = -1
        self.safety_snapshot_monotonic = 0.0
        self.safety_subscription_started_monotonic = time.monotonic()
        self.last_safety_subscription_rebind_monotonic = 0.0
        self.safety_status_subscription: Any = None
        self.last_manual_takeover_monotonic = 0.0
        self.pose: dict[str, float] | None = None
        self.odometry_trajectory: list[dict[str, Any]] = []
        self.trajectory_map_from_odom: tuple[float, float, float] | None = None
        self.last_trajectory_odom: tuple[float, float, float] | None = None
        self.trajectory_odom_epoch = 0
        self.execution_pose_guard_lock = threading.Lock()
        self.execution_pose_anchor_map: dict[str, float] | None = None
        self.execution_pose_anchor_odom: dict[str, float] | None = None
        self.execution_pose_discontinuity_since: float | None = None
        self.execution_pose_discontinuity_logged = False
        self.execution_pose_discontinuity_handled = False
        self.execution_pose_hold = False
        self.trail: list[dict[str, float]] = []
        self.localization_confidence = 0.0
        self.localization_started_monotonic = 0.0
        self.localization_phase_started_monotonic = 0.0
        self.last_amcl_monotonic = 0.0
        self.last_map_tf_monotonic = 0.0
        self.last_amcl_pose: tuple[float, float, float] | None = None
        self.last_amcl_covariance: list[float] = []
        self.pose_window_size = int(self.declare_parameter(
            "localization_pose_window_size", 8
        ).value)
        self.pose_window: deque[tuple[float, float, float, float]] = deque(
            maxlen=max(5, self.pose_window_size)
        )
        self.pose_stability_metrics: PoseStability = pose_stability(())
        self.scan_map_scores: deque[float] = deque(maxlen=max(
            3,
            int(self.declare_parameter("scan_map_score_window_size", 5).value),
        ))
        self.localization_consensus_window_size = max(3, int(
            self.declare_parameter(
                "localization_consensus_window_size", 7
            ).value
        ))
        self.localization_consensus_required_frames = max(2, min(
            self.localization_consensus_window_size,
            int(self.declare_parameter(
                "localization_consensus_required_frames", 5
            ).value),
        ))
        self.localization_consensus_position_tolerance = float(
            self.declare_parameter(
                "localization_consensus_position_tolerance", 0.10
            ).value
        )
        self.localization_consensus_yaw_tolerance = math.radians(float(
            self.declare_parameter(
                "localization_consensus_yaw_tolerance_degrees", 10.0
            ).value
        ))
        self.localization_evidence_frames: deque[
            LocalizationEvidenceFrame
        ] = deque(maxlen=self.localization_consensus_window_size)
        self.localization_consensus = LocalizationConsensus(
            False,
            "CONSENSUS_WINDOW_INCOMPLETE",
            0,
            self.localization_consensus_required_frames,
            0,
            0,
        )
        self.scan_map_median_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_p90_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_mean_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.raycast_static_match_ratios: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.raycast_dynamic_occlusion_ratios: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.raycast_contradiction_ratios: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.raycast_median_errors: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.raycast_p90_errors: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
        self.raycast_comparable_beams = 0
        self.raycast_static_matches = 0
        self.raycast_dynamic_occlusions = 0
        self.raycast_map_contradictions = 0
        self.raycast_inconclusive_map_hits = 0
        self.raycast_matched_beams = 0
        self.raycast_static_match_ratio = 0.0
        self.raycast_dynamic_occlusion_ratio = 0.0
        self.raycast_contradiction_ratio = 0.0
        self.raycast_match_ratio = 0.0
        self.raycast_median_error = math.inf
        self.raycast_p90_error = math.inf
        self.last_scan_map_monotonic = 0.0
        self.verification_scan_count = 0
        self.verification_started_monotonic = 0.0
        self.ready_evidence_since: float | None = None
        self.ready_evidence_invalid_since: float | None = None
        self.laser_in_base: tuple[float, float, float] | None = None
        self.low_confidence_since: float | None = None
        self.last_nomotion_request_monotonic = 0.0
        self.localization_seed_pose: dict[str, Any] | None = None
        self.localization_seed_approximate = False
        self.localization_operator_hint_active = False
        # A verified map<-odom anchor is the best short-lived prior after a
        # rescan. Keep the operator's approximate point as a fallback: odom
        # still has to pass the complete LiDAR verification gate and must
        # never replace the human hint after a rejected anchor.
        self.localization_pending_operator_hint: dict[str, float] | None = None
        self.localization_odometry_prior_active = False
        self.localization_odometry_prior_rejected_epoch: int | None = None
        self.global_search_requires_rotation = False
        self.global_search_rotation_pending = False
        self.global_search_untrusted = False
        self.stationary_global_candidate_ambiguous = False
        self.localization_attempt_sequence = 0
        self.localization_attempt_id = ""
        self.approximate_hint_allowed = False
        self.last_particle_cloud_monotonic = 0.0
        self.particle_uniqueness = ParticleCloudUniqueness(
            False,
            "PARTICLE_CLOUD_UNAVAILABLE",
            0,
            0,
            0.0,
            0.0,
            0.0,
        )
        self.global_scan_uniqueness = GlobalScanUniqueness(
            False, "GLOBAL_SCAN_NOT_EVALUATED", 0, 0, 0.0, 0.0, 0.0, 0.0
        )
        self.global_scan_uniqueness_in_progress = False
        self.global_scan_evaluation_generation = 0
        self.global_scan_evaluated_candidate: tuple[float, float, float] | None = None
        self.latest_localization_scan_snapshot: dict[str, Any] | None = None
        self.last_initial_pose_publish_monotonic = 0.0
        self.rotation_active = False
        self.rotation_angle = 0.0
        self.rotation_last_monotonic = 0.0
        self.rotation_yaw_progress = UnwrappedYawProgress()
        self.localization_rotation_cycle_start_angle = 0.0
        self.localization_next_observation_angle = 0.0
        self.localization_actual_yaw: float | None = None
        self.localization_settling_evidence_started = False
        # Passive by default: physical rotation always requires an explicit
        # command and is never implied by map load, reconnect or recovery.
        self.localization_rotation_authorized = False
        def configured(name: str, default: float, env_name: str = "") -> float:
            fallback = float(os.getenv(env_name, str(default))) if env_name else default
            return float(self.declare_parameter(name, fallback).value)

        self.localization_confidence_threshold = configured(
            "localization_confidence_threshold", 0.72,
            "LOCALIZATION_CONFIDENCE_THRESHOLD",
        )
        self.localization_low_threshold = configured(
            "localization_low_confidence_threshold", 0.30,
            "LOCALIZATION_LOW_CONFIDENCE_THRESHOLD",
        )
        self.localization_low_grace = configured(
            "localization_low_confidence_grace_seconds", 5.0,
            "LOCALIZATION_LOW_CONFIDENCE_GRACE_SECONDS",
        )
        self.localization_ready_hold = configured(
            "localization_ready_hold_seconds", 2.0
        )
        self.last_pose_timeout = configured(
            "localization_last_pose_timeout_seconds", 12.0,
            "LAST_POSE_TIMEOUT_SECONDS",
        )
        self.odometry_prior_timeout = configured(
            "localization_odometry_prior_timeout_seconds", 4.0
        )
        self.approximate_pose_timeout = configured(
            "localization_approximate_pose_timeout_seconds", 20.0
        )
        self.global_rotate_delay = configured(
            "global_localization_rotate_delay_seconds", 5.0,
            "GLOBAL_LOCALIZATION_ROTATE_DELAY_SECONDS",
        )
        self.stationary_global_retry_delay = configured(
            "localization_stationary_global_retry_seconds", 1.0
        )
        self.localization_rotation_settle = configured(
            "localization_rotation_settle_seconds", 1.0
        )
        self.localization_timeout = configured(
            "auto_localization_timeout_seconds", 45.0,
            "AUTO_LOCALIZATION_TIMEOUT_SECONDS",
        )
        self.localization_verify_timeout = configured(
            "localization_verify_timeout_seconds", 3.0
        )
        self.localization_verify_min_scans = int(self.declare_parameter(
            "localization_verify_min_scans", 3
        ).value)
        self.amcl_pose_freshness = configured(
            "localization_amcl_pose_freshness_seconds", 1.5
        )
        self.pose_minimum_samples = int(self.declare_parameter(
            "localization_pose_minimum_samples", 5
        ).value)
        self.pose_minimum_duration = configured(
            "localization_pose_minimum_duration_seconds", 1.0
        )
        self.pose_maximum_xy_spread = configured(
            "localization_pose_maximum_xy_spread", 0.12
        )
        self.pose_maximum_median_deviation = configured(
            "localization_pose_maximum_median_deviation", 0.06
        )
        self.pose_maximum_yaw_variance = configured(
            "localization_pose_maximum_yaw_variance", 0.025
        )
        self.pose_maximum_yaw_spread = configured(
            "localization_pose_maximum_yaw_spread", 0.20
        )
        self.scan_map_threshold = configured("scan_map_minimum_score", 0.35)
        self.tracking_scan_map_sanity_threshold = configured(
            "localization_tracking_scan_map_sanity_score", 0.12
        )
        self.global_scan_map_threshold = max(
            self.scan_map_threshold,
            configured("localization_global_scan_map_minimum_score", 0.80),
        )
        self.global_final_scan_map_threshold = max(
            self.scan_map_threshold,
            configured(
                "localization_global_final_scan_map_minimum_score", 0.70
            ),
        )
        self.localization_maximum_covariance_xy = configured(
            "localization_maximum_covariance_xy", 0.50
        )
        self.localization_maximum_covariance_yaw = configured(
            "localization_maximum_covariance_yaw", 0.50
        )
        self.scan_map_minimum_beams = int(self.declare_parameter(
            "scan_map_minimum_valid_beams", 25
        ).value)
        self.localization_final_minimum_residual_beams = max(1, int(
            self.declare_parameter(
                "localization_final_minimum_residual_beams", 20
            ).value
        ))
        self.scan_map_maximum_beams = int(self.declare_parameter(
            "scan_map_maximum_beams", 90
        ).value)
        self.localization_raycast_maximum_beams = max(1, int(
            self.declare_parameter(
                "localization_raycast_maximum_beams", 90
            ).value
        ))
        self.localization_raycast_minimum_beams = max(1, int(
            self.declare_parameter(
                "localization_raycast_minimum_comparable_beams", 25
            ).value
        ))
        self.localization_raycast_match_tolerance = configured(
            "localization_raycast_match_tolerance", 0.15
        )
        self.localization_raycast_minimum_reliable_structure_span = configured(
            "localization_raycast_minimum_reliable_structure_span", 0.75
        )
        self.localization_raycast_minimum_static_matches = max(1, int(
            self.declare_parameter(
                "localization_raycast_minimum_static_matches", 20
            ).value
        ))
        # A bounded operator hint already supplies the missing spatial prior.
        # Permit a smaller but still substantial static sample in sparse
        # end-of-route geometry, then require a strict multi-frame match ratio,
        # pose/covariance gates and a unique AMCL particle cluster below. Blind
        # global localization keeps the stricter acquisition thresholds above.
        self.localization_operator_hint_minimum_raycast_beams = min(
            self.localization_raycast_minimum_beams,
            max(1, int(self.declare_parameter(
                "localization_operator_hint_minimum_comparable_beams", 12
            ).value)),
        )
        self.localization_operator_hint_minimum_static_matches = min(
            self.localization_raycast_minimum_static_matches,
            max(1, int(self.declare_parameter(
                "localization_operator_hint_minimum_static_matches", 12
            ).value)),
        )
        self.localization_operator_hint_minimum_scan_score = configured(
            "localization_operator_hint_minimum_scan_score", 0.50
        )
        self.localization_operator_hint_minimum_explained_ratio = configured(
            "localization_operator_hint_minimum_explained_ratio", 0.65
        )
        self.localization_raycast_maximum_contradiction_ratio = configured(
            "localization_raycast_maximum_contradiction_ratio", 0.20
        )
        # Acquisition must remain strict, but a continuously tracked READY
        # pose can end a route facing movable furniture or another transient
        # occluder.  Navigation start uses this wider sanity ceiling together
        # with fresh AMCL/TF/scan, confidence and residual gates; it must not
        # silently rerun the first-acquisition contradiction test.
        self.localization_tracking_maximum_contradiction_ratio = configured(
            "localization_tracking_maximum_contradiction_ratio", 0.40
        )
        self.particle_cloud_freshness = configured(
            "localization_particle_cloud_freshness_seconds", 2.0
        )
        self.particle_cluster_radius = configured(
            "localization_particle_cluster_radius", 0.30
        )
        self.particle_alternative_separation = configured(
            "localization_particle_alternative_separation", 0.75
        )
        self.particle_minimum_best_weight = configured(
            "localization_particle_minimum_best_weight", 0.55
        )
        self.particle_minimum_dominance_ratio = configured(
            "localization_particle_minimum_dominance_ratio", 2.0
        )
        self.global_scan_maximum_beams = max(12, int(self.declare_parameter(
            "localization_global_scan_maximum_beams", 45
        ).value))
        self.global_scan_endpoint_tolerance = configured(
            "localization_global_scan_endpoint_tolerance", 0.15
        )
        self.global_scan_position_step = configured(
            "localization_global_scan_position_step", 0.20
        )
        self.global_scan_heading_step = math.radians(configured(
            "localization_global_scan_heading_step_degrees", 15.0
        ))
        self.global_scan_minimum_best_score = configured(
            "localization_global_scan_minimum_best_score", 0.45
        )
        self.global_scan_minimum_score_margin = configured(
            "localization_global_scan_minimum_score_margin", 0.12
        )
        self.global_scan_minimum_score_ratio = configured(
            "localization_global_scan_minimum_score_ratio", 1.15
        )
        self.global_scan_candidate_position_tolerance = configured(
            "localization_global_scan_candidate_position_tolerance", 0.45
        )
        self.global_scan_candidate_yaw_tolerance = math.radians(configured(
            "localization_global_scan_candidate_yaw_tolerance_degrees", 25.0
        ))
        self.global_scan_hint_radius = configured(
            "localization_global_scan_hint_radius", 1.25
        )
        self.operator_hint_search_radius = configured(
            "localization_operator_hint_search_radius", 0.60
        )
        self.scan_map_minimum_range = configured("scan_map_minimum_range", 0.20)
        self.scan_map_maximum_range = configured("scan_map_maximum_range", 6.0)
        self.localization_coarse_match_tolerance = configured(
            "localization_coarse_match_tolerance", 0.12
        )
        self.localization_final_max_median_residual = configured(
            "localization_final_max_median_residual", 0.075
        )
        self.localization_final_max_p90_residual = configured(
            "localization_final_max_p90_residual", 0.115
        )
        self.planning_static_match_tolerance = configured(
            "planning_static_match_tolerance", 0.08
        )
        self.dynamic_overlay_static_tolerance = configured(
            "dynamic_overlay_static_tolerance", 0.08
        )
        self.dynamic_overlay_ttl = configured(
            "dynamic_overlay_ttl_seconds", 2.0
        )
        self.dynamic_overlay_cluster_distance = configured(
            "dynamic_overlay_cluster_distance", 0.12
        )
        self.dynamic_obstacle_motion_threshold = configured(
            "dynamic_obstacle_motion_threshold", 0.12
        )
        self.dynamic_obstacle_stationary_confirmation = configured(
            "dynamic_obstacle_stationary_confirmation_seconds", 1.0
        )
        self.dynamic_obstacle_moving_confirmation_windows = max(2, int(
            self.declare_parameter(
                "dynamic_obstacle_moving_confirmation_windows", 2
            ).value
        ))
        self.dynamic_planning_minimum_observations = max(2, int(
            self.declare_parameter(
                "dynamic_planning_minimum_observations", 3
            ).value
        ))
        self.dynamic_overlay_max_cells = max(
            50,
            int(self.declare_parameter(
                "dynamic_overlay_max_cells", 600
            ).value),
        )
        self.dynamic_obstacle_wait = configured(
            "dynamic_obstacle_persistence_seconds", 1.5
        )
        self.dynamic_unconfirmed_blocker_timeout = max(
            self.dynamic_obstacle_wait,
            configured("dynamic_unconfirmed_blocker_timeout_seconds", 8.0),
        )
        self.dynamic_unconfirmed_blocker_log_interval = configured(
            "dynamic_unconfirmed_blocker_log_interval_seconds", 1.0
        )
        self.dynamic_moving_obstacle_max_wait = max(
            self.dynamic_unconfirmed_blocker_timeout,
            configured("dynamic_moving_obstacle_max_wait_seconds", 12.0),
        )
        self.dynamic_tracking_maximum_point_ratio = min(
            0.95,
            max(
                0.05,
                configured("dynamic_tracking_maximum_point_ratio", 0.40),
            ),
        )
        self.dynamic_tracking_minimum_static_matches = max(1, int(
            self.declare_parameter(
                "dynamic_tracking_minimum_static_matches", 20
            ).value
        ))
        self.dynamic_tracking_evidence_freshness = configured(
            "dynamic_tracking_evidence_freshness_seconds", 0.75
        )
        self.dynamic_clear_dwell = configured(
            "dynamic_obstacle_clear_dwell_seconds", 1.00
        )
        self.dynamic_replan_retry = configured(
            "dynamic_replan_retry_seconds", 2.0
        )
        self.dynamic_conflict_ttc_horizon = configured(
            "dynamic_obstacle_conflict_ttc_seconds", 3.0
        )
        self.start_escape_max_distance = configured(
            "start_escape_max_distance", 0.60
        )
        self.turn_bay_max_distance = configured(
            "turn_bay_max_relocation_distance", 0.80
        )
        self.scan_tf_wait = configured("scan_tf_wait_seconds", 0.02)
        self.scan_tf_fallback_max_age = configured(
            "scan_tf_fallback_max_age_seconds", 0.12
        )
        self.scan_map_freshness = configured("scan_map_freshness_seconds", 0.60)
        # A start command can arrive in the small gap between two LiDAR/AMCL
        # callbacks.  Wait for the next live evidence sample instead of
        # rejecting a robot which is still continuously maintained as READY.
        self.localization_start_evidence_wait = configured(
            "navigation_start_localization_evidence_wait_seconds", 1.20
        )
        self.sensor_time_invalid_grace = configured(
            "sensor_time_invalid_grace_seconds", 1.0
        )
        self.sensor_time_hard_failure_timeout = configured(
            "sensor_time_hard_failure_seconds", 15.0
        )
        self.rotation_minimum_obstacle_distance = configured(
            "localization_rotation_minimum_obstacle_distance", 0.03
        )
        self.localization_rotation_blocked_timeout = configured(
            "localization_rotation_blocked_timeout_seconds", 5.0
        )
        self.localization_tf_chain_timeout = configured(
            "localization_tf_chain_timeout_seconds", 8.0
        )
        self.localization_tf_unavailable_since: float | None = None
        self.last_localization_tf_log_monotonic = 0.0
        self.last_localization_tf_signature = ""
        self.last_navigation_scan_published_monotonic = 0.0
        self.localization_rotation_blocked_since: float | None = None
        self.global_heading_bin_count = max(4, int(self.declare_parameter(
            "localization_global_heading_bin_count", 8
        ).value))
        self.global_minimum_heading_bins = max(2, int(self.declare_parameter(
            "localization_global_min_heading_bins", 4
        ).value))
        self.global_minimum_heading_span = math.radians(configured(
            "localization_global_min_heading_span_degrees", 150.0
        ))
        self.global_strong_minimum_heading_bins = max(2, int(
            self.declare_parameter(
                "localization_global_strong_min_heading_bins", 2
            ).value
        ))
        self.global_strong_minimum_heading_span = math.radians(configured(
            "localization_global_strong_min_heading_span_degrees", 75.0
        ))
        # Kept as an observable physical-sweep diagnostic and max-sweep guard;
        # READY uses actual scan heading bins/span below, not commanded angle.
        self.global_observation_minimum_rotation = self.global_minimum_heading_span
        self.localization_evidence_headings: list[float] = []
        self.localization_heading_positions: dict[int, tuple[float, float]] = {}
        self.localization_heading_bins: tuple[int, ...] = ()
        self.localization_heading_span = 0.0
        self.rotation_speed = math.radians(
            configured(
                "auto_localization_rotation_degrees_per_second", 45.0,
                "AUTO_LOCALIZATION_ROTATION_DEG_S",
            )
        )
        self.rotation_max_angle = math.radians(
            configured(
                "auto_localization_max_angle_degrees", 270.0,
                "AUTO_LOCALIZATION_MAX_ANGLE_DEG",
            )
        )
        self.nearest_rotation_obstacle = math.inf
        self.motion_owner = "NONE"
        self.footprint_half_length = float(self.declare_parameter(
            "footprint_half_length", 0.15
        ).value)
        self.footprint_half_width = float(self.declare_parameter(
            "footprint_half_width", 0.10
        ).value)
        self.footprint = [
            {"x": self.footprint_half_length, "y": self.footprint_half_width},
            {"x": self.footprint_half_length, "y": -self.footprint_half_width},
            {"x": -self.footprint_half_length, "y": -self.footprint_half_width},
            {"x": -self.footprint_half_length, "y": self.footprint_half_width},
        ]
        self.planning_footprint_padding = float(self.declare_parameter(
            "planning_footprint_padding", 0.0
        ).value)
        self.corridor_hard_side_margin = configured(
            "corridor_hard_side_margin", 0.02
        )
        self.translation_lateral_margin = configured(
            "translation_lateral_margin", 0.01
        )
        self.corridor_side_margin = configured("corridor_side_margin", 0.07)
        self.stop_turn_minimum_route_side_clearance = configured(
            "stop_turn_minimum_route_side_clearance", 0.02
        )
        self.corridor_localization_uncertainty_max = configured(
            "corridor_localization_uncertainty_max", 0.04
        )
        self.corridor_front_clearance = configured(
            "corridor_front_clearance", 0.14
        )
        self.corridor_lookahead = configured("corridor_lookahead", 1.00)
        self.corridor_confirmation_samples = max(2, int(self.declare_parameter(
            "corridor_confirmation_samples", 5
        ).value))
        self.corridor_confirmation_duration = configured(
            "corridor_confirmation_duration_seconds", 0.40
        )
        self.failed_segment_ttl = configured("failed_segment_ttl_seconds", 20.0)
        self.failed_segment_radius = configured("failed_segment_radius", 0.12)
        self.failed_segment_forward_offset = configured(
            "failed_segment_forward_offset", 0.45
        )
        self.failed_segment_max_replans = max(1, int(self.declare_parameter(
            "failed_segment_max_replans", 3
        ).value))
        self.alternative_route_max_candidates = max(1, int(
            self.declare_parameter("alternative_route_max_candidates", 3).value
        ))
        self.alternative_route_overlap_threshold = configured(
            "alternative_route_overlap_threshold", 0.85
        )
        self.dynamic_local_bypass_minimum_overlap = configured(
            "dynamic_local_bypass_minimum_overlap", 0.35
        )
        self.dynamic_local_bypass_maximum_deviation = configured(
            "dynamic_local_bypass_maximum_deviation_m", 0.50
        )
        self.stop_turn_planning_budget = configured(
            "stop_turn_planning_budget_seconds", 12.0
        )
        self.stop_turn_live_obstacle_planning_budget = configured(
            "stop_turn_live_obstacle_planning_budget_seconds", 6.0
        )
        self.stop_turn_retry_planning_budget = configured(
            "stop_turn_retry_planning_budget_seconds", 15.0
        )
        self.stop_turn_retry_expansion_multiplier = max(1, int(
            self.declare_parameter(
                "stop_turn_retry_expansion_multiplier", 4
            ).value
        ))
        self.stop_turn_turn_robustness_radius = configured(
            "stop_turn_turn_robustness_radius", 0.01
        )
        self.stop_turn_require_final_yaw = bool(self.declare_parameter(
            "stop_turn_require_final_yaw", False
        ).value)
        self.stop_turn_final_position_tolerance = configured(
            "stop_turn_final_position_tolerance", 0.12
        )
        self.stop_turn_max_reanchors_per_segment = max(0, int(
            self.declare_parameter(
                "stop_turn_max_reanchors_per_segment", 1
            ).value
        ))
        self.alternative_route_keepout_radius = configured(
            "alternative_route_keepout_radius", 0.18
        )
        self.footprint_clearance = float(os.getenv("GOAL_CLEARANCE_METERS", "0.15"))
        self.goal_snap_max_distance = float(
            os.getenv("GOAL_SNAP_MAX_DISTANCE_METERS", "0.45")
        )
        self.latest_global_path: list[dict[str, float]] = []
        # /plan is planner diagnostics only. A route becomes visible/executable
        # exclusively after the maintained global-costmap footprint validator.
        self.latest_planner_raw_path: list[dict[str, float]] = []
        self.latest_dynamic_obstacles: list[dict[str, float]] = []
        self.dynamic_overlay = DynamicObstacleOverlay(
            ttl_seconds=self.dynamic_overlay_ttl,
            cluster_distance=self.dynamic_overlay_cluster_distance,
            motion_threshold=self.dynamic_obstacle_motion_threshold,
            stationary_confirmation_seconds=(
                self.dynamic_obstacle_stationary_confirmation
            ),
            moving_confirmation_windows=(
                self.dynamic_obstacle_moving_confirmation_windows
            ),
        )
        self.dynamic_wait_started: float | None = None
        self.dynamic_clear_started: float | None = None
        self.dynamic_last_replan = 0.0
        self.dynamic_block_reason = ""
        self.dynamic_blocked_route: list[dict[str, float]] = []
        self.dynamic_blocked_segment_directions: list[int] = []
        self.dynamic_blocked_keepout: tuple[float, float, float] | None = None
        self.dynamic_recovery_state = "IDLE"
        self.dynamic_blocker_id = ""
        self.dynamic_blocked_route_signature = ""
        self.dynamic_failed_route_signatures: dict[str, float] = {}
        self.dynamic_replan_attempt_count = 0
        self.dynamic_replan_requires_alternative = False
        self.dynamic_recovery_expires_monotonic = 0.0
        self.dynamic_last_unconfirmed_log_monotonic = 0.0
        self.dynamic_last_untrusted_log_monotonic = 0.0
        self.latest_global_costmap: OccupancyGrid | None = None
        self.latest_static_map: OccupancyGrid | None = None
        self.last_global_costmap_monotonic = 0.0
        self.global_costmap_generation = 0
        self.global_costmap_condition = threading.Condition()
        self.global_costmap_update = threading.Event()
        self.visualization_revision = 0
        self.mapping_started_monotonic = 0.0
        self.replan_timestamps: list[float] = []
        self.corridor_samples: deque[tuple[float, Any]] = deque(maxlen=30)
        self.latest_corridor: Any = evaluate_corridor(
            (),
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            side_margin=self.corridor_side_margin,
            front_clearance_required=self.corridor_front_clearance,
            rotation_margin=self.rotation_minimum_obstacle_distance,
            hard_side_margin=self.corridor_hard_side_margin,
            translation_lateral_margin=self.translation_lateral_margin,
        )
        self.last_corridor_log_monotonic = 0.0
        self.failed_segments: list[dict[str, Any]] = []
        self.navigation_recovery_attempts = 0
        self.execution_replan_attempts = 0
        self.navigation_corridor_clear_retried = False
        self.navigation_original_path_length = 0.0
        self.safety_stop_source = "UNKNOWN"
        self.safety_stop_reason = "UNKNOWN"
        self.last_logged_stop_source = ""
        self.route_candidates: dict[str, dict[str, Any]] = {}
        self.selected_route_id = ""
        self.route_selection_return_state = "READY"
        self.execution_points: list[dict[str, float]] = []
        self.execution_segment_directions: list[int] = []
        self.execution_segment_index = 0
        self.execution_segment_token = 0
        self.active_segment: ActiveSegment | None = None
        self.execution_phase = "IDLE"
        self.execution_phase_started = 0.0
        self.execution_target_heading = 0.0
        self.execution_turn_stable_since: float | None = None
        self.execution_turn_reentry_since: float | None = None
        self.execution_turn_blocked_since: float | None = None
        self.execution_turn_direction = 0
        self.turn_block_tracker = TurnBlockTracker(clear_dwell_seconds=0.30)
        self.execution_reanchor_after_turn = False
        self.execution_segment_reanchors = 0
        self.execution_final_turn = False
        self.execution_physical_final_turn = False
        self.execution_goal: dict[str, float] | None = None
        self.execution_relocation_reason = ""
        self.execution_relocation_plan: list[dict[str, float]] = []
        self.pending_start_escape: dict[str, Any] | None = None
        self.pending_segment_directions: list[int] = []
        self.execution_route_id = ""
        self.execution_pose_monotonic = 0.0
        self.segment_started_monotonic = 0.0
        self.segment_positive_travel = 0.0
        self.segment_last_travel_pose: tuple[float, float] | None = None
        self.straight_recovery_requested = ""
        self.last_turn_command = (0.0, 0.0)
        self.last_turn_command_log_monotonic = 0.0
        self.execution_settle_seconds = configured(
            "stop_turn_settle_seconds", 0.20
        )
        self.execution_velocity_settle_timeout = max(
            self.execution_settle_seconds,
            configured("stop_turn_velocity_settle_timeout_seconds", 0.80),
        )
        self.execution_turn_tolerance = math.radians(configured(
            "stop_turn_heading_tolerance_degrees", 3.0
        ))
        self.execution_turn_reentry_tolerance = math.radians(configured(
            "stop_turn_heading_reentry_tolerance_degrees", 6.0
        ))
        self.execution_turn_stable_dwell = configured(
            "stop_turn_stable_dwell_seconds", 0.40
        )
        self.execution_turn_reentry_dwell = configured(
            "stop_turn_reentry_dwell_seconds", 0.20
        )
        self.execution_turn_safety_block_timeout = configured(
            "stop_turn_safety_block_timeout_seconds", 5.0
        )
        self.execution_turn_kp = configured("stop_turn_heading_kp", 1.8)
        self.execution_turn_max_speed = configured(
            "stop_turn_max_angular_speed", 0.60
        )
        self.execution_turn_angular_deceleration = configured(
            "stop_turn_angular_deceleration", 2.0
        )
        self.execution_turn_reaction_time = configured(
            "stop_turn_angular_reaction_seconds", 0.12
        )
        self.straight_max_angular_correction = configured(
            "straight_max_angular_correction", 0.18
        )
        self.straight_path_pose_spacing = configured(
            "straight_path_pose_spacing", 0.05
        )
        self.execution_pose_freshness = configured(
            "execution_pose_freshness_seconds", 0.30
        )
        self.execution_pose_max_translation_residual = configured(
            "execution_pose_max_translation_residual_m", 0.12
        )
        self.execution_pose_max_yaw_residual = math.radians(configured(
            "execution_pose_max_yaw_residual_degrees", 20.0
        ))
        self.execution_pose_discontinuity_confirmation = configured(
            "execution_pose_discontinuity_confirmation_seconds", 0.15
        )
        self.straight_heading_kp = configured("straight_heading_kp", 1.2)
        self.straight_cross_track_kp = configured(
            "straight_cross_track_kp", 1.0
        )
        self.straight_heading_deadband = math.radians(configured(
            "straight_heading_deadband_degrees", 1.0
        ))
        self.straight_cross_track_deadband = configured(
            "straight_cross_track_deadband", 0.01
        )
        self.straight_hard_heading_error = math.radians(configured(
            "straight_hard_heading_error_degrees", 12.0
        ))
        self.straight_hard_cross_track = configured(
            "straight_hard_cross_track", 0.08
        )
        self.straight_endpoint_tolerance = configured(
            "straight_endpoint_tolerance", 0.03
        )
        self.straight_overshoot_epsilon = configured(
            "straight_overshoot_epsilon", 0.005
        )
        self.endpoint_reaction_time = configured(
            "straight_endpoint_reaction_time_seconds", 0.15
        )
        self.segment_watchdog_settle_allowance = configured(
            "segment_watchdog_settle_allowance_seconds", 2.0
        )
        self.segment_watchdog_time_factor = configured(
            "segment_watchdog_time_factor", 3.0
        )
        self.segment_watchdog_travel_factor = configured(
            "segment_watchdog_travel_factor", 2.0
        )
        self.segment_watchdog_travel_slack = configured(
            "segment_watchdog_travel_slack", 0.30
        )
        self.manual_handoff_reason = ""
        self.narrow_decision_in_progress = False

        self.speed_profiles = AutoNavigationProfiles.load(
            os.getenv(
                "AUTO_NAVIGATION_SPEED_PROFILES_PATH",
                "/opt/rovera/config/auto_navigation_speed_profiles.yaml",
            )
        )
        self.speed_mode_store = SpeedModeStore(
            os.getenv(
                "AUTO_NAVIGATION_SPEED_MODE_PATH",
                "/var/lib/rovera/navigation/auto-speed-mode.json",
            ),
            self.speed_profiles.default_mode,
        )
        self.auto_speed_mode = self.speed_mode_store.load()
        self.behavior_tree_paths = self.speed_profiles.write_behavior_trees(
            os.getenv(
                "AUTO_NAVIGATION_BT_DIRECTORY",
                "/var/lib/rovera/navigation/behavior_trees",
            )
        )
        self.profile_limiter = ProfileVelocityLimiter(
            pure_rotation_angular_max=self.speed_profiles.hardware.angular_max,
            pure_rotation_angular_decel=self.speed_profiles.hardware.angular_decel_max,
        )
        self.applied_speed_mode = self.auto_speed_mode if self.mode != "NAVIGATION" else ""
        self.profile_applied = self.mode != "NAVIGATION"
        self.profile_apply_error = ""
        self.profile_apply_pending = self.mode == "NAVIGATION"
        self.last_profile_apply_attempt = 0.0
        self.last_pipeline_log_monotonic = 0.0
        self.pipeline_samples: dict[str, tuple[float, float, float]] = {}
        self.last_controller_requested: tuple[float, float, float] | None = None
        self.controller_abort_history: deque[dict[str, Any]] = deque(maxlen=12)
        self.last_controller_blockage_monotonic = 0.0
        self.profile_clamp_reasons: tuple[str, ...] = ()
        self.nearest_forward_obstacle = math.inf
        self.nearest_left_obstacle = math.inf
        self.nearest_right_obstacle = math.inf
        self.obstacle_slowdown_active = False
        self.last_slowdown_obstacle_distance = math.inf
        self.last_replan_obstacle_distance = math.inf
        self.last_planning_failure: dict[str, Any] = {}
        self.profile_callback_latency_ms = 0.0
        self.scan_callback_latency_ms = 0.0
        self.localization_callback_latency_ms = 0.0
        self.planner_latency_ms = 0.0
        self.rotation_metric_active = False
        self.rotation_metric_started = 0.0
        self.rotation_metric_last_sample = 0.0
        self.rotation_metric_integrated_angle = 0.0
        self.rotation_metric_peak_requested = 0.0
        self.rotation_metric_peak_final = 0.0
        self.last_rotation_metrics: dict[str, float] = {}
        self.last_scan_filter_stats = {
            "scan_points_total": 0,
            "scan_points_valid": 0,
            "static_map_matches": 0,
            "dynamic_points_kept": 0,
            "raycast_unavailable": 0,
            "filtered": False,
        }
        self.last_scan_filter_monotonic = 0.0
        self.last_scan_filter_log_monotonic = 0.0
        self.last_localization_candidate_log_monotonic = 0.0
        self.last_localization_rotate_log_monotonic = 0.0
        self.last_tf_debug_monotonic: dict[str, float] = {}
        self.scan_transform_cache: dict[
            tuple[str, str, int], tuple[float, float, float] | None
        ] = {}
        self.declare_parameter(
            "cmd_vel_debug_enabled",
            self.navigation_debug_enabled and self.speed_profiles.debug_enabled,
        )
        self.declare_parameter(
            "cmd_vel_debug_throttle_seconds",
            self.speed_profiles.debug_throttle_seconds,
        )

        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.map_load_client = self.create_client(LoadMap, "/map_server/load_map")
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        # SLAM Toolbox exposes one toggle service for both directions. The
        # adapter state machine guarantees it is called only on MAPPING->PAUSED
        # or PAUSED->MAPPING, so the toggle remains deterministic.
        self.slam_pause_client = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.slam_save_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.slam_serialize_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )
        self.slam_deserialize_client = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map"
        )
        self.global_localization_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )
        self.nomotion_update_client = self.create_client(
            Empty, "/request_nomotion_update"
        )
        # Runtime profile changes wait for parameter-service responses. Keep
        # those clients and their retry timer in a re-entrant callback group so
        # the timer cannot block the very response callback it is waiting for,
        # or starve /map and AMCL callbacks during stack bootstrap.
        self.profile_callback_group = ReentrantCallbackGroup()
        self.scan_callback_group = MutuallyExclusiveCallbackGroup()
        # Costmap extraction can be CPU-heavy on the Pi. Keep it independent
        # from command/timer callbacks, and reserve a separate group for the
        # heartbeats that are allowed to revoke autonomous motion.
        self.costmap_callback_group = MutuallyExclusiveCallbackGroup()
        self.critical_status_callback_group = MutuallyExclusiveCallbackGroup()
        self.controller_parameter_client = self.create_client(
            SetParametersAtomically,
            "/controller_server/set_parameters_atomically",
            callback_group=self.profile_callback_group,
        )
        self.behavior_parameter_client = self.create_client(
            SetParametersAtomically,
            "/behavior_server/set_parameters_atomically",
            callback_group=self.profile_callback_group,
        )
        self.smoother_parameter_client = self.create_client(
            SetParametersAtomically,
            "/velocity_smoother/set_parameters_atomically",
            callback_group=self.profile_callback_group,
        )
        self.mapping_scan = self.create_publisher(
            LaserScan, "/scan_mapping", qos_profile_sensor_data
        )
        self.navigation_scan = self.create_publisher(
            LaserScan, "/scan_navigation", qos_profile_sensor_data
        )
        self.planning_scan = self.create_publisher(
            LaserScan, "/scan_planning", qos_profile_sensor_data
        )
        transient_map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.failed_segment_mask = self.create_publisher(
            OccupancyGrid, "/navigation/failed_segment_mask", transient_map_qos
        )
        self.initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        # Nav2 and recovery commands arrive on the private raw topic. Only the
        # selected Auto profile is applied here; the output remains the same
        # low-priority twist_mux input and still passes through the shared
        # smoother plus final motion-safety layer.
        self.navigation_velocity = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        self.localization_velocity = self.navigation_velocity
        self.create_subscription(
            Twist, "/cmd_vel_nav_raw", self._auto_velocity_callback, 1
        )
        self.create_subscription(
            Twist, "/cmd_vel_muxed", lambda message: self._record_pipeline("twist_mux", message), 1
        )
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", lambda message: self._record_pipeline("velocity_smoother", message), 1
        )
        self.create_subscription(
            Twist, "/cmd_vel", lambda message: self._record_pipeline("motion_safety", message), 1
        )
        # Map Server may publish its bootstrap map before this Python adapter
        # finishes starting. Transient-local QoS retrieves that retained map,
        # so the dedicated failed-segment StaticLayer always receives an
        # initial empty mask and does not remain uninitialized.
        self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, transient_map_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 5
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/slam_toolbox/pose",
            self._mapping_pose_callback,
            5,
        )
        self.create_subscription(
            # Nav2 AMCL publishes ParticleCloud as BEST_EFFORT. A default
            # RELIABLE reader is QoS-incompatible and silently receives no
            # hypotheses, which would leave uniqueness unavailable forever.
            ParticleCloud,
            "/particle_cloud",
            self._particle_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavigationPath, "/plan", self._path_callback, 1
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_callback, 1,
            callback_group=self.costmap_callback_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._global_costmap_callback,
            1,
        )
        self.create_subscription(
            LaserScan,
            "/scan/normalized",
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self.scan_callback_group,
        )
        self.create_subscription(
            String, "/sensors/time_status", self._sensor_time_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.create_subscription(
            String, "/safety/health", self._safety_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.safety_status_subscription = self.create_subscription(
            String, "/safety/status", self._safety_status_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.create_subscription(
            String, "/safety/stop_source", self._safety_source_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.create_subscription(
            Bool, "/safety/estop", self._estop_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.create_subscription(
            UInt8, "/safety/directional_mask", self._direction_mask_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self.create_subscription(
            Bool, "/safety/manual_takeover", self._manual_takeover_callback, 1
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._closing = threading.Event()
        self._server = self._open_server()
        self._server_thread = threading.Thread(
            target=self._serve, name="navigation-json-rpc", daemon=True
        )
        self._server_thread.start()
        self.create_timer(60.0, self._schedule_autosave)
        self.create_timer(0.2, self._update_pose)
        self.create_timer(0.2, self._localization_tick)
        self.create_timer(0.2, self._failed_segment_tick)
        self.create_timer(
            0.5,
            self._profile_runtime_tick,
            callback_group=self.profile_callback_group,
        )
        self.create_timer(0.2, self._cmd_vel_debug_tick)
        self.create_timer(0.05, self._segment_execution_tick)
        self.create_timer(0.20, self._dynamic_recovery_tick)
        self.create_timer(
            0.5,
            self._safety_subscription_watchdog_tick,
            callback_group=self.critical_status_callback_group,
        )
        self.get_logger().info(
            f"navigation adapter ready mode={self.mode} socket={self.socket_path} "
            f"auto_speed_mode={self.auto_speed_mode}"
        )
        self._nav_debug(
            "STARTUP",
            mode=self.mode,
            debug_log_path=str(self.navigation_debug_log.path),
            planning_static_match_tolerance=self.planning_static_match_tolerance,
        )

    def _nav_debug(self, event: str, **fields: Any) -> None:
        message = self.navigation_debug_log.event(event, **fields)
        if message:
            self.get_logger().info(message)

    def _set_state(self, state: str, reason: str) -> None:
        previous = self.current_state
        self.current_state = state
        if previous != state:
            self._nav_debug(
                "STATE",
                **{"from": previous, "to": state, "reason": reason},
            )

    @staticmethod
    def _parameter_request(values: dict[str, Any]) -> SetParametersAtomically.Request:
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        return request

    def _set_runtime_parameters(
        self,
        client: Any,
        node_name: str,
        values: dict[str, Any],
    ) -> None:
        if not client.wait_for_service(timeout_sec=1.0):
            raise AdapterError(
                "SPEED_PROFILE_UNAVAILABLE",
                f"{node_name} parameter service is unavailable",
            )
        response = self._wait(
            client.call_async(self._parameter_request(values)),
            3.0,
            "SPEED_PROFILE_TIMEOUT",
        )
        if not response.result.successful:
            raise AdapterError(
                "SPEED_PROFILE_REJECTED",
                f"{node_name} rejected speed profile: {response.result.reason}",
            )

    def _apply_speed_profile(self, mode: str) -> None:
        profile = self.speed_profiles.get(mode)
        previous_mode = self.applied_speed_mode
        applied: list[tuple[Any, str]] = []
        updates = [
            (
                self.controller_parameter_client,
                "controller_server",
                self.speed_profiles.controller_parameters(profile.mode),
            ),
            (
                self.behavior_parameter_client,
                "behavior_server",
                self.speed_profiles.behavior_parameters(profile.mode),
            ),
            (
                self.smoother_parameter_client,
                "velocity_smoother",
                self.speed_profiles.smoother_parameters(),
            ),
        ]
        try:
            for client, node_name, values in updates:
                self._set_runtime_parameters(client, node_name, values)
                applied.append((client, node_name))
        except AdapterError:
            # ROS has no transaction spanning multiple nodes. Roll already
            # updated nodes back to the last complete profile before exposing
            # the failure; the downstream smoother uses one invariant manual-
            # safe envelope for every profile.
            if previous_mode:
                previous = self.speed_profiles.get(previous_mode)
                rollback_values = {
                    "controller_server": self.speed_profiles.controller_parameters(
                        previous.mode
                    ),
                    "behavior_server": self.speed_profiles.behavior_parameters(
                        previous.mode
                    ),
                    "velocity_smoother": self.speed_profiles.smoother_parameters(),
                }
                for client, node_name in reversed(applied):
                    try:
                        self._set_runtime_parameters(
                            client,
                            node_name,
                            rollback_values[node_name],
                        )
                    except AdapterError as rollback_error:
                        self.get_logger().error(
                            f"speed profile rollback failed node={node_name}: {rollback_error}"
                        )
            raise
        self.applied_speed_mode = profile.mode
        self.profile_applied = True
        self.profile_apply_pending = False
        self.profile_apply_error = ""

    def _set_auto_speed_mode(self, requested_mode: object) -> dict[str, Any]:
        try:
            profile = self.speed_profiles.get(requested_mode)
        except SpeedProfileError as exc:
            raise AdapterError("INVALID_SPEED_MODE", str(exc)) from exc
        previous_mode = self.auto_speed_mode
        if self.mode == "NAVIGATION":
            try:
                self._apply_speed_profile(profile.mode)
            except AdapterError as exc:
                self.profile_apply_error = str(exc)
                self.auto_speed_mode = previous_mode
                raise
        else:
            # Mapping has no controller/behavior server. Persist the choice and
            # the next Navigation runtime applies it without restarting Nav2.
            self.profile_apply_pending = True
        try:
            self.speed_mode_store.save(profile.mode)
        except OSError as exc:
            if self.mode == "NAVIGATION" and previous_mode != profile.mode:
                try:
                    self._apply_speed_profile(previous_mode)
                except AdapterError as rollback_error:
                    self.get_logger().error(
                        f"speed profile persistence rollback failed: {rollback_error}"
                    )
            self.auto_speed_mode = previous_mode
            raise AdapterError(
                "SPEED_MODE_PERSIST_FAILED", f"Cannot persist speed mode: {exc}"
            ) from exc
        self.auto_speed_mode = profile.mode
        return {
            "status": "completed",
            "current_state": self.current_state,
            "mode": profile.mode,
            "profile": self._speed_profile_state(profile.mode),
            "state": self._state(),
        }

    def _profile_runtime_tick(self) -> None:
        if self.mode != "NAVIGATION" or not self.profile_apply_pending:
            return
        if not all(
            client.service_is_ready()
            for client in (
                self.controller_parameter_client,
                self.behavior_parameter_client,
                self.smoother_parameter_client,
            )
        ):
            return
        now = time.monotonic()
        if now - self.last_profile_apply_attempt < 1.0:
            return
        self.last_profile_apply_attempt = now
        try:
            self._apply_speed_profile(self.auto_speed_mode)
        except AdapterError as exc:
            message = str(exc)
            if message != self.profile_apply_error:
                self.get_logger().warning(f"Auto speed profile apply pending: {message}")
            self.profile_apply_error = message
        else:
            self.get_logger().info(
                f"Auto speed profile applied mode={self.auto_speed_mode} without Nav2 restart"
            )

    def _auto_velocity_callback(self, message: Twist) -> None:
        if self.motion_owner != "NAVIGATION":
            # Late Nav2/recovery commands must not compete with localization
            # or continue after a sensor fault/cancel on the shared mux input.
            return
        now = time.monotonic()
        self.last_controller_requested = (
            float(message.linear.x), float(message.angular.z), now
        )
        self._record_pipeline("controller_requested", message, now=now)
        if self.execution_phase == "TURN":
            # TURN is published directly by the segment state machine. A late
            # FollowPath callback must not overwrite that command with either
            # curvature or an apparent zero from an already-finished path.
            return
        profile = self.speed_profiles.get(self.auto_speed_mode)
        straight_phase = self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}
        linear, angular, reasons = self.profile_limiter.apply(
            message.linear.x,
            0.0 if straight_phase else message.angular.z,
            profile,
            now,
        )
        if straight_phase:
            active = self.active_segment
            pose = self.pose
            pose_fresh = (
                pose is not None
                and now - self.execution_pose_monotonic
                <= self.execution_pose_freshness
            )
            if active is None or not pose_fresh:
                linear = 0.0
                angular = 0.0
                reasons = tuple(reasons) + ("EXECUTION_POSE_STALE",)
            else:
                progress = straight_segment_progress(
                    active.effective_start,
                    active.endpoint,
                    pose,
                    overshoot_epsilon=self.straight_overshoot_epsilon,
                )
                hard_cross_track = max(
                    self.straight_hard_cross_track,
                    (self.saved_map.resolution if self.saved_map is not None else 0.0)
                    + self.planning_footprint_padding,
                )
                decision = straight_heading_lock(
                    active,
                    pose,
                    heading_kp=self.straight_heading_kp,
                    cross_track_kp=self.straight_cross_track_kp,
                    maximum_angular=self.straight_max_angular_correction,
                    heading_deadband=self.straight_heading_deadband,
                    cross_track_deadband=self.straight_cross_track_deadband,
                    hard_heading_error=max(
                        self.straight_hard_heading_error,
                        4.0 * self.execution_turn_tolerance,
                    ),
                    hard_cross_track=hard_cross_track,
                )
                angular = decision.angular
                if not decision.forward_allowed:
                    linear = 0.0
                    angular = 0.0
                    self.straight_recovery_requested = decision.reason
                    reasons = tuple(reasons) + (decision.reason,)
                elif (
                    progress.passed_endpoint
                    or progress.remaining_longitudinal <= 0.0
                    or progress.endpoint_distance <= self.straight_endpoint_tolerance
                ):
                    linear = 0.0
                    angular = 0.0
                    reasons = tuple(reasons) + ("GEOMETRIC_ENDPOINT_STOP",)
                else:
                    braking_limit = endpoint_braking_speed_limit(
                        progress.remaining_longitudinal,
                        deceleration=profile.linear_decel,
                        reaction_time=self.endpoint_reaction_time,
                    )
                    if linear > braking_limit:
                        reasons = tuple(reasons) + ("ENDPOINT_BRAKING_LIMIT",)
                    if active.motion_direction < 0:
                        reverse_limit = min(
                            profile.backup_speed,
                            braking_limit,
                        )
                        if linear < -reverse_limit:
                            reasons = tuple(reasons) + (
                                "REVERSE_BRAKING_OR_SPEED_LIMIT",
                            )
                        if linear > 0.0:
                            reasons = tuple(reasons) + (
                                "REVERSE_DIRECTION_AUTHORITY",
                            )
                        linear = min(0.0, max(linear, -reverse_limit))
                    else:
                        if linear < 0.0:
                            reasons = tuple(reasons) + (
                                "FORWARD_DIRECTION_AUTHORITY",
                            )
                        linear = max(0.0, min(linear, braking_limit))
                    direction_mask = 2 if active.motion_direction < 0 else 1
                    if (
                        self._atomic_safety_fresh(now)
                        and self.safety_direction_mask & direction_mask
                    ):
                        linear = 0.0
                        angular = 0.0
                        reasons = tuple(reasons) + (
                            "DIRECTIONAL_SAFETY_BLOCK",
                        )
                # RPP remains the linear/collision proposal only. Its changing
                # carrot curvature is intentionally never steering authority.
                if abs(float(message.angular.z) - angular) > 0.02:
                    reasons = tuple(reasons) + ("STRAIGHT_FIXED_HEADING_AUTHORITY",)
            if self.execution_phase == "NARROW_STRAIGHT":
                narrow_speed = self.speed_profiles.get("SLOW").linear_max
                linear = max(-narrow_speed, min(narrow_speed, linear))
        output = Twist()
        output.linear.x = linear
        output.linear.y = message.linear.y
        output.angular.z = angular
        self.profile_clamp_reasons = reasons
        self._record_pipeline("auto_profile", output, now=now)
        self.navigation_velocity.publish(output)
        self.profile_callback_latency_ms = round(
            (time.monotonic() - now) * 1000.0, 3
        )
        self._update_motion_metrics(message, output, now)

    def _update_motion_metrics(
        self,
        requested: Twist,
        output: Twist,
        now: float,
    ) -> None:
        profile = self.speed_profiles.get(self.auto_speed_mode)
        slowing = (
            requested.linear.x > 0.02
            and requested.linear.x < profile.linear_max * 0.85
        )
        if slowing and not self.obstacle_slowdown_active:
            self.last_slowdown_obstacle_distance = self.nearest_forward_obstacle
        self.obstacle_slowdown_active = slowing

        rotating = abs(requested.linear.x) < 0.02 and abs(requested.angular.z) > 0.08
        if rotating:
            if not self.rotation_metric_active:
                self.rotation_metric_active = True
                self.rotation_metric_started = now
                self.rotation_metric_last_sample = now
                self.rotation_metric_integrated_angle = 0.0
                self.rotation_metric_peak_requested = 0.0
                self.rotation_metric_peak_final = 0.0
            delta = max(0.0, min(0.25, now - self.rotation_metric_last_sample))
            self.rotation_metric_integrated_angle += abs(output.angular.z) * delta
            self.rotation_metric_last_sample = now
            self.rotation_metric_peak_requested = max(
                self.rotation_metric_peak_requested,
                abs(float(requested.angular.z)),
            )
        elif self.rotation_metric_active:
            self.last_rotation_metrics = {
                "duration_seconds": round(now - self.rotation_metric_started, 3),
                "angle_degrees": round(
                    math.degrees(self.rotation_metric_integrated_angle), 1
                ),
                "peak_requested_angular": round(
                    self.rotation_metric_peak_requested, 3
                ),
                "peak_final_angular": round(self.rotation_metric_peak_final, 3),
            }
            self.rotation_metric_active = False

    def _record_pipeline(
        self,
        stage: str,
        message: Twist,
        *,
        now: float | None = None,
    ) -> None:
        if not self.navigation_debug_enabled:
            return
        self.pipeline_samples[stage] = (
            float(message.linear.x),
            float(message.angular.z),
            time.monotonic() if now is None else now,
        )
        if stage == "motion_safety" and self.rotation_metric_active:
            self.rotation_metric_peak_final = max(
                self.rotation_metric_peak_final,
                abs(float(message.angular.z)),
            )

    @staticmethod
    def _pipeline_differs(
        left: tuple[float, float, float] | None,
        right: tuple[float, float, float] | None,
        tolerance: float = 0.02,
    ) -> bool:
        return bool(
            left
            and right
            and (
                abs(left[0] - right[0]) > tolerance
                or abs(left[1] - right[1]) > tolerance
            )
        )

    def _cmd_vel_debug_tick(self) -> None:
        if (
            not self.navigation_debug_enabled
            or not bool(self.get_parameter("cmd_vel_debug_enabled").value)
        ):
            return
        now = time.monotonic()
        throttle = max(
            0.2,
            float(self.get_parameter("cmd_vel_debug_throttle_seconds").value),
        )
        if now - self.last_pipeline_log_monotonic < throttle:
            return
        requested = self.pipeline_samples.get("controller_requested")
        if requested is None or now - requested[2] > 0.6:
            return
        profile_output = self.pipeline_samples.get("auto_profile")
        muxed = self.pipeline_samples.get("twist_mux")
        smoothed = self.pipeline_samples.get("velocity_smoother")
        final = self.pipeline_samples.get("motion_safety")
        reasons = list(self.profile_clamp_reasons)
        if self._pipeline_differs(profile_output, muxed):
            reasons.append("TWIST_MUX_OTHER_SOURCE")
        if self._pipeline_differs(muxed, smoothed):
            reasons.append("SMOOTHER_ACCEL_LIMIT")
        if self._pipeline_differs(smoothed, final):
            reasons.append(f"MOTION_SAFETY:{self.safety_health}")

        def values(sample: tuple[float, float, float] | None) -> str:
            return "unavailable" if sample is None else f"linear={sample[0]:.3f} angular={sample[1]:.3f}"

        pure_rotation = abs(requested[0]) < 0.02 and abs(requested[1]) > 0.08
        event = "ROTATE" if pure_rotation else "CMD_VEL"
        target_heading, heading_error = self._current_path_heading()
        self._nav_debug(
            event,
            profile=self.auto_speed_mode,
            rotation_mode="PURE_ROTATION" if pure_rotation else "PATH_FOLLOWING",
            heading_current=(self.pose or {}).get("yaw"),
            heading_target=target_heading,
            heading_error_deg=(
                None if heading_error is None else math.degrees(heading_error)
            ),
            controller_requested=values(requested),
            profile_limited=values(profile_output),
            twist_mux=values(muxed),
            velocity_smoother=values(smoothed),
            final_cmd=values(final),
            clamp_reason=list(dict.fromkeys(reasons)) or ["NONE"],
        )
        self.last_pipeline_log_monotonic = now

    def _current_path_heading(self) -> tuple[float | None, float | None]:
        if self.pose is None:
            return None, None
        current = float(self.pose.get("yaw", 0.0))
        if (
            self.execution_phase in {
                "STRAIGHT", "NARROW_STRAIGHT", "DISPATCHING_STRAIGHT",
            }
            and self.active_segment is not None
        ):
            target = self.active_segment.fixed_heading
            return target, self._yaw_delta(target, current)
        if self.execution_phase in {"TURN", "TURN_SETTLING"}:
            target = self.execution_target_heading
            return target, self._yaw_delta(target, current)
        pose_x = float(self.pose.get("x", 0.0))
        pose_y = float(self.pose.get("y", 0.0))
        # During a dynamic wait active_segment is intentionally cleared.  The
        # safety corridor must still be projected along the blocked route,
        # not along the chassis' temporary waiting orientation.  Otherwise a
        # sideways scan can look clear, resume the old segment, and encounter
        # the same obstacle immediately after turning back onto it.
        if (
            self.current_state in {
                "WAIT_FOR_DYNAMIC_CLEAR",
                "WAITING_FOR_DYNAMIC_CLEAR",
                "DYNAMIC_REPLAN",
            }
            and len(self.dynamic_blocked_route) >= 2
        ):
            for index, point in enumerate(self.dynamic_blocked_route[1:]):
                delta_x = float(point["x"]) - pose_x
                delta_y = float(point["y"]) - pose_y
                if math.hypot(delta_x, delta_y) < 0.08:
                    continue
                target = math.atan2(delta_y, delta_x)
                direction = (
                    self.dynamic_blocked_segment_directions[index]
                    if index < len(self.dynamic_blocked_segment_directions)
                    else 1
                )
                if direction < 0:
                    target = self._yaw_delta(target + math.pi, 0.0)
                return target, self._yaw_delta(target, current)
        for point in self.latest_global_path:
            delta_x = float(point["x"]) - pose_x
            delta_y = float(point["y"]) - pose_y
            if math.hypot(delta_x, delta_y) < 0.08:
                continue
            target = math.atan2(delta_y, delta_x)
            return target, self._yaw_delta(target, current)
        return None, None

    def _speed_profile_state(self, mode: str | None = None) -> dict[str, Any]:
        profile = self.speed_profiles.get(mode or self.auto_speed_mode)
        return {
            "mode": profile.mode,
            "linear_max": profile.linear_max,
            "angular_max": profile.angular_max,
            "linear_accel": profile.linear_accel,
            "angular_accel": profile.angular_accel,
            "regulated_min_speed": profile.regulated_min_speed,
            "collision_horizon": profile.collision_horizon,
            "replan_frequency": profile.replan_frequency,
            "recovery_wait": profile.recovery_wait,
            "backup_distance": profile.backup_distance,
            "backup_speed": profile.backup_speed,
            "runtime_applied": self.applied_speed_mode == profile.mode,
            "apply_error": self.profile_apply_error,
        }

    @staticmethod
    def _finite_metric(value: float) -> float | None:
        return round(value, 3) if math.isfinite(value) else None

    def _navigation_metrics(self) -> dict[str, Any]:
        return {
            "replan_frequency_hz": self._measured_replan_frequency(),
            "nearest_forward_obstacle_m": self._finite_metric(
                self.nearest_forward_obstacle
            ),
            "nearest_left_obstacle_m": self._finite_metric(
                self.nearest_left_obstacle
            ),
            "nearest_right_obstacle_m": self._finite_metric(
                self.nearest_right_obstacle
            ),
            "nearest_rotation_obstacle_m": self._finite_metric(
                self.nearest_rotation_obstacle
            ),
            "slowdown_obstacle_distance_m": self._finite_metric(
                self.last_slowdown_obstacle_distance
            ),
            "replan_obstacle_distance_m": self._finite_metric(
                self.last_replan_obstacle_distance
            ),
            "profile_callback_latency_ms": self.profile_callback_latency_ms,
            "scan_callback_latency_ms": self.scan_callback_latency_ms,
            "localization_callback_latency_ms": self.localization_callback_latency_ms,
            "planner_latency_ms": self.planner_latency_ms,
            "motion_owner": self.motion_owner,
            "last_rotation": dict(self.last_rotation_metrics),
            "last_planning_failure": dict(self.last_planning_failure),
            "scan_filter": dict(self.last_scan_filter_stats),
        }

    @staticmethod
    def _age_milliseconds(last_monotonic: float) -> float | None:
        if last_monotonic <= 0:
            return None
        return round(max(0.0, time.monotonic() - last_monotonic) * 1000, 1)

    def _localization_diagnostics(self) -> dict[str, Any]:
        covariance_xy = 0.0
        covariance_yaw = 0.0
        if len(self.last_amcl_covariance) >= 36:
            covariance_xy = max(0.0, float(self.last_amcl_covariance[0])) + max(
                0.0, float(self.last_amcl_covariance[7])
            )
            covariance_yaw = max(0.0, float(self.last_amcl_covariance[35]))
        metrics = self.pose_stability_metrics
        required_heading_bins, required_heading_span = (
            self._global_heading_requirement()
        )
        return {
            "localization_state": self.localization_state,
            "localization_confidence": self.localization_confidence,
            "amcl_covariance_xy": round(covariance_xy, 5),
            "amcl_covariance_yaw": round(covariance_yaw, 5),
            "pose_stability": {
                "passed": self._pose_is_stable(),
                "samples": metrics.sample_count,
                "duration_seconds": round(metrics.duration_seconds, 3),
                "xy_spread": self._finite_metric(metrics.xy_spread),
                "median_deviation": self._finite_metric(metrics.median_deviation),
                "yaw_circular_variance": self._finite_metric(
                    metrics.yaw_circular_variance
                ),
                "yaw_spread": self._finite_metric(metrics.yaw_spread),
            },
            "scan_map_score": self.scan_map_score,
            "scan_map_threshold": self.scan_map_threshold,
            "scan_map_score_required": self._required_localization_scan_score(),
            "scan_map_matched_beams": self.scan_map_matched_beams,
            "scan_map_valid_beams": self.scan_map_valid_beams,
            "scan_map_residual_beams": self.scan_map_residual_beams,
            "median_endpoint_residual_m": self._finite_metric(
                self.scan_map_median_residual
            ),
            "p90_endpoint_residual_m": self._finite_metric(
                self.scan_map_p90_residual
            ),
            "mean_endpoint_residual_m": self._finite_metric(
                self.scan_map_mean_residual
            ),
            "raycast_comparable_beams": self.raycast_comparable_beams,
            "raycast_static_matches": self.raycast_static_matches,
            "raycast_dynamic_occlusions": self.raycast_dynamic_occlusions,
            "raycast_map_contradictions": self.raycast_map_contradictions,
            "raycast_inconclusive_map_hits": (
                self.raycast_inconclusive_map_hits
            ),
            "raycast_static_match_ratio": self.raycast_static_match_ratio,
            "raycast_dynamic_occlusion_ratio": (
                self.raycast_dynamic_occlusion_ratio
            ),
            "raycast_contradiction_ratio": self.raycast_contradiction_ratio,
            # Compatibility aliases for existing telemetry consumers.
            "raycast_matches": self.raycast_matched_beams,
            "raycast_match_ratio": self.raycast_match_ratio,
            "raycast_median_error_m": self._finite_metric(
                self.raycast_median_error
            ),
            "raycast_p90_error_m": self._finite_metric(
                self.raycast_p90_error
            ),
            "ready_evidence_hold_ms": (
                None if self.ready_evidence_since is None else round(
                    max(0.0, time.monotonic() - self.ready_evidence_since) * 1000.0,
                    1,
                )
            ),
            "global_observation": {
                "requires_rotation": self.global_search_requires_rotation,
                "untrusted_global": self.global_search_untrusted,
                "candidate_class": (
                    "AMBIGUOUS"
                    if self.stationary_global_candidate_ambiguous
                    else "STRONG"
                ),
                "accumulated_rotation_degrees": round(
                    math.degrees(self.rotation_angle), 1
                ),
                "minimum_rotation_degrees": round(
                    math.degrees(self.global_observation_minimum_rotation), 1
                ),
                "heading_bins_observed": list(self.localization_heading_bins),
                "heading_bin_count": self.global_heading_bin_count,
                "minimum_heading_bins": required_heading_bins,
                "heading_span_degrees": round(
                    math.degrees(self.localization_heading_span), 1
                ),
                "minimum_heading_span_degrees": round(
                    math.degrees(required_heading_span), 1
                ),
                "sufficient": (
                    not self.global_search_untrusted
                    or (
                        self.particle_uniqueness.accepted
                        and self.global_scan_uniqueness.accepted
                    )
                ),
                "heading_diversity_sufficient": (
                    self._global_heading_diversity_ready()
                ),
            },
            "consensus": {
                "accepted": self.localization_consensus.accepted,
                "reason": self.localization_consensus.reason,
                "window_frames": self.localization_consensus.total_frames,
                "required_frames": self.localization_consensus.required_frames,
                "passing_frames": self.localization_consensus.passing_frames,
                "agreeing_frames": self.localization_consensus.agreeing_frames,
            },
            "uniqueness": {
                "accepted": self.particle_uniqueness.accepted,
                "reason": self.particle_uniqueness.reason,
                "particles": self.particle_uniqueness.particle_count,
                "clusters": self.particle_uniqueness.cluster_count,
                "best_weight": self.particle_uniqueness.best_weight,
                "alternative_weight": self.particle_uniqueness.alternative_weight,
                "dominance_ratio": self._finite_metric(
                    self.particle_uniqueness.dominance_ratio
                ),
                "age_ms": self._age_milliseconds(
                    self.last_particle_cloud_monotonic
                ),
            },
            "global_scan_uniqueness": {
                "accepted": self.global_scan_uniqueness.accepted,
                "reason": self.global_scan_uniqueness.reason,
                "in_progress": self.global_scan_uniqueness_in_progress,
                "evaluated_candidates": (
                    self.global_scan_uniqueness.evaluated_candidates
                ),
                "usable_beams": self.global_scan_uniqueness.usable_beams,
                "best_score": self.global_scan_uniqueness.best_score,
                "alternative_score": (
                    self.global_scan_uniqueness.alternative_score
                ),
                "score_margin": self.global_scan_uniqueness.score_margin,
                "score_ratio": self._finite_metric(
                    self.global_scan_uniqueness.score_ratio
                ),
                "best_pose": {
                    "x": self.global_scan_uniqueness.best_x,
                    "y": self.global_scan_uniqueness.best_y,
                    "yaw": self.global_scan_uniqueness.best_yaw,
                },
                "alternative_pose": {
                    "x": self.global_scan_uniqueness.alternative_x,
                    "y": self.global_scan_uniqueness.alternative_y,
                    "yaw": self.global_scan_uniqueness.alternative_yaw,
                },
            },
            "attempt_id": self.localization_attempt_id,
            "approximate_hint_allowed": self.approximate_hint_allowed,
            "scan_age_ms": self._age_milliseconds(self.last_scan_monotonic),
            "amcl_pose_age_ms": self._age_milliseconds(self.last_amcl_monotonic),
            "odom_age_ms": self._sensor_entry("odom").get("arrival_age_ms"),
            "imu_age_ms": self._sensor_entry("imu").get("arrival_age_ms"),
            "tf_age_ms": self._age_milliseconds(self.last_map_tf_monotonic),
            "sensor_time": dict(self.sensor_time_status),
        }

    def _open_server(self) -> socket.socket:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self.socket_path))
        socket_gid = int(os.getenv("NAVIGATION_SOCKET_GID", "1000"))
        os.chown(self.socket_path, -1, socket_gid)
        self.socket_path.chmod(0o660)
        server.listen(8)
        server.settimeout(0.5)
        return server

    def _serve(self) -> None:
        while not self._closing.is_set():
            try:
                connection, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                connection.settimeout(30)
                try:
                    raw = b""
                    while b"\n" not in raw and len(raw) <= 1_048_576:
                        chunk = connection.recv(65_536)
                        if not chunk:
                            break
                        raw += chunk
                    request_line = raw.split(b"\n", 1)[0]
                    if not request_line:
                        self.get_logger().warning(
                            "adapter client disconnected before sending a request"
                        )
                        continue
                    request = json.loads(request_line)
                    response = self._dispatch(
                        str(request["command"]), dict(request.get("payload") or {})
                    )
                except AdapterError as exc:
                    response = {
                        "status": "rejected",
                        "current_state": self.current_state,
                        "error_code": exc.code,
                        "error_message": str(exc),
                        "state": self._state(),
                    }
                except Exception as exc:
                    # RcutilsLogger in ROS 2 Humble has no ``exception`` method.
                    # Logging an invalid/abandoned request must never terminate
                    # the only JSON-RPC server thread.
                    self.get_logger().error(
                        f"adapter request failed: {exc}\n{traceback.format_exc()}"
                    )
                    response = {
                        "status": "rejected",
                        "current_state": "FAULT",
                        "error_code": "ADAPTER_ERROR",
                        "error_message": str(exc),
                        "state": self._state(),
                    }
                try:
                    connection.sendall(
                        json.dumps(response, separators=(",", ":")).encode() + b"\n"
                    )
                except OSError as exc:
                    # The edge agent may reconnect or time out while a long
                    # pose-graph save is still running. One abandoned client
                    # must not terminate the adapter's only RPC server thread.
                    self.get_logger().warning(f"adapter client disconnected: {exc}")

    def _state(self) -> dict[str, Any]:
        with self.state_lock:
            scan_time = self._sensor_entry("scan")
            odom_time = self._sensor_entry("odom")
            imu_time = self._sensor_entry("imu")
            (
                sensor_time_healthy,
                current_sensor_time_reason,
                current_sensor_time_diagnostics,
            ) = self._critical_sensor_time_status()
            navigation_runtime_ready = (
                self.mode == "NAVIGATION"
                and self.map_load_client.service_is_ready()
                and self.compute_path_client.server_is_ready()
                and self.follow_path_client.server_is_ready()
            )
            if self.mode == "MAPPING":
                nav2_state = "MAPPING"
            elif navigation_runtime_ready:
                nav2_state = "READY"
            elif self.current_state == "FAULT":
                nav2_state = "FAULT"
            else:
                nav2_state = "STARTING"
            return {
                "state": self.current_state,
                "mode": self.mode,
                "map_id": self.map_id,
                "map_version": self.map_version,
                "localized": self.localized,
                "localization_verification_version": (
                    2 if self.localized and self.localization_state == "READY" else 0
                ),
                "localization_state": self.localization_state,
                "localization_confidence": self.localization_confidence,
                "localization_rotation_authorized": self.localization_rotation_authorized,
                "nav2": nav2_state,
                "feedback": dict(self.latest_feedback),
                "scan_fresh": (
                    time.monotonic() - self.last_scan_monotonic <= 0.30
                    and sensor_time_healthy
                ),
                "sensor_clock_state": self.sensor_time_status.get(
                    "clock_state", "CLOCK_SYNCING"
                ),
                "sensor_time_healthy": sensor_time_healthy,
                "sensor_time_failure_reason": (
                    current_sensor_time_reason or self.sensor_time_failure_reason
                ),
                "sensor_time_diagnostics": (
                    current_sensor_time_diagnostics
                    if current_sensor_time_reason
                    else dict(self.sensor_time_failure_diagnostics)
                ),
                "scan_clock_skew_seconds": round(self.scan_clock_skew_seconds, 3),
                "scan_arrival_fresh": bool(scan_time.get("arrival_fresh")),
                "scan_timestamp_valid": bool(scan_time.get("timestamp_valid")),
                "odom_arrival_fresh": bool(odom_time.get("arrival_fresh")),
                "odom_timestamp_valid": bool(odom_time.get("timestamp_valid")),
                "odom_clock_skew_seconds": round(
                    float(odom_time.get("clock_skew_ms", 0.0)) / 1000.0, 3
                ),
                "imu_arrival_fresh": bool(imu_time.get("arrival_fresh")),
                "imu_timestamp_valid": bool(imu_time.get("timestamp_valid")),
                "imu_clock_skew_seconds": round(
                    float(imu_time.get("clock_skew_ms", 0.0)) / 1000.0, 3
                ),
                "odometry_ready": self.tf_buffer.can_transform(
                    "odom", "base_footprint", Time()
                ) and bool(odom_time.get("timestamp_valid")),
                # Validate the complete navigation chain. Checking only
                # base_link -> laser_frame hid a missing
                # base_footprint -> base_link transform and let AMCL discard
                # every scan while the container still reported healthy.
                "lidar_tf_ready": self.tf_buffer.can_transform(
                    "base_footprint", "laser_frame", Time()
                ),
                "safety": "HEALTHY" if self.safety_health.startswith("HEALTHY") else self.safety_health,
                "estop": self.estop_active,
                "mission_id": self.current_mission_id,
                "auto_speed_mode": self.auto_speed_mode,
                "auto_speed_profile": self._speed_profile_state(),
                "replan_frequency_hz": self._measured_replan_frequency(),
                "navigation_metrics": self._navigation_metrics(),
                "localization_diagnostics": self._localization_diagnostics(),
                "footprint": list(self.footprint),
                "corridor": {
                    "classification": self.latest_corridor.classification,
                    "reason": self.latest_corridor.reason,
                    "available_width": self._finite_metric(
                        self.latest_corridor.available_width
                    ),
                    "hard_required_width": self.latest_corridor.hard_required_width,
                    "auto_required_width": self.latest_corridor.auto_required_width,
                    "left_clearance": self._finite_metric(
                        self.latest_corridor.left_clearance
                    ),
                    "right_clearance": self._finite_metric(
                        self.latest_corridor.right_clearance
                    ),
                    "front_clearance": self._finite_metric(
                        self.latest_corridor.front_clearance
                    ),
                    "can_go_straight": self.latest_corridor.can_go_straight,
                    "can_rotate": self.latest_corridor.can_rotate,
                },
                "route_candidates": list(self.route_candidates.values()),
                "selected_route_id": self.selected_route_id,
                "manual_handoff_reason": self.manual_handoff_reason,
                # system.status is polled several times per second over a
                # newline-framed Unix socket and then forwarded by WebSocket.
                # Keep the complete history internally, but uniformly sample
                # the latest 200 points so this hot-path payload stays well
                # below the 64 KiB transport boundary.
                "trajectory": self._status_trajectory(),
                "mapping": {
                    "state": self.current_state,
                    "scanHealthy": (
                        time.monotonic() - self.last_scan_monotonic <= 0.30
                        and sensor_time_healthy
                    ),
                    "odomHealthy": self.tf_buffer.can_transform("odom", "base_footprint", Time()) and bool(odom_time.get("timestamp_valid")),
                    "tfHealthy": self.tf_buffer.can_transform("base_footprint", "laser_frame", Time()),
                    "slamHealthy": self.slam_save_client.service_is_ready() if self.mode == "MAPPING" else False,
                    "elapsedSeconds": (
                        round(time.monotonic() - self.mapping_started_monotonic)
                        if self.mapping_started_monotonic else 0
                    ),
                    "relocalization": dict(
                        self.mapping_relocalization_diagnostics
                    ),
                } if self.mode == "MAPPING" else None,
            }

    def _status_trajectory(self, maximum_points: int = 40) -> list[dict[str, Any]]:
        values = self.odometry_trajectory[-200:]
        limit = max(2, int(maximum_points))
        if len(values) <= limit:
            return list(values)
        last_index = len(values) - 1
        indexes = {
            round(index * last_index / (limit - 1))
            for index in range(limit)
        }
        return [values[index] for index in sorted(indexes)]

    @staticmethod
    def _wait(future: Any, timeout: float, error_code: str) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise AdapterError(error_code, "ROS operation timed out")
        exception = future.exception()
        if exception is not None:
            raise AdapterError(error_code, str(exception))
        return future.result()

    def _dispatch(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        expected_state = str(payload.get("expected_state", "")).upper()
        if command != "system.status":
            self._nav_debug(
                "COMMAND",
                command=command,
                expected_state=expected_state or None,
                current_state=self.current_state,
                map_id=str(payload.get("map_id") or self.map_id),
                map_version=(
                    payload.get("version") or payload.get("map_version")
                    or self.map_version
                ),
                request_id=(
                    payload.get("request_id") or payload.get("command_id")
                    or payload.get("message_id")
                ),
            )
        restartable_mapping_start = (
            command == "mapping.start"
            and expected_state == "IDLE"
            and self.current_state in {"CANCELED", "FINISHED", "FAULT", "MAPPING_ERROR"}
        )
        navigation_terminal_states = {
            "READY", "SUCCEEDED", "ARRIVED", "CANCELED", "CANCELLED",
            "FAILED", "BLOCKED",
        }
        restartable_navigation_plan = (
            command == "navigation.compute_path"
            and expected_state in navigation_terminal_states
            and self.current_state in navigation_terminal_states
        )
        unconditional_safety_command = command in {
            "navigation.cancel", "navigation.speed_mode", "map.deactivate"
        }
        if (
            expected_state
            and expected_state != self.current_state
            and not restartable_mapping_start
            and not restartable_navigation_plan
            and not unconditional_safety_command
        ):
            self._nav_debug(
                "COMMAND_REJECTED",
                command=command,
                expected_state=expected_state,
                current_state=self.current_state,
                map_id=str(payload.get("map_id") or self.map_id),
                map_version=(
                    payload.get("version") or payload.get("map_version")
                    or self.map_version
                ),
                request_id=(
                    payload.get("request_id") or payload.get("command_id")
                    or payload.get("message_id")
                ),
                reason="STATE_CONFLICT",
            )
            raise AdapterError(
                "STATE_CONFLICT",
                f"Expected {expected_state}, robot is {self.current_state}",
            )
        if command == "map.load":
            return self._load_map(payload)
        if command == "map.deactivate":
            return self._deactivate_map()
        if command == "map.set_initial_pose":
            return self._set_initial_pose(payload)
        if command == "map.relocalize":
            self._validate_command_map(payload)
            if self.current_goal_handle is not None:
                raise AdapterError(
                    "NAVIGATION_ACTIVE",
                    "Cancel navigation before localization verification",
                )
            allow_rotation = bool(payload.get("allow_rotation", False))
            force_global = bool(payload.get("force_global", False))
            self._nav_debug(
                "RELOCALIZE",
                reason="user_force_rescan" if force_global else "auto_pose_verification",
                force_global=force_global,
                rotation_allowed=allow_rotation,
                current_localization_state=self.localization_state,
            )
            if force_global:
                if not allow_rotation:
                    raise AdapterError(
                        "ROTATION_AUTHORIZATION_REQUIRED",
                        "Fresh global localization requires rotation authorization",
                    )
                if self.localization_state in {
                    "PASSIVE_LOCALIZING", "CANDIDATE", "VERIFYING",
                    "AMBIGUOUS", "LOCALIZING_GLOBAL",
                    "LOCALIZING_ROTATING", "LOCALIZING_SETTLING",
                }:
                    # The HTTP command is acknowledged before localization is
                    # complete, so an impatient retry can arrive while the
                    # same scan is still progressing. Authorize rotation but
                    # never erase AMCL particles or accumulated heading twice.
                    self.localization_rotation_authorized = True
                    return {
                        "status": "accepted",
                        "current_state": self.current_state,
                        "state": self._state(),
                    }
                # The route is stale once a rescan starts, but a verified
                # map<-odom anchor is stronger than an unbounded particle
                # reset. Recheck that narrow prior first; only its bounded
                # rejection may fall back to the authorized global search.
                self.paused_goal = None
                self.current_mission_id = ""
                self._clear_active_navigation_mission("operator_force_rescan")
                if self.latest_global_path:
                    self.latest_global_path = []
                    self.visualization_revision += 1
                self.localization_rotation_authorized = True
                odometry_pose = self._odometry_predicted_map_pose()
                if odometry_pose is not None:
                    self._begin_odometry_prior(
                        odometry_pose,
                        rotation_was_authorized=True,
                    )
                else:
                    self.localization_started_monotonic = time.monotonic()
                    self._start_global_localization()
                return {
                    "status": "accepted",
                    "current_state": self.current_state,
                    "state": self._state(),
                }
            elif self.localized and self.localization_state == "READY":
                # Initial convergence already passed the strict stationary
                # gates. READY is continuously maintained with lower-threshold
                # AMCL, scan, TF and sensor-time evidence, so reuse it without
                # destroying the particle cloud.
                return {
                    "status": "completed",
                    "current_state": "READY",
                    "state": self._state(),
                }
            elif self.localization_state in {
                "LOCALIZATION_INITIALIZING", "LOCALIZING_LAST_POSE",
                "LOCALIZING_APPROXIMATE_POSE", "VERIFYING",
            }:
                self._nav_debug(
                    "LOCALIZATION",
                    state=self.localization_state,
                    action="AUTO_READY_REUSE_UNAVAILABLE",
                    reason=self._localization_rejection_reason(time.monotonic()),
                )
                # Preserve a recent-pose/no-motion verification already in
                # progress. Auto Go only authorizes its later global fallback.
                self.localization_rotation_authorized = (
                    self.localization_rotation_authorized or allow_rotation
                )
            elif self.last_amcl_pose is not None:
                self._nav_debug(
                    "LOCALIZATION",
                    state=self.localization_state,
                    action="AUTO_READY_REUSE_UNAVAILABLE",
                    reason=self._localization_rejection_reason(time.monotonic()),
                )
                # Even when Auto Go authorizes a later rotation, first verify
                # the existing AMCL cloud without moving the chassis.
                self._begin_localization_verification(
                    allow_rotation=allow_rotation
                )
            else:
                self._nav_debug(
                    "LOCALIZATION",
                    state=self.localization_state,
                    action="AUTO_READY_REUSE_UNAVAILABLE",
                    reason=self._localization_rejection_reason(time.monotonic()),
                )
                self.localization_rotation_authorized = allow_rotation
                self.localization_started_monotonic = time.monotonic()
                self._start_global_localization()
            return {
                "status": "accepted",
                "current_state": self.current_state,
                "state": self._state(),
            }
        if command == "navigation.compute_path":
            return self._compute_path(payload)
        if command in {"navigation.start", "navigation.goal"}:
            self.current_mission_id = str(
                payload.get("mission_id") or payload.get("route_id") or ""
            )
            goal = dict(payload.get("goal") or {})
            if not goal and payload.get("points"):
                goal = dict(payload["points"][-1])
                goal["yaw"] = 0.0
            return self._navigate(goal, payload)
        if command == "navigation.pause":
            self._validate_command_map(payload)
            return self._pause_navigation()
        if command == "navigation.manual_handoff":
            self._validate_command_map(payload)
            return self._manual_handoff(str(payload.get("reason") or "NARROW_PATH"))
        if command == "navigation.alternatives":
            self._validate_command_map(payload)
            return self._compute_alternative_routes()
        if command == "navigation.select_route":
            self._validate_command_map(payload)
            return self._start_selected_route(str(payload.get("route_id") or ""))
        if command == "navigation.route_selection_back":
            self._validate_command_map(payload)
            target = self.route_selection_return_state
            self.route_candidates = {}
            self.selected_route_id = ""
            if target == "WAITING_FOR_DYNAMIC_CLEAR":
                self.dynamic_recovery_state = "WAITING_OLD_ROUTE_ONLY"
                self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
                self.latest_feedback["execution_phase"] = (
                    "WAITING_FOR_DYNAMIC_CLEAR"
                )
                self.latest_feedback["recovery_reason"] = (
                    "WAITING_FOR_OLD_ROUTE"
                )
                self.latest_global_path = list(self.dynamic_blocked_route)
                self.visualization_revision += 1
            self._set_state(target, "route_selection_back")
            self.navigation_velocity.publish(Twist())
            return {
                "status": "completed",
                "current_state": target,
                "destination_preserved": True,
                "state": self._state(),
            }
        if command == "navigation.resume":
            self._validate_command_map(payload)
            return self._resume_auto_from_current_pose()
        if command == "navigation.cancel":
            return self._cancel_navigation("CANCELED")
        if command == "navigation.speed_mode":
            return self._set_auto_speed_mode(payload.get("mode"))
        if command.startswith("mapping."):
            return self._mapping_command(command, payload)
        if command == "system.status":
            return {
                "status": "completed",
                "current_state": self.current_state,
                "state": self._state(),
                "pose": (
                    self.pose
                    if self.mode == "MAPPING"
                    or (self.localized and self.localization_state == "READY")
                    else None
                ),
                "visualization": {
                    "revision": self.visualization_revision,
                    "map_id": self.map_id,
                    "map_version": self.map_version,
                    "route_id": (
                        self.execution_route_id
                        if self.execution_route_id
                        else self.selected_route_id
                    ),
                    "global_path": list(self.latest_global_path),
                    "dynamic_obstacles": list(self.latest_dynamic_obstacles),
                } if self.mode == "NAVIGATION" else None,
            }
        raise AdapterError("UNSUPPORTED_COMMAND", f"Unsupported command: {command}")

    def _foreign_mapping_authorities(self) -> list[str]:
        """Return ROS nodes that could publish a second map authority."""
        conflicts: set[str] = set()
        map_publishers = self.get_publishers_info_by_topic("/map")
        own_slam_publishers = 0
        for publisher in map_publishers:
            node_name = str(publisher.node_name).lstrip("/")
            qualified = f"{publisher.node_namespace.rstrip('/')}/{node_name}"
            if node_name == "slam_toolbox":
                own_slam_publishers += 1
            else:
                conflicts.add(qualified or f"/{node_name}")
        if own_slam_publishers > 1:
            conflicts.add(f"duplicate /slam_toolbox ({own_slam_publishers})")
        for node_name, namespace in self.get_node_names_and_namespaces():
            normalized = str(node_name).lstrip("/")
            if normalized in {"slam_gmapping", "gmapping"}:
                conflicts.add(f"{str(namespace).rstrip('/')}/{normalized}")
        return sorted(conflicts)

    def _map_callback(self, message: OccupancyGrid) -> None:
        if int(message.info.width) > 0 and int(message.info.height) > 0:
            self.latest_static_map = message
            self.map_received_monotonic = time.monotonic()
            self._publish_failed_segments()

    @staticmethod
    def _yaw_from_quaternion(rotation: Any) -> float:
        return math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    @staticmethod
    def _yaw_delta(left: float, right: float) -> float:
        return math.atan2(math.sin(left - right), math.cos(left - right))

    def _mapping_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        """Start geometric confirmation from SLAM's corrected probe pose."""
        with self.state_lock:
            if (
                self.mode != "MAPPING"
                or not self.mapping_relocalization_active
                or self.mapping_relocalization_probe_count < 1
                or self.mapping_relocalization_hint is None
                or self.mapping_relocalization_corrected_pose is not None
            ):
                return
            pose = message.pose.pose
            corrected = (
                float(pose.position.x),
                float(pose.position.y),
                self._yaw_from_quaternion(pose.orientation),
            )
            assessment = mapping_pose_match_quality(
                self.mapping_relocalization_hint,
                corrected,
                list(message.pose.covariance),
                maximum_position_correction=(
                    self.mapping_relocalization_max_position_correction
                ),
                maximum_yaw_correction=self.mapping_relocalization_max_yaw_correction,
                maximum_xy_stddev=self.mapping_relocalization_max_xy_stddev,
                maximum_yaw_stddev=self.mapping_relocalization_max_yaw_stddev,
            )
            if not assessment["accepted"]:
                assessment.update({
                    "state": "REJECTED",
                    "hint_is_approximate": True,
                    "probe_scans": self.mapping_relocalization_probe_count,
                    "corrected_pose": {
                        "x": corrected[0],
                        "y": corrected[1],
                        "yaw": corrected[2],
                    },
                })
                self.mapping_relocalization_result = assessment
                self.mapping_relocalization_diagnostics = dict(assessment)
                self.mapping_relocalization_active = False
                self.mapping_relocalization_event.set()
                return
            self.mapping_relocalization_corrected_pose = corrected
            self.mapping_relocalization_diagnostics = {
                **assessment,
                "state": "VERIFYING_GEOMETRY",
                "hint_is_approximate": True,
                "probe_scans": self.mapping_relocalization_probe_count,
                "corrected_pose": {
                    "x": corrected[0], "y": corrected[1], "yaw": corrected[2]
                },
                "geometry_confirmations": 0,
                "required_confirmations": (
                    self.mapping_relocalization_required_confirmations
                ),
            }
            snapshot = self.mapping_relocalization_latest_snapshot
        if snapshot is not None:
            self._confirm_mapping_geometry_snapshot(snapshot, corrected)

    @staticmethod
    def _mapping_scan_snapshot(
        message: LaserScan,
        laser_in_base: tuple[float, float, float] | None,
    ) -> dict[str, Any]:
        return {
            "ranges": tuple(float(value) for value in message.ranges),
            "angle_min": float(message.angle_min),
            "angle_increment": float(message.angle_increment),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
            "laser_in_base": laser_in_base,
            "captured_monotonic": time.monotonic(),
        }

    def _mapping_geometry_assessment(
        self,
        snapshot: dict[str, Any],
        corrected_pose: tuple[float, float, float],
    ) -> dict[str, Any]:
        saved_map = self.mapping_relocalization_source_map
        extrinsic = snapshot.get("laser_in_base")
        if saved_map is None or extrinsic is None:
            return {"accepted": False, "reason": "MAPPING_GEOMETRY_UNAVAILABLE"}
        cosine, sine = math.cos(corrected_pose[2]), math.sin(corrected_pose[2])
        laser_x = corrected_pose[0] + cosine * float(extrinsic[0]) - sine * float(extrinsic[1])
        laser_y = corrected_pose[1] + sine * float(extrinsic[0]) + cosine * float(extrinsic[1])
        laser_yaw = corrected_pose[2] + float(extrinsic[2])
        match = scan_to_map_match(
            saved_map,
            snapshot["ranges"],
            angle_min=float(snapshot["angle_min"]),
            angle_increment=float(snapshot["angle_increment"]),
            range_min=float(snapshot["range_min"]),
            range_max=float(snapshot["range_max"]),
            laser_x=laser_x,
            laser_y=laser_y,
            laser_yaw=laser_yaw,
            maximum_beams=self.scan_map_maximum_beams,
            minimum_usable_range=self.scan_map_minimum_range,
            maximum_usable_range=self.scan_map_maximum_range,
            endpoint_tolerance=self.global_scan_endpoint_tolerance,
        )
        accepted = bool(
            match.valid_beams >= self.scan_map_minimum_beams
            and match.score >= self.scan_map_threshold
            and match.residual_beams >= self.localization_final_minimum_residual_beams
            and match.median_residual <= self.localization_coarse_match_tolerance
            and match.p90_residual <= self.global_scan_endpoint_tolerance
        )
        reason = (
            "GEOMETRY_CONFIRMED"
            if accepted
            else "SCAN_INSUFFICIENT_BEAMS"
            if match.valid_beams < self.scan_map_minimum_beams
            else "SCAN_MAP_SCORE_TOO_LOW"
            if match.score < self.scan_map_threshold
            else "SCAN_MAP_RESIDUAL_TOO_HIGH"
        )
        return {
            "accepted": accepted,
            "reason": reason,
            "scan_map_score": match.score,
            "valid_beams": match.valid_beams,
            "matched_beams": match.matched_beams,
            "median_residual_m": self._finite_metric(match.median_residual),
            "p90_residual_m": self._finite_metric(match.p90_residual),
        }

    def _confirm_mapping_geometry_snapshot(
        self,
        snapshot: dict[str, Any],
        corrected_pose: tuple[float, float, float],
    ) -> None:
        assessment = self._mapping_geometry_assessment(snapshot, corrected_pose)
        with self.state_lock:
            if (
                not self.mapping_relocalization_active
                or corrected_pose != self.mapping_relocalization_corrected_pose
            ):
                return
            self.mapping_relocalization_geometry_samples += 1
            if assessment["accepted"]:
                self.mapping_relocalization_geometry_confirmations += 1
            else:
                # Require consecutive agreement so one shifted scan cannot be
                # hidden by earlier samples that happened to touch old walls.
                self.mapping_relocalization_geometry_confirmations = 0
            confirmations = self.mapping_relocalization_geometry_confirmations
            samples = self.mapping_relocalization_geometry_samples
            diagnostics = {
                **self.mapping_relocalization_diagnostics,
                **assessment,
                "state": "VERIFYING_GEOMETRY",
                "geometry_confirmations": confirmations,
                "geometry_samples": samples,
                "required_confirmations": (
                    self.mapping_relocalization_required_confirmations
                ),
            }
            if confirmations >= self.mapping_relocalization_required_confirmations:
                diagnostics.update({
                    "accepted": True,
                    "reason": "SLAM_POSE_AND_GEOMETRY_CONFIRMED",
                    "state": "CONFIRMED",
                })
                self.mapping_relocalization_result = diagnostics
                self.mapping_relocalization_diagnostics = dict(diagnostics)
                self.mapping_relocalization_active = False
                self.mapping_relocalization_event.set()
            elif samples >= self.mapping_relocalization_max_validation_scans:
                diagnostics.update({
                    "accepted": False,
                    "reason": "SCAN_MAP_GEOMETRY_UNSTABLE",
                    "state": "REJECTED",
                })
                self.mapping_relocalization_result = diagnostics
                self.mapping_relocalization_diagnostics = dict(diagnostics)
                self.mapping_relocalization_active = False
                self.mapping_relocalization_event.set()
            else:
                self.mapping_relocalization_diagnostics = diagnostics

    def _collect_mapping_pose_evidence(
        self,
        message: LaserScan,
        laser_in_base: tuple[float, float, float] | None,
    ) -> None:
        if self.mode != "MAPPING":
            return
        snapshot = self._mapping_scan_snapshot(message, laser_in_base)
        corrected_pose: tuple[float, float, float] | None = None
        with self.state_lock:
            if self.mapping_pose_search_active:
                self.mapping_pose_search_snapshot = snapshot
                self.mapping_pose_search_active = False
                self.mapping_pose_search_event.set()
            if self.mapping_relocalization_active:
                self.mapping_relocalization_latest_snapshot = snapshot
                corrected_pose = self.mapping_relocalization_corrected_pose
        if corrected_pose is not None:
            self._confirm_mapping_geometry_snapshot(snapshot, corrected_pose)

    def _record_mapping_change_evidence(self, message: LaserScan) -> None:
        """Remember old occupied cells repeatedly observed as free now."""
        source_map = self.mapping_relocalization_source_map
        if (
            self.mode != "MAPPING"
            or self.current_state not in {"MAPPING", "MAPPING_RUNNING"}
            or source_map is None
        ):
            return
        scan_pose = self._scan_transform("map", message)
        if scan_pose is None:
            return
        laser_x, laser_y, laser_yaw = scan_pose
        beam_stride = max(
            1,
            math.ceil(
                len(message.ranges) / float(self.mapping_change_maximum_beams)
            ),
        )
        trace_step = max(0.01, source_map.resolution * 0.5)
        free_cells: set[tuple[int, int]] = set()
        hit_cells: set[tuple[int, int]] = set()
        endpoint_cells = max(
            1,
            math.ceil(
                self.mapping_change_endpoint_protection / source_map.resolution
            ),
        )
        for index in range(0, len(message.ranges), beam_stride):
            distance = float(message.ranges[index])
            if (
                not math.isfinite(distance)
                or distance < max(float(message.range_min), self.scan_map_minimum_range)
                or distance > min(float(message.range_max), self.scan_map_maximum_range)
            ):
                continue
            angle = laser_yaw + float(message.angle_min) + (
                index * float(message.angle_increment)
            )
            cosine, sine = math.cos(angle), math.sin(angle)
            endpoint_x = laser_x + distance * cosine
            endpoint_y = laser_y + distance * sine
            endpoint = source_map.world_to_cell(endpoint_x, endpoint_y)
            if endpoint is not None:
                for row_offset in range(-endpoint_cells, endpoint_cells + 1):
                    for column_offset in range(-endpoint_cells, endpoint_cells + 1):
                        if math.hypot(column_offset, row_offset) > endpoint_cells:
                            continue
                        column = endpoint[0] + column_offset
                        row = endpoint[1] + row_offset
                        if (
                            0 <= column < source_map.width
                            and 0 <= row < source_map.height
                            and source_map.value_at(column, row) >= 65
                        ):
                            hit_cells.add((column, row))

            # Stop before the live endpoint so a real wall is never counted as
            # free due to raster/pose noise. Only historical occupied cells
            # need counters; ordinary free space is intentionally ignored.
            free_limit = distance - self.mapping_change_endpoint_protection
            sample_distance = max(
                float(message.range_min), self.scan_map_minimum_range
            )
            while sample_distance <= free_limit + 1e-9:
                cell = source_map.world_to_cell(
                    laser_x + sample_distance * cosine,
                    laser_y + sample_distance * sine,
                )
                if cell is not None and source_map.value_at(*cell) >= 65:
                    free_cells.add(cell)
                sample_distance += trace_step

        if not free_cells and not hit_cells:
            return
        with self.state_lock:
            # Count a cell at most once per scan so one dense LiDAR fan cannot
            # masquerade as repeated temporal evidence.
            for cell in free_cells:
                self.mapping_free_cell_observations[cell] = (
                    self.mapping_free_cell_observations.get(cell, 0) + 1
                )
            for cell in hit_cells:
                self.mapping_hit_cell_observations[cell] = (
                    self.mapping_hit_cell_observations.get(cell, 0) + 1
                )
            self.mapping_change_evidence_scans += 1

    def _apply_mapping_change_evidence(
        self,
        image: Image.Image,
        yaml_data: dict[str, Any],
    ) -> int:
        """Clear only stale source cells proven free in this continuation."""
        source_map = self.mapping_relocalization_source_map
        if source_map is None or not self.mapping_free_cell_observations:
            return 0
        resolution = float(yaml_data["resolution"])
        origin = list(yaml_data["origin"])
        origin_yaw = float(origin[2])
        cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
        pixels = image.load()
        cleared = 0
        minimum = self.mapping_change_minimum_free_observations
        for cell, free_count in self.mapping_free_cell_observations.items():
            hit_count = self.mapping_hit_cell_observations.get(cell, 0)
            if free_count < minimum or free_count < (3 * hit_count + minimum):
                continue
            world_x, world_y = source_map.cell_center(*cell)
            delta_x = world_x - float(origin[0])
            delta_y = world_y - float(origin[1])
            column = math.floor((cosine * delta_x + sine * delta_y) / resolution)
            row = math.floor((-sine * delta_x + cosine * delta_y) / resolution)
            if not (0 <= column < image.width and 0 <= row < image.height):
                continue
            image_row = image.height - 1 - row
            grayscale = int(pixels[column, image_row])
            probability = (255 - grayscale) / 255.0
            if probability < float(yaml_data.get("occupied_thresh", 0.65)):
                continue
            pixels[column, image_row] = 254
            cleared += 1
        return cleared

    @localization_callback
    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        callback_started = time.monotonic()
        pose = message.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        sample = (float(pose.position.x), float(pose.position.y), yaw)
        if self.last_amcl_pose is not None:
            jump = math.hypot(
                sample[0] - self.last_amcl_pose[0], sample[1] - self.last_amcl_pose[1]
            )
            yaw_jump = abs(self._yaw_delta(sample[2], self.last_amcl_pose[2]))
            if jump > self.pose_maximum_xy_spread * 2 or yaw_jump > self.pose_maximum_yaw_spread * 2:
                self.pose_window.clear()
                self.scan_map_scores.clear()
                self.scan_map_median_residuals.clear()
                self.scan_map_p90_residuals.clear()
                self.scan_map_mean_residuals.clear()
                self.raycast_static_match_ratios.clear()
                self.raycast_dynamic_occlusion_ratios.clear()
                self.raycast_contradiction_ratios.clear()
                self.raycast_median_errors.clear()
                self.raycast_p90_errors.clear()
                self.scan_map_score = 0.0
                self.scan_map_median_residual = math.inf
                self.scan_map_p90_residual = math.inf
                self.scan_map_mean_residual = math.inf
                self.raycast_comparable_beams = 0
                self.raycast_static_matches = 0
                self.raycast_dynamic_occlusions = 0
                self.raycast_map_contradictions = 0
                self.raycast_inconclusive_map_hits = 0
                self.raycast_matched_beams = 0
                self.raycast_static_match_ratio = 0.0
                self.raycast_dynamic_occlusion_ratio = 0.0
                self.raycast_contradiction_ratio = 0.0
                self.raycast_match_ratio = 0.0
                self.raycast_median_error = math.inf
                self.raycast_p90_error = math.inf
                # Physical verification deliberately changes chassis yaw.
                # Invalidate cross-heading corroboration only when AMCL moves
                # the spatial hypothesis to another location; ordinary yaw
                # progress is measured independently from odometry.
                if (
                    self.global_search_untrusted
                    and jump > self.pose_maximum_xy_spread * 2
                ):
                    self.localization_evidence_headings.clear()
                    self.localization_heading_positions.clear()
                    self.localization_heading_bins = ()
                    self.localization_heading_span = 0.0
                    self.ready_evidence_since = None
                    self._nav_debug(
                        "LOCALIZATION",
                        state=self.localization_state,
                        action="HEADING_CORROBORATION_RESET",
                        reason="SPATIAL_HYPOTHESIS_JUMP",
                        position_jump_m=jump,
                        yaw_jump_deg=math.degrees(yaw_jump),
                    )
        self.last_amcl_pose = sample
        self.last_amcl_monotonic = time.monotonic()
        self.pose_window.append((self.last_amcl_monotonic, *sample))
        self.pose_stability_metrics = pose_stability(self.pose_window)
        self.last_amcl_covariance = list(message.pose.covariance)
        self._refresh_localization_confidence()
        now = time.monotonic()
        if self.localized and self.localization_confidence < self.localization_low_threshold:
            if self.low_confidence_since is None:
                self.low_confidence_since = now
        else:
            self.low_confidence_since = None
        if (
            self.low_confidence_since is not None
            and now - self.low_confidence_since >= self.localization_low_grace
        ):
            self._localization_lost("AMCL confidence is too low")
        self.localization_callback_latency_ms = round(
            (time.monotonic() - callback_started) * 1000.0, 3
        )

    @localization_callback
    def _particle_cloud_callback(self, message: ParticleCloud) -> None:
        particles = [
            (
                float(item.pose.position.x),
                float(item.pose.position.y),
                float(item.weight),
            )
            for item in message.particles
        ]
        self.particle_uniqueness = particle_cloud_uniqueness(
            particles,
            cluster_radius=self.particle_cluster_radius,
            alternative_separation=self.particle_alternative_separation,
            minimum_best_weight=self.particle_minimum_best_weight,
            minimum_dominance_ratio=self.particle_minimum_dominance_ratio,
        )
        self.last_particle_cloud_monotonic = time.monotonic()

    def _request_global_scan_uniqueness(
        self,
        *,
        operator_seed: bool = False,
    ) -> None:
        """Run one bounded map-wide alias search outside ROS callbacks."""
        if (
            not self.global_search_untrusted
            or self.saved_map is None
            or self.latest_localization_scan_snapshot is None
            or self.global_scan_uniqueness_in_progress
            or (
                operator_seed
                and (
                    not self.localization_operator_hint_active
                    or not self.localization_seed_approximate
                    or self.localization_seed_pose is None
                )
            )
            or (not operator_seed and self.last_amcl_pose is None)
        ):
            return
        candidate = (
            (
                float(self.localization_seed_pose["x"]),
                float(self.localization_seed_pose["y"]),
                0.0,
            )
            if operator_seed and self.localization_seed_pose is not None
            else tuple(self.last_amcl_pose or ())
        )
        if self.global_scan_evaluated_candidate is not None:
            previous = self.global_scan_evaluated_candidate
            if (
                math.hypot(candidate[0] - previous[0], candidate[1] - previous[1])
                <= self.global_scan_candidate_position_tolerance
                and abs(self._yaw_delta(candidate[2], previous[2]))
                <= self.global_scan_candidate_yaw_tolerance
            ):
                return
            self.global_scan_evaluation_generation += 1
            self.global_scan_uniqueness = GlobalScanUniqueness(
                False,
                "GLOBAL_SCAN_CANDIDATE_CHANGED",
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self.global_scan_evaluated_candidate = None
        snapshot = dict(self.latest_localization_scan_snapshot)
        if (
            time.monotonic() - float(snapshot.get("captured_monotonic", 0.0))
            > self.scan_map_freshness
        ):
            return
        generation = self.global_scan_evaluation_generation
        attempt_id = self.localization_attempt_id
        saved_map = self.saved_map
        hint_center = (
            (
                float(self.localization_seed_pose["x"]),
                float(self.localization_seed_pose["y"]),
            )
            if self.localization_seed_approximate
            and self.localization_seed_pose is not None
            else None
        )
        self.global_scan_uniqueness_in_progress = True
        self.global_scan_uniqueness = GlobalScanUniqueness(
            False, "GLOBAL_SCAN_EVALUATING", 0, 0, 0.0, 0.0, 0.0, 0.0
        )

        def evaluate() -> None:
            try:
                result = global_scan_candidate_uniqueness(
                    saved_map,
                    snapshot["ranges"],
                    angle_min=float(snapshot["angle_min"]),
                    angle_increment=float(snapshot["angle_increment"]),
                    range_min=float(snapshot["range_min"]),
                    range_max=float(snapshot["range_max"]),
                    candidate_pose=candidate,
                    laser_x=float(
                        (snapshot.get("laser_in_base") or (0, 0, 0))[0]
                    ),
                    laser_y=float(
                        (snapshot.get("laser_in_base") or (0, 0, 0))[1]
                    ),
                    laser_yaw=float(
                        (snapshot.get("laser_in_base") or (0, 0, 0))[2]
                    ),
                    maximum_beams=self.global_scan_maximum_beams,
                    minimum_usable_range=self.scan_map_minimum_range,
                    maximum_usable_range=self.scan_map_maximum_range,
                    endpoint_tolerance=self.global_scan_endpoint_tolerance,
                    position_step=self.global_scan_position_step,
                    heading_step=self.global_scan_heading_step,
                    alternative_separation=self.particle_alternative_separation,
                    minimum_best_score=self.global_scan_minimum_best_score,
                    minimum_score_margin=self.global_scan_minimum_score_margin,
                    minimum_score_ratio=self.global_scan_minimum_score_ratio,
                    candidate_position_tolerance=(
                        self.global_scan_candidate_position_tolerance
                    ),
                    candidate_yaw_tolerance=(
                        self.global_scan_candidate_yaw_tolerance
                    ),
                    search_center=hint_center,
                    search_radius=(
                        self.operator_hint_search_radius
                        if operator_seed
                        else self.global_scan_hint_radius
                        if hint_center is not None else None
                    ),
                    # For an operator point, this independent search is used
                    # to correct AMCL's first local basin, so requiring that
                    # basin to already equal the best mode defeats the search.
                    # The result is only a new seed; READY still requires all
                    # strict multi-frame AMCL/raycast gates.
                    require_candidate_match=not operator_seed,
                    # A nearby-position hint resolves spatial aliases, but
                    # the scanner must still reject a competing orientation
                    # at that same position.
                    alternative_yaw_separation=(
                        math.radians(45.0) if operator_seed else None
                    ),
                )
            except Exception as exc:
                result = GlobalScanUniqueness(
                    False,
                    f"GLOBAL_SCAN_ERROR:{type(exc).__name__}",
                    0,
                    0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            with self.localization_lock:
                if (
                    generation != self.global_scan_evaluation_generation
                    or attempt_id != self.localization_attempt_id
                    or saved_map is not self.saved_map
                ):
                    return
                self.global_scan_uniqueness = result
                self.global_scan_evaluated_candidate = candidate
                self.global_scan_uniqueness_in_progress = False
                self._nav_debug(
                    "GLOBAL_SCAN_UNIQUENESS",
                    attempt_id=attempt_id,
                    accepted=result.accepted,
                    reason=result.reason,
                    evaluated_candidates=result.evaluated_candidates,
                    usable_beams=result.usable_beams,
                    candidate=candidate,
                    best=(result.best_x, result.best_y, result.best_yaw),
                    alternative=(
                        result.alternative_x,
                        result.alternative_y,
                        result.alternative_yaw,
                    ),
                    best_score=result.best_score,
                    alternative_score=result.alternative_score,
                    score_margin=result.score_margin,
                    score_ratio=self._finite_metric(result.score_ratio),
                    hint_center=hint_center,
                    operator_seed=operator_seed,
                )
                if (
                    operator_seed
                    and result.accepted
                    and result.best_x is not None
                    and result.best_y is not None
                    and result.best_yaw is not None
                ):
                    operator_pose = dict(self.localization_seed_pose or {})
                    rotation_was_authorized = (
                        self.localization_rotation_authorized
                    )
                    scan_seed = {
                        "x": float(result.best_x),
                        "y": float(result.best_y),
                        "yaw": float(result.best_yaw),
                        "covariance": 0.01,
                    }
                    self._reset_localization_evidence()
                    self.localization_rotation_authorized = (
                        rotation_was_authorized
                    )
                    self.localization_started_monotonic = time.monotonic()
                    self.localization_phase_started_monotonic = (
                        self.localization_started_monotonic
                    )
                    self.localization_seed_pose = scan_seed
                    self.localization_seed_approximate = False
                    self.localization_operator_hint_active = True
                    self.localization_pending_operator_hint = operator_pose
                    self.localization_odometry_prior_active = False
                    self.global_search_requires_rotation = False
                    self.global_search_untrusted = True
                    self.approximate_hint_allowed = False
                    self.localization_attempt_id = f"{attempt_id}:scan-seed"
                    # Retain the independent result for the UI/debug status;
                    # reset above deliberately discarded all old AMCL frames.
                    self.global_scan_uniqueness = result
                    self.global_scan_evaluated_candidate = (
                        scan_seed["x"], scan_seed["y"], scan_seed["yaw"]
                    )
                    self.global_scan_uniqueness_in_progress = False
                    self._publish_initial_pose(scan_seed, approximate=False)
                    self.localization_state = "LOCALIZING_LAST_POSE"
                    self._set_state(
                        "LOCALIZING_LAST_POSE", "operator_scan_seed"
                    )
                    self._nav_debug(
                        "LOCALIZATION_OPERATOR_SCAN_SEED",
                        attempt_id=self.localization_attempt_id,
                        operator_center=(
                            operator_pose.get("x"), operator_pose.get("y")
                        ),
                        scan_seed=scan_seed,
                        best_score=result.best_score,
                        score_margin=result.score_margin,
                        score_ratio=self._finite_metric(result.score_ratio),
                        strict_verification_required=True,
                    )

        threading.Thread(target=evaluate, daemon=True).start()

    def _pose_is_stable(self) -> bool:
        return self.pose_stability_metrics.passes(
            minimum_samples=self.pose_minimum_samples,
            minimum_duration_seconds=self.pose_minimum_duration,
            maximum_xy_spread=self.pose_maximum_xy_spread,
            maximum_median_deviation=self.pose_maximum_median_deviation,
            maximum_yaw_variance=self.pose_maximum_yaw_variance,
            maximum_yaw_spread=self.pose_maximum_yaw_spread,
        )

    def _pose_stability_score(self) -> float:
        metrics = self.pose_stability_metrics
        if metrics.sample_count == 0:
            return 0.0
        evidence = min(1.0, metrics.sample_count / max(1, self.pose_minimum_samples))
        duration = min(
            1.0,
            metrics.duration_seconds / max(0.01, self.pose_minimum_duration),
        )

        def bounded(value: float, maximum: float) -> float:
            if not math.isfinite(value):
                return 0.0
            return max(0.0, min(1.0, maximum / max(maximum, value)))

        return min(
            evidence,
            duration,
            bounded(metrics.xy_spread, self.pose_maximum_xy_spread),
            bounded(
                metrics.median_deviation,
                self.pose_maximum_median_deviation,
            ),
            bounded(
                metrics.yaw_circular_variance,
                self.pose_maximum_yaw_variance,
            ),
            bounded(metrics.yaw_spread, self.pose_maximum_yaw_spread),
        )

    def _navigation_in_progress(self) -> bool:
        return (
            self.current_state == "NAVIGATING"
            and self.execution_phase in {
                "STRAIGHT_PREPARE", "TURN", "TURN_SETTLING",
                "DISPATCHING_STRAIGHT", "RECOVERING",
                "STRAIGHT", "NARROW_STRAIGHT",
            }
        )

    def _sensor_entry(self, name: str) -> dict[str, Any]:
        sensors = self.sensor_time_status.get("sensors")
        if not isinstance(sensors, dict):
            return {}
        entry = sensors.get(name)
        return entry if isinstance(entry, dict) else {}

    def _critical_sensor_time_status(self) -> tuple[bool, str, dict[str, Any]]:
        """Return the safety decision plus the exact evidence behind it.

        The adapter used to reduce every timing fault to one generic boolean.
        That made a short status hiccup indistinguishable from a stale LiDAR
        or odometry stream, and left neither the operator nor the recovery
        state machine with enough information to act correctly.
        """
        status_age_ms = self._age_milliseconds(
            self.last_sensor_time_status_monotonic
        )

        def snapshot(name: str) -> dict[str, Any]:
            entry = self._sensor_entry(name)
            return {
                "arrival_age_ms": entry.get("arrival_age_ms"),
                "timestamp_age_ms": entry.get("corrected_age_ms"),
                "arrival_fresh": bool(entry.get("arrival_fresh")),
                "timestamp_valid": bool(entry.get("timestamp_valid")),
                "frame_valid": bool(entry.get("frame_valid")),
                "clock_state": str(entry.get("clock_state") or "UNKNOWN"),
                "invalid_streak": int(entry.get("invalid_streak", 0) or 0),
                "rejected_packets": int(entry.get("rejected_packets", 0) or 0),
                "last_rejection": str(entry.get("last_rejection") or ""),
            }

        diagnostics = {
            "status_age_ms": status_age_ms,
            "clock_state": str(
                self.sensor_time_status.get("clock_state") or "CLOCK_SYNCING"
            ),
            "scan": snapshot("scan"),
            "odom": snapshot("odom"),
        }
        if (
            self.last_sensor_time_status_monotonic <= 0.0
            or status_age_ms is None
            or status_age_ms > 600.0
        ):
            return False, "STATUS_STALE", diagnostics
        if diagnostics["clock_state"] != "SYNCED":
            return False, "CLOCK_NOT_SYNCED", diagnostics

        for name, prefix in (("scan", "SCAN"), ("odom", "ODOM")):
            entry = diagnostics[name]
            if not entry["arrival_fresh"]:
                return False, f"{prefix}_ARRIVAL_STALE", diagnostics
            if not entry["timestamp_valid"]:
                return False, f"{prefix}_TIMESTAMP_INVALID", diagnostics
            if not entry["frame_valid"]:
                return False, f"{prefix}_FRAME_INVALID", diagnostics
        return True, "", diagnostics

    def _critical_sensor_time_healthy(self) -> bool:
        healthy, _, _ = self._critical_sensor_time_status()
        return healthy

    def _refresh_localization_confidence(self) -> None:
        if not self.last_amcl_covariance:
            self.localization_confidence = 0.0
            return
        # Pose-window stability proves initial convergence only. Once READY,
        # both manual and autonomous motion necessarily spread that window and
        # must not revoke an otherwise fresh tracked pose.
        navigation_in_progress = self._navigation_in_progress()
        tracking_ready_pose = self.localized and self.localization_state == "READY"
        stability_score = (
            1.0
            if navigation_in_progress or tracking_ready_pose
            else self._pose_stability_score()
        )
        fresh_scan_map_score = (
            self.scan_map_score
            if time.monotonic() - self.last_scan_map_monotonic
            <= self.scan_map_freshness
            else 0.0
        )
        # The median window tolerates short dynamic outliers while a separate
        # lower tracking sanity bound keeps sustained severe static-map
        # mismatch effective. Never floor all moving evidence to the
        # acquisition threshold: that hid a genuinely wrong map pose.
        confidence_scan_map_score = (
            self.scan_map_threshold
            if (
                navigation_in_progress
                and fresh_scan_map_score
                >= self.tracking_scan_map_sanity_threshold
            )
            else fresh_scan_map_score
        )
        self.localization_confidence = localization_confidence(
            self.last_amcl_covariance,
            stability_score=stability_score,
            scan_map_score=confidence_scan_map_score,
            scan_map_threshold=self.scan_map_threshold,
            # Dynamic occlusion is neutral here. Contradictions reduce the
            # summary confidence while explicit static/contradiction gates
            # remain authoritative for READY.
            raycast_match_ratio=max(
                0.0, 1.0 - self.raycast_contradiction_ratio
            ),
            scan_fresh=time.monotonic() - self.last_scan_monotonic <= 0.30,
            tf_stable=time.monotonic() - self.last_map_tf_monotonic <= 0.60,
            odometry_healthy=self.tf_buffer.can_transform(
                "odom", "base_footprint", Time()
            ),
            sensor_time_valid=self._critical_sensor_time_healthy(),
        )

    def _path_callback(self, message: NavigationPath) -> None:
        now = time.monotonic()
        if self.current_state == "NAVIGATING":
            self.replan_timestamps.append(now)
            self.replan_timestamps = [
                timestamp
                for timestamp in self.replan_timestamps
                if now - timestamp <= 10.0
            ]
        path = [
            {"x": round(float(item.pose.position.x), 3), "y": round(float(item.pose.position.y), 3)}
            for item in message.poses
        ]
        # Nav2 publishes the raw planner candidate before this adapter can
        # validate the 0.30 x 0.20 m swept footprint. Keep it for diagnostics,
        # but never let it replace the frontend/FollowPath route.
        self.latest_planner_raw_path = path

    @staticmethod
    def _path_length(path: list[dict[str, float]]) -> float:
        return round(sum(
            math.hypot(
                float(right["x"]) - float(left["x"]),
                float(right["y"]) - float(left["y"]),
            )
            for left, right in zip(path, path[1:])
        ), 3)

    def _measured_replan_frequency(self) -> float:
        if len(self.replan_timestamps) < 2:
            return 0.0
        elapsed = self.replan_timestamps[-1] - self.replan_timestamps[0]
        if elapsed <= 0:
            return 0.0
        return round((len(self.replan_timestamps) - 1) / elapsed, 2)

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        if (
            self.mode == "NAVIGATION"
            and (not self.localized or self.localization_state != "READY")
        ):
            # The local grid is still consumed by Nav2/motion safety. It is not
            # allowed to create map-relative persistent planning evidence from
            # an untrusted map->odom hypothesis.
            return
        if not self._dynamic_tracking_evidence_trusted():
            # Raw LiDAR still reaches the local controller and the independent
            # motion-safety node. Only the map-relative, mission-level tracker
            # is suppressed: when most scan endpoints disagree with Saved Map,
            # rolling-costmap walls move with pose/TF error and are not valid
            # evidence of a walking person.
            now = time.monotonic()
            if (
                self.navigation_debug_enabled
                and now - self.dynamic_last_untrusted_log_monotonic >= 1.0
            ):
                valid = int(self.last_scan_filter_stats.get(
                    "scan_points_valid", 0
                ) or 0)
                dynamic = int(self.last_scan_filter_stats.get(
                    "dynamic_points_kept", 0
                ) or 0)
                self._nav_debug(
                    "DYNAMIC_TRACKING_SUPPRESSED",
                    reason="SCAN_MAP_MISMATCH",
                    dynamic_point_ratio=(
                        None if valid <= 0 else round(dynamic / valid, 3)
                    ),
                    scan_filter=dict(self.last_scan_filter_stats),
                    physical_motion_safety_retained=True,
                )
                self.dynamic_last_untrusted_log_monotonic = now
            return
        # Inspect the complete local costmap before applying the bounded UI /
        # tracker payload.  Capping the row-major grid first systematically
        # discarded obstacles in the upper part of the rolling window --
        # commonly the area in front of the robot.
        obstacles = compact_lethal_cells(message, max_cells=None)
        source_frame = str(message.header.frame_id or "map")
        if source_frame != "map":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", source_frame, Time.from_msg(message.header.stamp)
                )
            except TransformException:
                stamp_seconds = (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) / 1_000_000_000.0
                )
                now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
                message_age = abs(now_seconds - stamp_seconds)
                if (
                    stamp_seconds <= 0.0
                    or message_age > self.scan_tf_fallback_max_age
                ):
                    self._nav_debug(
                        "COSTMAP_TF_REJECTED",
                        source_frame=source_frame,
                        reason="MESSAGE_TIMESTAMP_TF_UNAVAILABLE",
                        message_age_ms=round(message_age * 1000.0, 1),
                    )
                    return
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map", source_frame, Time()
                    )
                except TransformException:
                    return
                transform_stamp = Time.from_msg(transform.header.stamp)
                transform_age = abs(
                    Time.from_msg(message.header.stamp).nanoseconds
                    - transform_stamp.nanoseconds
                ) / 1e9
                if (
                    transform_stamp.nanoseconds <= 0
                    or transform_age > self.scan_tf_fallback_max_age
                ):
                    self._nav_debug(
                        "COSTMAP_TF_REJECTED",
                        source_frame=source_frame,
                        reason="LATEST_TF_STALE_FOR_MESSAGE",
                        tf_age_ms=round(transform_age * 1000.0, 1),
                    )
                    return
                self._nav_debug(
                    "COSTMAP_TF_FALLBACK",
                    source_frame=source_frame,
                    message_age_ms=round(message_age * 1000.0, 1),
                    tf_age_ms=round(transform_age * 1000.0, 1),
                    fallback="BOUNDED_FRESH_LATEST",
                )
            translation = transform.transform.translation
            yaw = self._yaw_from_quaternion(transform.transform.rotation)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            obstacles = [
                {
                    "x": round(float(translation.x) + cosine * item["x"] - sine * item["y"], 3),
                    "y": round(float(translation.y) + sine * item["x"] + cosine * item["y"], 3),
                }
                for item in obstacles
            ]
        if self.saved_map is not None:
            obstacles = [
                item
                for item in obstacles
                if not self.saved_map.occupied_within(
                    float(item["x"]),
                    float(item["y"]),
                    self.dynamic_overlay_static_tolerance,
                )
            ]
        if self.pose is not None:
            pose_x = float(self.pose["x"])
            pose_y = float(self.pose["y"])
            obstacles.sort(
                key=lambda item: (
                    (float(item["x"]) - pose_x) ** 2
                    + (float(item["y"]) - pose_y) ** 2
                )
            )
        obstacles = obstacles[: self.dynamic_overlay_max_cells]
        self.dynamic_overlay.observe(
            ((float(item["x"]), float(item["y"])) for item in obstacles),
            now=time.monotonic(),
            # Static cells were removed before the distance-prioritized cap.
            saved_map=None,
        )
        self._refresh_dynamic_obstacle_view()

    def _dynamic_tracking_evidence_trusted(
        self, now: float | None = None
    ) -> bool:
        timestamp = time.monotonic() if now is None else float(now)
        stats = self.last_scan_filter_stats
        valid = int(stats.get("scan_points_valid", 0) or 0)
        static_matches = int(stats.get("static_map_matches", 0) or 0)
        dynamic_points = int(stats.get("dynamic_points_kept", 0) or 0)
        return bool(
            self.localized
            and self.localization_state == "READY"
            and timestamp - self.last_scan_filter_monotonic
            <= self.dynamic_tracking_evidence_freshness
            and bool(stats.get("filtered"))
            and stats.get("reason") == "EXPECTED_RANGE_MATCH"
            and valid > 0
            and static_matches >= self.dynamic_tracking_minimum_static_matches
            and dynamic_points / valid
            <= self.dynamic_tracking_maximum_point_ratio
        )

    def _refresh_dynamic_obstacle_view(self) -> None:
        obstacles = [
            {
                "id": int(item.id),
                "x": round(item.center_x, 3),
                "y": round(item.center_y, 3),
                "radius": round(item.radius, 3),
                "first_seen": item.first_seen,
                "last_seen": item.last_seen,
                "observation_count": int(item.observation_count),
                "confidence": round(item.confidence, 3),
                "velocity_x": round(item.velocity_x, 3),
                "velocity_y": round(item.velocity_y, 3),
                "speed": round(item.speed, 3),
                "motion_state": item.motion_state,
            }
            for item in self.dynamic_overlay.snapshot(time.monotonic())
        ]
        if obstacles != self.latest_dynamic_obstacles:
            self.latest_dynamic_obstacles = obstacles
            self.visualization_revision += 1

    def _dynamic_exclusions(self) -> tuple[tuple[float, float, float], ...]:
        if not self._dynamic_tracking_evidence_trusted():
            return ()
        inflation = (
            self.footprint_half_width
            + self.planning_footprint_padding
            + self.corridor_hard_side_margin
        )
        return tuple(
            (item.center_x, item.center_y, item.radius + inflation)
            for item in self.dynamic_overlay.snapshot(time.monotonic())
            if item.observation_count
            >= self.dynamic_planning_minimum_observations
            and item.motion_state in {"MOVING", "STATIONARY"}
        )

    def _dynamic_planning_exclusions(
        self,
    ) -> tuple[tuple[float, float, float], ...]:
        if self.dynamic_blocked_keepout is not None:
            # Recovery has stronger evidence than the general live overlay:
            # this keepout was located from repeated observations on the
            # segment which actually stopped.  Do not combine it with every
            # map-mismatch cluster in the live overlay.  Treating those noisy
            # clusters as simultaneous hard walls can falsely disconnect a
            # wide free route and leave the robot waiting beside the blocker.
            return (self.dynamic_blocked_keepout,)
        return self._dynamic_exclusions()

    def _live_front_keepout_for_route(
        self,
        points: list[dict[str, float]],
        segment_directions: list[int] | tuple[int, ...] = (),
        *,
        blocked_only: bool,
    ) -> tuple[float, float, float] | None:
        """Project fresh route-aligned front LiDAR evidence into map space."""
        if self.pose is None or len(points) < 2 or not self.corridor_samples:
            return None
        now = time.monotonic()
        recent_corridors = [
            (timestamp, corridor)
            for timestamp, corridor, _ in self.corridor_samples
            if now - float(timestamp) <= 0.60
            and math.isfinite(float(corridor.front_clearance))
        ]
        if not recent_corridors:
            return None
        # A thin chair leg can alternate between adjacent LiDAR beams while
        # the chassis is stationary.  Retain the nearest fresh return inside
        # the same 600 ms evidence window instead of letting the final beam of
        # the window erase a real obstacle.
        _, corridor = min(
            recent_corridors,
            key=lambda item: float(item[1].front_clearance),
        )
        if segment_directions and int(segment_directions[0]) < 0:
            return None
        pose_x = float(self.pose["x"])
        pose_y = float(self.pose["y"])
        target = next(
            (
                point
                for point in points[1:]
                if math.hypot(
                    float(point["x"]) - pose_x,
                    float(point["y"]) - pose_y,
                ) >= 0.08
            ),
            None,
        )
        if target is None:
            return None
        delta_x = float(target["x"]) - pose_x
        delta_y = float(target["y"]) - pose_y
        segment_length = math.hypot(delta_x, delta_y)
        route_heading = math.atan2(delta_y, delta_x)
        chassis_yaw = float(self.pose.get("yaw", route_heading))
        if abs(self._yaw_delta(route_heading, chassis_yaw)) > math.radians(20.0):
            return None
        front_clearance = float(corridor.front_clearance)
        if not math.isfinite(front_clearance) or front_clearance < 0.0:
            return None
        physically_blocked = bool(
            corridor.classification == "PHYSICALLY_BLOCKED"
            or not bool(corridor.physically_passable)
            or front_clearance
            <= self.corridor_front_clearance + self.straight_endpoint_tolerance
        )
        if blocked_only:
            if not physically_blocked:
                return None
        elif (
            front_clearance > min(0.60, self.corridor_lookahead)
            or segment_length
            <= front_clearance + self.straight_endpoint_tolerance
        ):
            return None
        obstacle_distance = self.footprint_half_length + front_clearance
        keepout_radius = min(
            self.alternative_route_keepout_radius,
            max(0.05, obstacle_distance - 0.02),
        )
        return (
            pose_x + obstacle_distance * math.cos(chassis_yaw),
            pose_y + obstacle_distance * math.sin(chassis_yaw),
            keepout_radius,
        )

    @staticmethod
    def _route_signature(points: list[dict[str, float]]) -> str:
        canonical = canonicalize_stop_turn_path(points)
        payload = [
            {"x": round(float(point["x"]), 3), "y": round(float(point["y"]), 3)}
            for point in canonical
        ]
        return hashlib.sha1(
            json.dumps(payload, sort_keys=True).encode()
        ).hexdigest()[:12]

    def _reset_dynamic_recovery(self) -> None:
        self.dynamic_wait_started = None
        self.dynamic_clear_started = None
        self.dynamic_block_reason = ""
        self.dynamic_blocked_route = []
        self.dynamic_blocked_segment_directions = []
        self.dynamic_blocked_keepout = None
        self.dynamic_recovery_state = "IDLE"
        self.dynamic_blocker_id = ""
        self.dynamic_blocked_route_signature = ""
        self.dynamic_failed_route_signatures = {}
        self.dynamic_replan_attempt_count = 0
        self.dynamic_replan_requires_alternative = False
        self.dynamic_recovery_expires_monotonic = 0.0
        self.dynamic_last_unconfirmed_log_monotonic = 0.0

    def _dynamic_affects_remaining_route(self) -> bool:
        return bool(self._dynamic_route_obstacles(
            minimum_observations=self.dynamic_planning_minimum_observations
        ))

    def _dynamic_route_obstacles(
        self, *, minimum_observations: int
    ) -> tuple[DynamicObstacle, ...]:
        if not self._dynamic_tracking_evidence_trusted():
            return ()
        remaining = self._remaining_execution_route()
        if len(remaining) < 2:
            return ()
        inflation = (
            self.footprint_half_width
            + self.planning_footprint_padding
            + self.corridor_hard_side_margin
        )
        return tuple(
            obstacle
            for obstacle in self.dynamic_overlay.snapshot(time.monotonic())
            if obstacle.observation_count >= max(1, int(minimum_observations))
            and dynamic_exclusions_intersect_route(
                remaining,
                (
                    (
                        obstacle.center_x,
                        obstacle.center_y,
                        obstacle.radius + inflation,
                    ),
                ),
                horizon=2.0,
            )
        )

    def _global_costmap_callback(self, message: OccupancyGrid) -> None:
        if int(message.info.width) <= 0 or int(message.info.height) <= 0:
            return
        with self.global_costmap_condition:
            self.latest_global_costmap = message
            self.last_global_costmap_monotonic = time.monotonic()
            self.global_costmap_generation += 1
            self.global_costmap_condition.notify_all()
        self.global_costmap_update.set()

    def _reset_pre_ready_planning_evidence(self) -> None:
        """Drop every map-relative dynamic artifact made before trusted pose."""
        self.dynamic_overlay = DynamicObstacleOverlay(
            ttl_seconds=self.dynamic_overlay_ttl,
            cluster_distance=self.dynamic_overlay_cluster_distance,
            motion_threshold=self.dynamic_obstacle_motion_threshold,
            stationary_confirmation_seconds=(
                self.dynamic_obstacle_stationary_confirmation
            ),
            moving_confirmation_windows=(
                self.dynamic_obstacle_moving_confirmation_windows
            ),
        )
        self.latest_dynamic_obstacles = []
        self.dynamic_blocked_keepout = None
        self.dynamic_blocker_id = ""
        self.dynamic_failed_route_signatures.clear()
        self.latest_global_costmap = None
        self.last_global_costmap_monotonic = 0.0
        self.visualization_revision += 1
        if self.clear_global_costmap_client.service_is_ready():
            self.clear_global_costmap_client.call_async(
                ClearEntireCostmap.Request()
            )
        self._nav_debug(
            "PLANNING_EVIDENCE_RESET",
            reason="LOCALIZATION_BECAME_READY",
            attempt_id=self.localization_attempt_id,
        )

    def _publish_failed_segments(self) -> bool:
        source = self.latest_static_map
        if source is None:
            return False
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info = source.info
        width = int(source.info.width)
        height = int(source.info.height)
        resolution = float(source.info.resolution)
        message.data = [0] * (width * height)
        if self.saved_map is None or resolution <= 0:
            self.failed_segment_mask.publish(message)
            return False

        # This planning-only StaticLayer is also a redundant copy of the
        # authoritative Saved Map. The ordinary Nav2 static layer remains in
        # place, but a missed/stale merge there must never let ThetaStar plan
        # through a wall that the executable-path validator will later reject.
        source_origin = source.info.origin
        geometry_matches = (
            width == self.saved_map.width
            and height == self.saved_map.height
            and math.isclose(resolution, self.saved_map.resolution, abs_tol=1e-9)
            and math.isclose(
                float(source_origin.position.x),
                self.saved_map.origin_x,
                abs_tol=1e-6,
            )
            and math.isclose(
                float(source_origin.position.y),
                self.saved_map.origin_y,
                abs_tol=1e-6,
            )
            and abs(self._yaw_delta(
                self._yaw_from_quaternion(source_origin.orientation),
                self.saved_map.origin_yaw,
            )) <= 1e-6
            and len(self.saved_map.occupancy) == width * height
        )
        if geometry_matches:
            # Reproduce all Saved Map semantics in this planning authority.
            # Unknown remains unknown, occupied remains lethal, and only known
            # free cells are free.
            message.data = [
                100 if int(value) >= 65 else -1 if int(value) < 0 else 0
                for value in self.saved_map.occupancy
            ]

            # Nav2's primary static layer clears the current footprint. Mirror
            # that bounded behavior here so a one-cell SLAM/localization
            # overlap cannot make the planner's start pose permanently lethal.
            # This changes only the planning supplement, never the Saved Map.
            if self.pose is not None:
                center = self.saved_map.world_to_cell(
                    float(self.pose["x"]), float(self.pose["y"])
                )
                if center is not None:
                    # Remove source cells far enough from the occupied start
                    # for Nav2's inscribed-cost band not to overlap the current
                    # footprint. The final exact Saved Map validator resumes
                    # after the smaller physical-footprint exemption below.
                    clear_radius = (
                        math.hypot(
                            self.footprint_half_length,
                            self.footprint_half_width,
                        )
                        + min(
                            self.footprint_half_length,
                            self.footprint_half_width,
                        )
                        + resolution * math.sqrt(2.0) / 2.0
                    )
                    cells = max(1, math.ceil(clear_radius / resolution))
                    for offset_y in range(-cells, cells + 1):
                        for offset_x in range(-cells, cells + 1):
                            column = center[0] + offset_x
                            row = center[1] + offset_y
                            if (
                                0 <= column < width
                                and 0 <= row < height
                                and math.hypot(offset_x, offset_y) * resolution
                                <= clear_radius
                            ):
                                message.data[row * width + column] = 0

        for segment in self.failed_segments:
            center = self.saved_map.world_to_cell(
                float(segment["x"]), float(segment["y"])
            )
            if center is None:
                continue
            radius = float(segment["radius"])
            cells = max(1, math.ceil(radius / resolution))
            for offset_y in range(-cells, cells + 1):
                for offset_x in range(-cells, cells + 1):
                    column = center[0] + offset_x
                    row = center[1] + offset_y
                    if (
                        0 <= column < width
                        and 0 <= row < height
                        and math.hypot(offset_x, offset_y) * resolution <= radius
                    ):
                        message.data[row * width + column] = 100
        self.failed_segment_mask.publish(message)
        return geometry_matches

    def _sync_planning_static_mask(self) -> None:
        """Wait until the planning master has consumed the Saved Map mask."""
        baseline_generation = self.global_costmap_generation
        if not self._publish_failed_segments():
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Saved Map geometry is not ready for the planning costmap",
            )
        if not self._wait_for_global_costmap_after(baseline_generation, 2.0):
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Global costmap did not consume the planning static mask",
            )
        # A publish can race with an update already being assembled. Waiting
        # for one more 2 Hz full update closes that generation boundary.
        first_generation = self.global_costmap_generation
        if not self._wait_for_global_costmap_after(first_generation, 2.0):
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Global costmap planning mask was not confirmed",
            )

    def _failed_segment_tick(self) -> None:
        now = time.monotonic()
        before = len(self.failed_segments)
        self.failed_segments = [
            segment
            for segment in self.failed_segments
            if float(segment["expires_monotonic"]) > now
        ]
        # StaticLayer consumes a transient-local full map, so republish only
        # when the mask actually changes. A 5 Hz full-map publication here
        # would waste CPU/network without adding freshness semantics.
        if before > len(self.failed_segments):
            self._publish_failed_segments()
        if before > len(self.failed_segments):
            self._nav_debug(
                "FAILED_SEGMENT",
                action="EXPIRED",
                remaining=len(self.failed_segments),
            )

    def _corridor_failure_evidence(self) -> tuple[str, Any]:
        now = time.monotonic()
        samples = [
            item for item in self.corridor_samples
            if now - float(item[0]) <= 1.5
            and abs(float(item[2])) <= math.radians(20)
        ]
        required = self.corridor_confirmation_samples
        if len(samples) < required:
            return "UNCONFIRMED", self.latest_corridor
        selected = samples[-required:]
        if selected[-1][0] - selected[0][0] < self.corridor_confirmation_duration:
            return "UNCONFIRMED", selected[-1][1]
        assessments = [item[1] for item in selected]
        if all(item.classification == "CLEAR" for item in assessments):
            return "CORRIDOR_CLEAR", assessments[-1]
        if all(
            item.classification == "NARROW_OR_UNCERTAIN"
            for item in assessments
        ):
            return "NARROW_OR_UNCERTAIN", assessments[-1]
        if all(
            item.classification == "PHYSICALLY_BLOCKED"
            for item in assessments
        ):
            return "PHYSICALLY_BLOCKED", assessments[-1]
        return "UNCONFIRMED", assessments[-1]

    def _enter_narrow_path_decision(self, reason: str, corridor: Any) -> None:
        """Pause Auto before a stable narrow segment while preserving goal."""
        with self.state_lock:
            if self.current_state != "NAVIGATING" or self.narrow_decision_in_progress:
                return
            self.narrow_decision_in_progress = True
            handle = self.current_goal_handle
            self.current_goal_handle = None
            self.navigation_goal_generation += 1
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_phase = "IDLE"
            self.execution_points = []
            self.execution_segment_directions = []
            self.motion_owner = "NONE"
            self.manual_handoff_reason = reason
            self._set_state("NARROW_PATH_DECISION", reason.lower())
            self.latest_feedback["terminal_reason"] = reason
        self.navigation_velocity.publish(Twist())
        self.profile_limiter.reset()
        self._nav_debug(
            "CORRIDOR",
            vehicle_width=2.0 * self.footprint_half_width,
            vehicle_length=2.0 * self.footprint_half_length,
            available_width=self._finite_metric(corridor.available_width),
            hard_required_width=corridor.hard_required_width,
            auto_required_width=corridor.auto_required_width,
            left_clearance=self._finite_metric(corridor.left_clearance),
            right_clearance=self._finite_metric(corridor.right_clearance),
            front_clearance=self._finite_metric(corridor.front_clearance),
            classification=corridor.classification,
            reason=reason,
        )
        if handle is not None:
            handle.cancel_goal_async()
        self.narrow_decision_in_progress = False

    def _mark_failed_segment(self, reason: str, corridor: Any) -> dict[str, Any]:
        pose = dict(self.pose or {})
        heading, _ = self._current_path_heading()
        yaw = float(pose.get("yaw", 0.0)) if heading is None else float(heading)
        distance = self.failed_segment_forward_offset
        if math.isfinite(corridor.front_clearance):
            distance = min(
                distance,
                max(0.30, self.footprint_half_length + corridor.front_clearance),
            )
        center_x = float(pose.get("x", 0.0)) + distance * math.cos(yaw)
        center_y = float(pose.get("y", 0.0)) + distance * math.sin(yaw)
        now = time.monotonic()
        for segment in self.failed_segments:
            if math.hypot(
                float(segment["x"]) - center_x,
                float(segment["y"]) - center_y,
            ) <= self.failed_segment_radius:
                segment["expires_monotonic"] = now + self.failed_segment_ttl
                return segment
        segment = {
            "x": center_x,
            "y": center_y,
            "radius": self.failed_segment_radius,
            "reason": reason,
            "created_monotonic": now,
            "expires_monotonic": now + self.failed_segment_ttl,
        }
        self.failed_segments.append(segment)
        self._publish_failed_segments()
        self._nav_debug(
            "FAILED_SEGMENT",
            reason=reason,
            center=(round(center_x, 3), round(center_y, 3)),
            radius=self.failed_segment_radius,
            available_width=self._finite_metric(corridor.available_width),
            required_width=corridor.required_width,
            ttl_sec=self.failed_segment_ttl,
        )
        return segment

    @staticmethod
    def _path_crosses_segment(
        path: list[dict[str, float]],
        segment: dict[str, Any],
    ) -> bool:
        center_x = float(segment["x"])
        center_y = float(segment["y"])
        radius = float(segment["radius"])
        for left, right in zip(path, path[1:]):
            ax, ay = float(left["x"]), float(left["y"])
            bx, by = float(right["x"]), float(right["y"])
            dx, dy = bx - ax, by - ay
            denominator = dx * dx + dy * dy
            ratio = 0.0 if denominator <= 1e-9 else max(
                0.0,
                min(1.0, ((center_x - ax) * dx + (center_y - ay) * dy) / denominator),
            )
            if math.hypot(
                ax + ratio * dx - center_x,
                ay + ratio * dy - center_y,
            ) <= radius:
                return True
        return False

    @staticmethod
    def _sample_route(
        path: list[dict[str, float]],
        spacing: float = 0.10,
    ) -> list[tuple[float, float, float]]:
        if not path:
            return []
        output: list[tuple[float, float, float]] = []
        for left, right in zip(path, path[1:]):
            ax, ay = float(left["x"]), float(left["y"])
            bx, by = float(right["x"]), float(right["y"])
            yaw = math.atan2(by - ay, bx - ax)
            distance = math.hypot(bx - ax, by - ay)
            count = max(1, math.ceil(distance / max(0.001, spacing)))
            output.extend(
                (ax + (bx - ax) * index / count, ay + (by - ay) * index / count, yaw)
                for index in range(count)
            )
        last = path[-1]
        output.append((float(last["x"]), float(last["y"]), output[-1][2] if output else 0.0))
        return output

    def _route_metadata(
        self,
        path: list[dict[str, float]],
        *,
        original: list[dict[str, float]],
        segment_directions: list[int] | tuple[int, ...] | None = None,
    ) -> dict[str, Any]:
        canonical = canonicalize_stop_turn_path(path)
        directions = (
            [1 for _ in range(max(0, len(canonical) - 1))]
            if segment_directions is None
            else [-1 if int(value) < 0 else 1 for value in segment_directions]
        )
        directions_valid = len(directions) == max(0, len(canonical) - 1)
        # Infrastructure failures (especially a stale/not-ready costmap) are
        # not invalid-route results. Preserve the AdapterError so callers can
        # report COSTMAP_NOT_READY instead of collapsing SUCCESS into an empty
        # candidate list.
        live_validation = self._validate_executable_path(
            canonical,
            context="ROUTE_METADATA",
        )
        valid = live_validation.valid and directions_valid
        if self.saved_map is None or self.map_navigation_geometry is None:
            valid = False
            geometry_metadata = None
        else:
            static_validation = validate_stop_turn_route(
                self.saved_map,
                canonical,
                half_length=self.footprint_half_length,
                half_width=(
                    self.footprint_half_width
                    + self.translation_lateral_margin
                ),
                padding=self.planning_footprint_padding,
                segment_directions=directions,
            )
            valid = valid and static_validation.valid
            geometry_metadata = route_geometry_metadata(
                self.saved_map,
                self.map_navigation_geometry,
                canonical,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
                linear_speed=self.speed_profiles.get(self.auto_speed_mode).linear_max,
                angular_speed=self.execution_turn_max_speed,
                start_yaw=(
                    None if self.pose is None else float(self.pose.get("yaw", 0.0))
                ),
                segment_directions=directions,
            )
            valid = valid and geometry_metadata.turn_safe
        length = self._path_length(canonical)
        overlap = path_overlap_ratio(path, original)
        metadata = {
            "valid": valid,
            "total_length": length,
            "estimated_time": max(1, round(length / 0.15)),
            "minimum_clearance": None,
            "minimum_passage_width": None,
            "minimum_static_clearance": None,
            "minimum_turn_clearance": None,
            "turn_count": 0,
            "total_turn_angle": 0.0,
            "initial_turn_angle": 0.0,
            "internal_turn_angle": 0.0,
            "final_turn_angle": 0.0,
            "execution_total_turn_angle": 0.0,
            "narrow_segments": [],
            "turn_safe": False,
            "overlap_with_original": overlap,
        }
        if geometry_metadata is not None:
            metadata.update(geometry_metadata.as_dict())
            metadata["minimum_clearance"] = metadata["minimum_static_clearance"]
        return metadata

    def _decision_keepout(self) -> dict[str, Any]:
        pose = dict(self.pose or {})
        heading, _ = self._current_path_heading()
        yaw = float(pose.get("yaw", 0.0)) if heading is None else float(heading)
        distance = self.failed_segment_forward_offset
        return {
            "x": float(pose.get("x", 0.0)) + distance * math.cos(yaw),
            "y": float(pose.get("y", 0.0)) + distance * math.sin(yaw),
            "radius": self.alternative_route_keepout_radius,
            "reason": "ALTERNATIVE_ROUTE_SEARCH",
            "created_monotonic": time.monotonic(),
            "expires_monotonic": time.monotonic() + 60.0,
        }

    def _serialize_stop_turn_candidates(
        self, planned: list[Any]
    ) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []
        for planned_route in planned:
            points = [dict(point) for point in planned_route.points]
            directions = list(
                planned_route.segment_directions
                or (1 for _ in range(max(0, len(points) - 1)))
            )
            metadata = self._route_metadata(
                points,
                original=points,
                segment_directions=directions,
            )
            if not metadata["valid"]:
                continue
            # Preserve live costmap validation above, but use the planner's
            # start-yaw-aware execution cost for ranking/diagnostics.
            valid = metadata["valid"]
            overlap = metadata["overlap_with_original"]
            metadata.update(planned_route.metadata.as_dict())
            metadata["valid"] = valid
            metadata["overlap_with_original"] = overlap
            digest = hashlib.sha1(
                json.dumps(
                    {
                        "points": points,
                        "segment_directions": directions,
                    },
                    sort_keys=True,
                ).encode()
            ).hexdigest()[:10]
            candidate = {
                "route_id": f"stop-turn-{digest}",
                "points": points,
                **metadata,
                "heading_bins": list(planned_route.heading_bins),
                "segment_directions": directions,
                "recommended": False,
            }
            if any(
                path_overlap_ratio(points, item["points"])
                >= self.alternative_route_overlap_threshold
                for item in candidates
            ):
                continue
            candidates.append(candidate)
        if candidates:
            candidates[0]["recommended"] = True
        return candidates

    def _compute_alternative_routes(self) -> dict[str, Any]:
        """Run the optional, bounded multi-route search only on request."""
        if self.paused_goal is None:
            raise AdapterError("STATE_CONFLICT", "No destination is available")
        if self.stop_turn_planner is None or self.pose is None:
            raise AdapterError("PLANNER_NOT_READY", "Stop-turn planner is unavailable")
        return_state = (
            "NARROW_PATH_DECISION"
            if self.current_state == "NARROW_PATH_DECISION"
            else "READY"
        )
        with self.state_lock:
            self.route_selection_return_state = return_state
            self._set_state("COMPUTING_ALTERNATIVES", "operator_requested_routes")
        request_planner = self._stop_turn_planner_for_clearance()
        planned = request_planner.plan_candidates(
            dict(self.pose),
            dict(self.paused_goal),
            maximum_candidates=self.alternative_route_max_candidates,
            overlap_threshold=self.alternative_route_overlap_threshold,
            planning_time_budget=self.stop_turn_planning_budget,
        )
        candidates = self._serialize_stop_turn_candidates(planned)
        self._nav_debug(
            "ROUTE_CANDIDATES",
            count=len(candidates),
            source="EXPLICIT_ALTERNATIVE_SEARCH",
        )
        if not candidates:
            with self.state_lock:
                self._set_state(return_state, "no_alternative_route")
            raise AdapterError("NO_ALTERNATIVE_ROUTE", "No safe alternative route was found")
        selected = candidates[0]
        with self.state_lock:
            self.route_candidates = {
                str(candidate["route_id"]): candidate for candidate in candidates
            }
            self.selected_route_id = str(selected["route_id"])
            self.latest_global_path = list(selected["points"])
            self.visualization_revision += 1
            self._set_state("ROUTE_SELECTION", "alternative_routes_ready")
        return {
            "status": "completed",
            "current_state": self.current_state,
            "candidates": candidates,
            "destination": dict(self.paused_goal),
            "route_id": self.selected_route_id,
            "points": list(selected["points"]),
            "destination_preserved": True,
            "state": self._state(),
        }

    def _wait_for_global_costmap_after(
        self,
        baseline_generation: int,
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        with self.global_costmap_condition:
            while self.global_costmap_generation <= baseline_generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.global_costmap_condition.wait(remaining)
            return True

    def _validate_executable_path(
        self,
        points: list[dict[str, float]],
        *,
        context: str,
        allow_monotonic_initial_overlap: bool = False,
    ) -> ExecutablePathValidation:
        message = self.latest_global_costmap
        if (
            message is None
            or time.monotonic() - self.last_global_costmap_monotonic > 1.5
        ):
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Global costmap is not fresh enough to validate the route",
            )
        origin = message.info.origin
        live_validation = validate_executable_grid_path(
            points,
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=self._yaw_from_quaternion(origin.orientation),
            data=message.data,
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            allow_unknown=False,
            # OccupancyGrid 99 is Nav2 INSCRIBED and already represents body
            # clearance around a lethal source. Check it at the path center;
            # expanding the full body over it would count the footprint twice.
            lethal_threshold=100,
            inscribed_threshold=99,
            allow_monotonic_initial_overlap=allow_monotonic_initial_overlap,
        )
        validation = live_validation
        validation_source = "GLOBAL_MASTER_COSTMAP"
        if live_validation.valid and self.saved_map is not None:
            # The live master is authoritative around the footprint the robot
            # currently occupies: static-map quantization may overlap that
            # already occupied pose by one cell. Beyond that bounded envelope,
            # the exact Saved Map supplements the master so a missing/stale
            # StaticLayer can never make a wall-crossing route executable.
            static_exemption = (
                math.hypot(
                    self.footprint_half_length,
                    self.footprint_half_width,
                )
                + self.saved_map.resolution * math.sqrt(2.0) / 2.0
            )
            static_points = (
                points
                if allow_monotonic_initial_overlap
                else self._path_after_initial_distance(points, static_exemption)
            )
            if len(static_points) >= 2:
                static_validation = validate_executable_grid_path(
                    static_points,
                    width=self.saved_map.width,
                    height=self.saved_map.height,
                    resolution=self.saved_map.resolution,
                    origin_x=self.saved_map.origin_x,
                    origin_y=self.saved_map.origin_y,
                    origin_yaw=self.saved_map.origin_yaw,
                    data=self.saved_map.occupancy,
                    half_length=self.footprint_half_length,
                    half_width=self.footprint_half_width,
                    allow_unknown=False,
                    lethal_threshold=65,
                    allow_monotonic_initial_overlap=(
                        allow_monotonic_initial_overlap
                    ),
                )
                if not static_validation.valid:
                    validation = static_validation
                    validation_source = "SAVED_STATIC_MAP"
        self._nav_debug(
            "PATH_VALIDATION",
            context=context,
            valid=validation.valid,
            code=validation.code or "EXECUTABLE",
            segment_index=validation.segment_index,
            sample_x=validation.sample_x,
            sample_y=validation.sample_y,
            sample_yaw=validation.sample_yaw,
            cell_cost=validation.cell_cost,
            collision_x=validation.collision_x,
            collision_y=validation.collision_y,
            collision_cells=len(validation.collision_cells),
            samples_checked=validation.samples_checked,
            source=validation_source,
            costmap_generation=self.global_costmap_generation,
            costmap_age_ms=self._age_milliseconds(
                self.last_global_costmap_monotonic
            ),
        )
        return validation

    @staticmethod
    def _path_after_initial_distance(
        points: list[dict[str, float]],
        distance_to_skip: float,
    ) -> list[dict[str, float]]:
        if len(points) < 2 or distance_to_skip <= 0.0:
            return [dict(point) for point in points]
        remaining = float(distance_to_skip)
        for index, (left, right) in enumerate(zip(points, points[1:])):
            left_x, left_y = float(left["x"]), float(left["y"])
            right_x, right_y = float(right["x"]), float(right["y"])
            length = math.hypot(right_x - left_x, right_y - left_y)
            if length <= 1e-9:
                continue
            if remaining >= length:
                remaining -= length
                continue
            ratio = remaining / length
            boundary = {
                "x": left_x + (right_x - left_x) * ratio,
                "y": left_y + (right_y - left_y) * ratio,
            }
            return [boundary] + [
                dict(point) for point in points[index + 1:]
            ]
        return []

    def _add_invalid_path_exclusion(
        self,
        validation: ExecutablePathValidation,
    ) -> bool:
        collision_centers = [
            (float(x), float(y))
            for x, y, _ in validation.collision_cells
        ]
        if not collision_centers:
            exclusion_x = validation.collision_x
            exclusion_y = validation.collision_y
            if exclusion_x is None or exclusion_y is None:
                exclusion_x = validation.sample_x
                exclusion_y = validation.sample_y
            if exclusion_x is not None and exclusion_y is not None:
                collision_centers = [(exclusion_x, exclusion_y)]
        if not collision_centers:
            return False
        now = time.monotonic()
        # Put the temporary exclusion on the cell that the physical footprint
        # actually hit. Centering it on the robot/path sample progressively
        # blocked the escape corridor instead of expanding the real obstacle.
        exclusion_radius = self.failed_segment_radius
        exclusion_count = 0
        for exclusion_x, exclusion_y in collision_centers:
            for segment in self.failed_segments:
                if math.hypot(
                    float(segment["x"]) - exclusion_x,
                    float(segment["y"]) - exclusion_y,
                ) <= exclusion_radius:
                    segment["expires_monotonic"] = now + self.failed_segment_ttl
                    break
            else:
                self.failed_segments.append({
                    "x": exclusion_x,
                    "y": exclusion_y,
                    "radius": exclusion_radius,
                    "reason": validation.code,
                    "created_monotonic": now,
                    "expires_monotonic": now + self.failed_segment_ttl,
                })
                exclusion_count += 1
        self._publish_failed_segments()
        self._nav_debug(
            "PLAN_RETRY",
            reason=validation.code,
            action="TEMPORARY_PATH_EXCLUSION",
            segment_index=validation.segment_index,
            center=collision_centers[0],
            radius=exclusion_radius,
            cells=len(collision_centers),
            exclusions_added=exclusion_count,
        )
        # The exclusion is already latched on its transient-local planning
        # topic. A diagnosed failure may now clear/rebuild the master grid;
        # generation semantics accept updates during the service call.
        self._refresh_global_costmap_for_planning()
        return True

    def _ensure_executable_path(
        self,
        points: list[dict[str, float]],
        goal: dict[str, Any],
        *,
        context: str,
        segment_directions: list[int] | tuple[int, ...] | None = None,
    ) -> list[dict[str, float]]:
        canonical = canonicalize_stop_turn_path(points)
        if len(canonical) < 2:
            raise AdapterError("NO_VALID_PATH", "Planner returned no usable path")
        try:
            validation = self._validate_executable_path(canonical, context=context)
        except AdapterError as exc:
            if exc.code != "COSTMAP_NOT_READY":
                raise
            self._refresh_global_costmap_for_planning()
            validation = self._validate_executable_path(
                canonical, context=f"{context}_AFTER_COSTMAP_REFRESH"
            )
        if not validation.valid:
            raise AdapterError(
                "NO_VALID_PATH",
                f"Preview route is no longer executable: {validation.code}",
            )
        if self.saved_map is None:
            raise AdapterError("MAP_MISSING", "Saved map is unavailable")
        static_validation = validate_stop_turn_route(
            self.saved_map,
            canonical,
            half_length=self.footprint_half_length,
            half_width=(
                self.footprint_half_width + self.translation_lateral_margin
            ),
            padding=self.planning_footprint_padding,
            segment_directions=segment_directions,
        )
        if not static_validation.valid:
            raise AdapterError(
                static_validation.code,
                "Route failed exact translation or turn-sweep validation",
            )
        return canonical

    def _global_cost_at(self, x: float, y: float) -> int | None:
        message = self.latest_global_costmap
        if message is None:
            return None
        origin = message.info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        delta_x = float(x) - float(origin.position.x)
        delta_y = float(y) - float(origin.position.y)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        resolution = float(message.info.resolution)
        if resolution <= 0:
            return None
        column = math.floor(local_x / resolution)
        row = math.floor(local_y / resolution)
        width = int(message.info.width)
        height = int(message.info.height)
        if column < 0 or row < 0 or column >= width or row >= height:
            return None
        return int(message.data[row * width + column])

    def _nearby_global_cost_counts(
        self,
        x: float,
        y: float,
        radius_m: float = 0.75,
    ) -> dict[str, int]:
        if not self.navigation_debug_enabled or self.latest_global_costmap is None:
            return {}
        message = self.latest_global_costmap
        origin = message.info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        delta_x = float(x) - float(origin.position.x)
        delta_y = float(y) - float(origin.position.y)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        resolution = float(message.info.resolution)
        if resolution <= 0:
            return {}
        center_column = math.floor((cosine * delta_x + sine * delta_y) / resolution)
        center_row = math.floor((-sine * delta_x + cosine * delta_y) / resolution)
        radius_cells = math.ceil(radius_m / resolution)
        width = int(message.info.width)
        height = int(message.info.height)
        counts = {"lethal": 0, "inflated": 0, "unknown": 0}
        for row in range(
            max(0, center_row - radius_cells),
            min(height, center_row + radius_cells + 1),
        ):
            for column in range(
                max(0, center_column - radius_cells),
                min(width, center_column + radius_cells + 1),
            ):
                if math.hypot(
                    column - center_column, row - center_row
                ) * resolution > radius_m:
                    continue
                value = int(message.data[row * width + column])
                if value < 0:
                    counts["unknown"] += 1
                elif value >= 99:
                    counts["lethal"] += 1
                elif value > 0:
                    counts["inflated"] += 1
        return counts

    def _laser_extrinsic(self, frame_id: str) -> tuple[float, float, float] | None:
        if self.laser_in_base is not None:
            return self.laser_in_base
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_footprint", frame_id, Time()
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        self.laser_in_base = (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )
        return self.laser_in_base

    def _scan_transform(
        self,
        target_frame: str,
        message: LaserScan,
    ) -> tuple[float, float, float] | None:
        """Resolve capture-time TF with one bounded, age-checked fallback."""
        source_frame = str(message.header.frame_id)
        scan_stamp = Time.from_msg(message.header.stamp)
        cache_key = (target_frame, source_frame, int(scan_stamp.nanoseconds))
        if cache_key in self.scan_transform_cache:
            return self.scan_transform_cache[cache_key]
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                scan_stamp,
                timeout=Duration(seconds=max(0.0, self.scan_tf_wait)),
            )
        except TransformException:
            self._tf_debug(
                "TF_AT_SCAN_MISS",
                target=target_frame,
                source=source_frame,
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time()
                )
            except TransformException:
                self._tf_debug(
                    "SCAN_REJECTED_TF",
                    target=target_frame,
                    source=source_frame,
                    reason="NO_TRANSFORM",
                )
                self.scan_transform_cache[cache_key] = None
                return None
            transform_stamp = Time.from_msg(transform.header.stamp)
            age_seconds = abs(
                scan_stamp.nanoseconds - transform_stamp.nanoseconds
            ) / 1e9
            if (
                transform_stamp.nanoseconds <= 0
                or age_seconds > self.scan_tf_fallback_max_age
            ):
                self._tf_debug(
                    "SCAN_REJECTED_TF",
                    target=target_frame,
                    source=source_frame,
                    tf_age_ms=age_seconds * 1000.0,
                    reason="TF_STALE",
                )
                self.scan_transform_cache[cache_key] = None
                return None
            self._tf_debug(
                "TF_FALLBACK",
                target=target_frame,
                source=source_frame,
                tf_age_ms=age_seconds * 1000.0,
            )
        else:
            self._tf_debug(
                "TF_AT_SCAN_OK",
                target=target_frame,
                source=source_frame,
                tf_age_ms=0.0,
            )
        translation = transform.transform.translation
        result = (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )
        self.scan_transform_cache[cache_key] = result
        return result

    def _tf_debug(self, event: str, **fields: Any) -> None:
        if not self.navigation_debug_enabled:
            return
        if (
            event in {"TF_AT_SCAN_MISS", "SCAN_REJECTED_TF"}
            and self.mode == "NAVIGATION"
            and bool(self.map_id)
            and self.localization_state != "READY"
        ):
            # Missing map links are an expected pre-READY condition. The
            # link-by-link LOCALIZATION_TF transition diagnostic below is the
            # single authority until a tracked pose exists.
            return
        now = time.monotonic()
        if event in {"TF_AT_SCAN_OK", "TF_FALLBACK"}:
            signature_fields = {
                key: fields.get(key) for key in ("target", "source")
            }
        else:
            signature_fields = fields
        signature = json.dumps(signature_fields, sort_keys=True, default=str)
        signature_key = f"{event}:{signature}"
        throttle = (
            30.0
            if event == "TF_AT_SCAN_OK"
            else 10.0
            if event in {"TF_AT_SCAN_MISS", "SCAN_REJECTED_TF", "TF_FALLBACK"}
            else 2.0
        )
        if now - self.last_tf_debug_monotonic.get(signature_key, 0.0) < throttle:
            return
        self._nav_debug(event, **fields)
        self.last_tf_debug_monotonic[signature_key] = now

    def _tf_link_diagnostic(
        self,
        target_frame: str,
        source_frame: str,
    ) -> dict[str, Any]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time()
            )
        except TransformException:
            return {"available": False, "age_ms": None}
        stamp = Time.from_msg(transform.header.stamp)
        age_ms = None
        if stamp.nanoseconds > 0:
            age_ms = round(max(
                0.0,
                (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e6,
            ), 1)
        return {"available": True, "age_ms": age_ms}

    def _monitor_localization_tf(self, now: float) -> bool:
        links = {
            "map_to_odom": self._tf_link_diagnostic("map", "odom"),
            "odom_to_base": self._tf_link_diagnostic(
                "odom", "base_footprint"
            ),
            "base_to_laser": self._tf_link_diagnostic(
                "base_footprint", "laser_frame"
            ),
            "map_to_laser": self._tf_link_diagnostic("map", "laser_frame"),
        }
        navigation_scan_age = self._age_milliseconds(
            self.last_navigation_scan_published_monotonic
        )
        diagnostics = {
            **links,
            "amcl_pose_age_ms": self._age_milliseconds(
                self.last_amcl_monotonic
            ),
            "scan_navigation_age_ms": navigation_scan_age,
            "scan_navigation_published": bool(
                navigation_scan_age is not None
                and navigation_scan_age <= 500.0
            ),
            "global_localization_service_ready": (
                self.global_localization_client.service_is_ready()
            ),
            "nomotion_update_service_ready": (
                self.nomotion_update_client.service_is_ready()
            ),
            "localization_state": self.localization_state,
        }
        signature = json.dumps({
            key: value["available"] for key, value in links.items()
        }, sort_keys=True) + f":{self.localization_state}"
        if (
            signature != self.last_localization_tf_signature
            or now - self.last_localization_tf_log_monotonic >= 30.0
        ):
            self._nav_debug("LOCALIZATION_TF", **diagnostics)
            self.last_localization_tf_signature = signature
            self.last_localization_tf_log_monotonic = now
        missing_link = next(
            (name for name, status in links.items() if not status["available"]),
            "",
        )
        global_active = self.localization_state in {
            "PASSIVE_LOCALIZING", "CANDIDATE", "VERIFYING", "AMBIGUOUS",
            "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING", "LOCALIZING_SETTLING"
        }
        if not global_active or not missing_link:
            self.localization_tf_unavailable_since = None
            return True
        if self.localization_tf_unavailable_since is None:
            self.localization_tf_unavailable_since = now
            return True
        if (
            now - self.localization_tf_unavailable_since
            < self.localization_tf_chain_timeout
        ):
            return True
        self._stop_localization_rotation()
        self.localized = False
        self.localization_state = "LOCALIZATION_FAILED"
        self._set_state(
            "LOCALIZATION_FAILED", "localization_tf_chain_unavailable"
        )
        self._nav_debug(
            "LOCALIZATION_TF",
            **diagnostics,
            failed=True,
            reason="LOCALIZATION_TF_CHAIN_UNAVAILABLE",
            missing_link=missing_link,
        )
        return False

    def _scan_heading_in_odom(self, message: LaserScan) -> float | None:
        scan_pose = self._scan_transform("odom", message)
        return None if scan_pose is None else scan_pose[2]

    def _record_heading_observation(
        self,
        message: LaserScan,
        *,
        heading_observation_valid: bool,
    ) -> None:
        """Record basic-quality headings without pre-accepting localization."""
        # Keep the exact pose sample that passed the freshness gate.  Never
        # read the mutable field again after the check: a socket command may
        # start a new attempt while this scan callback is running.
        pose_snapshot = self.last_amcl_pose
        amcl_monotonic_snapshot = self.last_amcl_monotonic
        scan_map_monotonic_snapshot = self.last_scan_map_monotonic
        stationary_observation = bool(
            self.global_search_untrusted
            and self.localization_state in {
                "PASSIVE_LOCALIZING", "CANDIDATE", "VERIFYING",
                "AMBIGUOUS", "LOCALIZING_GLOBAL", "LOCALIZING_SETTLING"
            }
            and not self.rotation_active
            and self.motion_owner != "LOCALIZATION"
            and (
                self.localization_state != "LOCALIZING_SETTLING"
                or self.localization_settling_evidence_started
            )
        )
        if (
            not stationary_observation
            or not heading_observation_valid
            or not self._pose_is_stable()
            or pose_snapshot is None
            or time.monotonic() - amcl_monotonic_snapshot
            > self.amcl_pose_freshness
            or time.monotonic() - scan_map_monotonic_snapshot
            > self.scan_map_freshness
            or not self._critical_sensor_time_healthy()
        ):
            return
        valid_beams = sum(
            1
            for distance in message.ranges
            if math.isfinite(distance)
            and message.range_min <= distance <= message.range_max
        )
        if valid_beams < self.scan_map_minimum_beams:
            return
        heading = self._scan_heading_in_odom(message)
        if heading is None:
            return
        heading_bin = min(
            self.global_heading_bin_count - 1,
            int(
                (heading % (2.0 * math.pi))
                / (2.0 * math.pi)
                * self.global_heading_bin_count
            ),
        )
        # ``pose_snapshot`` cannot become None underneath this unpack.
        candidate_x, candidate_y, _ = pose_snapshot
        maximum_position_spread = self.pose_maximum_xy_spread * 2.0
        position_spread = heading_position_spread([
            *self.localization_heading_positions.values(),
            (candidate_x, candidate_y),
        ])
        if position_spread > maximum_position_spread:
            self.localization_evidence_headings.clear()
            self.localization_heading_positions.clear()
            self.localization_heading_bins = ()
            self.localization_heading_span = 0.0
            self.ready_evidence_since = None
            self._nav_debug(
                "LOCALIZATION",
                state=self.localization_state,
                action="HEADING_CORROBORATION_RESET",
                reason="SPATIAL_HYPOTHESIS_INCONSISTENT_ACROSS_HEADINGS",
                position_spread_m=position_spread,
            )
        self.localization_evidence_headings = bounded_heading_evidence(
            self.localization_evidence_headings,
            heading,
            bin_count=self.global_heading_bin_count,
        )
        self.localization_heading_positions[heading_bin] = (
            candidate_x, candidate_y
        )
        diversity = heading_diversity(
            self.localization_evidence_headings,
            bin_count=self.global_heading_bin_count,
        )
        self.localization_heading_bins = diversity.observed_bins
        self.localization_heading_span = diversity.span_radians

    def _update_scan_map_match(self, message: LaserScan) -> bool:
        if self.saved_map is None:
            return False
        if self.global_search_untrusted:
            # Keep one immutable scan for the occasional background global
            # alias search. The expensive search never runs in this callback.
            self.latest_localization_scan_snapshot = {
                "ranges": tuple(float(value) for value in message.ranges),
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "laser_in_base": self._laser_extrinsic(
                    str(message.header.frame_id)
                ),
                "captured_monotonic": time.monotonic(),
            }
        scan_pose = self._scan_transform("map", message)
        if scan_pose is None:
            return False
        match = scan_to_map_match(
            self.saved_map,
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            laser_x=scan_pose[0],
            laser_y=scan_pose[1],
            laser_yaw=scan_pose[2],
            maximum_beams=self.scan_map_maximum_beams,
            minimum_usable_range=self.scan_map_minimum_range,
            maximum_usable_range=self.scan_map_maximum_range,
            endpoint_tolerance=self.localization_coarse_match_tolerance,
        )
        raycast = scan_raycast_consistency(
            self.saved_map,
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            laser_x=scan_pose[0],
            laser_y=scan_pose[1],
            laser_yaw=scan_pose[2],
            maximum_beams=self.localization_raycast_maximum_beams,
            minimum_usable_range=self.scan_map_minimum_range,
            maximum_usable_range=self.scan_map_maximum_range,
            match_tolerance=self.localization_raycast_match_tolerance,
            minimum_reliable_structure_span=(
                self.localization_raycast_minimum_reliable_structure_span
            ),
        )
        self.scan_map_matched_beams = match.matched_beams
        self.scan_map_valid_beams = match.valid_beams
        self.scan_map_residual_beams = match.residual_beams
        self.raycast_comparable_beams = raycast.comparable_beams
        self.raycast_static_matches = raycast.static_matches
        self.raycast_dynamic_occlusions = raycast.dynamic_occlusions
        self.raycast_map_contradictions = raycast.map_contradictions
        self.raycast_inconclusive_map_hits = raycast.inconclusive_map_hits
        self.raycast_matched_beams = raycast.matched_beams
        self.raycast_static_match_ratio = raycast.static_match_ratio
        self.raycast_dynamic_occlusion_ratio = raycast.dynamic_occlusion_ratio
        self.raycast_contradiction_ratio = raycast.contradiction_ratio
        self.raycast_match_ratio = raycast.match_ratio
        self.raycast_median_error = raycast.median_error
        self.raycast_p90_error = raycast.p90_error
        if self.last_amcl_pose is not None:
            self.localization_evidence_frames.append(
                LocalizationEvidenceFrame(
                    timestamp=time.monotonic(),
                    pose_x=float(self.last_amcl_pose[0]),
                    pose_y=float(self.last_amcl_pose[1]),
                    pose_yaw=float(self.last_amcl_pose[2]),
                    scan_score=match.score,
                    valid_beams=match.valid_beams,
                    residual_beams=match.residual_beams,
                    median_residual=match.median_residual,
                    p90_residual=match.p90_residual,
                    mean_residual=match.mean_residual,
                    raycast_comparable_beams=raycast.comparable_beams,
                    raycast_static_matches=raycast.static_matches,
                    raycast_dynamic_occlusions=raycast.dynamic_occlusions,
                    raycast_map_contradictions=raycast.map_contradictions,
                    raycast_inconclusive_map_hits=raycast.inconclusive_map_hits,
                    raycast_static_match_ratio=raycast.static_match_ratio,
                    raycast_dynamic_occlusion_ratio=(
                        raycast.dynamic_occlusion_ratio
                    ),
                    raycast_contradiction_ratio=raycast.contradiction_ratio,
                    raycast_median_error=raycast.median_error,
                    raycast_p90_error=raycast.p90_error,
                )
            )
        if match.valid_beams < self.scan_map_minimum_beams:
            return False
        self.scan_map_scores.append(match.score)
        self.scan_map_median_residuals.append(match.median_residual)
        self.scan_map_p90_residuals.append(match.p90_residual)
        self.scan_map_mean_residuals.append(match.mean_residual)
        if raycast.comparable_beams >= self.localization_raycast_minimum_beams:
            self.raycast_static_match_ratios.append(
                raycast.static_match_ratio
            )
            self.raycast_dynamic_occlusion_ratios.append(
                raycast.dynamic_occlusion_ratio
            )
            self.raycast_contradiction_ratios.append(
                raycast.contradiction_ratio
            )
            self.raycast_median_errors.append(raycast.median_error)
            self.raycast_p90_errors.append(raycast.p90_error)
        self.scan_map_score = round(statistics.median(self.scan_map_scores), 4)
        self.scan_map_median_residual = statistics.median(
            self.scan_map_median_residuals
        )
        self.scan_map_p90_residual = statistics.median(
            self.scan_map_p90_residuals
        )
        self.scan_map_mean_residual = statistics.median(
            self.scan_map_mean_residuals
        )
        if self.raycast_static_match_ratios:
            self.raycast_static_match_ratio = round(
                statistics.median(self.raycast_static_match_ratios), 4
            )
            self.raycast_dynamic_occlusion_ratio = round(
                statistics.median(
                    self.raycast_dynamic_occlusion_ratios
                ), 4
            )
            self.raycast_contradiction_ratio = round(
                statistics.median(self.raycast_contradiction_ratios), 4
            )
            self.raycast_match_ratio = self.raycast_static_match_ratio
            self.raycast_median_error = statistics.median(
                self.raycast_median_errors
            )
            self.raycast_p90_error = statistics.median(
                self.raycast_p90_errors
            )
        self.last_scan_map_monotonic = time.monotonic()
        if self.verification_started_monotonic:
            self.verification_scan_count += 1
        self._refresh_localization_confidence()
        return bool(
            match.score >= self._required_localization_scan_score()
            and match.residual_beams
            >= self.localization_final_minimum_residual_beams
            and match.median_residual
            <= self.localization_final_max_median_residual
            and match.p90_residual
            <= self.localization_final_max_p90_residual
        )

    def _planning_scan_message(self, message: LaserScan) -> LaserScan:
        now = time.monotonic()
        self.last_scan_filter_monotonic = now
        if not self.localized or self.localization_state != "READY":
            planning = LaserScan()
            planning.header = message.header
            planning.angle_min = message.angle_min
            planning.angle_max = message.angle_max
            planning.angle_increment = message.angle_increment
            planning.time_increment = message.time_increment
            planning.scan_time = message.scan_time
            planning.range_min = message.range_min
            planning.range_max = message.range_max
            # An all-clear planning-only scan prevents a wrong pre-READY
            # map->laser hypothesis from turning saved walls into persistent
            # dynamic keepouts. Raw /scan_navigation is still published to
            # AMCL and the local atomic safety pipeline.
            planning.ranges = [math.inf for _ in message.ranges]
            planning.intensities = list(message.intensities)
            valid = sum(
                1
                for distance in message.ranges
                if math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            )
            self.last_scan_filter_stats = {
                "scan_points_total": len(message.ranges),
                "scan_points_valid": valid,
                "static_map_matches": 0,
                "dynamic_points_kept": 0,
                "raycast_unavailable": valid,
                "filtered": False,
                "reason": "LOCALIZATION_UNTRUSTED",
            }
            return planning
        scan_pose = self._scan_transform("map", message)
        if self.saved_map is None or scan_pose is None:
            valid = sum(
                1
                for distance in message.ranges
                if math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            )
            self.last_scan_filter_stats = {
                "scan_points_total": len(message.ranges),
                "scan_points_valid": valid,
                "static_map_matches": 0,
                "dynamic_points_kept": valid,
                "raycast_unavailable": valid,
                "filtered": False,
                "reason": (
                    "MAP_UNAVAILABLE"
                    if self.saved_map is None
                    else "TF_AT_SCAN_UNAVAILABLE"
                ),
            }
            return message
        filtered = filter_static_map_scan(
            self.saved_map,
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            laser_x=scan_pose[0],
            laser_y=scan_pose[1],
            laser_yaw=scan_pose[2],
            expected_range_tolerance=self.planning_static_match_tolerance,
            minimum_usable_range=self.scan_map_minimum_range,
            maximum_usable_range=self.scan_map_maximum_range,
        )
        planning = LaserScan()
        planning.header = message.header
        planning.angle_min = message.angle_min
        planning.angle_max = message.angle_max
        planning.angle_increment = message.angle_increment
        planning.time_increment = message.time_increment
        planning.scan_time = message.scan_time
        planning.range_min = message.range_min
        planning.range_max = message.range_max
        planning.ranges = filtered.ranges
        planning.intensities = list(message.intensities)
        self.last_scan_filter_stats = {
            "scan_points_total": filtered.total_beams,
            "scan_points_valid": filtered.valid_beams,
            "static_map_matches": filtered.static_map_matches,
            "dynamic_points_kept": filtered.dynamic_points_kept,
            "raycast_unavailable": filtered.raycast_unavailable,
            "filtered": True,
            "reason": "EXPECTED_RANGE_MATCH",
        }
        if (
            self.navigation_debug_enabled
            and now - self.last_scan_filter_log_monotonic >= 10.0
        ):
            self._nav_debug(
                "SCAN_FILTER",
                **self.last_scan_filter_stats,
                tolerance_m=self.planning_static_match_tolerance,
                tf_timestamp="SCAN_CAPTURE_TIME",
            )
            self.last_scan_filter_log_monotonic = now
        return planning

    @localization_callback
    def _scan_callback(self, message: LaserScan) -> None:
        callback_started = time.monotonic()
        self.scan_transform_cache.clear()
        self.last_scan_monotonic = callback_started
        nearest_forward = math.inf
        nearest_left = math.inf
        nearest_right = math.inf
        nearest_rotation_obstacle = math.inf
        extrinsic = self._laser_extrinsic(str(message.header.frame_id))
        self._collect_mapping_pose_evidence(message, extrinsic)
        self._record_mapping_change_evidence(message)
        base_points: list[tuple[float, float]] = []
        for index, distance in enumerate(message.ranges):
            if (
                not math.isfinite(distance)
                or distance < message.range_min
                or distance > message.range_max
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            if extrinsic is not None:
                offset_x, offset_y, offset_yaw = extrinsic
                base_angle = offset_yaw + angle
                point_x = offset_x + float(distance) * math.cos(base_angle)
                point_y = offset_y + float(distance) * math.sin(base_angle)
                base_points.append((point_x, point_y))
                if (
                    abs(point_x) < self.footprint_half_length
                    and abs(point_y) < self.footprint_half_width
                ):
                    continue
                clearance = rotation_swept_clearance(
                    point_x,
                    point_y,
                    half_length=self.footprint_half_length,
                    half_width=self.footprint_half_width,
                )
                nearest_rotation_obstacle = min(
                    nearest_rotation_obstacle, clearance
                )
        _, heading_error = self._current_path_heading()
        path_error = 0.0 if heading_error is None else float(heading_error)
        cosine, sine = math.cos(path_error), math.sin(path_error)
        path_points = [
            (cosine * x + sine * y, -sine * x + cosine * y)
            for x, y in base_points
        ]
        covariance_allowance = 0.0
        if len(self.last_amcl_covariance) >= 36:
            covariance_allowance = math.sqrt(max(
                0.0,
                float(self.last_amcl_covariance[0]),
                float(self.last_amcl_covariance[7]),
            )) * 0.10
        confidence_allowance = max(
            0.0, 1.0 - float(self.localization_confidence)
        ) * 0.01
        localization_uncertainty = min(
            self.corridor_localization_uncertainty_max,
            covariance_allowance + confidence_allowance,
        )
        corridor = evaluate_corridor(
            path_points,
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            side_margin=self.corridor_side_margin,
            front_clearance_required=self.corridor_front_clearance,
            lookahead=self.corridor_lookahead,
            rotation_margin=self.rotation_minimum_obstacle_distance,
            hard_side_margin=self.corridor_hard_side_margin,
            translation_lateral_margin=self.translation_lateral_margin,
            localization_uncertainty=localization_uncertainty,
        )
        self.latest_corridor = corridor
        self.corridor_samples.append((callback_started, corridor, path_error))
        nearest_forward = corridor.front_clearance
        nearest_left = corridor.left_clearance
        nearest_right = corridor.right_clearance
        self.nearest_forward_obstacle = nearest_forward
        self.nearest_left_obstacle = nearest_left
        self.nearest_right_obstacle = nearest_right
        self.nearest_rotation_obstacle = nearest_rotation_obstacle
        if (
            self.navigation_debug_enabled
            and self.current_state == "NAVIGATING"
            and callback_started - self.last_corridor_log_monotonic >= 0.5
        ):
            requested = self.pipeline_samples.get("controller_requested")
            self._nav_debug(
                "CORRIDOR",
                robot_width=2.0 * self.footprint_half_width,
                robot_length=2.0 * self.footprint_half_length,
                left_clearance=self._finite_metric(corridor.left_clearance),
                right_clearance=self._finite_metric(corridor.right_clearance),
                available_width=self._finite_metric(corridor.available_width),
                hard_required_width=corridor.hard_required_width,
                auto_required_width=corridor.auto_required_width,
                front_clearance=self._finite_metric(corridor.front_clearance),
                linear_cmd=None if requested is None else requested[0],
                angular_cmd=None if requested is None else requested[1],
                heading_error_deg=math.degrees(path_error),
                can_go_straight=(
                    corridor.can_go_straight and abs(path_error) <= math.radians(20)
                ),
                can_rotate=corridor.can_rotate,
                classification=corridor.classification,
                reason=corridor.reason,
            )
            self.last_corridor_log_monotonic = callback_started
        if (
            self.current_state == "NAVIGATING"
            and corridor.reason != "FRONT_CLEARANCE"
        ):
            evidence_reason, confirmed = self._corridor_failure_evidence()
            # A statically validated narrow segment is an autonomous slow,
            # heading-locked mode. Only confirmed physical blockage may end
            # the segment; uncertainty alone must not hand off to Manual.
            if evidence_reason == "PHYSICALLY_BLOCKED":
                self._enter_narrow_path_decision(evidence_reason, confirmed)
        heading_observation_valid = self._update_scan_map_match(message)
        self._record_heading_observation(
            message, heading_observation_valid=heading_observation_valid
        )
        publish_mapping_scan = False
        if self.mode == "MAPPING":
            with self.state_lock:
                if self.current_state in {"MAPPING", "MAPPING_RUNNING"}:
                    publish_mapping_scan = True
                elif (
                    self.current_state == "MAPPING_LOCALIZING"
                    and self.mapping_relocalization_active
                    and self.mapping_relocalization_probe_count
                    < self.mapping_relocalization_max_probes
                ):
                    # Only a few probe scans may reach SLAM before its corrected
                    # pose is verified. A rejected attempt is rolled back by
                    # deserializing the immutable source graph again.
                    self.mapping_relocalization_probe_count += 1
                    publish_mapping_scan = True
        if publish_mapping_scan:
            self.mapping_scan.publish(message)
        elif self.mode == "NAVIGATION":
            self.navigation_scan.publish(message)
            self.last_navigation_scan_published_monotonic = callback_started
            self.planning_scan.publish(self._planning_scan_message(message))
        self.scan_callback_latency_ms = round(
            (time.monotonic() - callback_started) * 1000.0, 3
        )

    def _sensor_time_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        self.sensor_time_status = status
        self.last_sensor_time_status_monotonic = time.monotonic()
        scan = self._sensor_entry("scan")
        self.scan_clock_skew_seconds = float(scan.get("clock_skew_ms", 0.0)) / 1000.0
    def _safety_callback(self, message: String) -> None:
        # Kept for external compatibility only. Navigation decisions are made
        # exclusively from /safety/status so reason/mask/output share one seq.
        if self.safety_snapshot_sequence < 0:
            self.safety_health = str(message.data)

    def _safety_subscription_watchdog_tick(self) -> None:
        """Recover when Motion Safety restarts its process-local sequence."""
        now = time.monotonic()
        reference = (
            self.safety_snapshot_monotonic
            if self.safety_snapshot_monotonic > 0.0
            else self.safety_subscription_started_monotonic
        )
        if now - reference <= 1.0:
            return
        if now - self.last_safety_subscription_rebind_monotonic <= 2.0:
            return
        old_subscription = self.safety_status_subscription
        if old_subscription is not None:
            self.destroy_subscription(old_subscription)
        # Motion Safety starts again at sequence one after a restart. Reset
        # the old high-water mark before accepting its new epoch.
        self.safety_snapshot = {}
        self.safety_snapshot_sequence = -1
        self.safety_snapshot_monotonic = 0.0
        self.safety_health = "UNKNOWN"
        self.safety_stop_reason = "UNKNOWN"
        self.safety_stop_source = "UNKNOWN"
        self.safety_direction_mask = 0
        self.safety_subscription_started_monotonic = now
        self.last_safety_subscription_rebind_monotonic = now
        self.safety_status_subscription = self.create_subscription(
            String, "/safety/status", self._safety_status_callback, 1,
            callback_group=self.critical_status_callback_group,
        )
        self._nav_debug(
            "SAFETY_SUBSCRIPTION_REBIND",
            reason="STALE_SNAPSHOT_OR_PUBLISHER_RESTART",
        )

    def _safety_status_callback(self, message: String) -> None:
        try:
            snapshot = json.loads(message.data)
            sequence = int(snapshot["seq"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if sequence <= self.safety_snapshot_sequence:
            return
        self.safety_snapshot = dict(snapshot)
        self.safety_snapshot_sequence = sequence
        self.safety_snapshot_monotonic = time.monotonic()
        self.safety_health = str(snapshot.get("health") or "UNKNOWN")
        self.safety_stop_reason = str(snapshot.get("reason") or "UNKNOWN").upper()
        self.safety_stop_source = str(snapshot.get("source") or "UNKNOWN")
        self.safety_direction_mask = int(snapshot.get("direction_mask") or 0)

        blocker_x = snapshot.get("blocking_point_x")
        blocker_y = snapshot.get("blocking_point_y")
        if (
            blocker_x is not None
            and blocker_y is not None
            and self.pose is not None
            and self.localized
            and self.localization_state == "READY"
            and self.safety_stop_source == "MOTION_SAFETY"
        ):
            yaw = float(self.pose.get("yaw", 0.0))
            cosine, sine = math.cos(yaw), math.sin(yaw)
            map_x = float(self.pose["x"]) + cosine * float(blocker_x) - sine * float(blocker_y)
            map_y = float(self.pose["y"]) + sine * float(blocker_x) + cosine * float(blocker_y)
            self.dynamic_overlay.observe(
                ((map_x, map_y),),
                now=time.monotonic(),
                saved_map=self.saved_map,
                static_tolerance=self.dynamic_overlay_static_tolerance,
            )
            self._refresh_dynamic_obstacle_view()

        dynamic_stop = (
            bool(snapshot.get("stop"))
            and self.safety_stop_source == "MOTION_SAFETY"
            and self.safety_stop_reason in {
                "FRONT_SWEEP_COLLISION",
                "REAR_SWEEP_COLLISION",
                "LEFT_TURN_CLEARANCE",
                "RIGHT_TURN_CLEARANCE",
                "ROTATION_SWEEP_COLLISION",
            }
        )
        if (
            dynamic_stop
            and self.current_state == "NAVIGATING"
            and self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}
        ):
            self._enter_dynamic_wait("MOTION_SAFETY_DYNAMIC_BLOCK")

    def _safety_source_callback(self, message: String) -> None:
        if self.safety_snapshot_sequence >= 0:
            return
        source = str(message.data or "UNKNOWN")
        self.safety_stop_source = source
        if (
            not self.navigation_debug_enabled
            or source in {"NONE", "COMMAND_TIMEOUT"}
            or source == self.last_logged_stop_source
        ):
            if source == "NONE":
                self.last_logged_stop_source = ""
            return
        requested = self.pipeline_samples.get("controller_requested")
        final = self.pipeline_samples.get("motion_safety")
        self._nav_debug(
            "STOP",
            reason=(self.safety_stop_reason if self.safety_stop_reason != "NONE" else "SAFETY"),
            source=source,
            direction_mask=self.safety_direction_mask,
            last_path_follow_command=requested,
            current_turn_command=self.last_turn_command,
            final_output_command=final,
        )
        self.last_logged_stop_source = source

    def _estop_callback(self, message: Bool) -> None:
        self.estop_active = bool(message.data)
        if self.estop_active:
            self._stop_localization_rotation()
            with self.state_lock:
                handle = self.current_goal_handle
                self.current_goal_handle = None
                self.motion_owner = "NONE"
                self.navigation_goal_generation += 1
                self.execution_segment_token += 1
                self.active_segment = None
                self.execution_phase = "IDLE"
                if self.current_state == "NAVIGATING":
                    self._set_state("CANCELED", "emergency_stop")
            self.navigation_velocity.publish(Twist())
            if handle is not None:
                handle.cancel_goal_async()

    def _direction_mask_callback(self, message: UInt8) -> None:
        if self.safety_snapshot_sequence < 0:
            self.safety_direction_mask = int(message.data)

    def _atomic_safety_fresh(self, now: float | None = None) -> bool:
        timestamp = time.monotonic() if now is None else float(now)
        return (
            self.safety_snapshot_sequence >= 0
            and timestamp - self.safety_snapshot_monotonic <= 0.35
        )

    def _safety_blocks_turn(self, direction: int) -> bool:
        key = "rotation_left_blocked" if int(direction) > 0 else "rotation_right_blocked"
        mask = 4 if int(direction) > 0 else 8
        return bool(
            self.safety_snapshot.get(key, False)
            or self.safety_direction_mask & mask
        )

    def _manual_takeover_callback(self, message: Bool) -> None:
        if message.data:
            self.last_manual_takeover_monotonic = time.monotonic()
            self._stop_localization_rotation()
        active_navigation = (
            message.data
            and self.mode == "NAVIGATION"
            and self.current_state in {
                "NAVIGATING", "PAUSED", "BLOCKED", "NARROW_PATH_DECISION"
            }
        )
        if active_navigation:
            self.get_logger().warning("manual takeover: canceling Nav2 goal")
            # This subscription shares the scan callback group. Waiting three
            # seconds for Nav2's cancel response starved /scan_navigation and
            # made both costmaps blind during a takeover. The mux already gives
            # manual the higher priority, so invalidate Auto synchronously and
            # let the cancel acknowledgement finish asynchronously.
            with self.state_lock:
                handle = self.current_goal_handle
                self.current_goal_handle = None
                self._set_state("MANUAL_BYPASS", "manual_takeover")
                self.navigation_goal_generation += 1
                self.execution_segment_token += 1
                self.active_segment = None
                self.execution_phase = "IDLE"
                self.manual_handoff_reason = "MANUAL_TAKEOVER"
                self.visualization_revision += 1
            self.profile_limiter.reset()
            self.motion_owner = "NONE"
            self.rotation_metric_active = False
            self.obstacle_slowdown_active = False
            self.navigation_velocity.publish(Twist())
            if handle is not None:
                future = handle.cancel_goal_async()

                def canceled(cancel_future: Any) -> None:
                    try:
                        cancel_future.result()
                    except Exception as exc:
                        self.get_logger().error(
                            f"manual takeover asynchronous cancel failed: {exc}"
                        )

                future.add_done_callback(canceled)
        elif message.data and self.mode == "NAVIGATION":
            # A takeover can land in the short window while Nav2 is accepting
            # a goal but before current_state becomes NAVIGATING. Invalidate
            # that in-flight generation so _navigate cancels the late handle.
            with self.state_lock:
                self.navigation_goal_generation += 1

    def _record_odometry_trajectory(self, transform: Any) -> None:
        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        if self.last_trajectory_odom is not None and (
            math.hypot(
                x - self.last_trajectory_odom[0],
                y - self.last_trajectory_odom[1],
            ) > 0.75
            or abs(self._yaw_delta(yaw, self.last_trajectory_odom[2]))
            > math.radians(90.0)
        ):
            # A discontinuous odom frame cannot share the map<-odom anchor of
            # the preceding segment.  Start a new epoch; only this epoch may
            # be reconstructed once localization is trusted again.
            self.trajectory_odom_epoch += 1
            self.trajectory_map_from_odom = None
            self._nav_debug(
                "TRAJECTORY_ODOM_DISCONTINUITY",
                odom_epoch=self.trajectory_odom_epoch,
                previous=list(self.last_trajectory_odom),
                current=[x, y, yaw],
            )
        if (
            self.last_trajectory_odom is not None
            and math.hypot(
                x - self.last_trajectory_odom[0],
                y - self.last_trajectory_odom[1],
            ) < 0.03
            and abs(self._yaw_delta(yaw, self.last_trajectory_odom[2]))
            < math.radians(5.0)
        ):
            return
        stamp = getattr(getattr(transform, "header", None), "stamp", None)
        sensor_timestamp = (
            float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
            if stamp is not None and (int(stamp.sec) or int(stamp.nanosec))
            else time.time()
        )
        point: dict[str, Any] = {
            "timestamp": sensor_timestamp,
            "odom_x": x,
            "odom_y": y,
            "odom_yaw": yaw,
            "odom_epoch": self.trajectory_odom_epoch,
            "frame": "odom",
            "quality": "UNANCHORED",
        }
        if self.trajectory_map_from_odom is not None:
            offset_x, offset_y, offset_yaw = self.trajectory_map_from_odom
            cosine, sine = math.cos(offset_yaw), math.sin(offset_yaw)
            point.update({
                "x": offset_x + cosine * x - sine * y,
                "y": offset_y + sine * x + cosine * y,
                "yaw": self._yaw_delta(yaw + offset_yaw, 0.0),
                "frame": "map",
                "quality": "TRUSTED",
                "map_id": self.map_id,
                "map_version": self.map_version,
            })
        self.odometry_trajectory.append(point)
        self.odometry_trajectory = self.odometry_trajectory[-2000:]
        self.last_trajectory_odom = (x, y, yaw)

    def _anchor_odometry_trajectory(self) -> None:
        if self.pose is None:
            return
        try:
            odom = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException:
            return
        odom_x = float(odom.transform.translation.x)
        odom_y = float(odom.transform.translation.y)
        odom_yaw = self._yaw_from_quaternion(odom.transform.rotation)
        offset_yaw = self._yaw_delta(float(self.pose["yaw"]), odom_yaw)
        cosine, sine = math.cos(offset_yaw), math.sin(offset_yaw)
        offset_x = float(self.pose["x"]) - cosine * odom_x + sine * odom_y
        offset_y = float(self.pose["y"]) - sine * odom_x - cosine * odom_y
        self.trajectory_map_from_odom = (offset_x, offset_y, offset_yaw)
        self.localization_odometry_prior_rejected_epoch = None
        reconstructed = 0
        for point in self.odometry_trajectory:
            if (
                point.get("quality") != "UNANCHORED"
                or int(point.get("odom_epoch", 0)) != self.trajectory_odom_epoch
            ):
                continue
            x = float(point["odom_x"])
            y = float(point["odom_y"])
            point.update({
                "x": offset_x + cosine * x - sine * y,
                "y": offset_y + sine * x + cosine * y,
                "yaw": self._yaw_delta(
                    float(point["odom_yaw"]) + offset_yaw, 0.0
                ),
                "frame": "map",
                "quality": "RECONSTRUCTED",
                "map_id": self.map_id,
                "map_version": self.map_version,
            })
            reconstructed += 1
        self._nav_debug(
            "TRAJECTORY_ANCHORED",
            attempt_id=self.localization_attempt_id,
            reconstructed_points=reconstructed,
            trusted_map_id=self.map_id,
            trusted_map_version=self.map_version,
        )

    def _odometry_predicted_map_pose(self) -> dict[str, float] | None:
        """Project the current measured odometry through a trusted map anchor."""
        if (
            self.trajectory_map_from_odom is None
            or self.saved_map is None
            or not self.map_id
            or not self._critical_sensor_time_healthy()
            or self.localization_odometry_prior_rejected_epoch
            == self.trajectory_odom_epoch
        ):
            return None
        try:
            odom = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException:
            return None
        odom_x = float(odom.transform.translation.x)
        odom_y = float(odom.transform.translation.y)
        odom_yaw = self._yaw_from_quaternion(odom.transform.rotation)
        offset_x, offset_y, offset_yaw = self.trajectory_map_from_odom
        cosine, sine = math.cos(offset_yaw), math.sin(offset_yaw)
        predicted = {
            "x": offset_x + cosine * odom_x - sine * odom_y,
            "y": offset_y + sine * odom_x + cosine * odom_y,
            "yaw": self._yaw_delta(odom_yaw + offset_yaw, 0.0),
            # The anchor was created only after READY. Keep enough uncertainty
            # for AMCL correction without broadening into another corridor.
            "covariance": 0.01,
        }
        if not all(math.isfinite(value) for value in predicted.values()):
            return None
        if self.saved_map.world_to_cell(predicted["x"], predicted["y"]) is None:
            return None
        return predicted

    def _update_pose(self) -> None:
        if (
            self.mode == "NAVIGATION"
            and self.current_state == "STARTING"
            and self.map_load_client.service_is_ready()
            and self.compute_path_client.server_is_ready()
            and self.follow_path_client.server_is_ready()
        ):
            with self.state_lock:
                # Nav2 processes being healthy does not mean a saved map is
                # loaded. Reporting READY here made Center treat a registry
                # assignment as an active map after every stack restart.
                self._set_state("NO_ACTIVE_MAP", "nav2_ready_without_active_map")
        try:
            odom_transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
            self._record_odometry_trajectory(odom_transform)
        except TransformException:
            pass
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
        except TransformException:
            return
        self.last_map_tf_monotonic = time.monotonic()
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation)
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": yaw,
        }
        if not self._execution_pose_candidate_accepted(pose):
            return
        if (
            (
                (
                    self.mode == "MAPPING"
                    and self.current_state in {"MAPPING", "MAPPING_RUNNING"}
                )
                or (
                    self.mode == "NAVIGATION"
                    and self.localized
                    and self.localization_state == "READY"
                )
            )
            and (
                not self.trail
                or math.hypot(
                    pose["x"] - self.trail[-1]["x"],
                    pose["y"] - self.trail[-1]["y"],
                )
                >= 0.03
            )
        ):
            self.trail.append({"x": pose["x"], "y": pose["y"]})
            self.trail = self.trail[-2000:]
        self.pose = pose
        if (
            self.mode == "NAVIGATION"
            and self.localized
            and self.localization_state == "READY"
            and self.trajectory_map_from_odom is None
        ):
            self._anchor_odometry_trajectory()

    def _publish_initial_pose(
        self,
        pose: dict[str, Any],
        *,
        approximate: bool,
    ) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        # Stamp the hint at a transform that already exists. AMCL converts a
        # zero initial-pose stamp to its callback time; on the Pi that raced
        # the EKF by ~70 ms and intermittently discarded the seed as a future
        # extrapolation. Using the latest actual odom transform is causal and
        # does not rewrite any sensor capture timestamp.
        try:
            latest_odom = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
            message.header.stamp = latest_odom.header.stamp
        except TransformException:
            # Startup may load the saved map before the first normalized odom
            # sample reaches this listener. Publishing a zero stamp here makes
            # AMCL substitute its newer callback time and reject the hint.
            # Leave it unsent; the bounded retry in _localization_tick will
            # publish as soon as a causal odom transform exists.
            self.initial_pose_requested = False
            self.last_initial_pose_publish_monotonic = time.monotonic()
            return
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        message.pose.pose.orientation = quaternion_from_yaw(float(pose.get("yaw", 0)))
        # An approximate point means "search near here", not "trust this
        # position and heading".  A one-metre position variance scattered too
        # few particles across this small indoor map, while the old 30-degree
        # heading variance excluded the real pose whenever an operator did not
        # know which way the robot was facing.  Keep the search local enough to
        # converge from stationary scans and cover the complete heading range.
        if approximate:
            position_variance = 0.36
            yaw_variance = math.pi ** 2 / 3.0
        else:
            # A recent navigation pose has already passed the sustained
            # scan/map, covariance and stability gates. The former 20 cm
            # standard deviation let stationary AMCL jump far enough for the
            # robot centre to land in an adjacent 5 cm wall cell after every
            # restart. Preserve at least 10 cm of uncertainty for correction,
            # but do not deliberately broaden an already verified pose.
            position_variance = max(0.01, float(pose.get("covariance", 0.25)))
            yaw_variance = 0.02
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[35] = yaw_variance
        self.initial_pose.publish(message)
        self.initial_pose_requested = True
        self.last_initial_pose_publish_monotonic = time.monotonic()

    @localization_serialized
    def _reset_localization_evidence(self) -> None:
        """Discard every sample that could validate a previous AMCL hypothesis."""
        self.localized = False
        self.localization_confidence = 0.0
        self.last_amcl_pose = None
        self.last_amcl_covariance = []
        self.pose_window.clear()
        self.pose_stability_metrics = pose_stability(())
        self.scan_map_scores.clear()
        self.scan_map_median_residuals.clear()
        self.scan_map_p90_residuals.clear()
        self.scan_map_mean_residuals.clear()
        self.raycast_static_match_ratios.clear()
        self.raycast_dynamic_occlusion_ratios.clear()
        self.raycast_contradiction_ratios.clear()
        self.raycast_median_errors.clear()
        self.raycast_p90_errors.clear()
        self.localization_evidence_frames.clear()
        self.localization_consensus = LocalizationConsensus(
            False,
            "CONSENSUS_WINDOW_INCOMPLETE",
            0,
            self.localization_consensus_required_frames,
            0,
            0,
        )
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
        self.raycast_comparable_beams = 0
        self.raycast_static_matches = 0
        self.raycast_dynamic_occlusions = 0
        self.raycast_map_contradictions = 0
        self.raycast_inconclusive_map_hits = 0
        self.raycast_matched_beams = 0
        self.raycast_static_match_ratio = 0.0
        self.raycast_dynamic_occlusion_ratio = 0.0
        self.raycast_contradiction_ratio = 0.0
        self.raycast_match_ratio = 0.0
        self.raycast_median_error = math.inf
        self.raycast_p90_error = math.inf
        self.localization_evidence_headings.clear()
        self.localization_heading_positions.clear()
        self.localization_heading_bins = ()
        self.localization_heading_span = 0.0
        self.last_scan_map_monotonic = 0.0
        self.verification_scan_count = 0
        self.verification_started_monotonic = 0.0
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None
        self.low_confidence_since = None
        self.last_nomotion_request_monotonic = 0.0
        self.localization_rotation_blocked_since = None
        self.global_search_rotation_pending = False
        self.global_search_untrusted = False
        self.stationary_global_candidate_ambiguous = False
        self.localization_operator_hint_active = False
        self.localization_pending_operator_hint = None
        self.localization_odometry_prior_active = False
        self.approximate_hint_allowed = False
        self.last_particle_cloud_monotonic = 0.0
        self.particle_uniqueness = ParticleCloudUniqueness(
            False,
            "PARTICLE_CLOUD_UNAVAILABLE",
            0,
            0,
            0.0,
            0.0,
            0.0,
        )
        self.global_scan_evaluation_generation += 1
        self.global_scan_uniqueness = GlobalScanUniqueness(
            False, "GLOBAL_SCAN_NOT_EVALUATED", 0, 0, 0.0, 0.0, 0.0, 0.0
        )
        self.global_scan_uniqueness_in_progress = False
        self.global_scan_evaluated_candidate = None
        self.latest_localization_scan_snapshot = None

    def _begin_localization_verification(self, *, allow_rotation: bool) -> None:
        """Verify the current AMCL cloud without resetting its particles."""
        now = time.monotonic()
        self._stop_localization_rotation()
        # This verification begins only after autonomous motion has been
        # revoked.  Its stability window must therefore contain fresh
        # stationary AMCL samples, not the trailing samples from the route
        # that was just paused.
        self._reset_stationary_verification_evidence()
        self.localized = False
        self.localization_state = "VERIFYING"
        self._set_state("VERIFYING", "verify_existing_amcl_pose")
        self.localization_started_monotonic = now
        self.localization_phase_started_monotonic = now
        self.verification_started_monotonic = now
        self.verification_scan_count = 0
        self.localization_rotation_authorized = allow_rotation
        self.global_search_requires_rotation = False
        self.global_search_rotation_pending = False
        self.global_search_untrusted = False
        self.last_nomotion_request_monotonic = 0.0
        self.low_confidence_since = None
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None

    @localization_serialized
    def _begin_auto_localization(self, last_pose: Any) -> None:
        now = time.monotonic()
        self._reset_localization_evidence()
        self.localization_rotation_authorized = False
        self.localization_started_monotonic = now
        self.localization_phase_started_monotonic = now
        self.rotation_angle = 0.0
        self.rotation_active = False
        self.rotation_yaw_progress.reset(
            None if self.pose is None else float(self.pose.get("yaw", 0.0))
        )
        self.localization_seed_pose = None
        self.localization_seed_approximate = False
        self.global_search_requires_rotation = False
        self.global_search_untrusted = False
        if isinstance(last_pose, dict) and all(
            axis in last_pose and math.isfinite(float(last_pose[axis]))
            for axis in ("x", "y", "yaw")
        ):
            self.localization_seed_pose = dict(last_pose)
            recent_verified_pose = (
                str(last_pose.get("source", "")) == "recent_navigation_pose"
                and int(last_pose.get("verification_version", 0)) >= 2
                and 0.0 <= time.time() - float(last_pose.get("timestamp", 0.0)) <= 3600.0
                and float(last_pose.get("covariance", 1.0)) <= 0.25
            )
            # A recent navigation pose already survived 30 seconds of scan/map,
            # covariance, stability and sensor-clock gates before persistence.
            # Recheck it locally with fresh scans and its bounded heading first;
            # a mismatch times out into the multi-heading global search below.
            # Mapping terminal poses and legacy records remain broad hints.
            self.localization_seed_approximate = not recent_verified_pose
            self.global_search_untrusted = self.localization_seed_approximate
            self.global_search_requires_rotation = False
            self._publish_initial_pose(
                self.localization_seed_pose,
                approximate=self.localization_seed_approximate,
            )
            self.localization_state = "LOCALIZING_LAST_POSE"
            self._set_state("LOCALIZING_LAST_POSE", "recent_pose_seed")
        else:
            # Particle search is passive and does not imply velocity ownership.
            # It begins as soon as the active Saved Map is available; only the
            # later disambiguating rotation remains operator-authorized.
            self._start_global_localization()

    @localization_serialized
    def _start_global_localization(self) -> None:
        self._stop_localization_rotation()
        self._reset_localization_evidence()
        self.localization_attempt_sequence += 1
        self.localization_attempt_id = (
            f"{self.map_id or 'no-map'}:{self.map_version}:"
            f"{self.localization_attempt_sequence}"
        )
        self.localization_settling_evidence_started = False
        self.rotation_angle = 0.0
        self.rotation_yaw_progress.reset(
            None if self.pose is None else float(self.pose.get("yaw", 0.0))
        )
        self.localization_seed_pose = None
        self.localization_seed_approximate = False
        # Stationary-first: reset/search globally, request fresh no-motion
        # updates, and accept a strong stable candidate before using rotation
        # to resolve genuine ambiguity.
        self.global_search_requires_rotation = False
        self.global_search_rotation_pending = True
        self.global_search_untrusted = True
        self.stationary_global_candidate_ambiguous = False
        self.localization_rotation_cycle_start_angle = 0.0
        self.localization_next_observation_angle = 0.0
        self.localization_tf_unavailable_since = None
        if not self.global_localization_client.wait_for_service(timeout_sec=3.0):
            self.localization_state = "LOCALIZATION_FAILED"
            self._set_state("LOCALIZATION_FAILED", "global_localization_service_unavailable")
            raise AdapterError(
                "GLOBAL_LOCALIZATION_UNAVAILABLE",
                "AMCL global localization service is unavailable",
            )
        # This method can run from the localization timer, which shares the
        # node's default callback group with the service response. Waiting on
        # that future here deadlocks the response callback, then retries every
        # tick and repeatedly resets AMCL's particle filter. Transition first
        # and let the executor deliver the response asynchronously.
        self.localization_phase_started_monotonic = time.monotonic()
        self.localization_state = "PASSIVE_LOCALIZING"
        self._set_state("PASSIVE_LOCALIZING", "passive_global_localization_started")
        self._nav_debug(
            "LOCALIZATION_ATTEMPT",
            attempt_id=self.localization_attempt_id,
            mode="PASSIVE_GLOBAL",
            rotation_authorized=self.localization_rotation_authorized,
        )
        self.initial_pose_requested = False
        future = self.global_localization_client.call_async(Empty.Request())

        def completed(response_future: Any) -> None:
            try:
                response_future.result()
            except Exception as exc:
                self.get_logger().error(f"global localization failed: {exc}")
                self._stop_localization_rotation()
                self.localization_state = "LOCALIZATION_FAILED"
                self._set_state("LOCALIZATION_FAILED", "global_localization_call_failed")

        future.add_done_callback(completed)

    def _safe_to_rotate(self) -> bool:
        commanded_direction = 4 if self.rotation_speed > 0 else 8
        return (
            time.monotonic() - self.last_scan_monotonic <= 0.30
            and self._critical_sensor_time_healthy()
            and self.tf_buffer.can_transform(
                "base_footprint", "laser_frame", Time()
            )
            and (
                self.safety_health.startswith("HEALTHY")
                or self.safety_health.startswith("BLOCKED")
            )
            and not self.estop_active
            and not (self.safety_direction_mask & commanded_direction)
            and time.monotonic() - self.last_manual_takeover_monotonic > 0.5
            and self.current_goal_handle is None
            and self.current_state != "NAVIGATING"
            # Global localization starts precisely when AMCL has no trusted
            # pose. Requiring last_amcl_pose or scan/map agreement here creates
            # a deadlock because both depend on the map->base transform that
            # rotation is intended to discover. They remain mandatory in the
            # READY evidence gate; rotation safety itself uses live raw LiDAR,
            # base->laser TF, the safety node and clearance below.
            and self.nearest_rotation_obstacle
            >= self.rotation_minimum_obstacle_distance
        )

    def _stop_localization_rotation(self) -> None:
        if self.rotation_active or self.motion_owner == "LOCALIZATION":
            self.localization_velocity.publish(Twist())
        self.rotation_active = False
        self.rotation_last_monotonic = 0.0
        if self.motion_owner == "LOCALIZATION":
            self.motion_owner = "NONE"

    def _degrade_localization(
        self,
        reason: str,
        diagnostics: dict[str, Any],
    ) -> None:
        """Safely pause an active mission for a recoverable timing fault."""
        if self.localization_state == "SENSOR_TIME_INVALID":
            return
        now = time.monotonic()
        self._stop_localization_rotation()
        with self.state_lock:
            handle = self.current_goal_handle
            mission_active = (
                self.paused_goal is not None
                and self.current_state in {
                    "NAVIGATING",
                    "WAIT_FOR_DYNAMIC_CLEAR",
                    "WAITING_FOR_DYNAMIC_CLEAR",
                    "DYNAMIC_REPLAN",
                    "RECOVERY",
                    "TURN_BAY_RECOVERY",
                }
            )
            if mission_active:
                self.sensor_time_resume_context = {
                    "goal": dict(self.paused_goal),
                    "mission_id": self.current_mission_id,
                    "route_id": self.execution_route_id or self.selected_route_id,
                    "path": list(
                        self.dynamic_blocked_route or self.latest_global_path
                    ),
                }
            self.sensor_time_previous_localization_state = self.localization_state
            self.sensor_time_pause_started_monotonic = now
            self.sensor_time_failure_reason = reason
            self.sensor_time_failure_diagnostics = dict(diagnostics)
            self.sensor_time_resume_in_progress = False
            self.sensor_time_hard_failed = False
            self.localized = False
            self.trajectory_map_from_odom = None
            self.localization_state = "SENSOR_TIME_INVALID"
            self._set_state("SENSOR_TIME_INVALID", "sensor_time_safe_pause")
            self.latest_feedback["terminal_reason"] = f"SENSOR_TIME_PAUSED:{reason}"
            self.current_goal_handle = None
            self.navigation_goal_generation += 1
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_phase = "IDLE"
            self.motion_owner = "NONE"

        self.get_logger().error(
            "localization timing fault; safely pausing navigation: "
            f"reason={reason} diagnostics={json.dumps(diagnostics, sort_keys=True)}"
        )
        self._nav_debug(
            "SENSOR_TIME_PAUSE",
            reason=reason,
            status_age_ms=diagnostics.get("status_age_ms"),
            scan=diagnostics.get("scan"),
            odom=diagnostics.get("odom"),
            destination_preserved=bool(self.paused_goal),
            mission_id=self.current_mission_id,
            route_id=self.selected_route_id,
        )
        # Revoke Nav2 ownership before waiting for its asynchronous cancel
        # result.  This makes stale controller messages harmless immediately.
        self.navigation_velocity.publish(Twist())
        self.localization_velocity.publish(Twist())
        self.profile_limiter.reset()
        if handle is not None:
            handle.cancel_goal_async()
        self.sensor_time_invalid_since = now

    def _restore_after_sensor_time_pause(self, now: float) -> None:
        """Resume verification only after synchronized sensor evidence returns."""
        previous = self.sensor_time_previous_localization_state
        paused_at = self.sensor_time_pause_started_monotonic
        paused_duration = (
            0.0 if paused_at is None else max(0.0, now - paused_at)
        )
        self.sensor_time_invalid_since = None
        self.sensor_time_pause_started_monotonic = None
        self.sensor_time_failure_reason = ""
        self.sensor_time_failure_diagnostics = {}

        # A Force Rescan may have already accepted a candidate/ready hold when
        # a short timestamp glitch arrived.  Resume that exact evidence phase
        # rather than discarding AMCL particles or the active ready hold.
        preserved_verification_states = {
            "VERIFYING",
            "LOCALIZING_LAST_POSE",
            "LOCALIZING_APPROXIMATE_POSE",
            "PASSIVE_LOCALIZING",
            "CANDIDATE",
            "AMBIGUOUS",
            "LOCALIZING_GLOBAL",
            "LOCALIZING_ROTATING",
            "LOCALIZING_SETTLING",
        }
        if previous in preserved_verification_states:
            if self.localization_phase_started_monotonic:
                self.localization_phase_started_monotonic += paused_duration
            if self.localization_started_monotonic:
                self.localization_started_monotonic += paused_duration
            if self.verification_started_monotonic:
                self.verification_started_monotonic += paused_duration
            if self.ready_evidence_since is not None:
                self.ready_evidence_since += paused_duration
            self.localization_state = previous
            self._set_state(previous, "sensor_time_recovered_continue_verification")
            self.sensor_time_previous_localization_state = ""
            self._nav_debug(
                "SENSOR_TIME_RECOVERED",
                action="CONTINUE_VERIFICATION",
                previous_localization_state=previous,
                pause_ms=round(paused_duration * 1000.0, 1),
            )
            return

        self.sensor_time_previous_localization_state = ""
        if self.last_amcl_pose is not None:
            # The FollowPath action is canceled above, so the fresh stationary
            # window in _begin_localization_verification can safely replace
            # moving samples from before the pause.
            self._begin_localization_verification(allow_rotation=False)
            self._nav_debug(
                "SENSOR_TIME_RECOVERED",
                action="VERIFY_CURRENT_AMCL_POSE",
                pause_ms=round(paused_duration * 1000.0, 1),
            )
            return
        try:
            self._start_global_localization()
        except AdapterError as exc:
            self.localization_state = "LOCALIZATION_FAILED"
            self._set_state(
                "LOCALIZATION_FAILED", "sensor_time_recovery_global_unavailable"
            )
            self.get_logger().error(str(exc))

    def _fail_sustained_sensor_time_pause(self, reason: str) -> None:
        """Require operator localization after a bounded, sustained outage."""
        self.sensor_time_resume_in_progress = False
        self.sensor_time_hard_failed = True
        self.localized = False
        self.trajectory_map_from_odom = None
        # Keep the timing-fault state until synchronized data returns. The
        # recovery path will then verify the surviving AMCL cloud or start a
        # new passive global search; it must not become stuck waiting for UI.
        self.localization_state = "SENSOR_TIME_INVALID"
        self._set_state("SENSOR_TIME_INVALID", "sensor_time_sustained_failure")
        self.latest_feedback["terminal_reason"] = f"SENSOR_TIME_SUSTAINED:{reason}"
        self._nav_debug(
            "SENSOR_TIME_FAILURE",
            reason=reason,
            action="LOCALIZATION_REQUIRED",
            destination_preserved=bool(self.paused_goal),
        )

    def _resume_sensor_time_navigation_if_ready(self) -> None:
        """Replan once from the verified current pose to the preserved goal."""
        with self.state_lock:
            context = self.sensor_time_resume_context
            if (
                context is None
                or self.sensor_time_resume_in_progress
                or not self.localized
                or self.localization_state != "READY"
                or not self._critical_sensor_time_healthy()
            ):
                return
            self.sensor_time_resume_in_progress = True
            goal = dict(context["goal"])
            route_id = str(context.get("route_id") or "sensor-time-resume")
            mission_id = str(context.get("mission_id") or self.current_mission_id)
            self._set_state("PLANNING", "sensor_time_recovery_replan")

        def resume() -> None:
            try:
                if (
                    not self.localized
                    or self.localization_state != "READY"
                    or not self._critical_sensor_time_healthy()
                ):
                    raise AdapterError(
                        "LOCALIZATION_UNRELIABLE",
                        "Localization is not ready to resume the paused mission",
                    )
                points = self._plan_stop_turn_from_current(goal)
                if len(points) < 2:
                    raise AdapterError(
                        "NO_VALID_PATH",
                        "No executable path remains to the preserved destination",
                    )
                self._navigate(
                    goal,
                    {
                        "map_id": self.map_id,
                        "version": self.map_version,
                        "mission_id": mission_id,
                        "route_id": route_id,
                        "points": points,
                    },
                )
            except AdapterError as exc:
                with self.state_lock:
                    self._set_state("PAUSED", "sensor_time_recovery_replan_failed")
                    self.latest_feedback["terminal_reason"] = (
                        f"SENSOR_TIME_RECOVERY_{exc.code}"
                    )
                    self.sensor_time_resume_context = None
                    self.sensor_time_resume_in_progress = False
                self._nav_debug(
                    "SENSOR_TIME_RECOVERED",
                    action="REPLAN_FAILED",
                    error=exc.code,
                )
                return
            with self.state_lock:
                self.sensor_time_resume_context = None
                self.sensor_time_resume_in_progress = False
            self._nav_debug(
                "SENSOR_TIME_RECOVERED",
                action="REPLAN_RESUMED",
                destination=goal,
                route_id=route_id,
                mission_id=mission_id,
            )

        threading.Thread(target=resume, daemon=True).start()

    def _resume_localization_navigation_if_ready(self) -> None:
        """Exact-revalidate the preserved route, then replan if it changed."""
        now = time.monotonic()
        with self.state_lock:
            context = self.localization_resume_context
            if (
                context is None
                or self.localization_resume_in_progress
                or not self.localized
                or self.localization_state != "READY"
                or now - self.last_localization_resume_attempt_monotonic < 2.0
            ):
                return
            self.localization_resume_in_progress = True
            self.last_localization_resume_attempt_monotonic = now
            goal = dict(context["goal"])
            mission_id = str(context.get("mission_id") or self.current_mission_id)
            old_route_id = str(context.get("route_id") or "localization-resume")
            original_points = [dict(point) for point in context.get("path") or []]
            original_directions = list(context.get("segment_directions") or [])
            self._set_state("PLANNING", "localization_recovery_revalidate")

        def resume() -> None:
            action = "REPLAN_RESUMED"
            route_id = f"{old_route_id}-relocalized"
            try:
                if self.pose is None:
                    raise AdapterError(
                        "LOCALIZATION_UNRELIABLE",
                        "Verified pose is unavailable for route revalidation",
                    )
                points = list(original_points)
                if len(points) >= 2:
                    points[0] = {
                        "x": float(self.pose["x"]),
                        "y": float(self.pose["y"]),
                    }
                    points = canonicalize_stop_turn_path(points)
                    if len(original_directions) != max(0, len(points) - 1):
                        original_directions = [
                            1 for _ in range(max(0, len(points) - 1))
                        ]
                    try:
                        self._navigate(
                            goal,
                            {
                                "map_id": self.map_id,
                                "version": self.map_version,
                                "mission_id": mission_id,
                                "route_id": route_id,
                                "points": points,
                                "segment_directions": original_directions,
                            },
                            recovery_attempt=True,
                        )
                    except AdapterError:
                        points = []
                    else:
                        action = "ORIGINAL_ROUTE_REVALIDATED_AND_RESUMED"
                if len(points) < 2:
                    replanned = self._plan_stop_turn_from_current(goal)
                    if len(replanned) < 2:
                        raise AdapterError(
                            "NO_VALID_PATH",
                            "No exact-valid route remains after relocalization",
                        )
                    route_id = (
                        f"{old_route_id}-relocalized-"
                        f"{self._route_signature(replanned)[:8]}"
                    )
                    self._navigate(
                        goal,
                        {
                            "map_id": self.map_id,
                            "version": self.map_version,
                            "mission_id": mission_id,
                            "route_id": route_id,
                            "points": replanned,
                        },
                        recovery_attempt=True,
                    )
            except AdapterError as exc:
                with self.state_lock:
                    self.localization_resume_in_progress = False
                    self._set_state("PAUSED", "localization_recovery_route_wait")
                    self.latest_feedback["recovery_reason"] = exc.code
                    self.latest_feedback["destination_preserved"] = True
                self._nav_debug(
                    "LOCALIZATION_RECOVERY",
                    action="WAIT_AND_RETRY",
                    error=exc.code,
                    destination=goal,
                    destination_preserved=True,
                )
                return
            with self.state_lock:
                self.localization_resume_context = None
                self.localization_resume_in_progress = False
            self._nav_debug(
                "LOCALIZATION_RECOVERY",
                action=action,
                destination=goal,
                route_id=route_id,
                mission_id=mission_id,
            )

        threading.Thread(target=resume, daemon=True).start()

    def _localization_lost(self, reason: str) -> None:
        if not self.localized:
            return
        self.get_logger().error(f"localization lost; stopping Nav2: {reason}")
        mission_active = bool(
            self.paused_goal is not None
            and self.current_state in {
                "NAVIGATING",
                "WAIT_FOR_DYNAMIC_CLEAR",
                "WAITING_FOR_DYNAMIC_CLEAR",
                "DYNAMIC_REPLAN",
                "RECOVERY",
                "TURN_BAY_RECOVERY",
            }
        )
        if mission_active:
            remaining_route = (
                self.dynamic_blocked_route or self._remaining_execution_route()
            )
            remaining_directions = list(
                self.execution_segment_directions[self.execution_segment_index:]
            )
            self.localization_resume_context = {
                "goal": dict(self.paused_goal),
                "mission_id": self.current_mission_id,
                "route_id": self.execution_route_id or self.selected_route_id,
                "path": list(remaining_route),
                "segment_directions": remaining_directions,
            }
            self.localization_resume_in_progress = False
        self.localized = False
        self.trajectory_map_from_odom = None
        self.localization_state = "LOCALIZATION_LOST"
        self._set_state("LOCALIZATION_LOST", "localization_evidence_lost")
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.navigation_goal_generation += 1
        self.execution_segment_token += 1
        self.active_segment = None
        self.execution_phase = "IDLE"
        if handle is not None:
            handle.cancel_goal_async()
        self.motion_owner = "NONE"
        self.localization_velocity.publish(Twist())
        self.latest_global_path = []
        self.visualization_revision += 1
        self._nav_debug(
            "LOCALIZATION_RECOVERY",
            action="MISSION_PAUSED_FOR_RELOCALIZATION",
            reason=reason,
            destination_preserved=bool(self.localization_resume_context),
            mission_id=self.current_mission_id,
        )
        try:
            # Preserve an explicit operator rotation authorization across a
            # failed verification; passive session checks remain passive.
            self.localization_started_monotonic = time.monotonic()
            self._start_global_localization()
        except AdapterError as exc:
            self.get_logger().error(str(exc))

    def _global_heading_requirement(self) -> tuple[int, float]:
        if self.stationary_global_candidate_ambiguous:
            return self.global_minimum_heading_bins, self.global_minimum_heading_span
        return (
            self.global_strong_minimum_heading_bins,
            self.global_strong_minimum_heading_span,
        )

    def _global_heading_diversity_ready(self) -> bool:
        required_bins, required_span = self._global_heading_requirement()
        return bool(
            len(self.localization_heading_bins) >= required_bins
            and self.localization_heading_span >= required_span
        )

    def _required_localization_scan_score(self) -> float:
        """Keep global-origin candidates on global criteria through READY."""
        if self.localization_operator_hint_active:
            # A bounded position hint plus strict residual, contradiction,
            # covariance, consensus and particle-uniqueness gates supplies
            # stronger local evidence than this occlusion-sensitive coarse
            # endpoint score. Keep a substantial floor without applying the
            # blind global-search score to a cluttered route endpoint.
            return self.localization_operator_hint_minimum_scan_score
        if not self.global_search_untrusted:
            return self.scan_map_threshold
        if (
            self.localization_state in {"PASSIVE_LOCALIZING", "LOCALIZING_GLOBAL"}
            and self.global_search_rotation_pending
        ):
            return self.global_scan_map_threshold
        return self.global_final_scan_map_threshold

    def _localization_verdict(
        self,
        now: float,
        *,
        required_scan_score: float,
        require_heading: bool,
    ) -> LocalizationVerification:
        hinted_candidate = bool(self.localization_operator_hint_active)
        trusted_continuity_candidate = bool(
            not self.global_search_untrusted
            and not self.localization_seed_approximate
            and (
                self.localization_odometry_prior_active
                or self.localization_state
                in {"LOCALIZING_LAST_POSE", "VERIFYING"}
            )
        )
        maximum_raycast_contradiction_ratio = (
            self.localization_tracking_maximum_contradiction_ratio
            if trusted_continuity_candidate
            else self.localization_raycast_maximum_contradiction_ratio
        )
        minimum_raycast_beams = (
            self.localization_operator_hint_minimum_raycast_beams
            if hinted_candidate
            else self.localization_raycast_minimum_beams
        )
        minimum_raycast_static_matches = (
            self.localization_operator_hint_minimum_static_matches
            if hinted_candidate
            else self.localization_raycast_minimum_static_matches
        )
        consensus = localization_evidence_consensus(
            self.localization_evidence_frames,
            window_size=self.localization_consensus_window_size,
            required_frames=self.localization_consensus_required_frames,
            candidate_position_tolerance=(
                self.localization_consensus_position_tolerance
            ),
            candidate_yaw_tolerance=self.localization_consensus_yaw_tolerance,
            minimum_scan_beams=self.scan_map_minimum_beams,
            required_scan_score=required_scan_score,
            minimum_residual_beams=self.localization_final_minimum_residual_beams,
            maximum_median_residual=self.localization_final_max_median_residual,
            maximum_p90_residual=self.localization_final_max_p90_residual,
            minimum_raycast_beams=minimum_raycast_beams,
            minimum_raycast_static_matches=minimum_raycast_static_matches,
            maximum_raycast_contradiction_ratio=(
                maximum_raycast_contradiction_ratio
            ),
        )
        self.localization_consensus = consensus
        if not consensus.accepted:
            return LocalizationVerification(False, consensus.reason)
        if (
            hinted_candidate
            and (
                consensus.raycast_static_match_ratio
                + consensus.raycast_dynamic_occlusion_ratio
            ) < self.localization_operator_hint_minimum_explained_ratio
        ):
            return LocalizationVerification(
                False, "OPERATOR_HINT_EXPLAINED_RATIO_TOO_LOW"
            )
        self.scan_map_score = round(consensus.scan_score, 4)
        self.scan_map_valid_beams = consensus.valid_beams
        self.scan_map_residual_beams = consensus.residual_beams
        self.scan_map_median_residual = consensus.median_residual
        self.scan_map_p90_residual = consensus.p90_residual
        self.scan_map_mean_residual = consensus.mean_residual
        self.raycast_comparable_beams = consensus.raycast_comparable_beams
        self.raycast_static_matches = consensus.raycast_static_matches
        self.raycast_dynamic_occlusions = consensus.raycast_dynamic_occlusions
        self.raycast_map_contradictions = (
            consensus.raycast_map_contradictions
        )
        self.raycast_inconclusive_map_hits = (
            consensus.raycast_inconclusive_map_hits
        )
        self.raycast_static_match_ratio = consensus.raycast_static_match_ratio
        self.raycast_dynamic_occlusion_ratio = (
            consensus.raycast_dynamic_occlusion_ratio
        )
        self.raycast_contradiction_ratio = consensus.raycast_contradiction_ratio
        self.raycast_match_ratio = consensus.raycast_static_match_ratio
        self.raycast_median_error = consensus.raycast_median_error
        self.raycast_p90_error = consensus.raycast_p90_error
        covariance_xy: float | None = None
        covariance_yaw: float | None = None
        if len(self.last_amcl_covariance) >= 36:
            covariance_xy = max(
                0.0, float(self.last_amcl_covariance[0])
            ) + max(0.0, float(self.last_amcl_covariance[7]))
            covariance_yaw = max(
                0.0, float(self.last_amcl_covariance[35])
            )
        verdict = localization_verification(
            confidence=self.localization_confidence,
            confidence_threshold=self.localization_confidence_threshold,
            pose_stable=self._pose_is_stable(),
            covariance_xy=covariance_xy,
            covariance_yaw=covariance_yaw,
            maximum_covariance_xy=self.localization_maximum_covariance_xy,
            maximum_covariance_yaw=self.localization_maximum_covariance_yaw,
            scan_valid_beams=self.scan_map_valid_beams,
            minimum_scan_beams=self.scan_map_minimum_beams,
            scan_score=self.scan_map_score,
            required_scan_score=required_scan_score,
            residual_beams=self.scan_map_residual_beams,
            minimum_residual_beams=self.localization_final_minimum_residual_beams,
            median_residual=self.scan_map_median_residual,
            maximum_median_residual=self.localization_final_max_median_residual,
            p90_residual=self.scan_map_p90_residual,
            maximum_p90_residual=self.localization_final_max_p90_residual,
            raycast_comparable_beams=self.raycast_comparable_beams,
            minimum_raycast_beams=minimum_raycast_beams,
            raycast_static_matches=self.raycast_static_matches,
            minimum_raycast_static_matches=minimum_raycast_static_matches,
            raycast_contradiction_ratio=self.raycast_contradiction_ratio,
            maximum_raycast_contradiction_ratio=(
                maximum_raycast_contradiction_ratio
            ),
            heading_required=require_heading,
            heading_ready=self._global_heading_diversity_ready(),
            amcl_fresh=now - self.last_amcl_monotonic <= self.amcl_pose_freshness,
            scan_map_fresh=(
                now - self.last_scan_map_monotonic <= self.scan_map_freshness
            ),
            scan_fresh=now - self.last_scan_monotonic <= 0.30,
            tf_valid=(
                now - self.last_map_tf_monotonic <= 0.60
                and self.tf_buffer.can_transform(
                    "odom", "base_footprint", Time()
                )
            ),
            sensor_time_valid=self._critical_sensor_time_healthy(),
        )
        if not verdict.accepted:
            return verdict
        if self.global_search_untrusted:
            if (
                now - self.last_particle_cloud_monotonic
                > self.particle_cloud_freshness
            ):
                return LocalizationVerification(False, "PARTICLE_CLOUD_STALE")
            if not self.particle_uniqueness.accepted:
                return LocalizationVerification(
                    False, self.particle_uniqueness.reason
                )
            if self.localization_operator_hint_active:
                operator_region = (
                    self.localization_pending_operator_hint
                    or self.localization_seed_pose
                )
                if (
                    operator_region is None
                    or self.last_amcl_pose is None
                    or math.hypot(
                        float(self.last_amcl_pose[0])
                        - float(operator_region["x"]),
                        float(self.last_amcl_pose[1])
                        - float(operator_region["y"]),
                    ) > self.operator_hint_search_radius
                ):
                    return LocalizationVerification(
                        False, "OPERATOR_HINT_CANDIDATE_OUTSIDE_REGION"
                    )
                # An explicit nearby-position hint supplies the spatial prior
                # that a blind global search lacks. Accept it only after every
                # strict multi-frame local geometry, pose/covariance and
                # particle-uniqueness gate above has passed. Requiring the
                # chassis to rotate here deadlocks a valid hinted candidate in
                # a tight bay, even though the operator already bounded its
                # position and stationary LiDAR evidence is independently
                # strong.
                # The coarse map-wide scorer is intentionally not a veto here:
                # movable objects depress its absolute score even when the
                # candidate's fine residual/raycast evidence is unequivocal.
                return verdict
            self._request_global_scan_uniqueness()
            if self.global_scan_uniqueness_in_progress:
                return LocalizationVerification(
                    False, "GLOBAL_SCAN_UNIQUENESS_EVALUATING"
                )
            if self.global_scan_evaluated_candidate is None:
                return LocalizationVerification(
                    False, "GLOBAL_SCAN_UNIQUENESS_UNAVAILABLE"
                )
            if not self.global_scan_uniqueness.accepted:
                return LocalizationVerification(
                    False, self.global_scan_uniqueness.reason
                )
        return verdict

    def _localization_quality_ready(
        self,
        now: float,
        *,
        required_scan_score: float,
    ) -> bool:
        """Quality/stability gate independent from global uniqueness."""
        return self._localization_verdict(
            now,
            required_scan_score=required_scan_score,
            require_heading=False,
        ).accepted

    def _actual_odom_yaw(self) -> float | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException:
            return None
        return self._yaw_from_quaternion(transform.transform.rotation)

    def _start_next_localization_rotation(self, now: float) -> None:
        required_bins, required_span = self._global_heading_requirement()
        remaining_bins = max(
            1, required_bins - len(self.localization_heading_bins)
        )
        remaining_span = max(
            0.0, required_span - self.localization_heading_span
        )
        increment = max(
            math.radians(15.0),
            remaining_span / remaining_bins,
        )
        increment = min(math.radians(30.0), increment)
        self.localization_rotation_cycle_start_angle = self.rotation_angle
        self.localization_next_observation_angle = min(
            self.rotation_max_angle,
            self.rotation_angle + increment,
        )
        actual_yaw = self._actual_odom_yaw()
        self.localization_actual_yaw = actual_yaw
        self.rotation_yaw_progress.reset(actual_yaw)
        self.localization_state = "LOCALIZING_ROTATING"
        self.localization_phase_started_monotonic = now
        self.localization_settling_evidence_started = False
        self._set_state("LOCALIZING_ROTATING", "global_search_needs_new_heading")

    def _localization_evidence_ready(self, now: float) -> bool:
        return self._localization_verdict(
            now,
            required_scan_score=self._required_localization_scan_score(),
            # Uniqueness is proven from spatial AMCL particle clusters.
            # Heading diversity remains diagnostic evidence and is collected
            # only when an ambiguous cloud needs an authorized physical turn.
            require_heading=False,
        ).accepted

    def _localization_rejection_reason(self, now: float) -> str:
        return self._localization_verdict(
            now,
            required_scan_score=self._required_localization_scan_score(),
            require_heading=False,
        ).reason

    def _localization_tracking_evidence_ready(self, now: float) -> bool:
        """Maintain an acquired pose with hysteresis, not acquisition gates.

        Entering READY still requires the high confidence threshold, a stable
        stationary window, scan/map agreement and global heading observation.
        Tracking only drops after the lower confidence threshold or fresh
        AMCL/scan/TF/sensor evidence has been absent for the configured grace.
        """
        return (
            self.localization_confidence >= self.localization_low_threshold
            and now - self.last_amcl_monotonic <= self.amcl_pose_freshness
            and now - self.last_scan_monotonic <= 0.30
            and now - self.last_map_tf_monotonic <= 0.60
            and self._critical_sensor_time_healthy()
        )

    def _localization_start_evidence_ready(self, now: float) -> bool:
        """Recheck READY tracking evidence without rerunning acquisition.

        Acquisition deliberately requires 25 comparable / 20 exact static
        raycasts across a consensus window. A valid robot can have fewer rays
        at a route endpoint simply because of its final heading. Reapplying
        those acquisition counts here falsely invalidated a continuously
        tracked READY pose between two consecutive journeys.
        """
        return bool(
            self.localized
            and self.localization_state == "READY"
            and self._localization_tracking_evidence_ready(now)
            and now - self.last_scan_map_monotonic <= self.scan_map_freshness
            and self.scan_map_valid_beams >= self.scan_map_minimum_beams
            and self.scan_map_score >= self.tracking_scan_map_sanity_threshold
            and self.scan_map_residual_beams
            >= self.localization_final_minimum_residual_beams
            and self.scan_map_median_residual
            <= self.localization_final_max_median_residual
            and self.scan_map_p90_residual
            <= self.localization_final_max_p90_residual
            and self.raycast_comparable_beams > 0
            and self.raycast_static_matches > 0
            and self.raycast_contradiction_ratio
            <= self.localization_tracking_maximum_contradiction_ratio
        )

    def _wait_for_localization_start_evidence(self) -> bool:
        """Bridge the bounded scheduling gap between command and sensor callbacks.

        The socket command server runs independently from the ROS executor, so
        LiDAR, AMCL and TF callbacks continue updating the evidence while this
        bounded wait is active.  No threshold is weakened: navigation starts
        only after the complete fresh-evidence predicate passes.
        """
        deadline = time.monotonic() + max(
            0.0, self.localization_start_evidence_wait
        )
        while True:
            now = time.monotonic()
            if self._localization_start_evidence_ready(now):
                return True
            remaining = deadline - now
            if remaining <= 0.0:
                return False
            time.sleep(min(0.025, remaining))

    def _begin_localization_settling(
        self,
        now: float,
        *,
        reason: str = "heading_observation_reached",
    ) -> None:
        """Stop after the global sweep and wait for stationary AMCL samples."""
        self._stop_localization_rotation()
        self.localization_state = "LOCALIZING_SETTLING"
        self._set_state("LOCALIZING_SETTLING", reason)
        self.localization_phase_started_monotonic = now
        self.localization_settling_evidence_started = False

    def _start_localization_settling_evidence(self) -> None:
        """Discard moving samples without resetting AMCL's particle cloud."""
        self._reset_stationary_verification_evidence()
        self.localization_settling_evidence_started = True

    def _reset_stationary_verification_evidence(self) -> None:
        """Require a new stationary evidence window while retaining AMCL."""
        self.localization_confidence = 0.0
        self.last_amcl_monotonic = 0.0
        self.pose_window.clear()
        self.pose_stability_metrics = pose_stability(())
        self.scan_map_scores.clear()
        self.scan_map_median_residuals.clear()
        self.scan_map_p90_residuals.clear()
        self.scan_map_mean_residuals.clear()
        self.raycast_static_match_ratios.clear()
        self.raycast_dynamic_occlusion_ratios.clear()
        self.raycast_contradiction_ratios.clear()
        self.raycast_median_errors.clear()
        self.raycast_p90_errors.clear()
        self.localization_evidence_frames.clear()
        self.localization_consensus = LocalizationConsensus(
            False,
            "CONSENSUS_WINDOW_INCOMPLETE",
            0,
            self.localization_consensus_required_frames,
            0,
            0,
        )
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
        self.raycast_comparable_beams = 0
        self.raycast_static_matches = 0
        self.raycast_dynamic_occlusions = 0
        self.raycast_map_contradictions = 0
        self.raycast_inconclusive_map_hits = 0
        self.raycast_matched_beams = 0
        self.raycast_static_match_ratio = 0.0
        self.raycast_dynamic_occlusion_ratio = 0.0
        self.raycast_contradiction_ratio = 0.0
        self.raycast_match_ratio = 0.0
        self.raycast_median_error = math.inf
        self.raycast_p90_error = math.inf
        self.last_scan_map_monotonic = 0.0
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None
        self.low_confidence_since = None
        self.last_nomotion_request_monotonic = 0.0
        self.last_particle_cloud_monotonic = 0.0

    def _nomotion_update_due(
        self,
        now: float,
        *,
        navigation_in_progress: bool,
    ) -> bool:
        """Keep AMCL publishing when an active controller is safely stationary.

        AMCL normally publishes often enough while odometry crosses its motion
        thresholds. A controller collision can leave a FollowPath action
        active without any chassis motion, however, so AMCL emits no new pose
        and the independent freshness gate would incorrectly revoke a valid
        localization. Force one scan update only after half the freshness
        budget has elapsed; ordinary moving updates therefore remain untouched.
        """
        if now - self.last_nomotion_request_monotonic < 0.5:
            return False
        if not self.nomotion_update_client.service_is_ready():
            return False
        if not navigation_in_progress:
            return True
        refresh_age = max(0.25, self.amcl_pose_freshness * 0.5)
        return now - self.last_amcl_monotonic >= refresh_age

    def _localization_checkpoint_observed(self, now: float) -> bool:
        """Confirm a fresh stationary checkpoint before rotating again.

        A failed score/raycast gate must not count as heading evidence, but it
        must be allowed to trigger the next authorized observation angle so
        AMCL can correct a nearby wrong hypothesis instead of stalling in
        LOCALIZING_SETTLING until the global timeout.
        """
        return bool(
            self._pose_is_stable()
            and now - self.last_amcl_monotonic <= self.amcl_pose_freshness
            and now - self.last_scan_map_monotonic <= self.scan_map_freshness
            and now - self.last_scan_monotonic <= 0.30
            and now - self.last_map_tf_monotonic <= 0.60
            and self.scan_map_valid_beams >= self.scan_map_minimum_beams
            and self._critical_sensor_time_healthy()
        )

    @localization_callback
    def _localization_tick(self) -> None:
        if self.mode != "NAVIGATION":
            return
        now = time.monotonic()
        if self.map_id and not self._monitor_localization_tf(now):
            return
        (
            sensor_time_healthy,
            sensor_time_reason,
            sensor_time_diagnostics,
        ) = self._critical_sensor_time_status()
        if self.map_id and not sensor_time_healthy:
            if self.sensor_time_invalid_since is None:
                self.sensor_time_invalid_since = now
            if (
                now - self.sensor_time_invalid_since
                >= self.sensor_time_invalid_grace
                and self.localization_state not in {"IDLE", "SENSOR_TIME_INVALID"}
                and not self.sensor_time_hard_failed
            ):
                self._degrade_localization(
                    sensor_time_reason,
                    sensor_time_diagnostics,
                )
            if (
                self.localization_state == "SENSOR_TIME_INVALID"
                and not self.sensor_time_hard_failed
                and now - self.sensor_time_invalid_since
                >= self.sensor_time_hard_failure_timeout
            ):
                self._fail_sustained_sensor_time_pause(sensor_time_reason)
            return
        if sensor_time_healthy:
            self.sensor_time_invalid_since = None
            self.sensor_time_hard_failed = False
            if self.localization_state == "SENSOR_TIME_INVALID":
                self._restore_after_sensor_time_pause(now)
                return
        self._refresh_localization_confidence()
        if self.localized and self.localization_state == "READY":
            navigation_in_progress = self._navigation_in_progress()
            # During normal travel AMCL refreshes from odometry. If Nav2 keeps
            # an action active while collision checking holds the chassis at
            # zero, force a bounded scan update before the pose freshness gate
            # expires instead of reporting a fictitious localization loss.
            if self._nomotion_update_due(
                now,
                navigation_in_progress=navigation_in_progress,
            ):
                self.last_nomotion_request_monotonic = now
                self.nomotion_update_client.call_async(Empty.Request())
            if self._localization_tracking_evidence_ready(now):
                if self.ready_evidence_since is None:
                    self.ready_evidence_since = now
                self.ready_evidence_invalid_since = None
            else:
                self.ready_evidence_since = None
                if self.ready_evidence_invalid_since is None:
                    self.ready_evidence_invalid_since = now
                elif now - self.ready_evidence_invalid_since >= self.localization_low_grace:
                    amcl_age = self._age_milliseconds(self.last_amcl_monotonic)
                    scan_age = self._age_milliseconds(self.last_scan_monotonic)
                    tf_age = self._age_milliseconds(self.last_map_tf_monotonic)
                    self._localization_lost(
                        "localization evidence expired "
                        f"(navigating={navigation_in_progress}, "
                        f"confidence={self.localization_confidence:.4f}, "
                        f"amcl_age_ms={amcl_age}, scan_age_ms={scan_age}, "
                        f"tf_age_ms={tf_age}, "
                        f"sensor_time_healthy={self._critical_sensor_time_healthy()})"
                    )
            self._resume_localization_navigation_if_ready()
            return
        localizing_states = {
            "LOCALIZATION_INITIALIZING", "LOCALIZING_LAST_POSE",
            "PASSIVE_LOCALIZING", "CANDIDATE", "AMBIGUOUS",
            "LOCALIZING_GLOBAL",
            "LOCALIZING_APPROXIMATE_POSE", "LOCALIZING_ROTATING",
            "LOCALIZING_SETTLING", "LOW_CONFIDENCE", "LOCALIZATION_LOST", "VERIFYING",
            # A failed attempt is terminal for automatic rotation, but AMCL
            # must still be allowed to recover after an operator moves the
            # robot manually and supplies enough fresh, stable scans.
            "LOCALIZATION_FAILED",
        }
        if self.localization_state not in localizing_states:
            return
        if self.localization_state == "LOCALIZATION_INITIALIZING":
            # Map-load completion initializes the session clock and chooses
            # VERIFYING / LAST_POSE / REQUIRED.  A timer callback can race the
            # load response by a few milliseconds; treating the zero-valued
            # pre-session clock as elapsed time causes an immediate false
            # LOCALIZATION_FAILED transition at every stack recreate.
            return
        if (
            self.localization_state == "LOCALIZING_SETTLING"
            and not self.localization_settling_evidence_started
        ):
            # Let the downstream velocity smoother bring angular velocity to
            # zero, then require a completely fresh stationary sample window.
            if (
                now - self.localization_phase_started_monotonic
                < self.localization_rotation_settle
            ):
                return
            self._start_localization_settling_evidence()
        # A best-effort subscriber may miss a single startup publication while
        # AMCL is transitioning lifecycle state. Retry only until AMCL returns
        # a fresh estimate for this localization phase; this is bounded and
        # does not keep resetting a particle filter that is already updating.
        if (
            self.localization_state in {
                "LOCALIZING_LAST_POSE", "LOCALIZING_APPROXIMATE_POSE",
            }
            and self.localization_seed_pose is not None
            and self.last_amcl_monotonic < self.localization_phase_started_monotonic
            and now - self.last_initial_pose_publish_monotonic >= 1.0
        ):
            self._publish_initial_pose(
                self.localization_seed_pose,
                approximate=self.localization_seed_approximate,
            )
        # AMCL may publish its only stationary sample just before map->base is
        # visible. Re-evaluate it against current TF freshness on every tick.
        self._refresh_localization_confidence()
        # A stationary robot normally produces only one AMCL estimate because
        # update_min_d/a suppresses further filter updates. Ask AMCL for
        # bounded no-motion updates so a recent persisted pose is verified by
        # multiple LiDAR scans instead of either trusting its first covariance
        # spike or timing out without enough samples.
        if (
            self.localization_state != "LOCALIZING_ROTATING"
            and now - self.last_nomotion_request_monotonic >= 0.5
            and self.nomotion_update_client.service_is_ready()
        ):
            self.last_nomotion_request_monotonic = now
            self.nomotion_update_client.call_async(Empty.Request())
        if (
            self.localization_operator_hint_active
            and self.localization_seed_approximate
        ):
            # A fresh single-scan map search can pull AMCL out of a wrong local
            # basin near the click. It runs once in a worker and only supplies
            # a seed; it never grants localization authority by itself.
            self._request_global_scan_uniqueness(operator_seed=True)
        localization_ready = self._localization_evidence_ready(now)
        if localization_ready and self.localization_state in {
            "PASSIVE_LOCALIZING", "AMBIGUOUS",
        }:
            self.localization_state = "CANDIDATE"
            self._set_state("CANDIDATE", "strong_unique_stationary_candidate")
            self._nav_debug(
                "LOCALIZATION_CANDIDATE",
                attempt_id=self.localization_attempt_id,
                result="STRONG_UNIQUE_STATIONARY_CANDIDATE",
                rotation_required=False,
                consensus=(
                    f"{self.localization_consensus.agreeing_frames}/"
                    f"{self.localization_consensus.total_frames}"
                ),
                uniqueness_reason=self.particle_uniqueness.reason,
            )
        elif localization_ready and self.localization_state == "CANDIDATE":
            self.localization_state = "VERIFYING"
            self.localization_phase_started_monotonic = now
            self.verification_started_monotonic = now
            self.verification_scan_count = 0
            self._set_state("VERIFYING", "candidate_strict_verification")
        if (
            self.navigation_debug_enabled
            and now - self.last_localization_candidate_log_monotonic >= 1.0
        ):
            self._nav_debug(
                "LOCALIZATION_VERIFY",
                state=self.localization_state,
                candidate_pose=self.last_amcl_pose,
                pose_stability={
                    "passed": self._pose_is_stable(),
                    "samples": self.pose_stability_metrics.sample_count,
                    "duration_seconds": round(
                        self.pose_stability_metrics.duration_seconds, 3
                    ),
                    "xy_spread": self._finite_metric(
                        self.pose_stability_metrics.xy_spread
                    ),
                    "median_deviation": self._finite_metric(
                        self.pose_stability_metrics.median_deviation
                    ),
                },
                covariance_xy=(
                    None if len(self.last_amcl_covariance) < 36 else
                    float(self.last_amcl_covariance[0])
                    + float(self.last_amcl_covariance[7])
                ),
                covariance_yaw=(
                    None if len(self.last_amcl_covariance) < 36 else
                    float(self.last_amcl_covariance[35])
                ),
                confidence=self.localization_confidence,
                scan_score=self.scan_map_score,
                scan_score_required=self._required_localization_scan_score(),
                median_residual_m=self._finite_metric(
                    self.scan_map_median_residual
                ),
                p90_residual_m=self._finite_metric(
                    self.scan_map_p90_residual
                ),
                mean_residual_m=self._finite_metric(
                    self.scan_map_mean_residual
                ),
                raycast_comparable_beams=self.raycast_comparable_beams,
                raycast_static_matches=self.raycast_static_matches,
                raycast_dynamic_occlusions=self.raycast_dynamic_occlusions,
                raycast_map_contradictions=(
                    self.raycast_map_contradictions
                ),
                raycast_inconclusive_map_hits=(
                    self.raycast_inconclusive_map_hits
                ),
                raycast_static_match_ratio=self.raycast_static_match_ratio,
                raycast_dynamic_occlusion_ratio=(
                    self.raycast_dynamic_occlusion_ratio
                ),
                raycast_contradiction_ratio=(
                    self.raycast_contradiction_ratio
                ),
                # Compatibility fields retained for existing log parsers.
                raycast_matches=self.raycast_matched_beams,
                raycast_match_ratio=self.raycast_match_ratio,
                raycast_median_error_m=self._finite_metric(
                    self.raycast_median_error
                ),
                raycast_p90_error_m=self._finite_metric(
                    self.raycast_p90_error
                ),
                heading_bins=len(self.localization_heading_bins),
                heading_bin_ids=list(self.localization_heading_bins),
                heading_span_deg=round(
                    math.degrees(self.localization_heading_span), 1
                ),
                rotation_degrees=round(math.degrees(self.rotation_angle), 1),
                global_search_untrusted=self.global_search_untrusted,
                attempt_id=self.localization_attempt_id,
                consensus_frames=self.localization_consensus.total_frames,
                consensus_required=self.localization_consensus.required_frames,
                consensus_passing=self.localization_consensus.passing_frames,
                consensus_agreeing=self.localization_consensus.agreeing_frames,
                uniqueness_accepted=self.particle_uniqueness.accepted,
                uniqueness_reason=self.particle_uniqueness.reason,
                uniqueness_best_weight=self.particle_uniqueness.best_weight,
                uniqueness_alternative_weight=(
                    self.particle_uniqueness.alternative_weight
                ),
                uniqueness_dominance=self._finite_metric(
                    self.particle_uniqueness.dominance_ratio
                ),
                accepted=localization_ready,
                reason=(
                    "ACCEPTED"
                    if localization_ready
                    else self._localization_rejection_reason(now)
                ),
            )
            self.last_localization_candidate_log_monotonic = now
        if localization_ready:
            if self.ready_evidence_since is None:
                self.ready_evidence_since = now
        else:
            self.ready_evidence_since = None
        evidence_held = (
            self.ready_evidence_since is not None
            and now - self.ready_evidence_since >= self.localization_ready_hold
        )
        if evidence_held and (
            self.localization_state != "VERIFYING"
            or self.verification_scan_count >= self.localization_verify_min_scans
        ):
            accepted_scan_score_required = self._required_localization_scan_score()
            self._stop_localization_rotation()
            self.localized = True
            self.localization_state = "READY"
            self._reset_pre_ready_planning_evidence()
            # While a discontinuous AMCL hypothesis is being verified, the UI
            # and planner retain the last trusted pose. Replace it only after
            # the full stationary/multi-heading localization gate accepts the
            # new hypothesis.
            self._release_execution_pose_hold()
            self._anchor_odometry_trajectory()
            self._set_state("READY", "localization_verified")
            self._nav_debug(
                "LOCALIZATION_VERIFY",
                state=self.localization_state,
                candidate_pose=self.last_amcl_pose,
                pose_stability={
                    "passed": self._pose_is_stable(),
                    "samples": self.pose_stability_metrics.sample_count,
                    "duration_seconds": round(
                        self.pose_stability_metrics.duration_seconds, 3
                    ),
                    "xy_spread": self._finite_metric(
                        self.pose_stability_metrics.xy_spread
                    ),
                    "median_deviation": self._finite_metric(
                        self.pose_stability_metrics.median_deviation
                    ),
                },
                covariance_xy=(
                    None if len(self.last_amcl_covariance) < 36 else
                    float(self.last_amcl_covariance[0])
                    + float(self.last_amcl_covariance[7])
                ),
                covariance_yaw=(
                    None if len(self.last_amcl_covariance) < 36 else
                    float(self.last_amcl_covariance[35])
                ),
                confidence=self.localization_confidence,
                scan_score=self.scan_map_score,
                scan_score_required=accepted_scan_score_required,
                median_residual_m=self._finite_metric(
                    self.scan_map_median_residual
                ),
                p90_residual_m=self._finite_metric(
                    self.scan_map_p90_residual
                ),
                raycast_comparable_beams=self.raycast_comparable_beams,
                raycast_static_matches=self.raycast_static_matches,
                raycast_dynamic_occlusions=self.raycast_dynamic_occlusions,
                raycast_map_contradictions=(
                    self.raycast_map_contradictions
                ),
                raycast_inconclusive_map_hits=(
                    self.raycast_inconclusive_map_hits
                ),
                raycast_static_match_ratio=self.raycast_static_match_ratio,
                raycast_dynamic_occlusion_ratio=(
                    self.raycast_dynamic_occlusion_ratio
                ),
                raycast_contradiction_ratio=(
                    self.raycast_contradiction_ratio
                ),
                raycast_matches=self.raycast_matched_beams,
                raycast_match_ratio=self.raycast_match_ratio,
                raycast_median_error_m=self._finite_metric(
                    self.raycast_median_error
                ),
                raycast_p90_error_m=self._finite_metric(
                    self.raycast_p90_error
                ),
                heading_bins=len(self.localization_heading_bins),
                heading_bin_ids=list(self.localization_heading_bins),
                heading_span_deg=round(
                    math.degrees(self.localization_heading_span), 1
                ),
                rotation_degrees=round(math.degrees(self.rotation_angle), 1),
                global_search_untrusted=self.global_search_untrusted,
                accepted=True,
                reason="ACCEPTED",
            )
            self._nav_debug(
                "LOCALIZATION",
                state="READY",
                confidence=self.localization_confidence,
                pose=dict(self.pose or {}),
                source="amcl",
                scan_score=self.scan_map_score,
                scan_score_required=accepted_scan_score_required,
            )
            self.low_confidence_since = None
            self.localization_seed_pose = None
            self.localization_operator_hint_active = False
            self.localization_pending_operator_hint = None
            self.localization_odometry_prior_active = False
            self.localization_rotation_authorized = False
            self.global_search_requires_rotation = False
            self.global_search_rotation_pending = False
            self.global_search_untrusted = False
            self.verification_started_monotonic = 0.0
            self.verification_scan_count = 0
            self.ready_evidence_invalid_since = None
            # A temporary clock fault may have paused an active FollowPath.
            # Replan from this newly verified pose in a worker, never from the
            # localization timer callback that owns AMCL service responses.
            self._resume_sensor_time_navigation_if_ready()
            self._resume_localization_navigation_if_ready()
            return
        if self.localization_state == "LOCALIZATION_FAILED":
            # A failed active scan is terminal until fresh evidence actually
            # reaches READY above or the operator starts another attempt. Do
            # not let the generic session timeout rewrite it to AMBIGUOUS and
            # leave the UI waiting forever on a scan that already ended.
            return
        if (
            self.localization_state == "LOCALIZING_SETTLING"
            and self.localization_settling_evidence_started
            and self.global_search_untrusted
            and self._localization_checkpoint_observed(now)
            and not localization_ready
        ):
            if self.rotation_angle >= self.rotation_max_angle:
                self._stop_localization_rotation()
                self.localized = False
                self.localization_state = "LOCALIZATION_FAILED"
                self._set_state(
                    "LOCALIZATION_FAILED",
                    "global_verification_exhausted",
                )
                return
            if self.localization_rotation_authorized and self._safe_to_rotate():
                self._start_next_localization_rotation(now)
            elif self.localization_rotation_authorized:
                if self.localization_rotation_blocked_since is None:
                    self.localization_rotation_blocked_since = now
                elif (
                    now - self.localization_rotation_blocked_since
                    >= self.localization_rotation_blocked_timeout
                ):
                    self.localization_state = "AMBIGUOUS"
                    self.approximate_hint_allowed = True
                    self._set_state(
                        "AMBIGUOUS", "rotation_clearance_blocked"
                    )
            return
        if (
            self.localization_state == "VERIFYING"
            and now - self.localization_phase_started_monotonic
            >= self.localization_verify_timeout
            # A candidate accepted at the timeout boundary still needs the
            # READY hold.  It is evidence in progress, not a failed verify.
            and not localization_ready
            and self.ready_evidence_since is None
        ):
            # Verification never resets AMCL while evidence is good. A
            # bounded failure is the point where a real global search begins.
            try:
                self._start_global_localization()
            except AdapterError as exc:
                self.get_logger().error(str(exc))
            return
        if (
            now - self.localization_started_monotonic >= self.localization_timeout
            and not localization_ready
            and (
                self.localization_state != "AMBIGUOUS"
                or not self.approximate_hint_allowed
            )
        ):
            self._stop_localization_rotation()
            self.localized = False
            self.localization_state = "AMBIGUOUS"
            self.approximate_hint_allowed = True
            self._set_state("AMBIGUOUS", "passive_localization_needs_assistance")
            self._nav_debug(
                "LOCALIZATION_ASSISTANCE",
                attempt_id=self.localization_attempt_id,
                reason=self._localization_rejection_reason(now),
                approximate_hint_allowed=True,
                rotation_authorized=self.localization_rotation_authorized,
            )
            return
        if (
            self.localization_state == "AMBIGUOUS"
            and self.approximate_hint_allowed
            and now - self.localization_started_monotonic
            >= self.localization_timeout
        ):
            # The bounded active scan is over. Keep evaluating stationary
            # evidence, but do not repeatedly enter ROTATING only for the
            # timeout branch above to cancel it on the following tick.
            return
        broad_seed = (
            self.localization_state == "LOCALIZING_APPROXIMATE_POSE"
            or (
                self.localization_state == "LOCALIZING_LAST_POSE"
                and self.localization_seed_approximate
            )
        )
        seed_timeout = (
            self.odometry_prior_timeout
            if self.localization_odometry_prior_active
            else (
                self.approximate_pose_timeout
                if broad_seed else self.last_pose_timeout
            )
        )
        if (
            self.localization_state in {
                "LOCALIZING_LAST_POSE", "LOCALIZING_APPROXIMATE_POSE",
            }
            and now - self.localization_phase_started_monotonic >= seed_timeout
        ):
            if (
                self.localization_state == "LOCALIZING_LAST_POSE"
                and self.localization_odometry_prior_active
            ):
                rejected_pose = dict(self.localization_seed_pose or {})
                operator_pose = (
                    None
                    if self.localization_pending_operator_hint is None
                    else dict(self.localization_pending_operator_hint)
                )
                self.localization_odometry_prior_active = False
                self.localization_odometry_prior_rejected_epoch = (
                    self.trajectory_odom_epoch
                )
                self._nav_debug(
                    "ODOMETRY_PRIOR_REJECTED",
                    attempt_id=self.localization_attempt_id,
                    predicted_pose=rejected_pose,
                    rejection_reason=self._localization_rejection_reason(now),
                    fallback=(
                        "GLOBAL_SEARCH"
                        if operator_pose is None
                        else "OPERATOR_POSE_HINT"
                    ),
                )
                if operator_pose is None:
                    self._start_global_localization()
                else:
                    self._begin_operator_pose_hint(
                        operator_pose,
                        rotation_was_authorized=(
                            self.localization_rotation_authorized
                        ),
                    )
                return
            if broad_seed:
                # A broad operator/legacy hint is already a global hypothesis
                # with a useful spatial prior.  Keep AMCL particles and all
                # accumulated scan evidence when its initial time budget ends;
                # periodically replacing them with a uniform cloud prevents
                # convergence and races every active callback.
                self.localization_state = "AMBIGUOUS"
                self.approximate_hint_allowed = True
                self.global_search_rotation_pending = False
                self.global_search_requires_rotation = True
                self._set_state("AMBIGUOUS", "approximate_hint_needs_assistance")
                self._nav_debug(
                    "LOCALIZATION_ASSISTANCE",
                    attempt_id=self.localization_attempt_id,
                    reason=self._localization_rejection_reason(now),
                    approximate_hint_allowed=True,
                    evidence_preserved=True,
                    rotation_authorized=self.localization_rotation_authorized,
                )
                return
            try:
                self._start_global_localization()
            except AdapterError as exc:
                self.get_logger().error(str(exc))
            return
        if self.localization_state in {
            "PASSIVE_LOCALIZING", "AMBIGUOUS", "LOCALIZING_GLOBAL",
            "LOW_CONFIDENCE", "LOCALIZATION_LOST",
        }:
            if now - self.localization_phase_started_monotonic < self.global_rotate_delay:
                return
            if self.global_search_rotation_pending:
                # The stationary candidate has now had a bounded opportunity.
                stationary_reject_reason = self._localization_rejection_reason(now)
                strong_candidate = self._localization_quality_ready(
                    now, required_scan_score=self.global_scan_map_threshold
                )
                self.stationary_global_candidate_ambiguous = not strong_candidate
                self.global_search_rotation_pending = False
                self.global_search_requires_rotation = not strong_candidate
                self.ready_evidence_since = None
                required_bins, required_span = self._global_heading_requirement()
                self._nav_debug(
                    "LOCALIZATION",
                    state=(
                        "AMBIGUOUS_STATIONARY_CANDIDATE"
                        if self.stationary_global_candidate_ambiguous
                        else "STRONG_STATIONARY_CANDIDATE"
                    ),
                    action=(
                        "VERIFY_WITHOUT_ROTATION"
                        if strong_candidate
                        else "ROTATION_REQUIRED_FOR_AMBIGUITY"
                    ),
                    attempt_id=self.localization_attempt_id,
                    reject_reason=stationary_reject_reason,
                    required_heading_bins=required_bins,
                    required_heading_span_deg=round(
                        math.degrees(required_span), 1
                    ),
                )
                if strong_candidate:
                    return
                self.localization_state = "AMBIGUOUS"
                self._set_state("AMBIGUOUS", "stationary_candidate_ambiguous")
            if not self.localization_rotation_authorized:
                # Particle search and no-motion updates continue, but passive
                # map activation never obtains angular velocity ownership.
                return
            if not self._safe_to_rotate():
                # Do not claim progress while a live safety gate withholds
                # velocity. A stable geometric block is terminal with a
                # specific reason, not a generic localization timeout.
                if self.localization_rotation_blocked_since is None:
                    self.localization_rotation_blocked_since = now
                elif (
                    now - self.localization_rotation_blocked_since
                    >= self.localization_rotation_blocked_timeout
                ):
                    self._stop_localization_rotation()
                    self.localized = False
                    self.localization_state = "AMBIGUOUS"
                    self.approximate_hint_allowed = True
                    self._set_state(
                        "AMBIGUOUS",
                        (
                            "ambiguous_stationary_rotation_blocked"
                            if self.stationary_global_candidate_ambiguous
                            else "rotation_clearance_blocked"
                        ),
                    )
                    self._nav_debug(
                        "LOCALIZATION_ASSISTANCE",
                        attempt_id=self.localization_attempt_id,
                        reason="ROTATION_SWEEP_UNSAFE",
                        approximate_hint_allowed=True,
                    )
                return
            self.localization_rotation_blocked_since = None
            self._start_next_localization_rotation(now)
        if self.localization_state == "LOCALIZING_ROTATING":
            actual_yaw = self._actual_odom_yaw()
            if actual_yaw is not None:
                self.localization_actual_yaw = actual_yaw
                self.rotation_angle = (
                    self.localization_rotation_cycle_start_angle
                    + self.rotation_yaw_progress.update(actual_yaw)
                )
            if self.rotation_angle >= self.localization_next_observation_angle:
                self._begin_localization_settling(now)
                return
            if not self._safe_to_rotate():
                self._stop_localization_rotation()
                self.localization_state = "AMBIGUOUS"
                self._set_state("AMBIGUOUS", "rotation_safety_gate_closed")
                if self.localization_rotation_blocked_since is None:
                    self.localization_rotation_blocked_since = now
                return
            command = Twist()
            command.angular.z = self.rotation_speed
            self.motion_owner = "LOCALIZATION"
            self.localization_velocity.publish(command)
            self.rotation_active = True
            self.rotation_last_monotonic = now
            if now - self.last_localization_rotate_log_monotonic >= 0.5:
                final_output = self.pipeline_samples.get("motion_safety")
                required_bins, required_span = self._global_heading_requirement()
                self._nav_debug(
                    "LOCALIZATION_ROTATE",
                    state=self.localization_state,
                    phase="ROTATE_TO_NEXT_OBSERVATION",
                    current_actual_yaw=self.localization_actual_yaw,
                    accumulated_yaw_span=self.rotation_angle,
                    target_accumulated_yaw=self.localization_next_observation_angle,
                    heading_bins=list(self.localization_heading_bins),
                    required_heading_bins=required_bins,
                    required_heading_span=required_span,
                    scan_score=self.scan_map_score,
                    requested_angular=command.angular.z,
                    final_safety_output=(
                        None if final_output is None else final_output[1]
                    ),
                )
                self.last_localization_rotate_log_monotonic = now

    def _resolve_runtime_map_yaml(self, payload: dict[str, Any]) -> Path:
        try:
            runtime_root = self.map_root.resolve(strict=True)
            bundle_directory = Path(str(payload["map_path"])).resolve(strict=True)
            bundle_directory.relative_to(runtime_root)
        except (KeyError, OSError, ValueError) as exc:
            raise AdapterError(
                "MAP_PATH_INVALID",
                "Active map path must be inside the runtime map root",
            ) from exc
        map_yaml = bundle_directory / "map.yaml"
        if not map_yaml.is_file():
            raise AdapterError(
                "MAP_MISSING", "map.yaml is missing from verified cache"
            )
        return map_yaml.resolve(strict=True)

    def _prepare_nav2_map_yaml(
        self, canonical_yaml: Path, payload: dict[str, Any]
    ) -> Path:
        """Create a runtime-only YAML when an old bundle loses unknown cells.

        The verified bundle remains immutable.  Both Nav2 and the adapter load
        the same normalized sidecar, so their occupancy semantics cannot
        diverge while existing robot-created maps remain usable.
        """
        metadata = yaml.safe_load(canonical_yaml.read_text())
        if not isinstance(metadata, dict):
            raise AdapterError("MAP_INVALID", "map.yaml must contain an object")
        image_path = Path(str(metadata.get("image") or ""))
        if not image_path.is_absolute():
            image_path = canonical_yaml.parent / image_path
        try:
            with Image.open(image_path) as image:
                normalized, changed = normalize_trinary_unknown_metadata(
                    metadata, image_grayscale_values(image.convert("L"))
                )
        except OSError as exc:
            raise AdapterError("MAP_INVALID", f"Map image is invalid: {exc}") from exc
        if not changed:
            return canonical_yaml
        runtime_directory = (
            self.socket_path.parent
            / "runtime-maps"
            / f"{payload.get('map_id', 'map')}-v{int(payload.get('version', 0))}"
        )
        runtime_directory.mkdir(parents=True, exist_ok=True)
        normalized["image"] = str(image_path.resolve(strict=True))
        runtime_yaml = runtime_directory / "map.yaml"
        temporary_yaml = runtime_directory / ".map.yaml.tmp"
        temporary_yaml.write_text(yaml.safe_dump(normalized, sort_keys=False))
        os.replace(temporary_yaml, runtime_yaml)
        self._nav_debug(
            "MAP_SEMANTICS_NORMALIZED",
            canonical_map_yaml_path=str(canonical_yaml),
            runtime_map_yaml_path=str(runtime_yaml),
            original_free_thresh=metadata.get("free_thresh"),
            normalized_free_thresh=normalized["free_thresh"],
            preserved_unknown_grayscale=205,
        )
        return runtime_yaml

    def _log_active_map(
        self,
        payload: dict[str, Any],
        map_yaml: Path,
        saved_map: SavedOccupancyMap,
    ) -> None:
        yaml_metadata = yaml.safe_load(map_yaml.read_text()) or {}
        image_path = Path(str(yaml_metadata.get("image") or ""))
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        metadata_path = map_yaml.parent / "metadata.json"
        bundle_metadata: dict[str, Any] = {}
        try:
            loaded = json.loads(metadata_path.read_text())
            if isinstance(loaded, dict):
                bundle_metadata = loaded
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
        bundle_path = map_yaml.parent / "map-bundle.tar.gz"
        checksum = (
            payload.get("checksum")
            or bundle_metadata.get("checksum")
            or bundle_metadata.get("bundle_checksum")
            or bundle_metadata.get("sha256")
        )
        self._nav_debug(
            "MAP_ACTIVE",
            map_id=self.map_id,
            map_version=self.map_version,
            canonical_map_yaml_path=str(map_yaml),
            image_path=str(image_path.resolve()),
            bundle_path=(str(bundle_path.resolve()) if bundle_path.is_file() else None),
            checksum=checksum,
            resolution=saved_map.resolution,
            width=saved_map.width,
            height=saved_map.height,
            origin=[
                saved_map.origin_x,
                saved_map.origin_y,
                saved_map.origin_yaw,
            ],
        )

    def _read_active_navigation_mission(self) -> dict[str, Any] | None:
        """Read a small crash-recovery record without trusting its geometry."""
        path = self.active_navigation_mission_path
        try:
            value = json.loads(path.read_text())
            if not isinstance(value, dict) or int(value.get("schema_version", 0)) != 1:
                raise ValueError("unsupported active-mission schema")
            goal = value.get("goal")
            if not isinstance(goal, dict):
                raise ValueError("active mission has no destination")
            goal_x = float(goal["x"])
            goal_y = float(goal["y"])
            goal_yaw = float(goal.get("yaw", 0.0))
            if not all(math.isfinite(item) for item in (goal_x, goal_y, goal_yaw)):
                raise ValueError("active mission destination is not finite")
            value["goal"] = {"x": goal_x, "y": goal_y, "yaw": goal_yaw}
            value["map_id"] = str(value.get("map_id") or "")
            value["map_version"] = int(value.get("map_version", 0))
            value["resume_automatically"] = bool(
                value.get("resume_automatically", False)
            )
            return value
        except FileNotFoundError:
            return None
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._nav_debug(
                "MISSION_RECOVERY_RECORD",
                action="REJECTED",
                reason="INVALID_OR_UNREADABLE",
                error=str(exc),
            )
            self._clear_active_navigation_mission("invalid_recovery_record")
            return None

    def _persist_active_navigation_mission(
        self,
        *,
        resume_automatically: bool,
        goal: dict[str, Any] | None = None,
        route_id: str | None = None,
        path: list[dict[str, Any]] | None = None,
        segment_directions: list[int] | None = None,
    ) -> None:
        """Atomically preserve only the data needed after an adapter/Pi crash."""
        destination = dict(goal or self.execution_goal or self.paused_goal or {})
        if not self.map_id or not destination:
            return
        try:
            clean_goal = {
                "x": float(destination["x"]),
                "y": float(destination["y"]),
                "yaw": float(destination.get("yaw", 0.0)),
            }
            if not all(math.isfinite(item) for item in clean_goal.values()):
                return
        except (KeyError, TypeError, ValueError):
            return
        route = list(path if path is not None else self.execution_points)
        clean_path: list[dict[str, float]] = []
        for point in route[:4096]:
            try:
                x = float(point["x"])
                y = float(point["y"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                clean_path.append({"x": x, "y": y})
        directions = list(
            segment_directions
            if segment_directions is not None
            else self.execution_segment_directions
        )
        clean_directions: list[int] = []
        for item in directions[:4095]:
            try:
                clean_directions.append(-1 if int(item) < 0 else 1)
            except (TypeError, ValueError):
                continue
        payload = {
            "schema_version": 1,
            "map_id": self.map_id,
            "map_version": self.map_version,
            "mission_id": self.current_mission_id,
            "route_id": str(route_id or self.execution_route_id or ""),
            "goal": clean_goal,
            # Retain the accepted route for diagnostics. Startup recovery
            # deliberately replans from the newly verified physical pose, so
            # this prior-session geometry is never rendered or followed stale.
            "path": clean_path,
            "segment_directions": clean_directions,
            "resume_automatically": bool(resume_automatically),
            "updated_at_unix": time.time(),
        }
        target = self.active_navigation_mission_path
        temporary = target.with_name(
            f".{target.name}.{os.getpid()}.{threading.get_ident()}.tmp"
        )
        try:
            with self.active_navigation_mission_lock:
                target.parent.mkdir(parents=True, exist_ok=True)
                with temporary.open("w") as output:
                    json.dump(payload, output, separators=(",", ":"), sort_keys=True)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(temporary, target)
        except OSError as exc:
            self._nav_debug(
                "MISSION_RECOVERY_RECORD",
                action="WRITE_FAILED",
                error=str(exc),
            )
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                pass

    def _clear_active_navigation_mission(self, reason: str) -> None:
        try:
            with self.active_navigation_mission_lock:
                self.active_navigation_mission_path.unlink()
        except FileNotFoundError:
            return
        except OSError as exc:
            self._nav_debug(
                "MISSION_RECOVERY_RECORD",
                action="CLEAR_FAILED",
                reason=reason,
                error=str(exc),
            )
            return
        self._nav_debug(
            "MISSION_RECOVERY_RECORD",
            action="CLEARED",
            reason=reason,
        )

    def _load_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        canonical_map_yaml = self._resolve_runtime_map_yaml(payload)
        map_yaml = self._prepare_nav2_map_yaml(canonical_map_yaml, payload)
        try:
            candidate_grid = SavedOccupancyMap.load(map_yaml)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise AdapterError("MAP_INVALID", f"Saved map artifact is invalid: {exc}") from exc
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            raise AdapterError("MAP_SERVER_UNAVAILABLE", "Map Server load service unavailable")
        previous_path = self.active_map_path
        previous_identity = (self.map_id, self.map_version)
        previous_grid = self.saved_map
        previous_geometry = self.map_navigation_geometry
        previous_planner = self.stop_turn_planner
        previous_localized = self.localized
        previous_localization_state = self.localization_state

        def rollback_map() -> None:
            self._stop_localization_rotation()
            if previous_path is None or not previous_path.is_file():
                self.map_id = ""
                self.map_version = 0
                self.saved_map = None
                self.map_navigation_geometry = None
                self.stop_turn_planner = None
                self.failed_segments = []
                self._publish_failed_segments()
                self.active_map_path = None
                self.localized = False
                self.localization_state = "IDLE"
                self._set_state("NO_ACTIVE_MAP", "map_load_rollback_without_map")
                return
            rollback = LoadMap.Request()
            rollback.map_url = str(previous_path)
            rollback_response = self._wait(
                self.map_load_client.call_async(rollback), 10, "MAP_ROLLBACK_TIMEOUT"
            )
            if int(rollback_response.result) != 0:
                raise AdapterError("MAP_ROLLBACK_FAILED", "Map Server rejected the previous map")
            self.map_id, self.map_version = previous_identity
            self.saved_map = previous_grid
            self.map_navigation_geometry = previous_geometry
            self.stop_turn_planner = previous_planner
            self.active_map_path = previous_path
            self.localized = previous_localized
            self.localization_state = previous_localization_state
            self._set_state(
                "READY" if previous_localized else previous_localization_state,
                "map_load_rollback",
            )
        with self.state_lock:
            self._set_state("MAP_LOADING", "map_load_requested")
            self.localization_state = "LOCALIZATION_INITIALIZING"
        load_started = time.monotonic()
        request = LoadMap.Request()
        request.map_url = str(map_yaml)
        response = self._wait(
            self.map_load_client.call_async(request), 10, "MAP_LOAD_TIMEOUT"
        )
        if int(response.result) != 0:
            try:
                rollback_map()
            except AdapterError:
                self._set_state("FAILED", "map_load_and_rollback_failed")
            raise AdapterError("MAP_LOAD_FAILED", f"Map Server returned {response.result}")
        with self.state_lock:
            self.map_id = str(payload["map_id"])
            self.map_version = int(payload["version"])
            self.saved_map = candidate_grid
            self.map_navigation_geometry = candidate_grid.navigation_geometry
            if self.map_navigation_geometry is None:
                self.map_navigation_geometry = MapNavigationGeometry.build(
                    candidate_grid,
                    half_length=self.footprint_half_length,
                    half_width=self.footprint_half_width,
                    padding=self.planning_footprint_padding,
                )
            self.stop_turn_planner = StopTurnStateLatticePlanner(
                candidate_grid,
                self.map_navigation_geometry,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
                linear_speed=self.speed_profiles.get(self.auto_speed_mode).linear_max,
                angular_speed=self.execution_turn_max_speed,
                turn_bay_max_distance=self.turn_bay_max_distance,
                hard_side_margin=self.corridor_hard_side_margin,
                preferred_side_margin=self.corridor_side_margin,
            )
            self.failed_segments = []
            self._publish_failed_segments()
            self.active_map_path = map_yaml
            self.localized = False
            self.initial_pose_requested = False
            self.paused_goal = None
            self.current_mission_id = ""
            self.latest_global_path = []
            self.latest_dynamic_obstacles = []
            self._reset_dynamic_recovery()
            self.trajectory_map_from_odom = None
            self.localization_resume_context = None
            self.localization_resume_in_progress = False
            self.dynamic_overlay = DynamicObstacleOverlay(
                ttl_seconds=self.dynamic_overlay_ttl,
                cluster_distance=self.dynamic_overlay_cluster_distance,
                motion_threshold=self.dynamic_obstacle_motion_threshold,
                stationary_confirmation_seconds=(
                    self.dynamic_obstacle_stationary_confirmation
                ),
                moving_confirmation_windows=(
                    self.dynamic_obstacle_moving_confirmation_windows
                ),
            )
            self.visualization_revision += 1
            self._set_state("LOCALIZATION_INITIALIZING", "map_loaded")
            self.localization_state = "LOCALIZATION_INITIALIZING"
        map_deadline = time.monotonic() + 3.0
        while self.map_received_monotonic < load_started and time.monotonic() < map_deadline:
            time.sleep(0.05)
        if self.map_received_monotonic < load_started:
            try:
                rollback_map()
            except AdapterError:
                self._set_state("FAILED", "map_topic_and_rollback_failed")
            raise AdapterError("MAP_TOPIC_UNAVAILABLE", "Map Server did not publish /map")
        stored_mission = self._read_active_navigation_mission()
        if stored_mission is not None:
            requested_identity = (
                str(payload["map_id"]),
                int(payload["version"]),
            )
            stored_identity = (
                str(stored_mission["map_id"]),
                int(stored_mission["map_version"]),
            )
            if stored_identity != requested_identity:
                self._clear_active_navigation_mission("active_map_changed")
            else:
                restored_goal = dict(stored_mission["goal"])
                resume_automatically = bool(
                    stored_mission["resume_automatically"]
                )
                with self.state_lock:
                    self.paused_goal = restored_goal
                    self.current_mission_id = str(
                        stored_mission.get("mission_id") or ""
                    )
                    self.latest_feedback["destination_preserved"] = True
                    self.latest_feedback["mission_recovered_after_restart"] = True
                    if resume_automatically:
                        self.localization_resume_context = {
                            "goal": restored_goal,
                            "mission_id": self.current_mission_id,
                            "route_id": str(
                                stored_mission.get("route_id")
                                or "restart-recovery"
                            ),
                            # A route accepted before a process/Pi restart may
                            # begin behind the robot. Wait for exact pose
                            # verification and replan from that pose instead.
                            "path": [],
                            "segment_directions": [],
                        }
                self._nav_debug(
                    "MISSION_RECOVERY_RECORD",
                    action=(
                        "RESTORED_FOR_AUTO_REPLAN"
                        if resume_automatically
                        else "RESTORED_PAUSED"
                    ),
                    mission_id=self.current_mission_id,
                    route_id=stored_mission.get("route_id"),
                    destination=restored_goal,
                    stale_route_published=False,
                )
        self._log_active_map(payload, canonical_map_yaml, candidate_grid)
        self._begin_auto_localization(payload.get("last_known_pose"))
        return {
            "status": "completed",
            "current_state": self.current_state,
            "progress_percent": 100,
            "state": self._state(),
        }

    def _begin_odometry_prior(
        self,
        odometry_pose: dict[str, float],
        *,
        rotation_was_authorized: bool,
        fallback_operator_hint: dict[str, float] | None = None,
    ) -> None:
        """Verify the current pose through the last trusted map<-odom anchor."""
        self._reset_localization_evidence()
        self.localization_rotation_authorized = rotation_was_authorized
        self.localization_started_monotonic = time.monotonic()
        self.localization_phase_started_monotonic = (
            self.localization_started_monotonic
        )
        self.localization_seed_pose = dict(odometry_pose)
        self.localization_seed_approximate = False
        self.localization_operator_hint_active = False
        self.localization_pending_operator_hint = (
            None
            if fallback_operator_hint is None
            else dict(fallback_operator_hint)
        )
        self.localization_odometry_prior_active = True
        self.global_search_requires_rotation = False
        self.global_search_untrusted = False
        self.approximate_hint_allowed = False
        self.localization_attempt_sequence += 1
        self.localization_attempt_id = (
            f"{self.map_id}:{self.map_version}:odom-"
            f"{self.localization_attempt_sequence}"
        )
        self._publish_initial_pose(odometry_pose, approximate=False)
        self.localization_state = "LOCALIZING_LAST_POSE"
        self._set_state("LOCALIZING_LAST_POSE", "trusted_odometry_prior")
        self._nav_debug(
            "LOCALIZATION_ODOMETRY_PRIOR",
            attempt_id=self.localization_attempt_id,
            predicted_pose=odometry_pose,
            fallback=(
                "GLOBAL_SEARCH"
                if fallback_operator_hint is None
                else "OPERATOR_POSE_HINT"
            ),
            fallback_center=(
                None
                if fallback_operator_hint is None
                else (
                    fallback_operator_hint["x"],
                    fallback_operator_hint["y"],
                )
            ),
            strict_verification_required=True,
        )

    def _begin_operator_pose_hint(
        self,
        operator_pose: dict[str, float],
        *,
        rotation_was_authorized: bool,
    ) -> None:
        """Start the broad click-based fallback with completely fresh evidence."""
        # Never let confidence accumulated around a false, locally stable AMCL
        # hypothesis immediately validate an operator-supplied correction.
        # Require fresh AMCL samples around the new pose before returning READY.
        self._reset_localization_evidence()
        self.localization_rotation_authorized = rotation_was_authorized
        self.localization_started_monotonic = time.monotonic()
        self.localization_phase_started_monotonic = self.localization_started_monotonic
        self.localization_seed_pose = operator_pose
        self.localization_seed_approximate = True
        self.localization_operator_hint_active = True
        self.localization_pending_operator_hint = dict(operator_pose)
        self.localization_odometry_prior_active = False
        self.global_search_requires_rotation = False
        self.global_search_untrusted = True
        self.approximate_hint_allowed = False
        self.localization_attempt_sequence += 1
        self.localization_attempt_id = (
            f"{self.map_id}:{self.map_version}:hint-"
            f"{self.localization_attempt_sequence}"
        )
        self._publish_initial_pose(operator_pose, approximate=True)
        self.localization_state = "LOCALIZING_APPROXIMATE_POSE"
        self._set_state("LOCALIZING_APPROXIMATE_POSE", "operator_pose_hint")
        self._nav_debug(
            "LOCALIZATION_HINT",
            attempt_id=self.localization_attempt_id,
            center=(operator_pose["x"], operator_pose["y"]),
            yaw_trusted=False,
            localized=False,
            strict_verification_required=True,
        )

    @localization_serialized
    def _set_initial_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = dict(payload["pose"])
        self._validate_command_map(payload)
        if self.saved_map is None or self.saved_map.world_to_cell(float(pose["x"]), float(pose["y"])) is None:
            raise AdapterError("POSE_OUTSIDE_MAP", "Vị trí gần đúng nằm ngoài bản đồ")
        # The operator supplies a search region only. Deliberately discard yaw
        # so a click can never masquerade as a complete robot pose.
        operator_pose = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": 0.0,
        }
        # A pose hint may be supplied while an explicitly authorized global
        # rescan is already in progress. Preserve that explicit permission;
        # submitting the same hint again must not erase a converging cloud.
        rotation_was_authorized = self.localization_rotation_authorized
        active_hint_states = {
            "LOCALIZING_LAST_POSE", "LOCALIZING_APPROXIMATE_POSE",
            "CANDIDATE", "VERIFYING", "AMBIGUOUS", "LOCALIZING_GLOBAL",
            "LOCALIZING_ROTATING", "LOCALIZING_SETTLING",
        }
        previous_hint = self.localization_pending_operator_hint
        repeated_hint = bool(
            previous_hint is not None
            and math.hypot(
                operator_pose["x"] - float(previous_hint["x"]),
                operator_pose["y"] - float(previous_hint["y"]),
            ) <= 0.20
        )
        if repeated_hint and self.localization_state in active_hint_states:
            self.localization_rotation_authorized = rotation_was_authorized
            self._nav_debug(
                "LOCALIZATION_HINT_IGNORED",
                attempt_id=self.localization_attempt_id,
                reason="SAME_HINT_ATTEMPT_IN_PROGRESS",
                state=self.localization_state,
                evidence_preserved=True,
            )
            return {
                "status": "accepted",
                "current_state": self.current_state,
                "localized": False,
                "state": self._state(),
            }

        # This is an explicit operator correction after localization needs
        # assistance. Do not let a previously trusted odometry chain override
        # the newly supplied physical location. Automatic relocalization still
        # uses that odometry prior before asking the operator for a hint.
        self._begin_operator_pose_hint(
            operator_pose,
            rotation_was_authorized=rotation_was_authorized,
        )
        return {
            "status": "accepted",
            "current_state": "LOCALIZING_APPROXIMATE_POSE",
            "localized": False,
            "state": self._state(),
        }

    def _deactivate_map(self) -> dict[str, Any]:
        self._cancel_navigation("CANCELED")
        self._stop_localization_rotation()
        self.map_id = ""
        self.map_version = 0
        self.saved_map = None
        self.map_navigation_geometry = None
        self.stop_turn_planner = None
        self.execution_points = []
        self.execution_segment_directions = []
        self.execution_phase = "IDLE"
        self.failed_segments = []
        self._publish_failed_segments()
        self.active_map_path = None
        self.localized = False
        self.localization_rotation_authorized = False
        self.global_search_requires_rotation = False
        self.localization_state = "IDLE"
        self._set_state("NO_ACTIVE_MAP", "map_deactivated")
        self.current_mission_id = ""
        self.latest_global_path = []
        self.latest_dynamic_obstacles = []
        self._reset_dynamic_recovery()
        self.trajectory_map_from_odom = None
        self.localization_resume_context = None
        self.localization_resume_in_progress = False
        self.dynamic_overlay = DynamicObstacleOverlay(
            ttl_seconds=self.dynamic_overlay_ttl,
            cluster_distance=self.dynamic_overlay_cluster_distance,
            motion_threshold=self.dynamic_obstacle_motion_threshold,
            stationary_confirmation_seconds=(
                self.dynamic_obstacle_stationary_confirmation
            ),
            moving_confirmation_windows=(
                self.dynamic_obstacle_moving_confirmation_windows
            ),
        )
        self.visualization_revision += 1
        return {"status": "completed", "current_state": "NO_ACTIVE_MAP", "state": self._state()}

    def _goal_pose(self, goal: dict[str, Any]) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(goal["x"])
        message.pose.position.y = float(goal["y"])
        message.pose.orientation = quaternion_from_yaw(float(goal.get("yaw", 0)))
        return message

    def _validate_command_map(self, payload: dict[str, Any]) -> None:
        requested_map = str(payload.get("map_id") or "")
        requested_version = int(payload.get("version") or payload.get("map_version") or 0)
        if requested_map != self.map_id or requested_version != self.map_version:
            raise AdapterError(
                "MAP_MISMATCH",
                f"Map command {requested_map}/v{requested_version} does not match active {self.map_id}/v{self.map_version}",
            )

    def _validate_goal(self, goal: dict[str, Any]) -> None:
        if not self.localized or self.localization_state != "READY":
            raise AdapterError("NOT_LOCALIZED", "Robot chưa định vị READY trên saved map")
        if self.saved_map is None:
            raise AdapterError("NO_ACTIVE_MAP", "Map chưa được kích hoạt")
        allow_unknown = os.getenv("NAVIGATION_ALLOW_UNKNOWN_GOAL", "0").lower() in {
            "1", "true", "yes"
        }
        obstacles = tuple(
            (float(item["x"]), float(item["y"]))
            for item in self.latest_dynamic_obstacles
        )
        validation = self.saved_map.validate_goal(
            float(goal["x"]),
            float(goal["y"]),
            clearance_m=self.footprint_clearance,
            allow_unknown=allow_unknown,
            lethal_world_cells=obstacles,
        )
        if not validation.valid:
            raise AdapterError(validation.code, validation.message)
        if not self.stop_turn_require_final_yaw or goal.get("yaw") is None:
            # Position-only arrival is validated by the final directional
            # translation segment.  Requiring an arbitrary click yaw here
            # both rejects otherwise reachable goals and forces a needless
            # in-place turn after the destination has already been reached.
            return
        footprint_validation = self.saved_map.validate_footprint(
            float(goal["x"]),
            float(goal["y"]),
            float(goal.get("yaw", 0.0)),
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            padding=self.planning_footprint_padding,
            allow_unknown=allow_unknown,
            lethal_world_cells=obstacles,
            code_prefix="GOAL",
        )
        if not footprint_validation.valid:
            raise AdapterError(
                footprint_validation.code,
                footprint_validation.message,
            )

    def _resolve_planning_goal(self, goal: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        """Validate a click and, when needed, move it to a nearby safe cell."""
        try:
            self._validate_goal(goal)
            return goal, False
        except AdapterError as exc:
            if exc.code not in {
                "GOAL_UNKNOWN",
                "GOAL_OCCUPIED",
                "GOAL_CLEARANCE",
                "GOAL_LETHAL",
                "GOAL_FOOTPRINT_BLOCKED",
                "GOAL_FOOTPRINT_OUTSIDE_MAP",
            }:
                raise
            if self.saved_map is None:
                raise
            obstacles = tuple(
                (float(item["x"]), float(item["y"]))
                for item in self.latest_dynamic_obstacles
            )
            snapped = self.saved_map.nearest_valid_goal(
                float(goal["x"]),
                float(goal["y"]),
                clearance_m=self.footprint_clearance,
                max_distance_m=self.goal_snap_max_distance,
                allow_unknown=os.getenv("NAVIGATION_ALLOW_UNKNOWN_GOAL", "0").lower()
                in {"1", "true", "yes"},
                lethal_world_cells=obstacles,
                yaw=float(goal.get("yaw", 0.0)),
                footprint_half_length=(
                    self.footprint_half_length
                    if self.stop_turn_require_final_yaw
                    else None
                ),
                footprint_half_width=(
                    self.footprint_half_width
                    if self.stop_turn_require_final_yaw
                    else None
                ),
                footprint_padding=self.planning_footprint_padding,
                reachable_from=(
                    float(self.pose["x"]), float(self.pose["y"])
                ) if self.pose is not None else None,
            )
            if snapped is None:
                raise
            resolved = dict(goal)
            resolved["x"], resolved["y"] = snapped
            self._validate_goal(resolved)
            return resolved, True

    def _execution_destination(self, goal: dict[str, Any]) -> dict[str, Any]:
        destination = dict(goal)
        if not self.stop_turn_require_final_yaw:
            destination.pop("yaw", None)
        return destination

    def _record_planning_failure(
        self,
        error: AdapterError,
        requested_goal: dict[str, Any],
        resolved_goal: dict[str, Any],
        planning_started: float,
    ) -> None:
        start_pose = dict(self.pose or {})
        start_cost = (
            self._global_cost_at(
                float(start_pose.get("x", 0.0)),
                float(start_pose.get("y", 0.0)),
            )
            if start_pose
            else None
        )
        goal_cost = self._global_cost_at(
            float(resolved_goal.get("x", 0.0)),
            float(resolved_goal.get("y", 0.0)),
        )
        corridor_clearance = (
            self.nearest_left_obstacle + self.nearest_right_obstacle
            if math.isfinite(self.nearest_left_obstacle)
            and math.isfinite(self.nearest_right_obstacle)
            else math.inf
        )
        details = {
            "code": error.code,
            "message": str(error),
            "map_id": self.map_id,
            "map_version": self.map_version,
            "start_pose": start_pose,
            "requested_goal": dict(requested_goal),
            "resolved_goal": dict(resolved_goal),
            "footprint": list(self.footprint),
            "start_cell_cost": start_cost,
            "goal_cell_cost": goal_cost,
            "global_costmap_age_ms": self._age_milliseconds(
                self.last_global_costmap_monotonic
            ),
            "scan_age_ms": self._age_milliseconds(self.last_scan_monotonic),
            "nearest_forward_obstacle_m": self._finite_metric(
                self.nearest_forward_obstacle
            ),
            "left_clearance_m": self._finite_metric(self.nearest_left_obstacle),
            "right_clearance_m": self._finite_metric(self.nearest_right_obstacle),
            "corridor_clearance_m": self._finite_metric(corridor_clearance),
            "nearest_rotation_obstacle_m": self._finite_metric(
                self.nearest_rotation_obstacle
            ),
            "dynamic_obstacle_count": len(self.latest_dynamic_obstacles),
            "scan_filter": dict(self.last_scan_filter_stats),
            "near_start_cost_cells": (
                self._nearby_global_cost_counts(
                    float(start_pose.get("x", 0.0)),
                    float(start_pose.get("y", 0.0)),
                )
                if start_pose
                else {}
            ),
            "latency_ms": round(
                (time.monotonic() - planning_started) * 1000.0,
                3,
            ),
        }
        self.last_planning_failure = details
        self.planner_latency_ms = float(details["latency_ms"])
        self.get_logger().warning(
            "planning failure "
            + json.dumps(details, ensure_ascii=False, separators=(",", ":"))
        )
        self._nav_debug("PLAN_RESULT", status="FAILED", **details)

    def _classify_empty_path(self, goal: dict[str, Any]) -> AdapterError:
        pose = self.pose or {}
        start_cost = (
            self._global_cost_at(float(pose["x"]), float(pose["y"]))
            if "x" in pose and "y" in pose
            else None
        )
        goal_cost = self._global_cost_at(float(goal["x"]), float(goal["y"]))
        costmap_ready = (
            self.latest_global_costmap is not None
            and time.monotonic() - self.last_global_costmap_monotonic <= 1.5
        )
        tf_ready = (
            bool(pose)
            and time.monotonic() - self.last_map_tf_monotonic <= 0.60
            and self.tf_buffer.can_transform("map", "base_footprint", Time())
        )
        crosses_unknown = bool(
            self.saved_map is not None
            and pose
            and self.saved_map.segment_crosses_unknown(
                float(pose["x"]),
                float(pose["y"]),
                float(goal["x"]),
                float(goal["y"]),
            )
        )
        code = classify_planning_failure(
            tf_ready=tf_ready,
            costmap_ready=costmap_ready,
            start_cost=start_cost,
            goal_cost=goal_cost,
            route_crosses_unknown=crosses_unknown,
        )
        messages = {
            "START_BLOCKED": "Vùng xuất phát đang bị global costmap đánh dấu là vật cản.",
            "GOAL_BLOCKED": "Điểm đến đang bị global costmap đánh dấu là vật cản.",
            "UNKNOWN_SPACE": "Không thể lập đường vì lộ trình đi qua vùng chưa được lập bản đồ.",
            "COSTMAP_NOT_READY": "Global costmap chưa có dữ liệu mới để lập đường.",
            "TF_ERROR": "Không thể xác định transform map tới robot để lập đường.",
            "NO_VALID_PATH": "Không tìm thấy đường hợp lệ tới điểm đích.",
        }
        return AdapterError(code, messages[code])

    def _refresh_global_costmap_for_planning(self) -> None:
        """Clear only after a diagnosed failure, then await a real update."""
        if not self.clear_global_costmap_client.wait_for_service(timeout_sec=2.0):
            raise AdapterError(
                "GLOBAL_COSTMAP_UNAVAILABLE",
                "Global costmap reset service is unavailable",
            )
        baseline_generation = self.global_costmap_generation
        self._wait(
            self.clear_global_costmap_client.call_async(
                ClearEntireCostmap.Request()
            ),
            3.0,
            "GLOBAL_COSTMAP_RESET_TIMEOUT",
        )
        if not self._wait_for_global_costmap_after(baseline_generation, 2.0):
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Global costmap did not publish an update after reset",
            )

    def _request_path_once(
        self,
        goal: dict[str, Any],
        *,
        sync_planning_mask: bool = True,
        planner_id: str = "GridBased",
    ) -> list[dict[str, float]]:
        if sync_planning_mask:
            self._sync_planning_static_mask()
        request = ComputePathToPose.Goal()
        request.goal = self._goal_pose(goal)
        request.use_start = False
        request.planner_id = planner_id
        handle = self._wait(
            self.compute_path_client.send_goal_async(request),
            5,
            "PLANNER_TIMEOUT",
        )
        if not handle.accepted:
            raise AdapterError("PLAN_REJECTED", "Planner rejected goal")
        wrapped = self._wait(
            handle.get_result_async(), 15, "PLANNER_TIMEOUT"
        )
        return [
            {"x": pose.pose.position.x, "y": pose.pose.position.y}
            for pose in wrapped.result.path.poses
        ]

    def _localization_lateral_uncertainty(self) -> float:
        if not self.localized or self.localization_state != "READY":
            return self.corridor_localization_uncertainty_max
        covariance_allowance = 0.0
        if len(self.last_amcl_covariance) >= 36:
            covariance_allowance = math.sqrt(max(
                0.0,
                float(self.last_amcl_covariance[0]),
                float(self.last_amcl_covariance[7]),
            )) * 0.10
        confidence_allowance = max(
            0.0, 1.0 - float(self.localization_confidence)
        ) * 0.01
        return min(
            self.corridor_localization_uncertainty_max,
            covariance_allowance + confidence_allowance,
        )

    def _hard_route_side_clearance(self) -> float:
        """Physical minimum; the 7 cm corridor margin remains ranking-only."""
        map_resolution_allowance = (
            0.0 if self.saved_map is None else self.saved_map.resolution * 0.25
        )
        return (
            max(
                self.corridor_hard_side_margin,
                self.stop_turn_minimum_route_side_clearance,
            )
            + self._localization_lateral_uncertainty()
            + map_resolution_allowance
        )

    def _stop_turn_planner_for_clearance(
        self, required_side_clearance: float | None = None
    ) -> StopTurnStateLatticePlanner:
        """Create a request-local planner with the complete hard reserve.

        The map-level planner owns the configured 2 cm reserve.  A live route
        additionally needs localization and raster allowances.  Applying
        those only after planning can reject a perfectly routable wide aisle:
        the search has never been allowed to move its candidate farther from
        the wall.  Keep the shared geometry, but put the full hard requirement
        into every exact segment and turn check from the start.
        """
        base = self.stop_turn_planner
        if base is None:
            raise AdapterError(
                "PLANNER_NOT_READY", "Static stop-turn planner is unavailable"
            )
        requested = (
            self._hard_route_side_clearance()
            if required_side_clearance is None
            else max(0.0, float(required_side_clearance))
        )
        # Round upward, never downward, so tiny covariance jitter neither
        # weakens safety nor changes the search bands for sub-millimetre noise.
        hard_margin = math.ceil(requested * 1000.0 - 1e-9) / 1000.0
        if abs(hard_margin - base.hard_side_margin) <= 1e-9:
            return base
        return StopTurnStateLatticePlanner(
            base.saved_map,
            base.geometry,
            half_length=base.half_length,
            half_width=base.half_width,
            padding=base.padding,
            primitive_length=base.primitive_length,
            linear_speed=base.linear_speed,
            angular_speed=base.angular_speed,
            max_expansions=base.max_expansions,
            turn_robustness_radius=base.turn_robustness_radius,
            turn_bay_max_distance=base.turn_bay_max_distance,
            hard_side_margin=hard_margin,
            preferred_side_margin=max(
                hard_margin, base.preferred_side_margin
            ),
        )

    def _compute_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        planning_started = time.monotonic()
        self._validate_command_map(payload)
        requested_goal = dict(payload["goal"])
        resolved_goal = self._execution_destination(requested_goal)

        def mark_plan_failed(reason: str) -> None:
            # A preview failure occurs before motion and therefore leaves the
            # robot READY. Only an exhausted active FollowPath recovery may
            # enter BLOCKED.
            with self.state_lock:
                self._set_state("READY", reason)
                if self.latest_global_path:
                    self.latest_global_path = []
                    self.visualization_revision += 1

        try:
            resolved_goal, goal_adjusted = self._resolve_planning_goal(
                resolved_goal
            )
        except AdapterError as exc:
            mark_plan_failed(f"plan_preflight_failed:{exc.code}")
            self._record_planning_failure(
                exc,
                requested_goal,
                resolved_goal,
                planning_started,
            )
            raise
        if self.stop_turn_planner is None or self.pose is None:
            error = AdapterError(
                "PLANNER_NOT_READY",
                "Static stop-turn planner geometry is unavailable",
            )
            mark_plan_failed("stop_turn_planner_not_ready")
            self._record_planning_failure(
                error, requested_goal, resolved_goal, planning_started
            )
            raise error
        with self.state_lock:
            self.current_mission_id = ""
            self.execution_route_id = ""
            self._reset_dynamic_recovery()
            self._set_state("PLANNING", "stop_turn_lattice_request")
        self._nav_debug(
            "PLAN_REQUEST",
            start=dict(self.pose),
            goal=resolved_goal,
            planner="StopTurnStateLattice24",
            retry=0,
        )
        start_pose = dict(self.pose)
        live_start_clear = bool(
            self._atomic_safety_fresh()
            and not (self.safety_direction_mask & 1)
            and not self.estop_active
        )
        chassis_yaw = float(start_pose.get("yaw", 0.0))
        front_probe_distance = max(
            0.80,
            self.corridor_lookahead + self.footprint_half_length,
        )
        live_front_keepout = self._live_front_keepout_for_route(
            [
                dict(start_pose),
                {
                    "x": float(start_pose["x"])
                    + front_probe_distance * math.cos(chassis_yaw),
                    "y": float(start_pose["y"])
                    + front_probe_distance * math.sin(chassis_yaw),
                },
            ],
            (1,),
            # Before a route exists, a merely nearby return is not enough to
            # invent a hard map obstacle. It may be a side wall that becomes
            # harmless after the first stop-turn. Only confirmed physical
            # blockage may constrain the topology search at this stage; the
            # selected first segment is checked against precise live evidence
            # again below before any motion is dispatched.
            blocked_only=True,
        )
        dynamic_exclusions = (
            (live_front_keepout,)
            if live_front_keepout is not None
            else self._dynamic_exclusions()
        )
        if live_front_keepout is not None:
            self._nav_debug(
                "PLAN_LIVE_FRONT_KEEPOUT",
                center=(
                    round(live_front_keepout[0], 3),
                    round(live_front_keepout[1], 3),
                ),
                radius=round(live_front_keepout[2], 3),
                front_clearance=self._finite_metric(
                    float(self.latest_corridor.front_clearance)
                ),
                action="APPLY_BEFORE_SEARCH",
            )
        live_obstacle_search = live_front_keepout is not None
        planning_time_budget = (
            min(
                self.stop_turn_planning_budget,
                self.stop_turn_live_obstacle_planning_budget,
            )
            if live_obstacle_search
            else self.stop_turn_planning_budget
        )
        hard_side_clearance = self._hard_route_side_clearance()
        request_planner = self._stop_turn_planner_for_clearance(
            hard_side_clearance
        )
        planner_result = request_planner.plan_result(
            start_pose,
            resolved_goal,
            exclusions=dynamic_exclusions,
            planning_time_budget=planning_time_budget,
            allow_start_escape=True,
            maximum_start_escape_distance=self.start_escape_max_distance,
            live_start_clear=live_start_clear,
        )
        if planner_result.status in {
            "SEARCH_EXPANSION_LIMIT", "SEARCH_TIME_BUDGET_EXCEEDED"
        } and not live_obstacle_search:
            retry_expansion_limit = (
                request_planner.max_expansions
                * self.stop_turn_retry_expansion_multiplier
            )
            self._nav_debug(
                "PLAN_SEARCH_RETRY",
                first_status=planner_result.status,
                first_expansions=planner_result.expansions,
                first_duration_ms=round(
                    planner_result.elapsed_seconds * 1000.0, 3
                ),
                planning_time_budget=self.stop_turn_retry_planning_budget,
                expansion_limit=retry_expansion_limit,
            )
            planner_result = request_planner.plan_result(
                start_pose,
                resolved_goal,
                exclusions=dynamic_exclusions,
                planning_time_budget=self.stop_turn_retry_planning_budget,
                search_expansion_limit=retry_expansion_limit,
                allow_start_escape=True,
                maximum_start_escape_distance=self.start_escape_max_distance,
                live_start_clear=live_start_clear,
            )
        if planner_result.route is not None:
            live_front_keepout = self._live_front_keepout_for_route(
                [dict(point) for point in planner_result.route.points],
                tuple(planner_result.route.segment_directions),
                blocked_only=False,
            )
            if live_front_keepout is not None:
                # The Saved Map can be stale around a chair, trolley or wall
                # edge.  If the selected first straight points directly into
                # fresh LiDAR evidence, replan before moving and use only that
                # precise local keepout; the broad live overlay contains many
                # map-mismatch clusters and is not a hard-wall authority.
                self._nav_debug(
                    "PLAN_LIVE_FRONT_KEEPOUT",
                    first_route=[
                        dict(point) for point in planner_result.route.points
                    ],
                    center=(
                        round(live_front_keepout[0], 3),
                        round(live_front_keepout[1], 3),
                    ),
                    radius=round(live_front_keepout[2], 3),
                    front_clearance=self._finite_metric(
                        float(self.latest_corridor.front_clearance)
                    ),
                    action="REPLAN_BEFORE_MOTION",
                )
                planner_result = request_planner.plan_result(
                    start_pose,
                    resolved_goal,
                    exclusions=(live_front_keepout,),
                    planning_time_budget=min(
                        self.stop_turn_planning_budget,
                        self.stop_turn_live_obstacle_planning_budget,
                    ),
                    allow_start_escape=True,
                    maximum_start_escape_distance=self.start_escape_max_distance,
                    live_start_clear=live_start_clear,
                )
        turn_diagnostics = dict(planner_result.diagnostics)
        if turn_diagnostics:
            rejections = list(
                turn_diagnostics.pop("turn_route_rejections", [])
            )
            turn_diagnostics["hard_clearance_required"] = hard_side_clearance
            self._nav_debug(
                "TURN_ROUTE_DIAGNOSTICS",
                **turn_diagnostics,
            )
            for rejection in rejections:
                self._nav_debug("TURN_ROUTE_REJECT", **rejection)
        if (
            planner_result.route is not None
            and planner_result.route.metadata.minimum_side_clearance + 1e-9
            < hard_side_clearance
        ):
            error = AdapterError(
                "ROUTE_CLEARANCE_INSUFFICIENT",
                (
                    "No route keeps the configured side clearance; "
                    f"best={planner_result.route.metadata.minimum_side_clearance:.3f}m "
                    f"hard_required={hard_side_clearance:.3f}m "
                    f"preferred={self.corridor_side_margin:.3f}m"
                ),
            )
            mark_plan_failed("route_clearance_insufficient")
            self._record_planning_failure(
                error, requested_goal, resolved_goal, planning_started
            )
            raise error
        planned = (
            []
            if planner_result.route is None
            else request_planner.plan_candidates(
                start_pose,
                resolved_goal,
                # Preview stays latency-sensitive; explicit alternatives are
                # still requested separately.
                maximum_candidates=1,
                overlap_threshold=self.alternative_route_overlap_threshold,
                planning_time_budget=self.stop_turn_planning_budget,
                exclusions=dynamic_exclusions,
                primary_route=planner_result.route,
            )
        )
        try:
            candidates = self._serialize_stop_turn_candidates(planned)
        except AdapterError as exc:
            mark_plan_failed(f"route_validation_failed:{exc.code}")
            self._record_planning_failure(
                exc, requested_goal, resolved_goal, planning_started
            )
            raise
        if candidates and planner_result.diagnostics:
            candidate_diagnostics = dict(planner_result.diagnostics)
            candidate_diagnostics[
                "hard_clearance_required"
            ] = hard_side_clearance
            candidates[0]["planning_diagnostics"] = candidate_diagnostics
        if candidates and planner_result.start_escape is not None:
            escape = planner_result.start_escape
            planned_points = list(candidates[0]["points"])
            display_points = [dict(escape.start), dict(escape.end)]
            display_points.extend(planned_points[1:])
            candidates[0]["planned_points_after_escape"] = planned_points
            candidates[0]["points"] = canonicalize_stop_turn_path(display_points)
            candidates[0]["start_escape"] = {
                "start": dict(escape.start),
                "end": dict(escape.end),
                "yaw": escape.yaw,
                "distance": escape.distance,
                "motion_direction": escape.motion_direction,
                "initial_overlap_cells": [list(cell) for cell in escape.initial_overlap_cells],
            }
            candidates[0]["total_length"] = round(
                float(candidates[0]["total_length"]) + escape.distance, 4
            )
        if not candidates:
            code = planner_result.reason or planner_result.status
            if code == "SUCCESS":
                code = "NO_FEASIBLE_ROUTE"
            public_code = (
                "GOAL_PHYSICALLY_UNREACHABLE"
                if code == "GOAL_DISCONNECTED"
                else code
            )
            error = AdapterError(
                public_code,
                planner_result.message or "No exact stop-turn route is available",
            )
            mark_plan_failed("stop_turn_no_valid_path")
            self._record_planning_failure(
                error, requested_goal, resolved_goal, planning_started
            )
            raise error
        # plan_candidates() already applies the geometry-aware ranking.  Keep
        # that order here; re-sorting by an unbounded room-width ray would
        # reintroduce needless elbows after clearance is already ample.
        selected = candidates[0]
        with self.state_lock:
            self.route_candidates = {
                str(candidate["route_id"]): candidate for candidate in candidates
            }
            self.selected_route_id = str(selected["route_id"])
            self.paused_goal = dict(resolved_goal)
            self.route_selection_return_state = "READY"
            self.latest_global_path = list(selected["points"])
            self.visualization_revision += 1
            self.planner_latency_ms = round(
                (time.monotonic() - planning_started) * 1000.0, 3
            )
            self.last_planning_failure = {}
            self._set_state("READY", "stop_turn_plan_success")
        self._nav_debug(
            "PLAN_RESULT",
            status="SUCCESS",
            planner="StopTurnStateLattice24",
            candidates=len(candidates),
            route_id=self.selected_route_id,
            points=len(selected["points"]),
            length=selected["total_length"],
            minimum_side_clearance=selected.get("minimum_side_clearance"),
            minimum_passage_width=selected.get("minimum_passage_width"),
            duration_ms=self.planner_latency_ms,
            goal_adjusted=goal_adjusted,
            segment_directions=selected.get("segment_directions", []),
        )
        return {
            "status": "completed",
            "current_state": "READY",
            "route_id": self.selected_route_id,
            "points": list(selected["points"]),
            "route_candidates": candidates,
            "distance_m": selected["total_length"],
            "goal": resolved_goal,
            "requested_goal": requested_goal,
            "goal_adjusted": goal_adjusted,
            "state": self._state(),
        }

    def _navigate(
        self,
        goal_payload: dict[str, Any],
        command_payload: dict[str, Any],
        *,
        recovery_attempt: bool = False,
    ) -> dict[str, Any]:
        self._validate_command_map(command_payload)
        goal_payload = self._execution_destination(goal_payload)
        self._validate_goal(goal_payload)
        if (
            not recovery_attempt
            and not self._wait_for_localization_start_evidence()
        ):
            self._nav_debug(
                "NAVIGATION_START_REJECTED",
                reason="LOCALIZATION_UNRELIABLE",
                localization_state=self.localization_state,
                localized=self.localized,
                confidence=self.localization_confidence,
                scan_age_ms=self._age_milliseconds(self.last_scan_monotonic),
                amcl_age_ms=self._age_milliseconds(self.last_amcl_monotonic),
                tf_age_ms=self._age_milliseconds(self.last_map_tf_monotonic),
                scan_map_age_ms=self._age_milliseconds(
                    self.last_scan_map_monotonic
                ),
                sensor_time_healthy=self._critical_sensor_time_healthy(),
                raycast_comparable_beams=self.raycast_comparable_beams,
                raycast_static_matches=self.raycast_static_matches,
                raycast_contradiction_ratio=self.raycast_contradiction_ratio,
                evidence_wait_seconds=self.localization_start_evidence_wait,
            )
            raise AdapterError(
                "LOCALIZATION_UNRELIABLE",
                "Localization quality must be freshly verified before navigation",
            )
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            raise AdapterError("NAV2_UNAVAILABLE", "FollowPath action unavailable")
        route_id = str(
            command_payload.get("route_id")
            or command_payload.get("mission_id")
            or self.current_mission_id
            or "planned-route"
        )
        candidate = self.route_candidates.get(route_id)
        points = list(command_payload.get("points") or [])
        segment_directions = list(
            command_payload.get("segment_directions") or []
        )
        if candidate is not None:
            points = list(candidate.get("points") or [])
            segment_directions = list(
                candidate.get("segment_directions") or []
            )
            if candidate.get("start_escape"):
                return self._start_escape_execution(
                    goal_payload,
                    route_id=route_id,
                    display_points=points,
                    recovery_attempt=recovery_attempt,
                )
        if not points and route_id == self.selected_route_id:
            points = list(self.latest_global_path)
        points = canonicalize_stop_turn_path(points)
        if len(points) < 2:
            raise AdapterError(
                "NO_VALID_PATH",
                "Selected preview route has no canonical stop-turn geometry",
            )
        if not segment_directions:
            segment_directions = [1 for _ in range(len(points) - 1)]
        segment_directions = [
            -1 if int(value) < 0 else 1 for value in segment_directions
        ]
        if len(segment_directions) != len(points) - 1:
            raise AdapterError(
                "ROUTE_DIRECTION_INVALID",
                "Selected route motion directions do not match its segments",
            )
        points = self._ensure_executable_path(
            points,
            goal_payload,
            context="RECOVERY_FOLLOW_PATH" if recovery_attempt else "PRE_FOLLOW_PATH",
            segment_directions=segment_directions,
        )
        if self.saved_map is None:
            raise AdapterError("MAP_MISSING", "Saved map geometry is unavailable")
        static_validation = validate_stop_turn_route(
            self.saved_map,
            points,
            half_length=self.footprint_half_length,
            half_width=(
                self.footprint_half_width + self.translation_lateral_margin
            ),
            padding=self.planning_footprint_padding,
            segment_directions=segment_directions,
        )
        if not static_validation.valid:
            raise AdapterError(static_validation.code, "Selected route failed swept-footprint validation")
        route_metadata = self._route_metadata(
            points,
            original=points,
            segment_directions=segment_directions,
        )
        if not route_metadata["valid"]:
            raise AdapterError("ROUTE_INVALIDATED", "Selected route is no longer safe")
        with self.state_lock:
            self.navigation_goal_generation += 1
            self.latest_feedback = {
                "recoveries": 0,
                "execution_phase": "STRAIGHT_PREPARE",
            }
            if not recovery_attempt:
                self.navigation_recovery_attempts = 0
                self.execution_replan_attempts = 0
                self.navigation_corridor_clear_retried = False
                self.navigation_original_path_length = self._path_length(points)
                self.corridor_samples.clear()
                self.controller_abort_history.clear()
                self.last_controller_blockage_monotonic = 0.0
            self.execution_points = list(points)
            self.execution_segment_directions = list(segment_directions)
            self.execution_segment_index = 0
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_phase = "STRAIGHT_PREPARE"
            self.execution_phase_started = time.monotonic()
            self.execution_turn_stable_since = None
            self.execution_turn_reentry_since = None
            self.execution_turn_blocked_since = None
            self.execution_reanchor_after_turn = False
            self.execution_segment_reanchors = 0
            self.segment_started_monotonic = 0.0
            self.segment_positive_travel = 0.0
            self.segment_last_travel_pose = None
            self.straight_recovery_requested = ""
            self.execution_final_turn = False
            self.execution_physical_final_turn = False
            self.execution_goal = dict(goal_payload)
            self.execution_relocation_reason = ""
            self.execution_relocation_plan = []
            self.execution_route_id = route_id
            self.execution_narrow_segments = {
                int(item["segment_index"])
                for item in route_metadata.get("narrow_segments", [])
            }
            self.current_goal_handle = None
            self.paused_goal = dict(goal_payload)
            self.latest_global_path = list(points)
            self.selected_route_id = route_id
            if not recovery_attempt:
                self.dynamic_block_reason = ""
                self.dynamic_blocked_route = []
                self.dynamic_blocked_segment_directions = []
                self.dynamic_wait_started = None
                self.dynamic_clear_started = None
                self.dynamic_recovery_state = "IDLE"
                self.dynamic_replan_requires_alternative = False
            self.last_controller_blockage_monotonic = 0.0
            self.motion_owner = "NONE"
            self._set_state("NAVIGATING", "stop_turn_route_accepted")
            self.visualization_revision += 1
        self._persist_active_navigation_mission(resume_automatically=True)
        self.profile_limiter.reset()
        self.navigation_velocity.publish(Twist())
        self._nav_debug(
            "ROUTE_SELECTED",
            route_id=route_id,
            actual_execution_path_route_id=route_id,
            points=len(points),
            length=self._path_length(points),
            executor="StopTurnSegmentExecutor",
            segment_directions=segment_directions,
        )
        return {
            "status": "accepted",
            "current_state": "NAVIGATING",
            "route_id": route_id,
            "points": points,
            "goal": dict(goal_payload),
            "state": self._state(),
        }

    def _start_escape_execution(
        self,
        goal: dict[str, Any],
        *,
        route_id: str,
        display_points: list[dict[str, float]],
        recovery_attempt: bool,
    ) -> dict[str, Any]:
        if self.saved_map is None or self.pose is None:
            raise AdapterError("PLANNER_NOT_READY", "Saved Map or current pose is unavailable")
        if not self._atomic_safety_fresh() or self.estop_active:
            self.navigation_velocity.publish(Twist())
            raise AdapterError(
                "START_ESCAPE_UNAVAILABLE",
                "Start escape safety data is unavailable",
            )
        if self.safety_direction_mask & 1:
            self.navigation_velocity.publish(Twist())
            raise AdapterError(
                "START_ESCAPE_UNAVAILABLE",
                "The forward start-escape direction is blocked by live safety",
            )
        escape = find_start_escape(
            self.saved_map,
            dict(self.pose),
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            padding=self.planning_footprint_padding,
            maximum_distance=self.start_escape_max_distance,
            directions=(1,),
        )
        if escape is None:
            raise AdapterError(
                "START_ESCAPE_UNAVAILABLE",
                "No monotonic-overlap straight escape remains from the current pose",
            )
        with self.state_lock:
            self.navigation_goal_generation += 1
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_points = [dict(escape.start), dict(escape.end)]
            self.execution_segment_directions = [escape.motion_direction]
            self.execution_segment_index = 0
            self.execution_phase = "STRAIGHT_PREPARE"
            self.execution_phase_started = time.monotonic()
            self.execution_goal = dict(goal)
            self.paused_goal = dict(goal)
            self.execution_route_id = route_id
            self.execution_relocation_reason = "START_ESCAPE"
            self.execution_relocation_plan = list(display_points)
            self.execution_narrow_segments = set()
            self.latest_global_path = list(display_points)
            self.selected_route_id = route_id
            self.current_goal_handle = None
            self.motion_owner = "NONE"
            self.latest_feedback = {
                "recoveries": int(recovery_attempt),
                "execution_phase": "START_ESCAPE",
                "recovery_reason": "START_STATIC_OVERLAP",
            }
            self._set_state("NAVIGATING", "start_escape_accepted")
            self.visualization_revision += 1
        self._persist_active_navigation_mission(
            resume_automatically=True,
            path=display_points,
        )
        self.navigation_velocity.publish(Twist())
        self._nav_debug(
            "START_ESCAPE",
            status="BEGIN",
            route_id=route_id,
            start=escape.start,
            end=escape.end,
            yaw=escape.yaw,
            distance=escape.distance,
            motion=("REVERSE" if escape.motion_direction < 0 else "FORWARD"),
            initial_overlap_cells=escape.initial_overlap_cells,
        )
        return {
            "status": "accepted",
            "current_state": "NAVIGATING",
            "route_id": route_id,
            "points": list(display_points),
            "goal": dict(goal),
            "recovery": "START_ESCAPE",
            "state": self._state(),
        }

    def _final_position_distance(
        self, pose: dict[str, float]
    ) -> float | None:
        goal = self.execution_goal
        final_translation = bool(
            goal is not None
            and not self.execution_relocation_reason
            and len(self.execution_points) >= 2
            and self.execution_segment_index == len(self.execution_points) - 2
            and (
                not self.stop_turn_require_final_yaw
                or goal.get("yaw") is None
            )
        )
        if not final_translation:
            return None
        distance = math.hypot(
            float(pose["x"]) - float(goal["x"]),
            float(pose["y"]) - float(goal["y"]),
        )
        return distance if math.isfinite(distance) else None

    def _complete_final_position_if_reached(
        self,
        pose: dict[str, float],
        goal_generation: int,
    ) -> bool:
        if goal_generation != self.navigation_goal_generation:
            return False
        distance = self._final_position_distance(pose)
        if (
            distance is None
            or not position_within_tolerance(
                pose,
                self.execution_goal or {},
                self.stop_turn_final_position_tolerance,
            )
        ):
            return False
        self._finish_execution_success(
            0.0,
            position_distance=distance,
            cancel_active=True,
        )
        return True

    def _segment_execution_tick(self) -> None:
        if self.current_state != "NAVIGATING" or len(self.execution_points) < 2:
            return
        generation = self.navigation_goal_generation
        now = time.monotonic()
        pose = self._fresh_execution_pose()
        if pose is None:
            self.motion_owner = "NONE"
            self.navigation_velocity.publish(Twist())
            return

        if self.execution_phase == "STRAIGHT_PREPARE":
            self.motion_owner = "NONE"
            self.navigation_velocity.publish(Twist())
            if self._complete_final_position_if_reached(pose, generation):
                return
            phase_elapsed = now - self.execution_phase_started
            final_velocity = self.pipeline_samples.get("motion_safety")
            final_velocity_settled = bool(
                final_velocity is not None
                and now - final_velocity[2] <= 0.30
                and abs(final_velocity[0]) <= 0.02
                and abs(final_velocity[1]) <= 0.03
            )
            if (
                not final_velocity_settled
                and phase_elapsed < self.execution_velocity_settle_timeout
            ):
                return
            if phase_elapsed < self.execution_settle_seconds:
                return
            self._prepare_active_segment(generation, pose)
            return

        if self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}:
            if self._complete_final_position_if_reached(pose, generation):
                return
            active = self.active_segment
            if active is None:
                self.straight_recovery_requested = "ACTIVE_SEGMENT_MISSING"
            if self.straight_recovery_requested:
                reason = self.straight_recovery_requested
                self.straight_recovery_requested = ""
                self._restart_segment_from_current(reason, generation)
                return
            assert active is not None
            progress = straight_segment_progress(
                active.effective_start,
                active.endpoint,
                pose,
                overshoot_epsilon=self.straight_overshoot_epsilon,
            )
            current_xy = (float(pose["x"]), float(pose["y"]))
            if self.segment_last_travel_pose is not None:
                self.segment_positive_travel += math.hypot(
                    current_xy[0] - self.segment_last_travel_pose[0],
                    current_xy[1] - self.segment_last_travel_pose[1],
                )
            self.segment_last_travel_pose = current_xy
            if progress.endpoint_distance <= self.straight_endpoint_tolerance:
                self._complete_active_segment(
                    "GEOMETRIC_ENDPOINT", generation, cancel_action=True
                )
                return
            if progress.passed_endpoint:
                self.navigation_velocity.publish(Twist())
                if abs(progress.signed_cross_track) <= self.straight_endpoint_tolerance:
                    self._complete_active_segment(
                        "GEOMETRIC_OVERSHOOT", generation, cancel_action=True
                    )
                else:
                    self._schedule_execution_replan(
                        "ENDPOINT_OVERSHOOT_CROSS_TRACK", generation
                    )
                return
            profile = self.speed_profiles.get(
                "SLOW" if active.narrow else self.auto_speed_mode
            )
            watchdog = segment_travel_watchdog(
                segment_length=active.segment_length,
                elapsed=max(0.0, now - self.segment_started_monotonic),
                positive_travel=self.segment_positive_travel,
                expected_speed=profile.linear_max,
                settle_allowance=self.segment_watchdog_settle_allowance,
                travel_factor=self.segment_watchdog_travel_factor,
                minimum_travel_slack=max(
                    self.segment_watchdog_travel_slack,
                    2.0 * self.footprint_half_length,
                ),
                time_factor=self.segment_watchdog_time_factor,
            )
            if watchdog.exceeded:
                self._nav_debug(
                    "SEGMENT_WATCHDOG",
                    reason=watchdog.reason,
                    segment_index=active.segment_index,
                    segment_token=active.segment_token,
                    elapsed=now - self.segment_started_monotonic,
                    elapsed_limit=watchdog.elapsed_limit,
                    positive_travel=self.segment_positive_travel,
                    travel_limit=watchdog.travel_limit,
                    along_track=progress.along_track,
                )
                self._schedule_execution_replan(watchdog.reason, generation)
            return

        if self.execution_phase not in {
            "TURN", "TURN_SETTLING", "WAIT_FOR_TURN_CLEAR"
        }:
            return
        target = self.execution_target_heading
        current_yaw = float(pose.get("yaw", target))
        error = self._yaw_delta(target, current_yaw)
        if self.execution_phase == "TURN_SETTLING":
            self.motion_owner = "NONE"
            self.last_turn_command = (0.0, 0.0)
            self.navigation_velocity.publish(Twist())
            # TURN_SETTLING is a Schmitt-trigger band.  The tighter tolerance
            # admitted us here; only leaving the wider re-entry band resets
            # the zero-command dwell.  Resetting at the tight threshold left
            # a 3-6 degree dead zone that could wait forever at a corner.
            if abs(error) > self.execution_turn_reentry_tolerance:
                self.execution_turn_stable_since = None
                if self.execution_turn_reentry_since is None:
                    self.execution_turn_reentry_since = now
            elif self.execution_turn_stable_since is None:
                self.execution_turn_reentry_since = None
                self.execution_turn_stable_since = now
            else:
                self.execution_turn_reentry_since = None
            transition = turn_hysteresis_transition(
                "TURN_SETTLING",
                error,
                completion_tolerance=self.execution_turn_tolerance,
                reentry_tolerance=self.execution_turn_reentry_tolerance,
                stable_elapsed=(
                    0.0
                    if self.execution_turn_stable_since is None
                    else now - self.execution_turn_stable_since
                ),
                stable_dwell=self.execution_turn_stable_dwell,
            )
            if transition == "TURN":
                if (
                    self.execution_turn_reentry_since is None
                    or now - self.execution_turn_reentry_since
                    < self.execution_turn_reentry_dwell
                ):
                    return
                # The chassis can coast through the target while settling.
                # Reusing the TURN_BEGIN direction after that sign change
                # drives away from the target and can produce endless full
                # rotations. Re-evaluate both direction and safety from the
                # current pose before applying another angular command.
                previous_direction = self.execution_turn_direction
                left_static = self._turn_static_safe(pose, target, 1)
                right_static = self._turn_static_safe(pose, target, -1)
                snapshot_fresh = self._atomic_safety_fresh(now)
                left_live = snapshot_fresh and not self._safety_blocks_turn(1)
                right_live = snapshot_fresh and not self._safety_blocks_turn(-1)
                direction = choose_turn_direction(
                    error,
                    left_static_safe=left_static,
                    right_static_safe=right_static,
                    left_live_safe=left_live,
                    right_live_safe=right_live,
                )
                self.execution_turn_direction = (
                    direction if direction else (1 if error > 0.0 else -1)
                )
                self.execution_turn_reentry_since = None
                self.execution_phase = (
                    "TURN" if direction else "WAIT_FOR_TURN_CLEAR"
                )
                self.execution_phase_started = now
                self.execution_turn_blocked_since = None if direction else now
                self.turn_block_tracker = TurnBlockTracker(
                    clear_dwell_seconds=0.30
                )
                self.latest_feedback["execution_phase"] = self.execution_phase
                self._nav_debug(
                    "EXECUTION_PHASE",
                    phase=(
                        "TURN_REENTER"
                        if direction
                        else "WAIT_FOR_TURN_CLEAR"
                    ),
                    segment_index=self.execution_segment_index,
                    heading_error=error,
                    previous_direction=previous_direction,
                    direction=self.execution_turn_direction,
                )
                return
            if transition != "STRAIGHT_PREPARE":
                return
            if self.execution_final_turn:
                self._finish_execution_success(error)
                return
            self._nav_debug(
                "EXECUTION_PHASE",
                phase="TURN_END",
                segment_index=self.execution_segment_index,
                heading_error=error,
            )
            if self.execution_reanchor_after_turn:
                # TURN/settling may have translated the chassis. Re-anchor
                # once from a fresh pose before FollowPath dispatch. A hard
                # cap prevents localization noise from creating an endless
                # TURN -> re-anchor -> TURN loop at a corner.
                self.execution_reanchor_after_turn = False
                self.execution_segment_reanchors += 1
                self.active_segment = None
                self.execution_phase = "STRAIGHT_PREPARE"
                self.execution_phase_started = now - self.execution_settle_seconds
                self.latest_feedback["execution_phase"] = "STRAIGHT_PREPARE"
            else:
                self._dispatch_prepared_segment(generation)
            return

        direction = self.execution_turn_direction or (1 if error > 0.0 else -1)
        atomic_fresh = self._atomic_safety_fresh(now)
        live_safe_sample = (
            now - self.last_scan_monotonic <= 0.30
            and self._critical_sensor_time_healthy()
            and not self.estop_active
            and atomic_fresh
            and not self._safety_blocks_turn(direction)
            and now - self.last_manual_takeover_monotonic > 0.5
        )
        held_blocked = (
            not atomic_fresh
            or self.turn_block_tracker.update(
                sequence=self.safety_snapshot_sequence,
                blocked=not live_safe_sample,
                now=now,
            )
        )
        if held_blocked:
            self.execution_phase = "WAIT_FOR_TURN_CLEAR"
            self.latest_feedback["execution_phase"] = "WAIT_FOR_TURN_CLEAR"
            self.motion_owner = "NONE"
            self.last_turn_command = (0.0, 0.0)
            self.navigation_velocity.publish(Twist())
            if self.execution_turn_blocked_since is None:
                self.execution_turn_blocked_since = now
            elif (
                now - self.execution_turn_blocked_since
                >= self.execution_turn_safety_block_timeout
            ):
                self._nav_debug(
                    "TURN_RECOVERY",
                    reason="PERSISTENT_SAFETY_BLOCK",
                    blocked_since=self.execution_turn_blocked_since,
                    safety_sequence=self.safety_snapshot_sequence,
                )
                # A persistent block is geometric evidence that this pose is
                # not a usable turning bay. Switching rotation direction here
                # can merely drive to the other blocked side, then switch back
                # forever. Keep the chassis heading fixed and relocate along
                # it instead; the bay search considers both forward and
                # reverse and motion-safety still gates the chosen translation.
                self._nav_debug(
                    "TURN_RECOVERY",
                    action="RELOCATE_TO_TURN_BAY",
                    blocked_direction=direction,
                    target_heading=target,
                )
                self._start_turn_bay_recovery(pose, generation)
            return
        self.execution_phase = "TURN"
        self.latest_feedback["execution_phase"] = "TURN"
        self.execution_turn_blocked_since = None
        transition = turn_hysteresis_transition(
            "TURN",
            error,
            completion_tolerance=self.execution_turn_tolerance,
            reentry_tolerance=self.execution_turn_reentry_tolerance,
            stable_elapsed=0.0,
            stable_dwell=self.execution_turn_stable_dwell,
        )
        if transition == "TURN_SETTLING":
            self.motion_owner = "NONE"
            self.last_turn_command = (0.0, 0.0)
            self.navigation_velocity.publish(Twist())
            self.execution_phase = "TURN_SETTLING"
            self.execution_phase_started = now
            self.execution_turn_stable_since = now
            self.execution_turn_reentry_since = None
            self.latest_feedback["execution_phase"] = "TURN_SETTLING"
            return
        command = Twist()
        command.linear.x = 0.0
        measured = self.pipeline_samples.get("motion_safety")
        measured_angular = (
            0.0
            if (
                measured is None
                or now - measured[2] > 0.25
                or direction * measured[1] <= 0.0
            )
            else abs(measured[1])
        )
        braking_limit = turn_braking_speed_limit(
            error,
            completion_tolerance=self.execution_turn_tolerance,
            angular_deceleration=self.execution_turn_angular_deceleration,
            reaction_time=self.execution_turn_reaction_time,
            current_angular_speed=measured_angular,
        )
        command.angular.z = direction * min(
            self.execution_turn_max_speed,
            braking_limit,
            max(0.08, self.execution_turn_kp * abs(error)),
        )
        self.motion_owner = "NAVIGATION"
        self.last_turn_command = (0.0, float(command.angular.z))
        self.navigation_velocity.publish(command)
        if now - self.last_turn_command_log_monotonic >= 0.5:
            self._nav_debug(
                "TURN_CMD",
                segment_index=self.execution_segment_index,
                target_heading=target,
                current_heading=current_yaw,
                heading_error=error,
                requested_angular=command.angular.z,
                published_angular=command.angular.z,
                final_output_angular=(None if measured is None else measured[1]),
            )
            self.last_turn_command_log_monotonic = now

    def _start_turn_bay_recovery(
        self, pose: dict[str, float], goal_generation: int
    ) -> None:
        if (
            goal_generation != self.navigation_goal_generation
            or self.execution_phase == "TURN_BAY_SEARCH"
        ):
            return
        self.execution_phase = "TURN_BAY_SEARCH"
        self.latest_feedback["execution_phase"] = "TURN_BAY_RECOVERY"
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        self._set_state("TURN_BAY_RECOVERY", "persistent_turn_safety_block")
        threading.Thread(
            target=self._find_and_start_turn_bay,
            args=(dict(pose), goal_generation),
            daemon=True,
        ).start()

    def _find_and_start_turn_bay(
        self, pose: dict[str, float], goal_generation: int
    ) -> None:
        planner = (
            None
            if self.stop_turn_planner is None
            else self._stop_turn_planner_for_clearance()
        )
        saved_map = self.saved_map
        goal = dict(self.execution_goal or self.paused_goal or {})
        if planner is None or saved_map is None or not goal:
            return
        if (
            not self._atomic_safety_fresh()
            or self.estop_active
        ):
            self._set_state(
                "WAITING_FOR_DYNAMIC_CLEAR", "turn_bay_safety_unavailable"
            )
            self.dynamic_wait_started = time.monotonic()
            return
        yaw = float(pose.get("yaw", 0.0))
        step = max(saved_map.resolution, 0.025)
        exclusions = self._dynamic_exclusions()
        selected: tuple[dict[str, float], Any, int] | None = None
        directions = tuple(
            direction
            for direction in preferred_turn_bay_directions(pose, goal)
            if not (
                self.safety_direction_mask & (2 if direction < 0 else 1)
            )
        )
        for direction in directions:
            for index in range(
                1, math.floor(self.turn_bay_max_distance / step) + 1
            ):
                distance = direction * index * step
                candidate = {
                    "x": float(pose["x"]) + distance * math.cos(yaw),
                    "y": float(pose["y"]) + distance * math.sin(yaw),
                    "yaw": yaw,
                }
                straight = validate_stop_turn_route(
                    saved_map,
                    (pose, candidate),
                    half_length=self.footprint_half_length,
                    half_width=(
                        self.footprint_half_width
                        + self.translation_lateral_margin
                    ),
                    padding=self.planning_footprint_padding,
                    segment_directions=(direction,),
                )
                if (
                    not straight.valid
                    or planner._segment_excluded(pose, candidate, exclusions)
                ):
                    continue
                result = planner.plan_result(
                    candidate,
                    goal,
                    exclusions=exclusions,
                    planning_time_budget=min(
                        3.0, self.stop_turn_planning_budget
                    ),
                )
                if result.success:
                    selected = (candidate, result.route, direction)
                    break
            if selected is not None:
                break
        if goal_generation != self.navigation_goal_generation:
            return
        if selected is None:
            self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
            self.latest_feedback["execution_phase"] = "WAITING_FOR_DYNAMIC_CLEAR"
            self.latest_feedback["recovery_reason"] = (
                "TURN_BLOCKED_NO_SAFE_RELOCATION"
            )
            self.dynamic_wait_started = time.monotonic()
            self._set_state(
                "WAITING_FOR_DYNAMIC_CLEAR", "turn_blocked_no_safe_relocation"
            )
            self._nav_debug(
                "TURN_BAY_RECOVERY",
                result="WAIT",
                reason="TURN_BLOCKED_NO_SAFE_RELOCATION",
                destination_preserved=True,
            )
            return
        candidate, continuation, direction = selected
        display = [dict(pose), dict(candidate), *[dict(p) for p in continuation.points[1:]]]
        self.execution_segment_token += 1
        self.active_segment = None
        self.execution_points = [dict(pose), dict(candidate)]
        self.execution_segment_directions = [direction]
        self.execution_segment_index = 0
        self.execution_phase = "STRAIGHT_PREPARE"
        self.execution_phase_started = time.monotonic()
        self.execution_relocation_reason = "TURN_BAY"
        self.execution_relocation_plan = canonicalize_stop_turn_path(display)
        self.execution_narrow_segments = set()
        self.latest_global_path = list(self.execution_relocation_plan)
        self._set_state("NAVIGATING", "move_to_turn_bay")
        self._nav_debug(
            "TURN_BAY_RECOVERY",
            result="MOVE_TO_TURN_BAY",
            candidate=candidate,
            motion=("REVERSE" if direction < 0 else "FORWARD"),
            distance=math.hypot(
                candidate["x"] - float(pose["x"]),
                candidate["y"] - float(pose["y"]),
            ),
            destination=goal,
        )

    def _odom_pose(self) -> dict[str, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException:
            return None
        return {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": self._yaw_from_quaternion(transform.transform.rotation),
        }

    def _execution_pose_guard_active(self) -> bool:
        return bool(
            self.current_state == "NAVIGATING"
            and self.paused_goal is not None
            and self.execution_phase != "IDLE"
        )

    def _execution_pose_candidate_accepted(
        self, candidate: dict[str, float]
    ) -> bool:
        """Freeze motion/UI before a map-pose jump can steer the route.

        AMCL is allowed to correct the map->odom transform during travel, but
        that correction must agree with the relative wheel-odometry motion.
        One transient mismatch stops output immediately; only a sustained
        mismatch pauses the mission for full localization verification.
        """
        odom = self._odom_pose()
        now = time.monotonic()
        assessment: Any = None
        observed = False
        recovered = False
        confirmed = False
        with self.execution_pose_guard_lock:
            if self.execution_pose_hold:
                return False
            if not self._execution_pose_guard_active():
                self.execution_pose_anchor_map = dict(candidate)
                self.execution_pose_anchor_odom = (
                    None if odom is None else dict(odom)
                )
                self.execution_pose_discontinuity_since = None
                self.execution_pose_discontinuity_logged = False
                self.execution_pose_discontinuity_handled = False
                return True
            if odom is None:
                return False
            if (
                self.execution_pose_anchor_map is None
                or self.execution_pose_anchor_odom is None
            ):
                self.execution_pose_anchor_map = dict(candidate)
                self.execution_pose_anchor_odom = dict(odom)
                return True
            assessment = execution_pose_continuity(
                self.execution_pose_anchor_map,
                candidate,
                self.execution_pose_anchor_odom,
                odom,
                maximum_translation_residual=(
                    self.execution_pose_max_translation_residual
                ),
                maximum_yaw_residual=self.execution_pose_max_yaw_residual,
            )
            if assessment.consistent:
                recovered = self.execution_pose_discontinuity_since is not None
                self.execution_pose_anchor_map = dict(candidate)
                self.execution_pose_anchor_odom = dict(odom)
                self.execution_pose_discontinuity_since = None
                self.execution_pose_discontinuity_logged = False
            else:
                if self.execution_pose_discontinuity_since is None:
                    self.execution_pose_discontinuity_since = now
                if not self.execution_pose_discontinuity_logged:
                    observed = True
                    self.execution_pose_discontinuity_logged = True
                if (
                    not self.execution_pose_discontinuity_handled
                    and now - self.execution_pose_discontinuity_since
                    >= self.execution_pose_discontinuity_confirmation
                ):
                    self.execution_pose_discontinuity_handled = True
                    self.execution_pose_hold = True
                    confirmed = True

        if recovered:
            self._nav_debug(
                "EXECUTION_POSE_CONTINUITY",
                status="RECOVERED_TRANSIENT",
                accepted_pose=candidate,
            )
        if assessment is None or assessment.consistent:
            return True

        # Do this on the first bad sample, before waiting for confirmation.
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        if observed:
            self._nav_debug(
                "EXECUTION_POSE_CONTINUITY",
                status="REJECTED_TRANSIENT",
                trusted_pose=self.execution_pose_anchor_map,
                rejected_pose=candidate,
                translation_residual_m=assessment.translation_residual,
                yaw_residual_deg=math.degrees(assessment.yaw_residual),
                map_translation_m=assessment.map_translation,
                odom_translation_m=assessment.odom_translation,
            )
        if confirmed:
            self._pause_for_execution_pose_discontinuity(
                candidate, assessment
            )
        return False

    def _pause_for_execution_pose_discontinuity(
        self, rejected_pose: dict[str, float], assessment: Any
    ) -> None:
        with self.state_lock:
            if self.current_state != "NAVIGATING":
                return
            handle = self.current_goal_handle
            goal = dict(self.execution_goal or self.paused_goal or {})
            if goal:
                self.sensor_time_resume_context = {
                    "goal": goal,
                    "mission_id": self.current_mission_id,
                    "route_id": self.selected_route_id,
                    "path": list(self.latest_global_path),
                    "recovery_reason": "EXECUTION_POSE_DISCONTINUITY",
                }
            self.current_goal_handle = None
            self.navigation_goal_generation += 1
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_phase = "IDLE"
            self.motion_owner = "NONE"
            self.latest_feedback["recovery_reason"] = (
                "EXECUTION_POSE_DISCONTINUITY"
            )

        self.navigation_velocity.publish(Twist())
        self.localization_velocity.publish(Twist())
        self.profile_limiter.reset()
        if handle is not None:
            handle.cancel_goal_async()
        self._nav_debug(
            "EXECUTION_POSE_CONTINUITY",
            status="CONFIRMED_PAUSE",
            trusted_pose=self.execution_pose_anchor_map,
            rejected_pose=rejected_pose,
            translation_residual_m=assessment.translation_residual,
            yaw_residual_deg=math.degrees(assessment.yaw_residual),
            destination=goal,
            destination_preserved=bool(goal),
            display_pose_preserved=True,
        )
        # Auto navigation already authorizes bounded, safety-gated recovery.
        # Verify stationary first; rotate only if the existing localization
        # contract proves that multiple headings are necessary.
        self._begin_localization_verification(allow_rotation=True)

    def _release_execution_pose_hold(self) -> None:
        verified = self.last_amcl_pose
        with self.execution_pose_guard_lock:
            if not self.execution_pose_hold:
                return
            self.execution_pose_hold = False
            self.execution_pose_anchor_map = None
            self.execution_pose_anchor_odom = None
            self.execution_pose_discontinuity_since = None
            self.execution_pose_discontinuity_logged = False
            self.execution_pose_discontinuity_handled = False
        if verified is not None:
            self.pose = {
                "x": float(verified[0]),
                "y": float(verified[1]),
                "yaw": float(verified[2]),
            }
        self._nav_debug(
            "EXECUTION_POSE_CONTINUITY",
            status="VERIFIED_RELEASE",
            accepted_pose=self.pose,
        )

    def _fresh_execution_pose(self) -> dict[str, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
        except TransformException:
            return None
        now_clock = self.get_clock().now()
        stamp = transform.header.stamp
        stamp_nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age = (
            0.0
            if stamp_nanoseconds <= 0
            else (now_clock.nanoseconds - stamp_nanoseconds) / 1_000_000_000.0
        )
        if age > self.execution_pose_freshness:
            return None
        rotation = transform.transform.rotation
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": self._yaw_from_quaternion(rotation),
        }
        if not self._execution_pose_candidate_accepted(pose):
            return None
        self.pose = pose
        self.last_map_tf_monotonic = time.monotonic()
        self.execution_pose_monotonic = self.last_map_tf_monotonic
        return pose

    def _prepare_active_segment(
        self,
        goal_generation: int,
        pose: dict[str, float],
    ) -> None:
        if goal_generation != self.navigation_goal_generation:
            return
        if self._complete_final_position_if_reached(pose, goal_generation):
            return
        if self.execution_segment_index >= len(self.execution_points) - 1:
            if not self.execution_final_turn or self.execution_goal is None:
                return
            target = float(
                self.execution_goal.get("yaw", self.execution_target_heading)
            )
            self.execution_target_heading = target
            heading_error = self._yaw_delta(
                target, float(pose.get("yaw", target))
            )
            if abs(heading_error) > self.execution_turn_tolerance:
                self.execution_physical_final_turn = True
                self._nav_debug(
                    "EXECUTION_PHASE",
                    phase="FINAL_TURN_BEGIN",
                    heading_error=heading_error,
                    route_id=self.execution_route_id,
                )
            self._begin_turn_or_settling(pose, target, goal_generation)
            return
        planned_start = self.execution_points[self.execution_segment_index]
        endpoint = self.execution_points[self.execution_segment_index + 1]
        motion_direction = (
            self.execution_segment_directions[self.execution_segment_index]
            if self.execution_segment_index < len(self.execution_segment_directions)
            else 1
        )
        planned_progress = straight_segment_progress(
            planned_start,
            endpoint,
            pose,
            overshoot_epsilon=self.straight_overshoot_epsilon,
        )
        if planned_progress.endpoint_distance <= self.straight_endpoint_tolerance:
            self._complete_active_segment(
                "ALREADY_AT_ENDPOINT", goal_generation, cancel_action=False
            )
            return
        if planned_progress.passed_endpoint:
            self._schedule_execution_replan(
                "PLANNED_ENDPOINT_ALREADY_PASSED", goal_generation
            )
            return
        candidate = [
            {"x": float(pose["x"]), "y": float(pose["y"])},
            {"x": float(endpoint["x"]), "y": float(endpoint["y"])},
        ]
        start_escape = self.execution_relocation_reason == "START_ESCAPE"
        if start_escape and self.saved_map is not None:
            refreshed_escape = find_start_escape(
                self.saved_map,
                pose,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
                maximum_distance=self.start_escape_max_distance,
                directions=(motion_direction,),
            )
            if refreshed_escape is None:
                current_footprint = self.saved_map.validate_footprint(
                    float(pose["x"]),
                    float(pose["y"]),
                    float(pose.get("yaw", 0.0)),
                    half_length=self.footprint_half_length,
                    half_width=self.footprint_half_width,
                    padding=self.planning_footprint_padding,
                    allow_unknown=False,
                    code_prefix="START",
                )
                if current_footprint.valid:
                    self._nav_debug(
                        "START_ESCAPE",
                        status="ALREADY_CLEAR",
                        action="REPLAN_ORIGINAL_DESTINATION",
                    )
                    self._complete_active_segment(
                        "START_ESCAPE_ALREADY_CLEAR",
                        goal_generation,
                        cancel_action=False,
                    )
                    return
                self._enter_dynamic_wait("START_ESCAPE_LIVE_OR_STATIC_BLOCK")
                return
            candidate[1] = dict(refreshed_escape.end)
            self.execution_points[1] = dict(refreshed_escape.end)
            self.execution_segment_directions[0] = (
                refreshed_escape.motion_direction
            )
        try:
            live_validation = self._validate_executable_path(
                candidate,
                context="STRAIGHT_REANCHOR",
                allow_monotonic_initial_overlap=start_escape,
            )
        except AdapterError:
            self._schedule_execution_replan(
                "REANCHOR_COSTMAP_UNAVAILABLE", goal_generation
            )
            return
        static_validation = (
            None
            if self.saved_map is None or start_escape
            else validate_stop_turn_route(
                self.saved_map,
                candidate,
                half_length=self.footprint_half_length,
                half_width=(
                    self.footprint_half_width
                    + self.translation_lateral_margin
                ),
                padding=self.planning_footprint_padding,
                segment_directions=(motion_direction,),
            )
        )
        if (
            not live_validation.valid
            or (
                not start_escape
                and (static_validation is None or not static_validation.valid)
            )
        ):
            self._schedule_execution_replan(
                "REANCHOR_TRANSLATION_INVALID", goal_generation
            )
            return
        self.execution_segment_token += 1
        token = self.execution_segment_token
        active = ActiveSegment.create(
            planned_start=planned_start,
            effective_start=candidate[0],
            endpoint=endpoint,
            segment_index=self.execution_segment_index,
            route_id=self.execution_route_id,
            segment_token=token,
            narrow=self.execution_segment_index in self.execution_narrow_segments,
            motion_direction=motion_direction,
        )
        current_yaw = float(pose.get("yaw", active.fixed_heading))
        heading_delta = self._yaw_delta(active.fixed_heading, current_yaw)
        straight_entry_after_reanchor = bool(
            self.execution_segment_reanchors > 0
            and not post_turn_reanchor_requires_turn(
                heading_delta,
                planned_progress.signed_cross_track,
                straight_entry_heading_limit=self.straight_hard_heading_error,
                straight_entry_cross_track_limit=self.straight_hard_cross_track,
            )
        )
        if start_escape:
            if abs(self._yaw_delta(active.fixed_heading, current_yaw)) > self.execution_turn_tolerance:
                self.navigation_velocity.publish(Twist())
                self._enter_dynamic_wait("START_ESCAPE_HEADING_CHANGED")
                return
            self.active_segment = active
            self.execution_target_heading = active.fixed_heading
            self.execution_phase = "DISPATCHING_STRAIGHT"
            self.latest_feedback["execution_phase"] = "START_ESCAPE"
            self._nav_debug(
                "START_ESCAPE",
                status="VALIDATED",
                start=active.effective_start,
                end=active.endpoint,
                fixed_heading=active.fixed_heading,
            )
            self._dispatch_prepared_segment(goal_generation)
            return
        if not straight_entry_after_reanchor:
            start_turn = validate_rotation_sweep(
                self.saved_map,
                float(pose["x"]),
                float(pose["y"]),
                current_yaw,
                active.fixed_heading,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
            )
            if not start_turn.valid:
                self._schedule_execution_replan(
                    "REANCHOR_START_TURN_INVALID", goal_generation
                )
                return
        if self.execution_segment_index + 2 < len(self.execution_points):
            following = self.execution_points[self.execution_segment_index + 2]
            next_heading = math.atan2(
                float(following["y"]) - float(endpoint["y"]),
                float(following["x"]) - float(endpoint["x"]),
            )
            next_direction = (
                self.execution_segment_directions[
                    self.execution_segment_index + 1
                ]
                if self.execution_segment_index + 1
                < len(self.execution_segment_directions)
                else 1
            )
            if next_direction < 0:
                next_heading = self._yaw_delta(next_heading + math.pi, 0.0)
            endpoint_turn = validate_rotation_sweep(
                self.saved_map,
                float(endpoint["x"]),
                float(endpoint["y"]),
                active.fixed_heading,
                next_heading,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
            )
            if not endpoint_turn.valid:
                self._schedule_execution_replan(
                    "REANCHOR_ENDPOINT_TURN_INVALID", goal_generation
                )
                return
        self.active_segment = active
        self.execution_target_heading = active.fixed_heading
        self._nav_debug(
            "SEGMENT_REANCHOR",
            planned_start=active.planned_start,
            effective_start=active.effective_start,
            endpoint=active.endpoint,
            fixed_heading=active.fixed_heading,
            segment_length=active.segment_length,
            segment_index=active.segment_index,
            route_id=active.route_id,
            segment_token=active.segment_token,
            narrow=active.narrow,
            motion=("REVERSE" if active.motion_direction < 0 else "FORWARD"),
        )
        if straight_entry_after_reanchor:
            self.execution_reanchor_after_turn = False
            self._nav_debug(
                "EXECUTION_PHASE",
                phase="STRAIGHT_ENTRY_CORRECTION",
                segment_index=self.execution_segment_index,
                heading_error=heading_delta,
                cross_track=planned_progress.signed_cross_track,
                reason="POST_TURN_REANCHOR_WITHIN_CONTROLLER_BAND",
            )
            self._dispatch_prepared_segment(goal_generation)
            return
        self._begin_turn_or_settling(
            pose, active.fixed_heading, goal_generation
        )

    def _begin_turn_or_settling(
        self,
        pose: dict[str, float],
        target: float,
        goal_generation: int,
    ) -> None:
        if goal_generation != self.navigation_goal_generation:
            return
        current_yaw = float(pose.get("yaw", target))
        error = self._yaw_delta(target, current_yaw)
        now = time.monotonic()
        if abs(error) > self.execution_turn_tolerance:
            left_static = self._turn_static_safe(pose, target, 1)
            right_static = self._turn_static_safe(pose, target, -1)
            snapshot_fresh = self._atomic_safety_fresh(now)
            left_live = snapshot_fresh and not self._safety_blocks_turn(1)
            right_live = snapshot_fresh and not self._safety_blocks_turn(-1)
            direction = choose_turn_direction(
                error,
                left_static_safe=left_static,
                right_static_safe=right_static,
                left_live_safe=left_live,
                right_live_safe=right_live,
            )
            if not left_static and not right_static:
                self._schedule_execution_replan(
                    "START_TURN_BLOCKED_STATIC", goal_generation
                )
                return
            self.execution_turn_direction = (
                direction if direction else (1 if error > 0.0 else -1)
            )
            self.execution_phase = "TURN" if direction else "WAIT_FOR_TURN_CLEAR"
            self.execution_phase_started = now
            self.execution_turn_stable_since = None
            self.execution_turn_reentry_since = None
            self.execution_turn_blocked_since = None if direction else now
            self.turn_block_tracker = TurnBlockTracker(clear_dwell_seconds=0.30)
            self.execution_reanchor_after_turn = (
                self.execution_segment_reanchors
                < self.stop_turn_max_reanchors_per_segment
            )
            self.latest_feedback["execution_phase"] = self.execution_phase
            self.motion_owner = "NAVIGATION"
            if direction:
                self._nav_debug(
                    "EXECUTION_PHASE",
                    phase="TURN_BEGIN",
                    segment_index=self.execution_segment_index,
                    target_heading=target,
                    direction=self.execution_turn_direction,
                )
            else:
                self._nav_debug(
                    "EXECUTION_PHASE",
                    phase="WAIT_FOR_TURN_CLEAR",
                    segment_index=self.execution_segment_index,
                    target_heading=target,
                    direction=self.execution_turn_direction,
                )
            return
        self.execution_phase = "TURN_SETTLING"
        self.execution_phase_started = now
        self.execution_turn_stable_since = now
        self.execution_turn_reentry_since = None
        self.execution_reanchor_after_turn = False
        self.latest_feedback["execution_phase"] = "TURN_SETTLING"

    def _turn_static_safe(
        self, pose: dict[str, float], target: float, direction: int
    ) -> bool:
        return bool(
            self.saved_map is not None
            and validate_rotation_sweep(
                self.saved_map,
                float(pose["x"]),
                float(pose["y"]),
                float(pose.get("yaw", target)),
                target,
                half_length=self.footprint_half_length,
                half_width=self.footprint_half_width,
                padding=self.planning_footprint_padding,
                direction=direction,
            ).valid
        )

    def _dispatch_prepared_segment(self, goal_generation: int) -> None:
        active = self.active_segment
        if active is None or goal_generation != self.navigation_goal_generation:
            return
        self.execution_phase = "DISPATCHING_STRAIGHT"
        self.latest_feedback["execution_phase"] = "DISPATCHING_STRAIGHT"
        threading.Thread(
            target=self._send_current_straight_segment,
            args=(goal_generation, active.segment_token, active.segment_index),
            daemon=True,
        ).start()

    def _complete_active_segment(
        self,
        reason: str,
        goal_generation: int,
        *,
        cancel_action: bool,
    ) -> None:
        if goal_generation != self.navigation_goal_generation:
            return
        active = self.active_segment
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        self.execution_segment_token += 1
        completed_index = self.execution_segment_index
        self.active_segment = None
        self.execution_segment_index += 1
        self.execution_segment_reanchors = 0
        self.execution_final_turn = bool(
            self.execution_segment_index >= len(self.execution_points) - 1
            and self.execution_goal is not None
            and self.stop_turn_require_final_yaw
            and self.execution_goal.get("yaw") is not None
        )
        self.execution_physical_final_turn = False
        self.execution_phase = "STRAIGHT_PREPARE"
        self.execution_phase_started = time.monotonic()
        self.latest_feedback["execution_phase"] = "STRAIGHT_PREPARE"
        self.segment_started_monotonic = 0.0
        self.segment_positive_travel = 0.0
        self.segment_last_travel_pose = None
        self.profile_limiter.reset()
        self._nav_debug(
            "EXECUTION_PHASE",
            phase="STRAIGHT_END",
            segment_index=completed_index,
            segment_token=(None if active is None else active.segment_token),
            reason=reason,
        )
        if cancel_action and handle is not None:
            handle.cancel_goal_async()
        relocation_reason = self.execution_relocation_reason
        if (
            relocation_reason
            and self.execution_segment_index >= len(self.execution_points) - 1
        ):
            self.execution_relocation_reason = ""
            self.execution_relocation_plan = []
            self.execution_phase = "RECOVERING"
            self.latest_feedback["execution_phase"] = "DYNAMIC_REPLAN"
            self._nav_debug(
                relocation_reason,
                status="COMPLETE",
                action="REPLAN_ORIGINAL_DESTINATION",
            )
            threading.Thread(
                target=self._replan_execution_from_current,
                args=(f"{relocation_reason}_COMPLETE", goal_generation),
                daemon=True,
            ).start()
            return
        if (
            self.execution_segment_index >= len(self.execution_points) - 1
            and not self.execution_final_turn
        ):
            self._finish_execution_success(0.0)

    def _finish_execution_success(
        self,
        heading_error: float,
        *,
        position_distance: float | None = None,
        cancel_active: bool = False,
    ) -> None:
        # Capture transaction fields before clearing them. The completion log
        # runs after cleanup; reading the cleared route/goal there previously
        # raised NameError and killed the adapter immediately after SUCCEEDED.
        completed_route_id = self.execution_route_id
        completed_goal = (
            None if self.execution_goal is None else dict(self.execution_goal)
        )
        physical_final_turn = bool(
            self.stop_turn_require_final_yaw
            and self.execution_final_turn
            and self.execution_physical_final_turn
        )
        if position_distance is None and self.pose is not None and self.execution_goal:
            position_distance = math.hypot(
                float(self.pose["x"]) - float(self.execution_goal["x"]),
                float(self.pose["y"]) - float(self.execution_goal["y"]),
            )
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.execution_segment_token += 1
        self.execution_final_turn = False
        self.execution_physical_final_turn = False
        self.execution_phase = "IDLE"
        self.active_segment = None
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        if cancel_active and handle is not None:
            handle.cancel_goal_async()
        self._set_state("SUCCEEDED", "goal_reached_by_stop_turn_segments")
        self.paused_goal = None
        self.execution_goal = None
        self.execution_route_id = ""
        self.execution_points = []
        self.execution_segment_directions = []
        self.execution_segment_index = 0
        self.execution_narrow_segments = set()
        self.route_candidates = {}
        self.selected_route_id = ""
        self.current_mission_id = ""
        self.route_selection_return_state = "READY"
        self.latest_global_path = []
        self.visualization_revision += 1
        self._clear_active_navigation_mission("goal_succeeded")
        if physical_final_turn:
            self._nav_debug(
                "EXECUTION_PHASE",
                phase="FINAL_TURN_END",
                heading_error=heading_error,
                route_id=completed_route_id,
            )
        elif self.stop_turn_require_final_yaw and completed_goal is not None:
            self._nav_debug(
                "GOAL_REACHED",
                mode="POSITION_AND_YAW",
                distance=position_distance,
                route_id=completed_route_id,
            )
        else:
            self._nav_debug(
                "GOAL_REACHED",
                mode="POSITION_ONLY",
                distance=position_distance,
                route_id=completed_route_id,
            )

    def _restart_segment_from_current(
        self,
        reason: str,
        goal_generation: int,
    ) -> None:
        if goal_generation != self.navigation_goal_generation:
            return
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.execution_segment_token += 1
        self.active_segment = None
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        self.profile_limiter.reset()
        self.execution_phase = "STRAIGHT_PREPARE"
        self.execution_phase_started = time.monotonic()
        self.latest_feedback["execution_phase"] = "STRAIGHT_PREPARE"
        self._nav_debug(
            "SEGMENT_RECOVERY",
            reason=reason,
            action="REANCHOR_REALIGN",
            segment_index=self.execution_segment_index,
        )
        if handle is not None:
            handle.cancel_goal_async()

    def _atomic_dynamic_blockage(self, now: float | None = None) -> bool:
        return bool(
            self._atomic_safety_fresh(now)
            and self.safety_snapshot.get("stop")
            and self.safety_snapshot.get("source") == "MOTION_SAFETY"
            and str(self.safety_snapshot.get("reason") or "").upper()
            in {
                "FRONT_SWEEP_COLLISION",
                "REAR_SWEEP_COLLISION",
                "LEFT_TURN_CLEARANCE",
                "RIGHT_TURN_CLEARANCE",
                "ROTATION_SWEEP_COLLISION",
            }
        )

    def _remaining_execution_route(self) -> list[dict[str, float]]:
        if self.pose is None or not self.execution_points:
            return []
        remaining = [{
            "x": float(self.pose["x"]),
            "y": float(self.pose["y"]),
        }]
        remaining.extend(
            {
                "x": float(point["x"]),
                "y": float(point["y"]),
            }
            for point in self.execution_points[
                min(self.execution_segment_index + 1, len(self.execution_points)):
            ]
        )
        return canonicalize_stop_turn_path(remaining)

    def _corridor_sample_fresh_for_path(self, now: float) -> bool:
        if not self.corridor_samples:
            return False
        timestamp, _, path_error = self.corridor_samples[-1]
        return bool(
            now - float(timestamp) <= 0.60
            and abs(float(path_error)) <= math.radians(20.0)
        )

    def _fresh_corridor_blockage(self, now: float) -> tuple[bool, float]:
        if not self.corridor_samples:
            return False, math.inf
        _, corridor, _ = self.corridor_samples[-1]
        fresh = self._corridor_sample_fresh_for_path(now)
        front_clearance = float(corridor.front_clearance)
        blockage_limit = (
            self.corridor_front_clearance + self.straight_endpoint_tolerance
        )
        return bool(
            fresh
            and (
                corridor.classification == "PHYSICALLY_BLOCKED"
                or not bool(corridor.physically_passable)
                or (
                    math.isfinite(front_clearance)
                    and front_clearance <= blockage_limit
                )
            )
        ), front_clearance

    def _record_controller_abort(self, now: float, zero_linear: bool) -> bool:
        pose = dict(self.pose or {})
        goal = dict(self.execution_goal or self.paused_goal or {})
        repeated = bool(
            zero_linear
            and "x" in pose
            and "y" in pose
            and any(
                now - float(item["timestamp"]) <= 10.0
                and bool(item["zero_linear"])
                and math.hypot(
                    float(item["x"]) - float(pose["x"]),
                    float(item["y"]) - float(pose["y"]),
                ) <= 0.25
                and math.hypot(
                    float(item["goal_x"]) - float(goal.get("x", math.inf)),
                    float(item["goal_y"]) - float(goal.get("y", math.inf)),
                ) <= self.stop_turn_final_position_tolerance
                for item in self.controller_abort_history
            )
        )
        if "x" in pose and "y" in pose and "x" in goal and "y" in goal:
            self.controller_abort_history.append({
                "timestamp": now,
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "goal_x": float(goal["x"]),
                "goal_y": float(goal["y"]),
                "zero_linear": zero_linear,
            })
        return repeated

    def _controller_abort_live_blockage(
        self,
        error_code: Any,
        error_msg: str,
    ) -> tuple[bool, dict[str, Any]]:
        now = time.monotonic()
        requested = self.last_controller_requested
        zero_linear = bool(
            requested is not None
            and now - requested[2] <= 0.75
            and abs(requested[0]) < 0.02
        )
        repeated = self._record_controller_abort(now, zero_linear)
        corridor_blocked, front_clearance = self._fresh_corridor_blockage(now)
        corridor_fresh = bool(
            self.corridor_samples
            and now - float(self.corridor_samples[-1][0]) <= 0.60
        )
        atomic_block = self._atomic_dynamic_blockage(now)
        dynamic_route_intersection = self._dynamic_affects_remaining_route()
        live = controller_abort_is_live_blockage(
            error_code=error_code,
            error_msg=error_msg,
            atomic_motion_safety_block=atomic_block,
            dynamic_route_intersection=dynamic_route_intersection,
            controller_zero_linear=zero_linear,
            repeated_zero_linear_abort=repeated,
            corridor_sample_fresh=corridor_fresh,
            corridor_front_clearance=front_clearance,
            corridor_blockage_limit=(
                self.corridor_front_clearance + self.straight_endpoint_tolerance
            ),
        )
        if live:
            self.last_controller_blockage_monotonic = now
        return live, {
            "atomic_motion_safety_block": atomic_block,
            "dynamic_route_intersection": dynamic_route_intersection,
            "controller_zero_linear": zero_linear,
            "repeated_zero_linear_abort": repeated,
            "corridor_sample_fresh": corridor_fresh,
            "corridor_front_clearance": front_clearance,
            "fresh_corridor_blockage": corridor_blocked,
        }

    def _runtime_live_blockage_reason(self) -> str:
        now = time.monotonic()
        if self._atomic_dynamic_blockage(now):
            return "MOTION_SAFETY_DYNAMIC_BLOCK"
        if self._dynamic_affects_remaining_route():
            return "DYNAMIC_ROUTE_INTERSECTION"
        if (
            self.last_controller_blockage_monotonic > 0.0
            and now - self.last_controller_blockage_monotonic
            <= max(5.0, self.dynamic_obstacle_wait + self.dynamic_replan_retry)
        ):
            return "CONTROLLER_ABORT_LIVE_BLOCKAGE"
        return ""

    def _observe_controller_blocker(self, *, force: bool = False) -> bool:
        """Latch only a blocker whose map position came from repeated sensor data.

        A controller stop without a located obstacle remains valid atomic safety
        evidence, but it is not enough to invent a map keepout in front of the
        chassis. ``force`` is retained for command compatibility and deliberately
        does not weaken the observation requirement.
        """
        now = time.monotonic()
        if self.dynamic_blocked_keepout is not None:
            # A short occlusion or an expiring overlay track must not erase a
            # blocker whose position was already confirmed.  Keep using the
            # latched map position until a replacement route is accepted or
            # the recovery state is explicitly reset.
            self._nav_debug(
                "DYNAMIC_BLOCKER",
                source="LATCHED_MULTI_OBSERVATION_SENSOR",
                result="KEEP_OUT_REUSED",
                blocker_id=self.dynamic_blocker_id or None,
                center=(
                    round(self.dynamic_blocked_keepout[0], 3),
                    round(self.dynamic_blocked_keepout[1], 3),
                ),
                radius=round(self.dynamic_blocked_keepout[2], 3),
                destination_preserved=bool(
                    self.execution_goal or self.paused_goal
                ),
            )
            return True
        corridor_blocked, front_clearance = self._fresh_corridor_blockage(now)
        candidates = self._dynamic_route_obstacles(
            minimum_observations=self.dynamic_planning_minimum_observations
        )
        if self.pose is None or not candidates:
            projected_keepout = self._live_front_keepout_for_route(
                self.dynamic_blocked_route,
                self.dynamic_blocked_segment_directions,
                blocked_only=True,
            )
            if projected_keepout is not None:
                self.dynamic_blocked_keepout = projected_keepout
                self.dynamic_blocker_id = "corridor-front-projection"
                self._nav_debug(
                    "DYNAMIC_BLOCKER",
                    source="ROUTE_ALIGNED_FRONT_LIDAR",
                    result="KEEP_OUT_CONFIRMED",
                    blocker_id=self.dynamic_blocker_id,
                    center=(
                        round(projected_keepout[0], 3),
                        round(projected_keepout[1], 3),
                    ),
                    radius=round(projected_keepout[2], 3),
                    front_clearance=self._finite_metric(front_clearance),
                    destination_preserved=bool(
                        self.execution_goal or self.paused_goal
                    ),
                )
                return True
            if (
                force
                or now - self.dynamic_last_unconfirmed_log_monotonic
                >= self.dynamic_unconfirmed_blocker_log_interval
            ):
                self.dynamic_last_unconfirmed_log_monotonic = now
                self._nav_debug(
                    "DYNAMIC_BLOCKER",
                    source="CONTROLLER_CORRIDOR",
                    result="POSITION_UNCONFIRMED",
                    corridor_blocked=corridor_blocked,
                    front_clearance=self._finite_metric(front_clearance),
                    forced=bool(force),
                    keepout_created=False,
                    destination_preserved=bool(
                        self.execution_goal or self.paused_goal
                    ),
                )
            return False
        obstacle = min(
            candidates,
            key=lambda item: math.hypot(
                item.center_x - float(self.pose["x"]),
                item.center_y - float(self.pose["y"]),
            ),
        )
        footprint_inflation = (
            self.footprint_half_width
            + self.planning_footprint_padding
            + self.corridor_hard_side_margin
        )
        self.dynamic_blocked_keepout = (
            obstacle.center_x,
            obstacle.center_y,
            max(
                obstacle.radius + footprint_inflation,
                self.alternative_route_keepout_radius,
            ),
        )
        self.dynamic_blocker_id = f"dynamic-{obstacle.id}"
        self._nav_debug(
            "DYNAMIC_BLOCKER",
            source="MULTI_OBSERVATION_SENSOR",
            result="KEEP_OUT_CONFIRMED",
            blocker_id=self.dynamic_blocker_id,
            center=(round(obstacle.center_x, 3), round(obstacle.center_y, 3)),
            radius=round(self.dynamic_blocked_keepout[2], 3),
            observation_count=obstacle.observation_count,
            motion_state=obstacle.motion_state,
            front_clearance=self._finite_metric(front_clearance),
            destination_preserved=bool(self.execution_goal or self.paused_goal),
        )
        return True

    def _stop_unconfirmed_dynamic_recovery(self) -> None:
        """End an evidence-deadlocked wait without inventing a map obstacle."""
        goal = dict(self.execution_goal or self.paused_goal or {})
        if goal:
            self.paused_goal = goal
        self.dynamic_recovery_state = "BLOCKED"
        self.dynamic_replan_requires_alternative = False
        self.execution_phase = "IDLE"
        self.latest_feedback["execution_phase"] = "IDLE"
        self.latest_feedback["recovery_reason"] = (
            "BLOCKER_POSITION_UNCONFIRMED"
        )
        self.latest_feedback["terminal_reason"] = (
            "BLOCKER_POSITION_UNCONFIRMED"
        )
        self.latest_feedback["destination_preserved"] = bool(goal)
        self.motion_owner = "NONE"
        self._set_state("BLOCKED", "blocker_position_unconfirmed")
        self.navigation_velocity.publish(Twist())
        self._nav_debug(
            "DYNAMIC_OBSTACLE",
            action="BLOCKED_UNCONFIRMED",
            reason="BLOCKER_POSITION_UNCONFIRMED",
            wait_seconds=(
                None
                if self.dynamic_wait_started is None
                else round(time.monotonic() - self.dynamic_wait_started, 3)
            ),
            destination=goal or None,
            destination_preserved=bool(goal),
            keepout_created=False,
        )
        if goal:
            self._persist_active_navigation_mission(
                resume_automatically=False,
                goal=goal,
                path=self.dynamic_blocked_route or self.latest_global_path,
                segment_directions=self.dynamic_blocked_segment_directions,
            )

    def _stop_persistent_moving_dynamic_recovery(self) -> None:
        """Bound a moving-obstacle wait without discarding its destination."""
        goal = dict(self.execution_goal or self.paused_goal or {})
        if goal:
            self.paused_goal = goal
        waited = (
            None
            if self.dynamic_wait_started is None
            else round(time.monotonic() - self.dynamic_wait_started, 3)
        )
        self.dynamic_recovery_state = "BLOCKED"
        self.dynamic_replan_requires_alternative = False
        self.execution_phase = "IDLE"
        self.latest_feedback["execution_phase"] = "IDLE"
        self.latest_feedback["recovery_reason"] = (
            "MOVING_OBSTACLE_WAIT_TIMEOUT"
        )
        self.latest_feedback["terminal_reason"] = (
            "MOVING_OBSTACLE_WAIT_TIMEOUT"
        )
        self.latest_feedback["destination_preserved"] = bool(goal)
        self.motion_owner = "NONE"
        self._set_state("BLOCKED", "moving_obstacle_wait_timeout")
        self.navigation_velocity.publish(Twist())
        self._nav_debug(
            "DYNAMIC_OBSTACLE",
            action="BLOCKED_PERSISTENT_MOVING",
            reason="MOVING_OBSTACLE_WAIT_TIMEOUT",
            wait_seconds=waited,
            destination=goal or None,
            destination_preserved=bool(goal),
        )
        if goal:
            self._persist_active_navigation_mission(
                resume_automatically=False,
                goal=goal,
                path=self.dynamic_blocked_route or self.latest_global_path,
                segment_directions=self.dynamic_blocked_segment_directions,
            )

    def _schedule_execution_replan(
        self,
        reason: str,
        goal_generation: int,
    ) -> None:
        if (
            goal_generation != self.navigation_goal_generation
            or self.execution_phase == "RECOVERING"
        ):
            return
        live_blockage_reason = self._runtime_live_blockage_reason()
        if not live_blockage_reason and reason == "CONTROLLER_ABORT:UNCONFIRMED":
            live_blockage_reason = "CONTROLLER_ABORT_UNCONFIRMED"
        if live_blockage_reason:
            self._observe_controller_blocker()
            self._enter_dynamic_wait(live_blockage_reason)
            return
        if self.execution_replan_attempts >= self.failed_segment_max_replans:
            # Exhausting attempts proves neither a static disconnection nor a
            # hard fault. Preserve the mission and continue bounded periodic
            # replans after the dynamic recovery cooldown.
            self._enter_dynamic_wait("REPLAN_BUDGET_COOLDOWN")
            return
        self.execution_replan_attempts += 1
        self.latest_feedback["recoveries"] = (
            self.navigation_recovery_attempts + self.execution_replan_attempts
        )
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.execution_segment_token += 1
        self.active_segment = None
        self.execution_phase = "RECOVERING"
        self.latest_feedback["execution_phase"] = "RECOVERING"
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        self.profile_limiter.reset()
        if handle is not None:
            handle.cancel_goal_async()
        threading.Thread(
            target=self._replan_execution_from_current,
            args=(reason, goal_generation),
            daemon=True,
        ).start()

    def _enter_dynamic_wait(self, reason: str) -> None:
        if self.current_state in {
            "WAIT_FOR_DYNAMIC_CLEAR", "WAITING_FOR_DYNAMIC_CLEAR"
        }:
            return
        now = time.monotonic()
        new_physical_encounter = bool(
            self.dynamic_recovery_state == "IDLE"
            or now >= self.dynamic_recovery_expires_monotonic
        )
        blocked_route = self._remaining_execution_route()
        blocked_directions = list(
            self.execution_segment_directions[self.execution_segment_index:]
        )
        if len(blocked_directions) != max(0, len(blocked_route) - 1):
            blocked_directions = [1 for _ in range(max(0, len(blocked_route) - 1))]
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.execution_segment_token += 1
        self.active_segment = None
        self.execution_phase = "WAIT_FOR_DYNAMIC_CLEAR"
        self.latest_feedback["execution_phase"] = "WAIT_FOR_DYNAMIC_CLEAR"
        self.latest_feedback["recovery_reason"] = reason
        self.latest_feedback["destination_preserved"] = True
        self.motion_owner = "NONE"
        self.dynamic_block_reason = reason
        if new_physical_encounter:
            self.dynamic_failed_route_signatures = {}
            self.dynamic_replan_attempt_count = 0
            self.dynamic_blocked_keepout = None
            self.dynamic_blocker_id = ""
        if len(blocked_route) >= 2:
            self.dynamic_blocked_route = blocked_route
            self.dynamic_blocked_segment_directions = blocked_directions
            self.dynamic_blocked_route_signature = self._route_signature(blocked_route)
        if new_physical_encounter and dynamic_block_requires_alternative(reason):
            # This may deliberately return false. A controller abort has STOP
            # authority, but only repeated located sensor observations may
            # create the temporary planning keepout.
            self._observe_controller_blocker(force=True)
        if new_physical_encounter or self.dynamic_wait_started is None:
            self.dynamic_wait_started = now
        self.dynamic_recovery_expires_monotonic = now + max(
            self.dynamic_overlay_ttl,
            self.dynamic_obstacle_wait + 3.0 * self.dynamic_replan_retry,
        )
        self.dynamic_clear_started = None
        self.dynamic_recovery_state = "WAITING"
        self.dynamic_replan_requires_alternative = False
        self._set_state("WAIT_FOR_DYNAMIC_CLEAR", reason.lower())
        self.navigation_velocity.publish(Twist())
        if handle is not None:
            handle.cancel_goal_async()
        self._nav_debug(
            "DYNAMIC_OBSTACLE",
            action="WAIT_FOR_DYNAMIC_CLEAR",
            reason=reason,
            destination=self.execution_goal or self.paused_goal,
            route_id=self.execution_route_id,
            blocked_route_signature=self.dynamic_blocked_route_signature,
            blocker_id=self.dynamic_blocker_id or None,
            recovery_state=self.dynamic_recovery_state,
        )

    def _dynamic_recovery_tick(self) -> None:
        now = time.monotonic()
        self._refresh_dynamic_obstacle_view()
        if (
            self.current_state == "NAVIGATING"
            and self.execution_phase in {"STRAIGHT", "NARROW_STRAIGHT"}
            and self.active_segment is not None
            and self.current_goal_handle is not None
            and self._dynamic_tracking_evidence_trusted(now)
        ):
            profile_speed = self.speed_profiles.get(
                "SLOW" if self.execution_phase == "NARROW_STRAIGHT"
                else self.auto_speed_mode
            ).linear_max
            corridor_blocked, _ = self._fresh_corridor_blockage(now)
            immediate_physical_block = bool(
                self._atomic_dynamic_blockage(now) or corridor_blocked
            )
            predicted = tuple(
                (item, ttc)
                for item in self._dynamic_route_obstacles(
                    minimum_observations=(
                        self.dynamic_planning_minimum_observations
                    )
                )
                if item.motion_state == "MOVING"
                and (
                    ttc := dynamic_trajectory_conflict_ttc(
                        self._remaining_execution_route(),
                        item,
                        robot_speed=profile_speed,
                        footprint_inflation=(
                            self.footprint_half_width
                            + self.planning_footprint_padding
                            + self.corridor_hard_side_margin
                        ),
                        ttc_horizon=self.dynamic_conflict_ttc_horizon,
                    )
                ) is not None
                and (ttc > 0.0 or immediate_physical_block)
            )
            if predicted:
                # Only a confirmed moving track with a bounded swept-route TTC
                # has mission-level proactive stop authority. Static tracks and
                # nearby-only objects remain under controller + safety braking.
                self._nav_debug(
                    "DYNAMIC_OBSTACLE",
                    action="PROACTIVE_TRAJECTORY_CONFLICT",
                    obstacle_ids=[item.id for item, _ in predicted],
                    motion_states=[item.motion_state for item, _ in predicted],
                    ttc_seconds=[ttc for _, ttc in predicted],
                    destination=self.execution_goal or self.paused_goal,
                )
                self._enter_dynamic_wait("DYNAMIC_TRAJECTORY_CONFLICT")
                return
        awaiting_user_route = bool(
            self.current_state == "ROUTE_SELECTION"
            and self.dynamic_recovery_state
            == "AWAITING_USER_ROUTE_CONFIRMATION"
        )
        waiting_old_route_only = bool(
            self.current_state == "WAITING_FOR_DYNAMIC_CLEAR"
            and self.dynamic_recovery_state == "WAITING_OLD_ROUTE_ONLY"
        )
        if self.current_state not in {
            "WAIT_FOR_DYNAMIC_CLEAR",
            "WAITING_FOR_DYNAMIC_CLEAR",
        } and not awaiting_user_route:
            return
        self.navigation_velocity.publish(Twist())
        self.dynamic_recovery_state = "CLASSIFYING"
        fresh = self._atomic_safety_fresh(now)
        corridor_blocked, _ = self._fresh_corridor_blockage(now)
        corridor_evidence_fresh = self._corridor_sample_fresh_for_path(now)
        route_obstacles = self._dynamic_route_obstacles(
            minimum_observations=self.dynamic_planning_minimum_observations
        )
        stationary_route_blocked = any(
            item.motion_state == "STATIONARY" for item in route_obstacles
        )
        moving_route_blocked = any(
            item.motion_state == "MOVING" for item in route_obstacles
        )
        # A transient planner error must not erase fresh physical evidence.
        # Keep the current route blocked while its forward corridor remains
        # obstructed, regardless of the latest retry's error-code spelling.
        controller_corridor_blocked = bool(
            corridor_blocked
            and (self.execution_goal is not None or self.paused_goal is not None)
            and len(self.dynamic_blocked_route) >= 2
        )
        dynamic_blocked = bool(
            self._atomic_dynamic_blockage(now)
            or self._dynamic_affects_remaining_route()
            or controller_corridor_blocked
        )
        blocked_route_needs_corridor = len(self.dynamic_blocked_route) >= 2
        if (
            dynamic_blocked
            or not fresh
            or (blocked_route_needs_corridor and not corridor_evidence_fresh)
        ):
            self.dynamic_clear_started = None
        elif self.dynamic_clear_started is None:
            self.dynamic_clear_started = now
        clear_dwelled = bool(
            self.dynamic_clear_started is not None
            and now - self.dynamic_clear_started >= self.dynamic_clear_dwell
        )
        if awaiting_user_route and not clear_dwelled:
            # Alternatives remain a proposal only. Keep monitoring the old
            # route so removing the obstacle resumes it automatically, but do
            # not keep recomputing or silently start a proposed global route.
            self.dynamic_recovery_state = (
                "AWAITING_USER_ROUTE_CONFIRMATION"
            )
            return
        if waiting_old_route_only and not clear_dwelled:
            self.dynamic_recovery_state = "WAITING_OLD_ROUTE_ONLY"
            return
        persistent = bool(
            self.dynamic_wait_started is not None
            and now - self.dynamic_wait_started >= self.dynamic_obstacle_wait
        )
        replan_around = bool(
            persistent
            and (stationary_route_blocked or controller_corridor_blocked)
        )
        moving_wait_expired = bool(
            self.dynamic_wait_started is not None
            and now - self.dynamic_wait_started
            >= self.dynamic_moving_obstacle_max_wait
        )
        if (
            moving_route_blocked
            and not stationary_route_blocked
            and not clear_dwelled
            and moving_wait_expired
        ):
            # A confirmed person may legitimately remain on the route, but a
            # costmap track must never leave the transaction in an invisible
            # WAIT state forever. Stop explicitly, preserve the destination,
            # and let retry/resume collect a completely new evidence window.
            self._stop_persistent_moving_dynamic_recovery()
            return
        if (
            moving_route_blocked
            and not stationary_route_blocked
            and not clear_dwelled
        ):
            # A person/trolley that is still moving gets right of way. Keep
            # the original mission instead of oscillating between left/right
            # replans as its costmap cells move.
            self._set_state(
                "WAITING_FOR_DYNAMIC_CLEAR", "moving_obstacle_has_priority"
            )
            self.latest_feedback["execution_phase"] = (
                "WAITING_FOR_DYNAMIC_CLEAR"
            )
            self.latest_feedback["recovery_reason"] = (
                "MOVING_OBSTACLE_HAS_PRIORITY"
            )
            self.dynamic_recovery_state = "WAITING"
            return
        retry_due = now - self.dynamic_last_replan >= self.dynamic_replan_retry
        if not retry_due or (not clear_dwelled and not replan_around):
            self.dynamic_recovery_state = "WAITING"
            return
        requires_alternative = bool(not clear_dwelled and replan_around)
        if requires_alternative and not self._observe_controller_blocker():
            # Continue collecting evidence. Never run an alternative search
            # against a fabricated keepout near the robot. The wait is bounded:
            # if no sensor can locate the blocker, stop in an explicit BLOCKED
            # state and preserve the destination for operator retry/resume.
            self.dynamic_recovery_state = "WAITING"
            self.latest_feedback["recovery_reason"] = "BLOCKER_POSITION_UNCONFIRMED"
            if (
                self.dynamic_wait_started is not None
                and now - self.dynamic_wait_started
                >= self.dynamic_unconfirmed_blocker_timeout
            ):
                self._stop_unconfirmed_dynamic_recovery()
            return
        self.dynamic_recovery_state = "REPLAN_PENDING"
        self.dynamic_replan_requires_alternative = requires_alternative
        self.dynamic_last_replan = now
        self.dynamic_replan_attempt_count += 1
        self.dynamic_recovery_state = "REPLAN_RUNNING"
        self._set_state("DYNAMIC_REPLAN", "dynamic_replan_due")
        self.latest_feedback["execution_phase"] = "DYNAMIC_REPLAN"
        generation = self.navigation_goal_generation
        threading.Thread(
            target=self._attempt_dynamic_replan,
            args=(generation, clear_dwelled),
            daemon=True,
        ).start()

    def _dynamic_route_relation(
        self, points: list[dict[str, float]]
    ) -> tuple[bool, float, float]:
        """Classify a safe route as local bypass versus global replacement."""
        if len(points) < 2 or len(self.dynamic_blocked_route) < 2:
            return False, 0.0, math.inf
        overlap = path_overlap_ratio(self.dynamic_blocked_route, points)
        maximum_deviation = path_maximum_deviation(
            points,
            self.dynamic_blocked_route,
        )
        local_bypass = bool(
            overlap >= self.dynamic_local_bypass_minimum_overlap
            and maximum_deviation
            <= self.dynamic_local_bypass_maximum_deviation
        )
        return local_bypass, overlap, maximum_deviation

    def _present_dynamic_route_selection(
        self,
        candidates: list[dict[str, Any]],
        goal: dict[str, Any],
    ) -> None:
        """Stop and expose global alternatives; never execute one implicitly."""
        if not candidates:
            return
        for index, candidate in enumerate(candidates):
            candidate["recommended"] = index == 0
            candidate["requires_user_confirmation"] = True
            candidate["recovery_route_kind"] = "GLOBAL_ALTERNATIVE"
        selected = candidates[0]
        with self.state_lock:
            self.route_candidates = {
                str(candidate["route_id"]): candidate
                for candidate in candidates
            }
            self.selected_route_id = str(selected["route_id"])
            self.latest_global_path = list(selected["points"])
            self.route_selection_return_state = "WAITING_FOR_DYNAMIC_CLEAR"
            self.execution_phase = "IDLE"
            self.latest_feedback["execution_phase"] = "ROUTE_SELECTION"
            self.latest_feedback["recovery_reason"] = (
                "USER_ROUTE_CONFIRMATION_REQUIRED"
            )
            self.latest_feedback["destination_preserved"] = True
            self.dynamic_recovery_state = (
                "AWAITING_USER_ROUTE_CONFIRMATION"
            )
            self.dynamic_replan_requires_alternative = False
            self.motion_owner = "NONE"
            self._set_state(
                "ROUTE_SELECTION",
                "dynamic_alternative_confirmation_required",
            )
            self.visualization_revision += 1
        self.navigation_velocity.publish(Twist())
        self._persist_active_navigation_mission(
            resume_automatically=False,
            goal=goal,
            route_id=self.execution_route_id,
            path=self.dynamic_blocked_route,
            segment_directions=self.dynamic_blocked_segment_directions,
        )
        self._nav_debug(
            "DYNAMIC_ROUTE_SELECTION",
            action="USER_CONFIRMATION_REQUIRED",
            candidate_count=len(candidates),
            candidate_route_ids=[item["route_id"] for item in candidates],
            blocked_route_signature=self.dynamic_blocked_route_signature,
            destination=goal,
            destination_preserved=True,
            autonomous_global_route_started=False,
        )

    def _attempt_dynamic_local_bypass_or_selection(
        self,
        expected_generation: int,
        goal: dict[str, Any],
    ) -> None:
        """Prefer a bounded old-corridor bypass, otherwise ask the operator."""
        try:
            if self.stop_turn_planner is None or self.pose is None:
                raise AdapterError(
                    "PLANNER_NOT_READY",
                    "Stop-turn planner is unavailable for dynamic recovery",
                )
            recovery_planner = self._stop_turn_planner_for_clearance()
            planned = recovery_planner.plan_candidates(
                dict(self.pose),
                goal,
                maximum_candidates=self.alternative_route_max_candidates,
                overlap_threshold=self.alternative_route_overlap_threshold,
                planning_time_budget=self.stop_turn_live_obstacle_planning_budget,
                exclusions=self._dynamic_planning_exclusions(),
            )
            candidates = self._serialize_stop_turn_candidates(planned)
        except AdapterError as exc:
            if expected_generation != self.navigation_goal_generation:
                return
            self.dynamic_recovery_state = "WAITING"
            self._set_state(
                "WAITING_FOR_DYNAMIC_CLEAR",
                "dynamic_bypass_search_unavailable",
            )
            self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
            self.latest_feedback["execution_phase"] = (
                "WAITING_FOR_DYNAMIC_CLEAR"
            )
            self.latest_feedback["recovery_reason"] = exc.code
            self._nav_debug(
                "DYNAMIC_LOCAL_BYPASS",
                result="WAIT",
                error=exc.code,
                destination=goal,
                destination_preserved=True,
            )
            return
        if expected_generation != self.navigation_goal_generation:
            return

        now = time.monotonic()
        self.dynamic_failed_route_signatures = {
            signature: expires
            for signature, expires in self.dynamic_failed_route_signatures.items()
            if expires > now
        }
        local_candidates: list[dict[str, Any]] = []
        global_candidates: list[dict[str, Any]] = []
        hard_clearance = self._hard_route_side_clearance()
        blocker = (
            ()
            if self.dynamic_blocked_keepout is None
            else (self.dynamic_blocked_keepout,)
        )
        for candidate in candidates:
            points = [dict(point) for point in candidate.get("points") or []]
            signature = self._route_signature(points)
            length = sum(
                math.hypot(
                    float(right["x"]) - float(left["x"]),
                    float(right["y"]) - float(left["y"]),
                )
                for left, right in zip(points, points[1:])
            )
            still_hits_blocker = bool(
                blocker
                and dynamic_exclusions_intersect_route(
                    points,
                    blocker,
                    horizon=length + 0.01,
                )
            )
            minimum_clearance = candidate.get("minimum_side_clearance")
            rejected = bool(
                len(points) < 2
                or still_hits_blocker
                or signature == self.dynamic_blocked_route_signature
                or signature in self.dynamic_failed_route_signatures
                or (
                    minimum_clearance is not None
                    and float(minimum_clearance) + 1e-9 < hard_clearance
                )
            )
            if rejected:
                self.dynamic_failed_route_signatures[signature] = (
                    now + max(
                        self.dynamic_overlay_ttl,
                        4.0 * self.dynamic_replan_retry,
                    )
                )
                self._nav_debug(
                    "DYNAMIC_ROUTE_CANDIDATE",
                    result="REJECTED",
                    route_signature=signature,
                    still_hits_blocker=still_hits_blocker,
                    same_blocked_route=(
                        signature == self.dynamic_blocked_route_signature
                    ),
                    minimum_side_clearance=minimum_clearance,
                    hard_clearance_required=hard_clearance,
                )
                continue
            local_bypass, overlap, maximum_deviation = (
                self._dynamic_route_relation(points)
            )
            candidate["overlap_with_blocked_route"] = overlap
            candidate["maximum_deviation_from_blocked_route"] = round(
                maximum_deviation, 3
            )
            candidate["recovery_route_kind"] = (
                "LOCAL_BYPASS"
                if local_bypass
                else "GLOBAL_ALTERNATIVE"
            )
            if local_bypass:
                local_candidates.append(candidate)
            else:
                global_candidates.append(candidate)

        if local_candidates:
            selected = local_candidates[0]
            points = [dict(point) for point in selected["points"]]
            directions = list(selected.get("segment_directions") or [])
            signature = self._route_signature(points)
            route_root = self.execution_route_id.split(
                "-local-bypass", 1
            )[0]
            route_id = f"{route_root}-local-bypass-{signature[:10]}"
            self.route_candidates = {}
            self.selected_route_id = ""
            self.dynamic_recovery_state = "RESUME"
            self._nav_debug(
                "DYNAMIC_LOCAL_BYPASS",
                result="AUTO_RESUME_IN_OLD_CORRIDOR",
                route_id=route_id,
                route_signature=signature,
                overlap_with_blocked_route=(
                    selected["overlap_with_blocked_route"]
                ),
                maximum_deviation_m=(
                    selected["maximum_deviation_from_blocked_route"]
                ),
                destination=goal,
                global_route_started=False,
            )
            self._navigate(
                goal,
                {
                    "map_id": self.map_id,
                    "version": self.map_version,
                    "mission_id": self.current_mission_id,
                    "route_id": route_id,
                    "points": points,
                    "segment_directions": directions,
                },
                recovery_attempt=True,
            )
            return

        if global_candidates:
            self._present_dynamic_route_selection(global_candidates, goal)
            return

        self.dynamic_recovery_state = "WAITING"
        self._set_state(
            "WAITING_FOR_DYNAMIC_CLEAR",
            "no_safe_local_bypass_or_alternative",
        )
        self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
        self.latest_feedback["execution_phase"] = (
            "WAITING_FOR_DYNAMIC_CLEAR"
        )
        self.latest_feedback["recovery_reason"] = (
            "NO_SAFE_LOCAL_BYPASS_OR_ALTERNATIVE"
        )
        self._nav_debug(
            "DYNAMIC_LOCAL_BYPASS",
            result="NO_ROUTE_WAIT_FOR_OLD_ROUTE",
            candidate_count=len(candidates),
            destination=goal,
            destination_preserved=True,
            autonomous_global_route_started=False,
        )

    def _attempt_dynamic_replan(
        self, expected_generation: int, obstacle_cleared: bool
    ) -> None:
        if expected_generation != self.navigation_goal_generation:
            return
        goal = dict(self.execution_goal or self.paused_goal or {})
        requires_alternative = bool(self.dynamic_replan_requires_alternative)
        if (
            obstacle_cleared
            and not requires_alternative
            and len(self.dynamic_blocked_route) >= 2
        ):
            resume_points = [dict(point) for point in self.dynamic_blocked_route]
            if self.pose is not None:
                resume_points[0] = {
                    "x": float(self.pose["x"]),
                    "y": float(self.pose["y"]),
                }
            resume_points = canonicalize_stop_turn_path(resume_points)
            resume_directions = list(self.dynamic_blocked_segment_directions)
            if len(resume_directions) != max(0, len(resume_points) - 1):
                resume_directions = [1 for _ in range(max(0, len(resume_points) - 1))]
            route_root = self.execution_route_id.split("-clear-resume", 1)[0]
            resume_route_id = f"{route_root}-clear-resume"
            try:
                self.route_candidates = {}
                self.selected_route_id = ""
                self._navigate(
                    goal,
                    {
                        "map_id": self.map_id,
                        "version": self.map_version,
                        "mission_id": self.current_mission_id,
                        "route_id": resume_route_id,
                        "points": resume_points,
                        "segment_directions": resume_directions,
                    },
                    recovery_attempt=True,
                )
            except AdapterError as exc:
                self._nav_debug(
                    "DYNAMIC_REPLAN",
                    result="ORIGINAL_ROUTE_INVALIDATED",
                    error=exc.code,
                    destination=goal,
                )
            else:
                self.dynamic_recovery_state = "RESUME"
                self._nav_debug(
                    "DYNAMIC_REPLAN",
                    result="RESUME_ORIGINAL_ROUTE",
                    obstacle_cleared=True,
                    route_id=resume_route_id,
                    destination=goal,
                )
                return
        if requires_alternative:
            # A located persistent blocker first gets one bounded search for a
            # same-corridor bypass. A genuinely different route is only
            # offered to the UI and can start solely through select_route.
            self._attempt_dynamic_local_bypass_or_selection(
                expected_generation,
                goal,
            )
            return
        try:
            points = self._plan_stop_turn_from_current(goal)
            if expected_generation != self.navigation_goal_generation:
                return
            now = time.monotonic()
            self.dynamic_failed_route_signatures = {
                signature: expires
                for signature, expires in self.dynamic_failed_route_signatures.items()
                if expires > now
            }
            candidate_signature = self._route_signature(points)
            same_blocked_route = bool(
                len(self.dynamic_blocked_route) >= 2
                and len(points) >= 2
                and path_overlap_ratio(
                    self.dynamic_blocked_route, points
                ) >= self.alternative_route_overlap_threshold
            )
            route_length = sum(
                math.hypot(
                    float(right["x"]) - float(left["x"]),
                    float(right["y"]) - float(left["y"]),
                )
                for left, right in zip(points, points[1:])
            )
            still_hits_blocker = bool(
                self.dynamic_blocked_keepout is not None
                and dynamic_exclusions_intersect_route(
                    points,
                    (self.dynamic_blocked_keepout,),
                    horizon=route_length + 0.01,
                )
            )
            previously_failed = candidate_signature in self.dynamic_failed_route_signatures
            candidate_rejected = bool(
                requires_alternative
                and (
                    same_blocked_route
                    or still_hits_blocker
                    or previously_failed
                    or candidate_signature == self.dynamic_blocked_route_signature
                )
            )
            if (
                candidate_rejected
                and self.stop_turn_planner is not None
                and self.pose is not None
            ):
                # The primary solver is deterministic. Search its bounded set
                # of exact-valid, geometrically distinct candidates now instead
                # of feeding the same rejected route to the controller again.
                recovery_planner = self._stop_turn_planner_for_clearance()
                alternatives = recovery_planner.plan_candidates(
                    dict(self.pose),
                    goal,
                    maximum_candidates=self.alternative_route_max_candidates,
                    overlap_threshold=self.alternative_route_overlap_threshold,
                    planning_time_budget=self.stop_turn_planning_budget,
                    exclusions=self._dynamic_planning_exclusions(),
                )
                for alternative in alternatives:
                    alternative_points = [
                        dict(point) for point in alternative.points
                    ]
                    alternative_signature = self._route_signature(
                        alternative_points
                    )
                    alternative_length = sum(
                        math.hypot(
                            float(right["x"]) - float(left["x"]),
                            float(right["y"]) - float(left["y"]),
                        )
                        for left, right in zip(
                            alternative_points, alternative_points[1:]
                        )
                    )
                    alternative_overlap = bool(
                        len(self.dynamic_blocked_route) >= 2
                        and path_overlap_ratio(
                            self.dynamic_blocked_route, alternative_points
                        ) >= self.alternative_route_overlap_threshold
                    )
                    alternative_hits_blocker = bool(
                        self.dynamic_blocked_keepout is not None
                        and dynamic_exclusions_intersect_route(
                            alternative_points,
                            (self.dynamic_blocked_keepout,),
                            horizon=alternative_length + 0.01,
                        )
                    )
                    if (
                        alternative_overlap
                        or alternative_hits_blocker
                        or alternative_signature
                        in self.dynamic_failed_route_signatures
                        or alternative_signature
                        == self.dynamic_blocked_route_signature
                        or alternative.metadata.minimum_side_clearance + 1e-9
                        < self._hard_route_side_clearance()
                    ):
                        self.dynamic_failed_route_signatures[
                            alternative_signature
                        ] = now + max(
                            self.dynamic_overlay_ttl,
                            4.0 * self.dynamic_replan_retry,
                        )
                        continue
                    points = alternative_points
                    candidate_signature = alternative_signature
                    same_blocked_route = False
                    still_hits_blocker = False
                    previously_failed = False
                    candidate_rejected = False
                    self.pending_start_escape = None
                    self.pending_segment_directions = list(
                        alternative.segment_directions
                        or (1 for _ in range(max(0, len(points) - 1)))
                    )
                    self._nav_debug(
                        "DYNAMIC_REPLAN",
                        result="DISTINCT_ALTERNATIVE_SELECTED",
                        route_signature=candidate_signature,
                        blocked_route_signature=(
                            self.dynamic_blocked_route_signature
                        ),
                        blocker_id=self.dynamic_blocker_id or None,
                    )
                    break
            if (
                candidate_rejected
            ):
                self.dynamic_failed_route_signatures[candidate_signature] = (
                    now + max(
                        self.dynamic_overlay_ttl,
                        4.0 * self.dynamic_replan_retry,
                    )
                )
                self.dynamic_recovery_state = "WAITING"
                self._set_state("WAITING_FOR_DYNAMIC_CLEAR", "same_blocked_route")
                self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
                self.latest_feedback["execution_phase"] = (
                    "WAITING_FOR_DYNAMIC_CLEAR"
                )
                self.latest_feedback["recovery_reason"] = "SAME_BLOCKED_ROUTE"
                self._nav_debug(
                    "DYNAMIC_REPLAN",
                    result="ALTERNATIVE_REJECTED",
                    reason=(
                        "STILL_INTERSECTS_BLOCKER"
                        if still_hits_blocker
                        else "FAILED_ROUTE_SIGNATURE"
                        if previously_failed
                        else "SAME_BLOCKED_ROUTE"
                    ),
                    route_signature=candidate_signature,
                    blocked_route_signature=self.dynamic_blocked_route_signature,
                    blocker_id=self.dynamic_blocker_id or None,
                    attempt_count=self.dynamic_replan_attempt_count,
                    destination=goal,
                    destination_preserved=True,
                )
                return
            digest = candidate_signature[:10]
            route_id = f"{self.execution_route_id}-dynamic-{digest}"
            if self.pending_start_escape is not None:
                self.route_candidates[route_id] = {
                    "route_id": route_id,
                    **self.pending_start_escape,
                }
            elif self.pending_segment_directions:
                self.route_candidates[route_id] = {
                    "route_id": route_id,
                    "points": list(points),
                    "segment_directions": list(
                        self.pending_segment_directions
                    ),
                }
            self._nav_debug(
                "DYNAMIC_REPLAN",
                result="SUCCESS",
                obstacle_cleared=obstacle_cleared,
                route_id=route_id,
                destination=goal,
                route_signature=candidate_signature,
                blocked_route_signature=self.dynamic_blocked_route_signature,
                blocker_id=self.dynamic_blocker_id or None,
            )
            self.dynamic_recovery_state = "RESUME"
            self._navigate(
                goal,
                {
                    "map_id": self.map_id,
                    "version": self.map_version,
                    "route_id": route_id,
                    "points": points,
                },
                recovery_attempt=True,
            )
        except AdapterError as exc:
            if expected_generation != self.navigation_goal_generation:
                return
            confirmed_stationary = any(
                item.motion_state == "STATIONARY"
                for item in self._dynamic_route_obstacles(
                    minimum_observations=self.dynamic_planning_minimum_observations
                )
            )
            topology_proven_unreachable = bool(
                exc.code == "GOAL_PHYSICALLY_UNREACHABLE"
                and requires_alternative
                and confirmed_stationary
                and self.dynamic_replan_attempt_count
                >= self.failed_segment_max_replans
            )
            if topology_proven_unreachable:
                self._set_recovery_terminal(
                    "FAILED", exc.code, expected_generation
                )
                self._nav_debug(
                    "DYNAMIC_REPLAN",
                    result="FAILED",
                    error=exc.code,
                    destination=goal,
                    destination_preserved=False,
                    attempt_count=self.dynamic_replan_attempt_count,
                    blocker_id=self.dynamic_blocker_id or None,
                )
                return
            else:
                self.dynamic_recovery_state = "WAITING"
                self._enter_dynamic_wait(
                    exc.code or "DYNAMIC_ROUTES_TEMPORARILY_BLOCKED"
                )
                self._set_state(
                    "WAITING_FOR_DYNAMIC_CLEAR",
                    "dynamic_routes_temporarily_blocked",
                )
                self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
                self.latest_feedback["execution_phase"] = (
                    "WAITING_FOR_DYNAMIC_CLEAR"
                )
                self.latest_feedback["recovery_reason"] = exc.code
            self._nav_debug(
                "DYNAMIC_REPLAN",
                result="WAIT",
                error=exc.code,
                destination=goal,
                destination_preserved=True,
                attempt_count=self.dynamic_replan_attempt_count,
                blocker_id=self.dynamic_blocker_id or None,
            )

    def _replan_execution_from_current(
        self,
        reason: str,
        goal_generation: int,
    ) -> None:
        if goal_generation != self.navigation_goal_generation:
            return
        goal = dict(self.execution_goal or self.paused_goal or {})
        old_route_id = self.execution_route_id
        blocked_route = self._remaining_execution_route()
        try:
            points = self._plan_stop_turn_from_current(goal)
            if len(points) < 2:
                raise AdapterError("NO_VALID_REPLAN", "Replan has no usable path")
            same_blocked_route = bool(
                len(blocked_route) >= 2
                and path_overlap_ratio(blocked_route, points)
                >= self.alternative_route_overlap_threshold
            )
            if same_blocked_route and (
                reason.startswith("CONTROLLER_ABORT")
                or bool(self._runtime_live_blockage_reason())
            ):
                self.execution_replan_attempts = max(
                    0, self.execution_replan_attempts - 1
                )
                self.dynamic_blocked_route = blocked_route
                self._enter_dynamic_wait("SAME_BLOCKED_ROUTE")
                self._nav_debug(
                    "REPLAN",
                    reason="SAME_BLOCKED_ROUTE",
                    from_current_pose=True,
                    alternative_found=False,
                    destination_preserved=True,
                )
                return
            digest = hashlib.sha1(
                json.dumps(points, sort_keys=True).encode()
            ).hexdigest()[:10]
            route_id = f"{old_route_id}-replan-{digest}"
            if self.pending_start_escape is not None:
                self.route_candidates[route_id] = {
                    "route_id": route_id,
                    **self.pending_start_escape,
                }
            elif self.pending_segment_directions:
                self.route_candidates[route_id] = {
                    "route_id": route_id,
                    "points": list(points),
                    "segment_directions": list(
                        self.pending_segment_directions
                    ),
                }
            self._nav_debug(
                "REPLAN",
                reason=reason,
                from_current_pose=True,
                old_route_id=old_route_id,
                route_id=route_id,
                points=len(points),
            )
            self._navigate(
                goal,
                {
                    "map_id": self.map_id,
                    "version": self.map_version,
                    "route_id": route_id,
                    "points": points,
                },
                recovery_attempt=True,
            )
        except AdapterError as exc:
            if exc.code == "GOAL_PHYSICALLY_UNREACHABLE":
                self._set_recovery_terminal(
                    "FAILED", exc.code or "NO_VALID_REPLAN", goal_generation
                )
            elif exc.code in {"PLANNER_NOT_READY", "MAP_MISSING"}:
                self._set_recovery_terminal(
                    "FAILED", exc.code, goal_generation
                )
            else:
                self._enter_dynamic_wait(
                    exc.code or "DYNAMIC_ROUTES_TEMPORARILY_BLOCKED"
                )
                self._set_state(
                    "WAITING_FOR_DYNAMIC_CLEAR",
                    "dynamic_routes_temporarily_blocked",
                )
                self.execution_phase = "WAITING_FOR_DYNAMIC_CLEAR"
                self.latest_feedback["execution_phase"] = (
                    "WAITING_FOR_DYNAMIC_CLEAR"
                )
                self.latest_feedback["recovery_reason"] = exc.code
            self._nav_debug(
                "REPLAN",
                reason=reason,
                from_current_pose=True,
                alternative_found=False,
                error=exc.code,
            )

    def _send_current_straight_segment(
        self,
        goal_generation: int,
        segment_token: int,
        segment_index: int,
    ) -> None:
        active = self.active_segment
        if (
            goal_generation != self.navigation_goal_generation
            or self.execution_phase != "DISPATCHING_STRAIGHT"
            or active is None
            or active.segment_token != segment_token
            or active.segment_index != segment_index
            or self.execution_segment_index != segment_index
        ):
            return
        segment_points = densify_straight_segment(
            active.effective_start,
            active.endpoint,
            spacing=self.straight_path_pose_spacing,
        )
        path = NavigationPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for point in segment_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.orientation = quaternion_from_yaw(active.fixed_heading)
            path.poses.append(pose)
        action_goal = FollowPath.Goal()
        action_goal.path = path
        action_goal.controller_id = "FollowPath"
        action_goal.goal_checker_id = "segment_goal_checker"
        future = self.follow_path_client.send_goal_async(
            action_goal,
            feedback_callback=(
                lambda feedback, generation=goal_generation,
                token=segment_token, index=segment_index:
                self._navigation_feedback(feedback, generation, token, index)
            ),
        )
        try:
            handle = self._wait(future, 5, "NAVIGATION_TIMEOUT")
        except AdapterError as exc:
            self._schedule_execution_replan(exc.code, goal_generation)
            return
        if not handle.accepted:
            self._schedule_execution_replan("GOAL_REJECTED", goal_generation)
            return
        with self.state_lock:
            if not self._segment_callback_current(
                goal_generation, segment_token, segment_index
            ):
                handle.cancel_goal_async()
                return
            self.current_goal_handle = handle
            self.execution_phase = (
                "NARROW_STRAIGHT"
                if active.narrow
                else "STRAIGHT"
            )
            self.latest_feedback["execution_phase"] = self.execution_phase
            self.motion_owner = "NAVIGATION"
            self.segment_started_monotonic = time.monotonic()
            self.segment_positive_travel = 0.0
            self.segment_last_travel_pose = (
                float((self.pose or active.effective_start)["x"]),
                float((self.pose or active.effective_start)["y"]),
            )
        self._nav_debug(
            "EXECUTION_PHASE",
            phase="STRAIGHT_BEGIN",
            segment_index=active.segment_index,
            segment_token=active.segment_token,
            route_id=active.route_id,
            narrow=active.narrow,
            path_pose_count=len(path.poses),
            planned_start=active.planned_start,
            effective_start=active.effective_start,
            end=active.endpoint,
            fixed_heading=active.fixed_heading,
            motion=("REVERSE" if active.motion_direction < 0 else "FORWARD"),
        )
        handle.get_result_async().add_done_callback(
            lambda result_future, generation=goal_generation,
            token=segment_token, index=segment_index:
            self._navigation_result(result_future, generation, token, index)
        )

    def _segment_callback_current(
        self,
        goal_generation: int,
        segment_token: int,
        segment_index: int,
    ) -> bool:
        active = self.active_segment
        return bool(
            goal_generation == self.navigation_goal_generation
            and active is not None
            and active.segment_token == segment_token
            and active.segment_index == segment_index
            and self.execution_segment_index == segment_index
        )

    def _navigation_feedback(
        self,
        feedback: Any,
        goal_generation: int,
        segment_token: int,
        segment_index: int,
    ) -> None:
        data = feedback.feedback
        with self.state_lock:
            if not self._segment_callback_current(
                goal_generation, segment_token, segment_index
            ):
                return
            self.latest_feedback = {
                "distance_remaining": float(getattr(data, "distance_to_goal", 0.0)),
                "navigation_time_seconds": float(
                    self.latest_feedback.get("navigation_time_seconds", 0.0)
                ) + 0.1,
                "recoveries": int(self.latest_feedback.get("recoveries", 0)),
                "speed": float(getattr(data, "speed", 0.0)),
                "execution_phase": self.execution_phase,
                "route_id": self.execution_route_id,
            }

    def _navigation_result(
        self,
        future: Any,
        goal_generation: int,
        segment_token: int,
        segment_index: int,
    ) -> None:
        result: Any = None
        error_code: Any = None
        error_msg = ""
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = getattr(wrapped, "result", None)
            error_code = getattr(result, "error_code", None)
            error_msg = str(getattr(result, "error_msg", "") or "")
        except Exception as exc:
            self.get_logger().error(f"FollowPath result failed: {exc}")
            status = GoalStatus.STATUS_ABORTED
            error_msg = str(exc)
        if not self._segment_callback_current(
            goal_generation, segment_token, segment_index
        ):
            return
        self.current_goal_handle = None
        self.motion_owner = "NONE"
        requested = (
            self.last_controller_requested
            or self.pipeline_samples.get("controller_requested")
        )
        final = self.pipeline_samples.get("motion_safety")
        dynamic_route_intersection = self._dynamic_affects_remaining_route()
        self._nav_debug(
            "FOLLOW_PATH_RESULT",
            status=status,
            error_code=error_code,
            error_msg=error_msg,
            segment_index=segment_index,
            segment_token=segment_token,
            controller_requested=requested,
            motion_safety_output=final,
            corridor_front_clearance=self._finite_metric(
                self.latest_corridor.front_clearance
            ),
            dynamic_route_intersection=dynamic_route_intersection,
        )
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._complete_active_segment(
                "FOLLOW_PATH_SUCCEEDED", goal_generation, cancel_action=False
            )
        elif status == GoalStatus.STATUS_CANCELED and self.current_state == "PAUSED":
            self.execution_phase = "IDLE"
        elif status == GoalStatus.STATUS_CANCELED:
            # A current, non-intentional cancel is recoverable from the fresh
            # pose. Intentional cancels invalidated the token before arrival.
            self._restart_segment_from_current(
                "UNEXPECTED_SEGMENT_CANCEL", goal_generation
            )
        else:
            evidence_reason, corridor = self._corridor_failure_evidence()
            localization_reliable = (
                self.localized
                and self.localization_state == "READY"
                and self.localization_confidence >= self.localization_low_threshold
            )
            live_blockage, live_diagnostics = (
                self._controller_abort_live_blockage(error_code, error_msg)
            )
            self._nav_debug(
                "STOP",
                reason=evidence_reason,
                source=(
                    self.safety_stop_source
                    if self.safety_stop_source not in {"NONE", "UNKNOWN"}
                    else "SEGMENT_CONTROLLER"
                ),
                segment_index=segment_index,
                segment_token=segment_token,
                last_path_follow_command=requested,
                current_turn_command=self.last_turn_command,
                final_output_command=final,
                controller_error_code=error_code,
                controller_error_msg=error_msg,
                live_blockage=live_blockage,
                live_blockage_diagnostics=live_diagnostics,
            )
            if live_blockage and localization_reliable:
                self._observe_controller_blocker()
                self._enter_dynamic_wait("CONTROLLER_ABORT_LIVE_BLOCKAGE")
                self.profile_limiter.reset()
                self.navigation_velocity.publish(Twist())
                return
            recoverable_evidence = (
                evidence_reason in {"CORRIDOR_CLEAR", "NARROW_OR_UNCERTAIN"}
                or evidence_reason == "UNCONFIRMED"
            )
            if (
                recoverable_evidence
                and localization_reliable
                and self.navigation_recovery_attempts < self.failed_segment_max_replans
            ):
                self.navigation_recovery_attempts += 1
                self.latest_feedback["recoveries"] = self.navigation_recovery_attempts
                self._set_state("NAVIGATING", "automatic_segment_retry")
                self._restart_segment_from_current(
                    f"CONTROLLER_ABORT:{evidence_reason}", goal_generation
                )
            elif recoverable_evidence and localization_reliable:
                self._schedule_execution_replan(
                    f"CONTROLLER_ABORT:{evidence_reason}", goal_generation
                )
            elif evidence_reason == "PHYSICALLY_BLOCKED" and localization_reliable:
                if self._dynamic_affects_remaining_route():
                    self._enter_dynamic_wait("CONFIRMED_DYNAMIC_ROUTE_BLOCK")
                else:
                    self._mark_failed_segment(evidence_reason, corridor)
                    self._schedule_execution_replan(
                        "CONFIRMED_STATIC_PHYSICAL_BLOCKAGE", goal_generation
                    )
            else:
                self.execution_segment_token += 1
                self.active_segment = None
                self.execution_phase = "IDLE"
                terminal = (
                    "LOCALIZATION_UNRELIABLE"
                    if not localization_reliable
                    else "SEGMENT_CONTROLLER_FAILURE"
                )
                self._set_state("FAILED", terminal.lower())
                self.latest_feedback["terminal_reason"] = terminal
                self.paused_goal = None
                self.latest_global_path = []
                self.visualization_revision += 1
                self._clear_active_navigation_mission(terminal.lower())
        self.profile_limiter.reset()
        self.navigation_velocity.publish(Twist())

    def _set_recovery_terminal(
        self,
        state: str,
        reason: str,
        expected_generation: int,
    ) -> None:
        with self.state_lock:
            if expected_generation != self.navigation_goal_generation:
                return
            self._set_state(state, reason.lower())
            self.latest_feedback["terminal_reason"] = reason
            self.paused_goal = None
            self.latest_global_path = []
            self.execution_segment_token += 1
            self.active_segment = None
            self.execution_phase = "IDLE"
            self.execution_final_turn = False
            self.execution_physical_final_turn = False
            self.execution_points = []
            self.execution_segment_directions = []
            self.execution_segment_index = 0
            self.visualization_revision += 1
        self._clear_active_navigation_mission(reason.lower())

    def _recover_navigation(
        self,
        goal: dict[str, Any],
        reason: str,
        segment: dict[str, Any] | None,
        expected_generation: int,
        old_path: list[dict[str, float]],
    ) -> None:
        if expected_generation != self.navigation_goal_generation:
            return
        recovery_points = list(old_path)
        if segment is not None:
            baseline_generation = self.global_costmap_generation
            self._publish_failed_segments()
            if not self._wait_for_global_costmap_after(
                baseline_generation, 2.0
            ):
                self._set_recovery_terminal(
                    "FAILED", "COSTMAP_NOT_READY", expected_generation
                )
                return
            try:
                alternative = self._request_path_once(goal)
            except AdapterError as exc:
                self._nav_debug(
                    "REPLAN",
                    reason="FAILED_SEGMENT",
                    alternative_found=False,
                    error=exc.code,
                )
                self._set_recovery_terminal(
                    "FAILED", exc.code, expected_generation
                )
                return
            if (
                not alternative
                or self._path_crosses_segment(alternative, segment)
            ):
                self._nav_debug(
                    "REPLAN",
                    reason="FAILED_SEGMENT",
                    alternative_found=False,
                    old_path_length=self._path_length(old_path),
                )
                self._set_recovery_terminal(
                    "BLOCKED", "NO_ALTERNATIVE_ROUTE", expected_generation
                )
                return
            recovery_points = list(alternative)
            self._nav_debug(
                "REPLAN",
                reason="FAILED_SEGMENT",
                alternative_found=True,
                old_path_length=self._path_length(old_path),
                new_path_length=self._path_length(alternative),
            )
        if expected_generation != self.navigation_goal_generation:
            return
        try:
            self._navigate(
                goal,
                {
                    "map_id": self.map_id,
                    "version": self.map_version,
                    "points": recovery_points,
                },
                recovery_attempt=True,
            )
        except AdapterError as exc:
            self._set_recovery_terminal("FAILED", exc.code, expected_generation)

    def _cancel_navigation(self, target: str) -> dict[str, Any]:
        with self.state_lock:
            preserved_goal = dict(self.execution_goal or self.paused_goal or {})
            preserved_route_id = self.execution_route_id
            preserved_path = list(self.execution_points or self.latest_global_path)
            preserved_directions = list(self.execution_segment_directions)
            handle = self.current_goal_handle
            self.current_goal_handle = None
            self.execution_segment_token += 1
            self.active_segment = None
            self.navigation_goal_generation += 1
            self.execution_phase = "IDLE"
            self._set_state(target, "operator_navigation_command")
            self.motion_owner = "NONE"
        # Revoke ownership and inject zero before waiting for Nav2's
        # acknowledgement; cancel latency must never extend chassis motion.
        self.navigation_velocity.publish(Twist())
        if handle is not None:
            self._wait(handle.cancel_goal_async(), 3, "CANCEL_TIMEOUT")
        with self.state_lock:
            # The requested terminal/paused state is already authoritative.
            # Invalidate the asynchronous Nav2 result so it cannot overwrite a
            # later resume, map deactivation, or manual takeover state.
            self.execution_points = []
            self.execution_segment_directions = []
            if target != "PAUSED":
                self.sensor_time_resume_context = None
                self.sensor_time_resume_in_progress = False
                self.paused_goal = None
                self.execution_goal = None
                self.execution_route_id = ""
                self.execution_narrow_segments = set()
                self.route_candidates = {}
                self.selected_route_id = ""
                self.current_mission_id = ""
                self.route_selection_return_state = "READY"
                self.latest_global_path = []
                self.visualization_revision += 1
        if target == "PAUSED" and preserved_goal:
            self._persist_active_navigation_mission(
                resume_automatically=False,
                goal=preserved_goal,
                route_id=preserved_route_id,
                path=preserved_path,
                segment_directions=preserved_directions,
            )
        elif target != "PAUSED":
            self._clear_active_navigation_mission(
                f"navigation_{target.lower()}"
            )
        self.profile_limiter.reset()
        self.rotation_metric_active = False
        self.obstacle_slowdown_active = False
        self.navigation_velocity.publish(Twist())
        return {"status": "completed", "current_state": target, "state": self._state()}

    def _pause_navigation(self) -> dict[str, Any]:
        recovery_active = self.current_state in {
            "WAIT_FOR_DYNAMIC_CLEAR",
            "WAITING_FOR_DYNAMIC_CLEAR",
            "DYNAMIC_REPLAN",
        }
        if (
            self.current_goal_handle is None
            and not recovery_active
            and self.execution_phase not in {
                "STRAIGHT_PREPARE", "TURN", "TURN_SETTLING",
                "DISPATCHING_STRAIGHT", "RECOVERING",
                "WAIT_FOR_DYNAMIC_CLEAR", "DYNAMIC_REPLAN",
            }
        ):
            raise AdapterError("STATE_CONFLICT", "Navigation is not active")
        return self._cancel_navigation("PAUSED")

    def _manual_handoff(self, reason: str) -> dict[str, Any]:
        with self.state_lock:
            if self.paused_goal is None:
                raise AdapterError("STATE_CONFLICT", "No destination is available for handoff")
            handle = self.current_goal_handle
            self.current_goal_handle = None
            self.navigation_goal_generation += 1
            self.execution_segment_token += 1
            self.active_segment = None
            self.motion_owner = "NONE"
            self.execution_phase = "IDLE"
            self.manual_handoff_reason = reason
            self._set_state("MANUAL_BYPASS", "operator_manual_handoff")
        self.profile_limiter.reset()
        self.navigation_velocity.publish(Twist())
        if handle is not None:
            handle.cancel_goal_async()
        self._persist_active_navigation_mission(
            resume_automatically=False,
            goal=self.paused_goal,
            route_id=self.execution_route_id,
            path=self.latest_global_path,
        )
        self._nav_debug(
            "NARROW_DECISION",
            choice="MANUAL",
            destination=self.paused_goal,
            route_id=self.selected_route_id,
        )
        return {
            "status": "completed",
            "current_state": "MANUAL_BYPASS",
            "destination_preserved": True,
            "goal": dict(self.paused_goal),
            "state": self._state(),
        }

    def _plan_stop_turn_from_current(
        self, goal: dict[str, Any]
    ) -> list[dict[str, float]]:
        if self.stop_turn_planner is None or self.pose is None:
            raise AdapterError("PLANNER_NOT_READY", "Stop-turn planner is unavailable")
        self.pending_start_escape = None
        self.pending_segment_directions = []
        hard_side_clearance = self._hard_route_side_clearance()
        request_planner = self._stop_turn_planner_for_clearance(
            hard_side_clearance
        )
        result = request_planner.plan_result(
            dict(self.pose),
            goal,
            exclusions=self._dynamic_planning_exclusions(),
            planning_time_budget=self.stop_turn_planning_budget,
            allow_start_escape=True,
            maximum_start_escape_distance=self.start_escape_max_distance,
            live_start_clear=bool(
                self._atomic_safety_fresh()
                and not (self.safety_direction_mask & 1)
                and not self.estop_active
            ),
        )
        if result.route is None:
            code = result.reason or result.status
            raise AdapterError(
                (
                    "GOAL_PHYSICALLY_UNREACHABLE"
                    if code == "GOAL_DISCONNECTED"
                    else code
                ),
                result.message or "No exact footprint-valid stop-turn path remains",
            )
        if (
            result.route.metadata.minimum_side_clearance + 1e-9
            < hard_side_clearance
        ):
            raise AdapterError(
                "ROUTE_CLEARANCE_INSUFFICIENT",
                (
                    "No recovery route keeps the configured side clearance; "
                    f"best={result.route.metadata.minimum_side_clearance:.3f}m "
                    f"hard_required={hard_side_clearance:.3f}m "
                    f"preferred={self.corridor_side_margin:.3f}m"
                ),
            )
        route_points = [dict(point) for point in result.route.points]
        self.pending_segment_directions = list(
            result.route.segment_directions
            or (1 for _ in range(max(0, len(route_points) - 1)))
        )
        if result.start_escape is not None:
            escape = result.start_escape
            display = [dict(escape.start), dict(escape.end), *route_points[1:]]
            self.pending_start_escape = {
                "points": canonicalize_stop_turn_path(display),
                "start_escape": {
                    "start": dict(escape.start),
                    "end": dict(escape.end),
                    "yaw": escape.yaw,
                    "distance": escape.distance,
                    "motion_direction": escape.motion_direction,
                },
            }
            self.pending_segment_directions = []
            return list(self.pending_start_escape["points"])
        return route_points

    def _resume_auto_from_current_pose(self) -> dict[str, Any]:
        if self.paused_goal is None:
            raise AdapterError("STATE_CONFLICT", "No paused goal to resume")
        if not self.localized or self.localization_state != "READY":
            raise AdapterError(
                "LOCALIZATION_UNRELIABLE",
                "Current pose must be verified before Auto can resume",
            )
        # Manual topics time out after at most 300 ms. Publish Auto zero and
        # wait one bounded interval so twist_mux cannot blend stale takeover
        # input with the new controller action.
        self.navigation_velocity.publish(Twist())
        time.sleep(0.35)
        goal = dict(self.paused_goal)
        with self.state_lock:
            self._set_state("PLANNING", "resume_from_current_pose")
        try:
            points = self._plan_stop_turn_from_current(goal)
        except AdapterError:
            self._set_state("MANUAL_BYPASS", "resume_path_unavailable")
            raise
        if len(points) < 2:
            self._set_state("MANUAL_BYPASS", "resume_path_unavailable")
            raise AdapterError("NO_VALID_PATH", "No safe path from current pose")
        route_id = f"resume-{hashlib.sha1(json.dumps(points, sort_keys=True).encode()).hexdigest()[:10]}"
        if self.pending_start_escape is not None:
            self.route_candidates[route_id] = {
                "route_id": route_id,
                **self.pending_start_escape,
            }
        elif self.pending_segment_directions:
            self.route_candidates[route_id] = {
                "route_id": route_id,
                "points": list(points),
                "segment_directions": list(
                    self.pending_segment_directions
                ),
            }
        self.manual_handoff_reason = ""
        return self._navigate(
            goal,
            {
                "map_id": self.map_id,
                "version": self.map_version,
                "route_id": route_id,
                "points": points,
            },
        )

    def _start_selected_route(self, route_id: str) -> dict[str, Any]:
        candidate = self.route_candidates.get(route_id)
        if candidate is None or self.paused_goal is None:
            raise AdapterError("ROUTE_NOT_FOUND", "Selected route is no longer available")
        points = list(candidate["points"])
        segment_directions = list(
            candidate.get("segment_directions") or []
        )
        current_validation = self._route_metadata(
            points,
            original=list(self.latest_global_path),
            segment_directions=(segment_directions or None),
        )
        if not current_validation["valid"]:
            raise AdapterError(
                "ROUTE_INVALIDATED",
                "Selected route is no longer clear; choose another route",
            )
        self._nav_debug("ROUTE_SELECTED", route_id=route_id, choice="USER")
        return self._navigate(
            dict(self.paused_goal),
            {
                "map_id": self.map_id,
                "version": self.map_version,
                "route_id": route_id,
                "points": points,
            },
        )

    def _mapping_command(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        if self.mode != "MAPPING":
            raise AdapterError("WRONG_MODE", "Navigation stack is not in MAPPING mode")
        if command == "mapping.start":
            readiness = self._state()
            mapping_conflicts = self._foreign_mapping_authorities()
            if mapping_conflicts:
                raise AdapterError(
                    "MAPPING_AUTHORITY_CONFLICT",
                    "Another ROS mapping authority is active: "
                    + ", ".join(mapping_conflicts),
                )
            if not readiness["scan_fresh"]:
                raise AdapterError("SCAN_STALE", "LiDAR /scan has no fresh publisher")
            if not readiness["odometry_ready"]:
                raise AdapterError(
                    "ODOMETRY_UNAVAILABLE",
                    "TF odom -> base_footprint is unavailable",
                )
            if not readiness["lidar_tf_ready"]:
                raise AdapterError(
                    "LIDAR_TF_UNAVAILABLE",
                    "TF base_footprint -> base_link -> laser_frame is unavailable",
                )
            if readiness["safety"] != "HEALTHY":
                raise AdapterError("SAFETY_UNHEALTHY", "Motion safety is not healthy")
            if readiness["estop"]:
                raise AdapterError("ESTOP_ACTIVE", "E-stop is active")
            if self.current_state in {"CANCELED", "FINISHED", "FAULT", "MAPPING_ERROR"}:
                self._restart_slam_runtime()
            with self.state_lock:
                self.mapping_payload = dict(payload)
                self.map_id = str(payload.get("map_id") or "")
                self.map_version = int(payload.get("version") or 1)
                self.trail = []
                self.pose = None
                self.mapping_started_monotonic = time.monotonic()
                self.mapping_relocalization_source_map = None
                self.mapping_free_cell_observations = {}
                self.mapping_hit_cell_observations = {}
                self.mapping_change_evidence_scans = 0
            posegraph_path = str(payload.get("posegraph_path") or "")
            if posegraph_path:
                self._verify_mapping_continuation_pose(
                    posegraph_path, payload.get("initial_pose")
                )
            else:
                with self.state_lock:
                    self.mapping_relocalization_diagnostics = {
                        "state": "NOT_REQUIRED",
                        "hint_is_approximate": True,
                    }
            with self.state_lock:
                self.current_state = "MAPPING_RUNNING"
            return {"status": "completed", "current_state": "MAPPING_RUNNING", "state": self._state()}
        if command == "mapping.stop":
            self._call_empty_like(self.slam_pause_client, Pause.Request(), "SLAM_STOP_FAILED")
            self.current_state = "MAPPING_STOPPED_UNSAVED"
            return {
                "status": "completed",
                "current_state": "MAPPING_STOPPED_UNSAVED",
                "state": self._state(),
            }
        if command == "mapping.pause":
            self._call_empty_like(self.slam_pause_client, Pause.Request(), "SLAM_PAUSE_FAILED")
            self.current_state = "PAUSED"
            return {"status": "completed", "current_state": "PAUSED", "state": self._state()}
        if command == "mapping.resume":
            self._call_empty_like(self.slam_pause_client, Pause.Request(), "SLAM_RESUME_FAILED")
            self.current_state = "MAPPING_RUNNING"
            return {"status": "completed", "current_state": "MAPPING_RUNNING", "state": self._state()}
        if command in {"mapping.save", "mapping.save_draft", "mapping.finish"}:
            state_before_save = self.current_state
            self.current_state = "MAPPING_SAVING"
            bundle = self._save_mapping_bundle(payload)
            self.current_state = state_before_save if command.endswith("save_draft") else "FINISHED"
            return {
                "status": "completed",
                "current_state": self.current_state,
                "bundle_path": str(bundle),
                "draft_saved": command.endswith("save_draft"),
                "state": self._state(),
            }
        if command in {"mapping.discard", "mapping.cancel"}:
            self.current_state = "CANCELED"
            return {"status": "completed", "current_state": "CANCELED", "state": self._state()}
        raise AdapterError("UNSUPPORTED_COMMAND", command)

    def _verify_mapping_continuation_pose(self, filename: str, initial_pose: Any) -> None:
        if not isinstance(initial_pose, dict) or not all(
            axis in initial_pose and math.isfinite(float(initial_pose[axis]))
            for axis in ("x", "y", "yaw")
        ):
            raise AdapterError(
                "MAPPING_POSE_HINT_REQUIRED",
                "Cần chọn vùng và hướng gần đúng trước khi tiếp tục map",
            )
        hint = (
            float(initial_pose["x"]),
            float(initial_pose["y"]),
            float(initial_pose["yaw"]),
        )
        source_yaml = Path(filename).parent / "map.yaml"
        try:
            source_map = SavedOccupancyMap.load(source_yaml)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise AdapterError(
                "MAPPING_SOURCE_MAP_INVALID",
                f"Không đọc được occupancy map nguồn: {exc}",
            ) from exc
        with self.state_lock:
            self.current_state = "MAPPING_LOCALIZING"
            self.mapping_relocalization_active = False
            self.mapping_relocalization_probe_count = 0
            self.mapping_relocalization_hint = None
            self.mapping_relocalization_result = None
            self.mapping_relocalization_event.clear()
            self.mapping_pose_search_event.clear()
            self.mapping_pose_search_active = True
            self.mapping_pose_search_snapshot = None
            self.mapping_relocalization_latest_snapshot = None
            self.mapping_relocalization_source_map = source_map
            self.mapping_relocalization_corrected_pose = None
            self.mapping_relocalization_geometry_confirmations = 0
            self.mapping_relocalization_geometry_samples = 0
            self.mapping_relocalization_diagnostics = {
                "state": "SEARCHING_SAVED_MAP",
                "hint_is_approximate": True,
                "probe_scans": 0,
                "operator_hint": {
                    "x": hint[0], "y": hint[1], "yaw": hint[2]
                },
            }

        if not self.mapping_pose_search_event.wait(min(
            3.0, self.mapping_relocalization_timeout
        )):
            with self.state_lock:
                self.mapping_pose_search_active = False
                self.current_state = "MAPPING_ERROR"
                self.mapping_relocalization_diagnostics.update({
                    "state": "SEARCH_TIMEOUT",
                    "reason": "NO_FRESH_SCAN_FOR_POSE_SEARCH",
                })
            raise AdapterError(
                "MAPPING_POSE_SEARCH_TIMEOUT",
                "Không nhận được scan mới để tìm hướng robot trên map cũ",
            )
        snapshot = dict(self.mapping_pose_search_snapshot or {})
        extrinsic = snapshot.get("laser_in_base")
        if not snapshot or extrinsic is None:
            with self.state_lock:
                self.current_state = "MAPPING_ERROR"
            raise AdapterError(
                "MAPPING_POSE_SEARCH_TF_UNAVAILABLE",
                "Không có TF base_footprint → laser_frame để tìm pose",
            )
        search = global_scan_candidate_uniqueness(
            source_map,
            snapshot["ranges"],
            angle_min=float(snapshot["angle_min"]),
            angle_increment=float(snapshot["angle_increment"]),
            range_min=float(snapshot["range_min"]),
            range_max=float(snapshot["range_max"]),
            candidate_pose=hint,
            laser_x=float(extrinsic[0]),
            laser_y=float(extrinsic[1]),
            laser_yaw=float(extrinsic[2]),
            maximum_beams=self.global_scan_maximum_beams,
            minimum_usable_range=self.scan_map_minimum_range,
            maximum_usable_range=self.scan_map_maximum_range,
            endpoint_tolerance=self.global_scan_endpoint_tolerance,
            position_step=self.global_scan_position_step,
            heading_step=self.global_scan_heading_step,
            alternative_separation=self.particle_alternative_separation,
            minimum_best_score=self.mapping_pose_search_minimum_score,
            minimum_score_margin=self.mapping_pose_search_minimum_margin,
            minimum_score_ratio=self.mapping_pose_search_minimum_ratio,
            candidate_position_tolerance=(
                self.global_scan_candidate_position_tolerance
            ),
            candidate_yaw_tolerance=self.global_scan_candidate_yaw_tolerance,
            search_center=(hint[0], hint[1]),
            search_radius=self.global_scan_hint_radius,
            require_candidate_match=False,
            alternative_yaw_separation=math.radians(45.0),
        )
        if (
            not search.accepted
            or search.best_x is None
            or search.best_y is None
            or search.best_yaw is None
        ):
            with self.state_lock:
                self.current_state = "MAPPING_ERROR"
                self.mapping_relocalization_diagnostics.update({
                    "state": "SEARCH_REJECTED",
                    "reason": search.reason,
                    "best_score": search.best_score,
                    "alternative_score": search.alternative_score,
                    "score_margin": search.score_margin,
                    "score_ratio": self._finite_metric(search.score_ratio),
                })
            raise AdapterError(
                "MAPPING_POSE_SEARCH_AMBIGUOUS",
                "LiDAR chưa xác định được một vị trí–hướng duy nhất; hãy chọn vùng gần hơn",
            )
        seed = {
            "x": float(search.best_x),
            "y": float(search.best_y),
            "yaw": float(search.best_yaw),
        }
        with self.state_lock:
            self.mapping_relocalization_hint = (
                seed["x"], seed["y"], seed["yaw"]
            )
            self.mapping_relocalization_diagnostics.update({
                "state": "SLAM_REFINING",
                "searched_pose": dict(seed),
                "best_score": search.best_score,
                "alternative_score": search.alternative_score,
                "score_margin": search.score_margin,
                "score_ratio": self._finite_metric(search.score_ratio),
                "evaluated_candidates": search.evaluated_candidates,
            })

        try:
            self._load_mapping_posegraph(filename, seed)
        except AdapterError:
            with self.state_lock:
                self.current_state = "MAPPING_ERROR"
                self.mapping_relocalization_diagnostics = {
                    "state": "LOAD_FAILED",
                    "hint_is_approximate": True,
                    "probe_scans": 0,
                }
            raise
        with self.state_lock:
            self.mapping_relocalization_active = True
        if not self.mapping_relocalization_event.wait(
            self.mapping_relocalization_timeout
        ):
            with self.state_lock:
                self.mapping_relocalization_active = False
                geometry_started = (
                    self.mapping_relocalization_corrected_pose is not None
                )
                self.mapping_relocalization_diagnostics = {
                    "state": "TIMEOUT",
                    "reason": (
                        "GEOMETRY_CONFIRMATION_TIMEOUT"
                        if geometry_started else "NO_CORRECTED_SLAM_POSE"
                    ),
                    "hint_is_approximate": True,
                    "probe_scans": self.mapping_relocalization_probe_count,
                    "geometry_confirmations": (
                        self.mapping_relocalization_geometry_confirmations
                    ),
                    "required_confirmations": (
                        self.mapping_relocalization_required_confirmations
                    ),
                }
            failure = AdapterError(
                "MAPPING_RELOCALIZATION_TIMEOUT",
                (
                    "Scan LiDAR chưa ổn định theo map cũ; hãy giữ robot đứng yên"
                    if geometry_started
                    else "SLAM không khớp được robot với map cũ; hãy chọn vùng gần hơn"
                ),
            )
        else:
            result = dict(self.mapping_relocalization_result or {})
            if result.get("accepted"):
                self.get_logger().info(
                    "SLAM confirmed the approximate continuation pose "
                    f"after {self.mapping_relocalization_probe_count} probe scan(s)"
                )
                return
            failure = AdapterError(
                "MAPPING_RELOCALIZATION_UNCERTAIN",
                "Kết quả khớp pose chưa đủ tin cậy; hãy chọn vị trí hoặc hướng gần hơn",
            )

        # Probe scans must never remain in the working graph after a failed
        # match. Reload the immutable source before exposing the error state.
        try:
            self._load_mapping_posegraph(filename, seed)
        except AdapterError as rollback_error:
            failure = AdapterError(
                failure.code,
                f"{failure}; không thể rollback pose-graph: {rollback_error}",
            )
        with self.state_lock:
            self.current_state = "MAPPING_ERROR"
        raise failure

    def _load_mapping_posegraph(self, filename: str, initial_pose: Any) -> None:
        posegraph = Path(filename)
        if not posegraph.is_absolute() or not posegraph.with_suffix(".posegraph").is_file() or not posegraph.with_suffix(".data").is_file():
            raise AdapterError("POSEGRAPH_MISSING", "Serialized pose-graph is incomplete")
        request = DeserializePoseGraph.Request()
        request.filename = str(posegraph)
        if isinstance(initial_pose, dict) and all(
            math.isfinite(float(initial_pose.get(axis, 0.0)))
            for axis in ("x", "y", "yaw")
        ):
            request.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
            request.initial_pose.x = float(initial_pose.get("x", 0.0))
            request.initial_pose.y = float(initial_pose.get("y", 0.0))
            request.initial_pose.theta = float(initial_pose.get("yaw", 0.0))
        else:
            # Existing bundles may predate terminal-pose metadata. Continuing
            # from the first graph node is safe when the robot is returned to
            # its original mapping start position.
            request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        response = self._call_empty_like(
            self.slam_deserialize_client,
            request,
            "POSEGRAPH_LOAD_FAILED",
        )
        if hasattr(response, "result") and not bool(response.result):
            raise AdapterError(
                "POSEGRAPH_LOAD_FAILED",
                "SLAM Toolbox rejected the serialized pose-graph",
            )
        self.get_logger().info(f"continued mapping from pose-graph {posegraph}")

    @staticmethod
    def _slam_process_ids() -> set[int]:
        process_ids: set[int] = set()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if b"async_slam_toolbox_node" in command:
                process_ids.add(int(entry.name))
        return process_ids

    def _restart_slam_runtime(self) -> None:
        previous = self._slam_process_ids()
        if not previous:
            raise AdapterError("SLAM_RESET_FAILED", "SLAM Toolbox process is unavailable")
        self.get_logger().info("resetting SLAM pose graph for the next mapping session")
        for process_id in previous:
            try:
                os.kill(process_id, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 12.0
        replacement: set[int] = set()
        while time.monotonic() < deadline:
            current = self._slam_process_ids()
            replacement = current - previous
            if replacement:
                break
            time.sleep(0.1)
        if not replacement:
            raise AdapterError("SLAM_RESET_FAILED", "SLAM Toolbox did not restart")
        if not self.slam_save_client.wait_for_service(timeout_sec=8.0):
            raise AdapterError("SLAM_RESET_FAILED", "Restarted SLAM Toolbox is not ready")

    def _call_empty_like(self, client: Any, request: Any, code: str) -> Any:
        if not client.wait_for_service(timeout_sec=3.0):
            raise AdapterError(code, "SLAM Toolbox service unavailable")
        return self._wait(client.call_async(request), 10, code)

    def _save_mapping_bundle(self, payload: dict[str, Any]) -> Path:
        map_id = str(payload.get("map_id") or self.mapping_payload.get("map_id"))
        version = int(payload.get("version") or self.mapping_payload.get("version", 1))
        if not map_id:
            raise AdapterError("MAPPING_SESSION_MISSING", "Mapping session has no map_id")
        staging = self.map_root / ".staging" / f"{map_id}-v{version}"
        shutil.rmtree(staging, ignore_errors=True)
        staging.mkdir(parents=True)
        prefix = staging / "map"
        save = SaveMap.Request()
        save.name.data = str(prefix)
        self._call_empty_like(self.slam_save_client, save, "MAP_SAVE_FAILED")
        serialize = SerializePoseGraph.Request()
        serialize.filename = str(staging / "posegraph")
        self._call_empty_like(self.slam_serialize_client, serialize, "POSEGRAPH_SAVE_FAILED")
        yaml_path = staging / "map.yaml"
        pgm_path = staging / "map.pgm"
        if not yaml_path.is_file() or not pgm_path.is_file():
            raise AdapterError("MAP_SAVE_FAILED", "SLAM Toolbox did not create map.yaml/map.pgm")
        yaml_data = yaml.safe_load(yaml_path.read_text())
        if not isinstance(yaml_data, dict):
            raise AdapterError("MAP_SAVE_FAILED", "SLAM Toolbox created invalid map.yaml")
        image = Image.open(pgm_path)
        cleared_stale_cells = self._apply_mapping_change_evidence(
            image, yaml_data
        )
        if cleared_stale_cells:
            # SaveMap serialized the historical pose-graph raster. Replace
            # only cells disproven by repeated current-session free rays.
            image.save(pgm_path)
            self._nav_debug(
                "MAPPING_STALE_CELLS_CLEARED",
                cleared_cells=cleared_stale_cells,
                evidence_scans=self.mapping_change_evidence_scans,
                free_evidence_cells=len(self.mapping_free_cell_observations),
                protected_hit_cells=len(self.mapping_hit_cell_observations),
            )
        yaml_data, semantics_changed = normalize_trinary_unknown_metadata(
            yaml_data, image_grayscale_values(image.convert("L"))
        )
        if semantics_changed:
            # New bundles are canonical at rest; the runtime-side normalization
            # remains only for already-saved maps made by the affected build.
            yaml_path.write_text(yaml.safe_dump(yaml_data, sort_keys=False))
        image.save(staging / "preview.png")
        metadata = {
            "map_id": map_id,
            "name": str(dict(self.mapping_payload.get("metadata") or {}).get("name") or map_id),
            "version": version,
            "robot_id": os.getenv("ROBOT_ID", ""),
            "resolution": float(yaml_data["resolution"]),
            "origin": {
                "x": float(yaml_data["origin"][0]),
                "y": float(yaml_data["origin"][1]),
                "yaw": float(yaml_data["origin"][2]),
            },
            "width": image.width,
            "height": image.height,
            "created_by_robot": os.getenv("ROBOT_ID", ""),
            "created_at": time.time(),
            "updated_at": time.time(),
            "frame_id": "map",
            "has_posegraph": True,
            "slam_mode": "slam_toolbox_online_async",
            "continued_map_cleanup": {
                "evidence_scans": self.mapping_change_evidence_scans,
                "cleared_stale_cells": cleared_stale_cells,
            },
            "terminal_pose": dict(self.pose) if self.pose else None,
            "files": {},
            "poi": [],
            "keepout_zones": [],
            "speed_zones": [],
        }
        for path in staging.iterdir():
            if path.is_file() and path.name != "metadata.json":
                metadata["files"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        metadata["checksum"] = metadata["files"][pgm_path.name]
        metadata["checksum_scope"] = pgm_path.name
        (staging / "metadata.json").write_text(json.dumps(metadata, indent=2))

        # Reject a corrupt or internally inconsistent local save before it is
        # ever queued for Center upload. This reads the exact saved grid (no
        # visualization downsampling) and verifies every declared artifact.
        try:
            saved_grid = SavedOccupancyMap.load(yaml_path)
            if (
                saved_grid.width != metadata["width"]
                or saved_grid.height != metadata["height"]
                or not math.isclose(saved_grid.resolution, metadata["resolution"])
            ):
                raise ValueError("saved grid dimensions/resolution do not match metadata")
            for filename, expected_sha256 in metadata["files"].items():
                artifact = staging / filename
                if not artifact.is_file():
                    raise ValueError(f"missing declared artifact {filename}")
                actual_sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
                if actual_sha256 != expected_sha256:
                    raise ValueError(f"checksum mismatch for {filename}")
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise AdapterError("MAP_SAVE_VALIDATION_FAILED", str(exc)) from exc

        bundle = staging / "map-bundle.tar.gz"
        with tarfile.open(bundle, "w:gz") as archive:
            for path in staging.iterdir():
                if path.is_file() and path != bundle:
                    archive.add(path, arcname=path.name)
        expected_members = set(metadata["files"]) | {"metadata.json"}
        with tarfile.open(bundle, "r:gz") as archive:
            actual_members = {member.name for member in archive.getmembers() if member.isfile()}
        if actual_members != expected_members:
            raise AdapterError(
                "MAP_SAVE_VALIDATION_FAILED",
                "generated bundle does not contain the exact declared artifact set",
            )
        final = self.map_root / "created" / map_id / f"v{version}"
        final.parent.mkdir(parents=True, exist_ok=True)
        previous = final.with_name(f".{final.name}.previous")
        shutil.rmtree(previous, ignore_errors=True)
        if final.exists():
            os.replace(final, previous)
        os.replace(staging, final)
        shutil.rmtree(previous, ignore_errors=True)
        return final / bundle.name

    def _schedule_autosave(self) -> None:
        if self.mode != "MAPPING" or self.current_state not in {"MAPPING", "MAPPING_RUNNING", "PAUSED"}:
            return
        threading.Thread(target=self._autosave_posegraph, daemon=True).start()

    def _autosave_posegraph(self) -> None:
        try:
            map_id = str(self.mapping_payload.get("map_id", "session"))
            autosave = self.map_root / ".autosave"
            autosave.mkdir(parents=True, exist_ok=True)
            request = SerializePoseGraph.Request()
            request.filename = str(autosave / f"{map_id}-latest")
            self._call_empty_like(self.slam_serialize_client, request, "AUTOSAVE_FAILED")
        except AdapterError as exc:
            self.get_logger().warning(str(exc))

    def destroy_node(self) -> bool:
        self._closing.set()
        try:
            self._server.close()
        except OSError:
            pass
        self.socket_path.unlink(missing_ok=True)
        self.navigation_debug_log.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NavigationAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
