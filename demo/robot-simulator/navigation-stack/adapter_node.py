from __future__ import annotations

from collections import deque
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
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
from PIL import Image
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from navigation_core import (
    NavigationDebugLog,
    PoseStability,
    SavedOccupancyMap,
    classify_planning_failure,
    compact_lethal_cells,
    environment_flag,
    evaluate_corridor,
    filter_static_map_scan,
    heading_diversity,
    localization_confidence,
    pose_stability,
    rotation_swept_clearance,
    scan_to_map_match,
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
        self.current_state = "IDLE" if self.mode == "MAPPING" else "STARTING"
        self.localization_state = "IDLE"
        self.localized = False
        self.initial_pose_requested = False
        self.map_id = ""
        self.map_version = 0
        self.mapping_payload: dict[str, Any] = {}
        self.current_goal_handle: Any = None
        self.navigation_goal_generation = 0
        self.paused_goal: dict[str, float] | None = None
        self.current_mission_id = ""
        self.latest_feedback: dict[str, Any] = {}
        self.saved_map: SavedOccupancyMap | None = None
        self.active_map_path: Path | None = None
        self.map_received_monotonic = 0.0
        self.last_scan_monotonic = 0.0
        self.scan_clock_skew_seconds = 0.0
        self.sensor_time_status: dict[str, Any] = {}
        self.last_sensor_time_status_monotonic = 0.0
        self.sensor_time_invalid_since: float | None = None
        self.safety_health = "UNKNOWN"
        self.estop_active = False
        self.safety_direction_mask = 0
        self.last_manual_takeover_monotonic = 0.0
        self.pose: dict[str, float] | None = None
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
        self.scan_map_median_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_p90_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_mean_residuals: deque[float] = deque(
            maxlen=self.scan_map_scores.maxlen
        )
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
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
        self.global_search_requires_rotation = False
        self.last_initial_pose_publish_monotonic = 0.0
        self.rotation_active = False
        self.rotation_angle = 0.0
        self.rotation_last_monotonic = 0.0
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
        self.approximate_pose_timeout = configured(
            "localization_approximate_pose_timeout_seconds", 20.0
        )
        self.global_rotate_delay = configured(
            "global_localization_rotate_delay_seconds", 5.0,
            "GLOBAL_LOCALIZATION_ROTATE_DELAY_SECONDS",
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
        self.global_scan_map_threshold = max(
            self.scan_map_threshold,
            configured("localization_global_scan_map_minimum_score", 0.80),
        )
        self.scan_map_minimum_beams = int(self.declare_parameter(
            "scan_map_minimum_valid_beams", 25
        ).value)
        self.scan_map_maximum_beams = int(self.declare_parameter(
            "scan_map_maximum_beams", 90
        ).value)
        self.scan_map_minimum_range = configured("scan_map_minimum_range", 0.20)
        self.scan_map_maximum_range = configured("scan_map_maximum_range", 6.0)
        self.localization_coarse_match_tolerance = configured(
            "localization_coarse_match_tolerance", 0.12
        )
        self.localization_final_max_median_residual = configured(
            "localization_final_max_median_residual", 0.05
        )
        self.localization_final_max_p90_residual = configured(
            "localization_final_max_p90_residual", 0.08
        )
        self.planning_static_match_tolerance = configured(
            "planning_static_match_tolerance", 0.08
        )
        self.dynamic_overlay_static_tolerance = configured(
            "dynamic_overlay_static_tolerance", 0.08
        )
        self.scan_map_freshness = configured("scan_map_freshness_seconds", 0.60)
        self.sensor_time_invalid_grace = configured(
            "sensor_time_invalid_grace_seconds", 1.0
        )
        self.rotation_minimum_obstacle_distance = configured(
            "localization_rotation_minimum_obstacle_distance", 0.25
        )
        self.global_heading_bin_count = max(4, int(self.declare_parameter(
            "localization_global_heading_bin_count", 8
        ).value))
        self.global_minimum_heading_bins = max(2, int(self.declare_parameter(
            "localization_global_min_heading_bins", 4
        ).value))
        self.global_minimum_heading_span = math.radians(configured(
            "localization_global_min_heading_span_degrees", 180.0
        ))
        # Kept as an observable physical-sweep diagnostic and max-sweep guard;
        # READY uses actual scan heading bins/span below, not commanded angle.
        self.global_observation_minimum_rotation = self.global_minimum_heading_span
        self.localization_evidence_headings: list[float] = []
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
            "footprint_half_length", 0.20
        ).value)
        self.footprint_half_width = float(self.declare_parameter(
            "footprint_half_width", 0.18
        ).value)
        self.footprint = [
            {"x": self.footprint_half_length, "y": self.footprint_half_width},
            {"x": self.footprint_half_length, "y": -self.footprint_half_width},
            {"x": -self.footprint_half_length, "y": -self.footprint_half_width},
            {"x": -self.footprint_half_length, "y": self.footprint_half_width},
        ]
        self.planning_footprint_padding = float(self.declare_parameter(
            "planning_footprint_padding", 0.01
        ).value)
        self.corridor_side_margin = configured("corridor_side_margin", 0.06)
        self.corridor_front_clearance = configured(
            "corridor_front_clearance", 0.20
        )
        self.corridor_lookahead = configured("corridor_lookahead", 0.80)
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
        self.footprint_clearance = float(os.getenv("GOAL_CLEARANCE_METERS", "0.15"))
        self.goal_snap_max_distance = float(
            os.getenv("GOAL_SNAP_MAX_DISTANCE_METERS", "0.45")
        )
        self.latest_global_path: list[dict[str, float]] = []
        self.latest_dynamic_obstacles: list[dict[str, float]] = []
        self.latest_global_costmap: OccupancyGrid | None = None
        self.latest_static_map: OccupancyGrid | None = None
        self.last_global_costmap_monotonic = 0.0
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
        )
        self.last_corridor_log_monotonic = 0.0
        self.failed_segments: list[dict[str, Any]] = []
        self.navigation_recovery_attempts = 0
        self.navigation_corridor_clear_retried = False
        self.navigation_original_path_length = 0.0
        self.safety_stop_source = "UNKNOWN"
        self.last_logged_stop_source = ""

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
        self.last_scan_filter_log_monotonic = 0.0
        self.last_localization_candidate_log_monotonic = 0.0
        self.declare_parameter(
            "cmd_vel_debug_enabled",
            self.navigation_debug_enabled and self.speed_profiles.debug_enabled,
        )
        self.declare_parameter(
            "cmd_vel_debug_throttle_seconds",
            self.speed_profiles.debug_throttle_seconds,
        )

        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
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
            NavigationPath, "/plan", self._path_callback, 1
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_callback, 1
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._global_costmap_callback,
            1,
        )
        self.create_subscription(
            LaserScan, "/scan/normalized", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/sensors/time_status", self._sensor_time_callback, 1
        )
        self.create_subscription(String, "/safety/health", self._safety_callback, 1)
        self.create_subscription(
            String, "/safety/stop_source", self._safety_source_callback, 1
        )
        self.create_subscription(Bool, "/safety/estop", self._estop_callback, 1)
        self.create_subscription(
            UInt8, "/safety/directional_mask", self._direction_mask_callback, 1
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
        self._record_pipeline("controller_requested", message, now=now)
        profile = self.speed_profiles.get(self.auto_speed_mode)
        linear, angular, reasons = self.profile_limiter.apply(
            message.linear.x,
            message.angular.z,
            profile,
            now,
        )
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
        pose_x = float(self.pose.get("x", 0.0))
        pose_y = float(self.pose.get("y", 0.0))
        for point in self.latest_global_path:
            delta_x = float(point["x"]) - pose_x
            delta_y = float(point["y"]) - pose_y
            if math.hypot(delta_x, delta_y) < 0.08:
                continue
            target = math.atan2(delta_y, delta_x)
            current = float(self.pose.get("yaw", 0.0))
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
            "ready_evidence_hold_ms": (
                None if self.ready_evidence_since is None else round(
                    max(0.0, time.monotonic() - self.ready_evidence_since) * 1000.0,
                    1,
                )
            ),
            "global_observation": {
                "requires_rotation": self.global_search_requires_rotation,
                "accumulated_rotation_degrees": round(
                    math.degrees(self.rotation_angle), 1
                ),
                "minimum_rotation_degrees": round(
                    math.degrees(self.global_observation_minimum_rotation), 1
                ),
                "heading_bins_observed": list(self.localization_heading_bins),
                "heading_bin_count": self.global_heading_bin_count,
                "minimum_heading_bins": self.global_minimum_heading_bins,
                "heading_span_degrees": round(
                    math.degrees(self.localization_heading_span), 1
                ),
                "minimum_heading_span_degrees": round(
                    math.degrees(self.global_minimum_heading_span), 1
                ),
                "sufficient": (
                    not self.global_search_requires_rotation
                    or (
                        len(self.localization_heading_bins)
                        >= self.global_minimum_heading_bins
                        and self.localization_heading_span
                        >= self.global_minimum_heading_span
                    )
                ),
            },
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
            sensor_time_healthy = self._critical_sensor_time_healthy()
            navigation_runtime_ready = (
                self.mode == "NAVIGATION"
                and self.map_load_client.service_is_ready()
                and self.compute_path_client.server_is_ready()
                and self.navigate_client.server_is_ready()
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
                } if self.mode == "MAPPING" else None,
            }

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
                    "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING",
                    "LOCALIZING_SETTLING",
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
                # An operator-forced rescan deliberately discards the old
                # route/mission and AMCL hypothesis. The map-coordinate goal
                # remains owned by Center so it can be planned again after
                # localization returns READY.
                self.paused_goal = None
                self.current_mission_id = ""
                if self.latest_global_path:
                    self.latest_global_path = []
                    self.visualization_revision += 1
                self.localization_rotation_authorized = True
                self.localization_started_monotonic = time.monotonic()
                self._start_global_localization()
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
                # Preserve a recent-pose/no-motion verification already in
                # progress. Auto Go only authorizes its later global fallback.
                self.localization_rotation_authorized = (
                    self.localization_rotation_authorized or allow_rotation
                )
            elif self.last_amcl_pose is not None:
                # Even when Auto Go authorizes a later rotation, first verify
                # the existing AMCL cloud without moving the chassis.
                self._begin_localization_verification(
                    allow_rotation=allow_rotation
                )
            else:
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
        if command == "navigation.resume":
            self._validate_command_map(payload)
            if self.paused_goal is None:
                raise AdapterError("STATE_CONFLICT", "No paused goal to resume")
            return self._navigate(self.paused_goal, {
                "map_id": self.map_id, "version": self.map_version,
            })
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
                "pose": self.pose,
                "visualization": {
                    "revision": self.visualization_revision,
                    "map_id": self.map_id,
                    "map_version": self.map_version,
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
                self.scan_map_score = 0.0
                self.scan_map_median_residual = math.inf
                self.scan_map_p90_residual = math.inf
                self.scan_map_mean_residual = math.inf
                if self.global_search_requires_rotation:
                    self.localization_evidence_headings.clear()
                    self.localization_heading_bins = ()
                    self.localization_heading_span = 0.0
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
            and self.current_goal_handle is not None
            and self.motion_owner == "NAVIGATION"
        )

    def _sensor_entry(self, name: str) -> dict[str, Any]:
        sensors = self.sensor_time_status.get("sensors")
        if not isinstance(sensors, dict):
            return {}
        entry = sensors.get(name)
        return entry if isinstance(entry, dict) else {}

    def _critical_sensor_time_healthy(self) -> bool:
        if time.monotonic() - self.last_sensor_time_status_monotonic > 0.60:
            return False
        if self.sensor_time_status.get("clock_state") != "SYNCED":
            return False
        return all(
            bool(self._sensor_entry(name).get("arrival_fresh"))
            and bool(self._sensor_entry(name).get("timestamp_valid"))
            and bool(self._sensor_entry(name).get("frame_valid"))
            for name in ("scan", "odom")
        )

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
        # Endpoint-to-static-map matching is strong stationary verification,
        # but during travel it also penalizes people and temporary obstacles.
        # AMCL covariance plus fresh scan/odom/TF timestamps remain mandatory;
        # do not turn a controller collision into a false localization loss.
        confidence_scan_map_score = (
            max(fresh_scan_map_score, self.scan_map_threshold)
            if navigation_in_progress
            else fresh_scan_map_score
        )
        self.localization_confidence = localization_confidence(
            self.last_amcl_covariance,
            stability_score=stability_score,
            scan_map_score=confidence_scan_map_score,
            scan_map_threshold=self.scan_map_threshold,
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
        if path != self.latest_global_path:
            had_previous_path = bool(self.latest_global_path)
            previous_path = list(self.latest_global_path)
            self.latest_global_path = path
            if had_previous_path and not math.isfinite(
                self.last_replan_obstacle_distance
            ):
                self.last_replan_obstacle_distance = self.nearest_forward_obstacle
            self.visualization_revision += 1
            if had_previous_path and self.current_state == "NAVIGATING":
                recoveries = int(self.latest_feedback.get("recoveries", 0) or 0)
                reason = (
                    "RECOVERY"
                    if recoveries > 0
                    else "DYNAMIC_OBSTACLE"
                    if self.nearest_forward_obstacle < 0.8
                    else "COSTMAP_CHANGED"
                )
                self._nav_debug(
                    "REPLAN",
                    reason=reason,
                    old_path_length=self._path_length(previous_path),
                    new_path_length=self._path_length(path),
                    nearest_forward_obstacle=self._finite_metric(
                        self.nearest_forward_obstacle
                    ),
                )

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
        obstacles = compact_lethal_cells(message)
        source_frame = str(message.header.frame_id or "map")
        if source_frame != "map":
            try:
                transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
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
            except TransformException:
                # Never draw or validate against odom-frame cells as though
                # they were map-frame coordinates.
                return
        # The black Saved Map already displays permanent walls. Sending the
        # same LiDAR returns again as red "dynamic obstacles" made open routes
        # look blocked and also made goal validation overly conservative.
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
        if obstacles != self.latest_dynamic_obstacles:
            self.latest_dynamic_obstacles = obstacles
            self.visualization_revision += 1

    def _global_costmap_callback(self, message: OccupancyGrid) -> None:
        if int(message.info.width) <= 0 or int(message.info.height) <= 0:
            return
        self.latest_global_costmap = message
        self.last_global_costmap_monotonic = time.monotonic()
        self.global_costmap_update.set()

    def _publish_failed_segments(self) -> None:
        source = self.latest_static_map
        if source is None:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info = source.info
        width = int(source.info.width)
        height = int(source.info.height)
        message.data = [0] * (width * height)
        resolution = float(source.info.resolution)
        if self.saved_map is None or resolution <= 0:
            self.failed_segment_mask.publish(message)
            return
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
                event="EXPIRED",
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
        if all(item.can_go_straight for item in assessments):
            return "CORRIDOR_CLEAR", assessments[-1]
        if all(
            not item.can_go_straight
            and (
                item.available_width < item.required_width
                or item.front_clearance <= self.corridor_front_clearance
            )
            for item in assessments
        ):
            reason = (
                "INSUFFICIENT_CLEARANCE"
                if assessments[-1].available_width < assessments[-1].required_width
                else "CONFIRMED_FRONT_OBSTACLE"
            )
            return reason, assessments[-1]
        return "UNCONFIRMED", assessments[-1]

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
        """Resolve a scan frame at capture time; never substitute stale AMCL."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                str(message.header.frame_id),
                Time.from_msg(message.header.stamp),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )

    def _scan_heading_in_odom(self, message: LaserScan) -> float | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                Time.from_msg(message.header.stamp),
            )
        except TransformException:
            return None
        return self._yaw_from_quaternion(transform.transform.rotation)

    def _update_scan_map_match(self, message: LaserScan) -> None:
        if self.saved_map is None:
            return
        scan_pose = self._scan_transform("map", message)
        if scan_pose is None:
            return
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
        self.scan_map_matched_beams = match.matched_beams
        self.scan_map_valid_beams = match.valid_beams
        self.scan_map_residual_beams = match.residual_beams
        if match.valid_beams < self.scan_map_minimum_beams:
            return
        self.scan_map_scores.append(match.score)
        self.scan_map_median_residuals.append(match.median_residual)
        self.scan_map_p90_residuals.append(match.p90_residual)
        self.scan_map_mean_residuals.append(match.mean_residual)
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
        self.last_scan_map_monotonic = time.monotonic()
        if self.verification_started_monotonic:
            self.verification_scan_count += 1
        if (
            self.global_search_requires_rotation
            and match.score >= self.global_scan_map_threshold
            and match.median_residual
            <= self.localization_final_max_median_residual
            and match.p90_residual <= self.localization_final_max_p90_residual
        ):
            heading = self._scan_heading_in_odom(message)
            if heading is not None:
                self.localization_evidence_headings.append(heading)
                diversity = heading_diversity(
                    self.localization_evidence_headings,
                    bin_count=self.global_heading_bin_count,
                )
                self.localization_heading_bins = diversity.observed_bins
                self.localization_heading_span = diversity.span_radians
        self._refresh_localization_confidence()

    def _planning_scan_message(self, message: LaserScan) -> LaserScan:
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
        now = time.monotonic()
        if (
            self.navigation_debug_enabled
            and now - self.last_scan_filter_log_monotonic >= 1.0
        ):
            self._nav_debug(
                "SCAN_FILTER",
                **self.last_scan_filter_stats,
                tolerance_m=self.planning_static_match_tolerance,
                tf_timestamp="SCAN_CAPTURE_TIME",
            )
            self.last_scan_filter_log_monotonic = now
        return planning

    def _scan_callback(self, message: LaserScan) -> None:
        callback_started = time.monotonic()
        self.last_scan_monotonic = callback_started
        nearest_forward = math.inf
        nearest_left = math.inf
        nearest_right = math.inf
        nearest_rotation_obstacle = math.inf
        extrinsic = self._laser_extrinsic(str(message.header.frame_id))
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
        corridor = evaluate_corridor(
            path_points,
            half_length=self.footprint_half_length,
            half_width=self.footprint_half_width,
            side_margin=self.corridor_side_margin,
            front_clearance_required=self.corridor_front_clearance,
            lookahead=self.corridor_lookahead,
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
                required_width=corridor.required_width,
                front_clearance=self._finite_metric(corridor.front_clearance),
                linear_cmd=None if requested is None else requested[0],
                angular_cmd=None if requested is None else requested[1],
                heading_error_deg=math.degrees(path_error),
                can_go_straight=(
                    corridor.can_go_straight and abs(path_error) <= math.radians(20)
                ),
                can_rotate=corridor.can_rotate,
            )
            self.last_corridor_log_monotonic = callback_started
        self._update_scan_map_match(message)
        if self.mode == "MAPPING" and self.current_state in {"MAPPING", "MAPPING_RUNNING"}:
            self.mapping_scan.publish(message)
        elif self.mode == "NAVIGATION":
            self.navigation_scan.publish(message)
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
        self.safety_health = message.data

    def _safety_source_callback(self, message: String) -> None:
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
        self._nav_debug(
            "STOP",
            reason=(
                "OBSTACLE"
                if "OBSTACLE" in source or source == "MOTION_SAFETY"
                else "SAFETY"
            ),
            source=source,
            direction_mask=self.safety_direction_mask,
            linear_cmd=None if requested is None else requested[0],
            angular_cmd=None if requested is None else requested[1],
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
                if self.current_state == "NAVIGATING":
                    self._set_state("CANCELED", "emergency_stop")
            self.navigation_velocity.publish(Twist())
            if handle is not None:
                handle.cancel_goal_async()

    def _direction_mask_callback(self, message: UInt8) -> None:
        self.safety_direction_mask = int(message.data)

    def _manual_takeover_callback(self, message: Bool) -> None:
        if message.data:
            self.last_manual_takeover_monotonic = time.monotonic()
            self._stop_localization_rotation()
        active_navigation = (
            message.data
            and self.mode == "NAVIGATION"
            and self.current_state in {"NAVIGATING", "PAUSED", "BLOCKED"}
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
                self._set_state("CANCELED", "manual_takeover")
                self.navigation_goal_generation += 1
                self.paused_goal = None
                self.latest_global_path = []
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

    def _update_pose(self) -> None:
        if (
            self.mode == "NAVIGATION"
            and self.current_state == "STARTING"
            and self.map_load_client.service_is_ready()
            and self.compute_path_client.server_is_ready()
            and self.navigate_client.server_is_ready()
        ):
            with self.state_lock:
                # Nav2 processes being healthy does not mean a saved map is
                # loaded. Reporting READY here made Center treat a registry
                # assignment as an active map after every stack restart.
                self._set_state("NO_ACTIVE_MAP", "nav2_ready_without_active_map")
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
            "x": transform.transform.translation.x,
            "y": transform.transform.translation.y,
            "yaw": yaw,
        }
        if (
            (self.mode != "MAPPING" or self.current_state in {"MAPPING", "MAPPING_RUNNING"})
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
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
        self.localization_evidence_headings.clear()
        self.localization_heading_bins = ()
        self.localization_heading_span = 0.0
        self.last_scan_map_monotonic = 0.0
        self.verification_scan_count = 0
        self.verification_started_monotonic = 0.0
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None
        self.low_confidence_since = None
        self.last_nomotion_request_monotonic = 0.0

    def _begin_localization_verification(self, *, allow_rotation: bool) -> None:
        """Verify the current AMCL cloud without resetting its particles."""
        now = time.monotonic()
        self._stop_localization_rotation()
        self.localized = False
        self.localization_state = "VERIFYING"
        self._set_state("VERIFYING", "verify_existing_amcl_pose")
        self.localization_started_monotonic = now
        self.localization_phase_started_monotonic = now
        self.verification_started_monotonic = now
        self.verification_scan_count = 0
        self.localization_rotation_authorized = allow_rotation
        self.global_search_requires_rotation = False
        self.last_nomotion_request_monotonic = 0.0
        self.low_confidence_since = None
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None

    def _begin_auto_localization(self, last_pose: Any) -> None:
        now = time.monotonic()
        self._reset_localization_evidence()
        self.localization_rotation_authorized = False
        self.localization_started_monotonic = now
        self.localization_phase_started_monotonic = now
        self.rotation_angle = 0.0
        self.rotation_active = False
        self.localization_seed_pose = None
        self.localization_seed_approximate = False
        self.global_search_requires_rotation = False
        if isinstance(last_pose, dict) and all(
            math.isfinite(float(last_pose.get(axis, 0.0))) for axis in ("x", "y", "yaw")
        ):
            self.localization_seed_pose = dict(last_pose)
            recent_verified_pose = (
                str(last_pose.get("source", "")) == "recent_navigation_pose"
                and 0.0 <= time.time() - float(last_pose.get("timestamp", 0.0)) <= 3600.0
                and float(last_pose.get("covariance", 1.0)) <= 0.25
            )
            # A recent navigation pose already survived 30 seconds of scan/map,
            # covariance, stability and sensor-clock gates before persistence.
            # Recheck it locally with fresh scans and its bounded heading first;
            # a mismatch times out into the multi-heading global search below.
            # Mapping terminal poses and legacy records remain broad hints.
            self.localization_seed_approximate = not recent_verified_pose
            self.global_search_requires_rotation = False
            self._publish_initial_pose(
                self.localization_seed_pose,
                approximate=self.localization_seed_approximate,
            )
            self.localization_state = "LOCALIZING_LAST_POSE"
            self._set_state("LOCALIZING_LAST_POSE", "recent_pose_seed")
        else:
            # Loading a map or opening Control must not start active global
            # localization. Auto Go (or the explicit UI action) authorizes it.
            self.localization_state = "LOCALIZATION_REQUIRED"
            self._set_state("LOCALIZATION_REQUIRED", "global_rotation_not_authorized")

    def _start_global_localization(self) -> None:
        self._stop_localization_rotation()
        if not self.localization_rotation_authorized:
            # Never enter a state whose READY gate requires rotation while
            # velocity ownership forbids it. Expose the required operator/Auto
            # action immediately instead of waiting for the 45-second timeout.
            self.global_search_requires_rotation = False
            self.localization_state = "LOCALIZATION_REQUIRED"
            self._set_state("LOCALIZATION_REQUIRED", "global_rotation_not_authorized")
            return
        self._reset_localization_evidence()
        self.localization_settling_evidence_started = False
        self.rotation_angle = 0.0
        self.localization_seed_pose = None
        self.localization_seed_approximate = False
        # A stationary scan on a repetitive indoor map can produce several
        # low-covariance, high endpoint-score poses. Global search therefore
        # needs observations from more than one heading before it can be READY.
        self.global_search_requires_rotation = True
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
        self.localization_state = "LOCALIZING_GLOBAL"
        self._set_state("LOCALIZING_GLOBAL", "global_localization_started")
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
        return (
            time.monotonic() - self.last_scan_monotonic <= 0.30
            and self._critical_sensor_time_healthy()
            and self.tf_buffer.can_transform(
                "base_footprint", "laser_frame", Time()
            )
            and self.safety_health.startswith("HEALTHY")
            and not self.estop_active
            and self.safety_direction_mask == 0
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

    def _degrade_localization(self, reason: str) -> None:
        if self.localization_state == "SENSOR_TIME_INVALID":
            return
        self.get_logger().error(f"localization degraded; stopping Nav2: {reason}")
        self.localized = False
        self.localization_state = "SENSOR_TIME_INVALID"
        self._set_state("SENSOR_TIME_INVALID", "sensor_time_invalid")
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.navigation_goal_generation += 1
        if handle is not None:
            handle.cancel_goal_async()
        self.motion_owner = "NONE"
        self.localization_velocity.publish(Twist())
        self.latest_global_path = []
        self.visualization_revision += 1
        self.sensor_time_invalid_since = time.monotonic()

    def _localization_lost(self, reason: str) -> None:
        if not self.localized:
            return
        self.get_logger().error(f"localization lost; stopping Nav2: {reason}")
        self.localized = False
        self.localization_state = "LOCALIZATION_LOST"
        self._set_state("LOCALIZATION_LOST", "localization_evidence_lost")
        handle = self.current_goal_handle
        self.current_goal_handle = None
        self.navigation_goal_generation += 1
        if handle is not None:
            handle.cancel_goal_async()
        self.motion_owner = "NONE"
        self.localization_velocity.publish(Twist())
        self.latest_global_path = []
        self.visualization_revision += 1
        try:
            # Preserve an explicit operator rotation authorization across a
            # failed verification; passive session checks remain passive.
            self.localization_started_monotonic = time.monotonic()
            self._start_global_localization()
        except AdapterError as exc:
            self.get_logger().error(str(exc))

    def _localization_evidence_ready(self, now: float) -> bool:
        required_scan_score = (
            self.global_scan_map_threshold
            if self.global_search_requires_rotation
            else self.scan_map_threshold
        )
        return (
            self.localization_confidence >= self.localization_confidence_threshold
            and self._pose_is_stable()
            and now - self.last_amcl_monotonic <= self.amcl_pose_freshness
            and self.scan_map_valid_beams >= self.scan_map_minimum_beams
            and self.scan_map_score >= required_scan_score
            and self.scan_map_residual_beams > 0
            and self.scan_map_median_residual
            <= self.localization_final_max_median_residual
            and self.scan_map_p90_residual
            <= self.localization_final_max_p90_residual
            and now - self.last_scan_map_monotonic <= self.scan_map_freshness
            and now - self.last_scan_monotonic <= 0.30
            and now - self.last_map_tf_monotonic <= 0.60
            and self._critical_sensor_time_healthy()
            and (
                not self.global_search_requires_rotation
                or (
                    len(self.localization_heading_bins)
                    >= self.global_minimum_heading_bins
                    and self.localization_heading_span
                    >= self.global_minimum_heading_span
                )
            )
        )

    def _localization_rejection_reason(self, now: float) -> str:
        required_scan_score = (
            self.global_scan_map_threshold
            if self.global_search_requires_rotation
            else self.scan_map_threshold
        )
        if self.scan_map_valid_beams < self.scan_map_minimum_beams:
            return "INSUFFICIENT_VALID_BEAMS"
        if self.scan_map_score < required_scan_score:
            return "SCAN_MATCH_SCORE_TOO_LOW"
        if self.scan_map_residual_beams <= 0:
            return "NO_ENDPOINT_RESIDUALS"
        if self.scan_map_median_residual > self.localization_final_max_median_residual:
            return "MEDIAN_RESIDUAL_TOO_HIGH"
        if self.scan_map_p90_residual > self.localization_final_max_p90_residual:
            return "P90_RESIDUAL_TOO_HIGH"
        if self.global_search_requires_rotation and (
            len(self.localization_heading_bins) < self.global_minimum_heading_bins
            or self.localization_heading_span < self.global_minimum_heading_span
        ):
            return "INSUFFICIENT_HEADING_DIVERSITY"
        if not self._pose_is_stable():
            return "POSE_UNSTABLE"
        if self.localization_confidence < self.localization_confidence_threshold:
            return "LOW_CONFIDENCE"
        if now - self.last_amcl_monotonic > self.amcl_pose_freshness:
            return "AMCL_STALE"
        if now - self.last_scan_map_monotonic > self.scan_map_freshness:
            return "SCAN_MAP_EVIDENCE_STALE"
        if not self._critical_sensor_time_healthy():
            return "SENSOR_TIME_INVALID"
        return "EVIDENCE_HOLD_PENDING"

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

    def _begin_localization_settling(self, now: float) -> None:
        """Stop after the global sweep and wait for stationary AMCL samples."""
        self._stop_localization_rotation()
        self.localization_state = "LOCALIZING_SETTLING"
        self._set_state("LOCALIZING_SETTLING", "rotation_sweep_complete")
        self.localization_phase_started_monotonic = now
        self.localization_settling_evidence_started = False

    def _start_localization_settling_evidence(self) -> None:
        """Discard moving samples without resetting AMCL's particle cloud."""
        self.localization_confidence = 0.0
        self.last_amcl_monotonic = 0.0
        self.pose_window.clear()
        self.pose_stability_metrics = pose_stability(())
        self.scan_map_scores.clear()
        self.scan_map_median_residuals.clear()
        self.scan_map_p90_residuals.clear()
        self.scan_map_mean_residuals.clear()
        self.scan_map_score = 0.0
        self.scan_map_matched_beams = 0
        self.scan_map_valid_beams = 0
        self.scan_map_residual_beams = 0
        self.scan_map_median_residual = math.inf
        self.scan_map_p90_residual = math.inf
        self.scan_map_mean_residual = math.inf
        self.last_scan_map_monotonic = 0.0
        self.ready_evidence_since = None
        self.ready_evidence_invalid_since = None
        self.low_confidence_since = None
        self.last_nomotion_request_monotonic = 0.0
        self.localization_settling_evidence_started = True

    def _nomotion_update_due(
        self,
        now: float,
        *,
        navigation_in_progress: bool,
    ) -> bool:
        """Keep AMCL publishing when an active controller is safely stationary.

        AMCL normally publishes often enough while odometry crosses its motion
        thresholds. A controller collision can leave a NavigateToPose action
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

    def _localization_tick(self) -> None:
        if self.mode != "NAVIGATION":
            return
        now = time.monotonic()
        if self.map_id and not self._critical_sensor_time_healthy():
            if self.sensor_time_invalid_since is None:
                self.sensor_time_invalid_since = now
            if (
                now - self.sensor_time_invalid_since
                >= self.sensor_time_invalid_grace
                and self.localization_state not in {"IDLE", "SENSOR_TIME_INVALID"}
            ):
                self._degrade_localization(
                    "scan/odometry source clock is not synchronized"
                )
            return
        if self._critical_sensor_time_healthy():
            self.sensor_time_invalid_since = None
            if self.localization_state == "SENSOR_TIME_INVALID":
                # Keep the AMCL cloud. Fresh synchronized sensor evidence must
                # verify it before navigation resumes.
                if self.last_amcl_pose is not None:
                    self._begin_localization_verification(
                        allow_rotation=self.localization_rotation_authorized
                    )
                elif self.localization_seed_pose is not None:
                    # map.load can legitimately run before the USB MCU has
                    # rejoined after a Pi reboot.  _degrade_localization keeps
                    # the verified last-known pose, so replay that bounded
                    # seed once scan + odometry become healthy instead of
                    # discarding it into an unauthorized global search.  This
                    # is stationary: only AMCL's /initialpose is republished.
                    seed_pose = dict(self.localization_seed_pose)
                    self._begin_auto_localization(seed_pose)
                else:
                    self.localization_started_monotonic = now
                    try:
                        self._start_global_localization()
                    except AdapterError as exc:
                        self.get_logger().error(str(exc))
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
            return
        localizing_states = {
            "LOCALIZATION_INITIALIZING", "LOCALIZING_LAST_POSE", "LOCALIZING_GLOBAL",
            "LOCALIZING_APPROXIMATE_POSE", "LOCALIZING_ROTATING",
            "LOCALIZING_SETTLING", "LOW_CONFIDENCE", "LOCALIZATION_LOST", "VERIFYING",
            # A failed attempt is terminal for automatic rotation, but AMCL
            # must still be allowed to recover after an operator moves the
            # robot manually and supplies enough fresh, stable scans.
            "LOCALIZATION_FAILED",
        }
        if self.localization_state not in localizing_states:
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
        localization_ready = self._localization_evidence_ready(now)
        if (
            self.navigation_debug_enabled
            and now - self.last_localization_candidate_log_monotonic >= 1.0
        ):
            self._nav_debug(
                "LOCALIZATION_VERIFY",
                state=self.localization_state,
                candidate_pose=self.last_amcl_pose,
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
                scan_score_required=(
                    self.global_scan_map_threshold
                    if self.global_search_requires_rotation
                    else self.scan_map_threshold
                ),
                median_residual_m=self._finite_metric(
                    self.scan_map_median_residual
                ),
                p90_residual_m=self._finite_metric(
                    self.scan_map_p90_residual
                ),
                mean_residual_m=self._finite_metric(
                    self.scan_map_mean_residual
                ),
                heading_bins=len(self.localization_heading_bins),
                heading_bin_ids=list(self.localization_heading_bins),
                heading_span_deg=round(
                    math.degrees(self.localization_heading_span), 1
                ),
                rotation_degrees=round(math.degrees(self.rotation_angle), 1),
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
            self._stop_localization_rotation()
            self.localized = True
            self.localization_state = "READY"
            self._set_state("READY", "localization_verified")
            self._nav_debug(
                "LOCALIZATION_VERIFY",
                candidate_pose=self.last_amcl_pose,
                scan_score=self.scan_map_score,
                median_residual_m=self._finite_metric(
                    self.scan_map_median_residual
                ),
                p90_residual_m=self._finite_metric(
                    self.scan_map_p90_residual
                ),
                heading_bins=len(self.localization_heading_bins),
                heading_span_deg=round(
                    math.degrees(self.localization_heading_span), 1
                ),
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
                scan_score_required=(
                    self.global_scan_map_threshold
                    if self.global_search_requires_rotation
                    else self.scan_map_threshold
                ),
            )
            self.low_confidence_since = None
            self.localization_seed_pose = None
            self.localization_rotation_authorized = False
            self.global_search_requires_rotation = False
            self.verification_started_monotonic = 0.0
            self.verification_scan_count = 0
            self.ready_evidence_invalid_since = None
            return
        if (
            self.localization_state == "VERIFYING"
            and now - self.localization_phase_started_monotonic
            >= self.localization_verify_timeout
        ):
            # Verification never resets AMCL while evidence is good. A
            # bounded failure is the point where a real global search begins.
            try:
                self._start_global_localization()
            except AdapterError as exc:
                self.get_logger().error(str(exc))
            return
        if now - self.localization_started_monotonic >= self.localization_timeout:
            self._stop_localization_rotation()
            self.localized = False
            self.localization_state = "LOCALIZATION_FAILED"
            self._set_state("LOCALIZATION_FAILED", "localization_timeout")
            return
        broad_seed = (
            self.localization_state == "LOCALIZING_APPROXIMATE_POSE"
            or (
                self.localization_state == "LOCALIZING_LAST_POSE"
                and self.localization_seed_approximate
            )
        )
        seed_timeout = (
            self.approximate_pose_timeout if broad_seed else self.last_pose_timeout
        )
        if (
            self.localization_state in {
                "LOCALIZING_LAST_POSE", "LOCALIZING_APPROXIMATE_POSE",
            }
            and now - self.localization_phase_started_monotonic >= seed_timeout
        ):
            try:
                self._start_global_localization()
            except AdapterError as exc:
                self.get_logger().error(str(exc))
            return
        if self.localization_state in {"LOCALIZING_GLOBAL", "LOW_CONFIDENCE", "LOCALIZATION_LOST"}:
            if now - self.localization_phase_started_monotonic < self.global_rotate_delay:
                return
            if not self.localization_rotation_authorized:
                # Keep evaluating passive AMCL updates until timeout. Merely
                # opening Control must never move an unchecked robot.
                return
            if not self._safe_to_rotate():
                # Do not claim that the chassis is rotating while a live
                # safety gate is withholding velocity ownership.
                return
            self.localization_state = "LOCALIZING_ROTATING"
            self._set_state("LOCALIZING_ROTATING", "global_search_needs_new_heading")
        if self.localization_state == "LOCALIZING_ROTATING":
            if self.rotation_angle >= self.rotation_max_angle:
                # Completing the sweep is success of the observation phase,
                # not a localization failure. Verify fresh stationary samples
                # without destroying AMCL's converged particle cloud.
                self._begin_localization_settling(now)
                return
            if not self._safe_to_rotate():
                self._stop_localization_rotation()
                self.localization_state = "LOCALIZING_GLOBAL"
                self._set_state("LOCALIZING_GLOBAL", "rotation_safety_gate_closed")
                return
            delta = now - self.rotation_last_monotonic if self.rotation_last_monotonic else 0.0
            self.rotation_angle += abs(self.rotation_speed) * max(0.0, min(delta, 0.5))
            command = Twist()
            command.angular.z = self.rotation_speed
            self.motion_owner = "LOCALIZATION"
            self.localization_velocity.publish(command)
            self.rotation_active = True
            self.rotation_last_monotonic = now

    def _load_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_yaml = Path(str(payload["map_path"])) / "map.yaml"
        if not map_yaml.is_file():
            raise AdapterError("MAP_MISSING", "map.yaml is missing from verified cache")
        try:
            candidate_grid = SavedOccupancyMap.load(map_yaml)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise AdapterError("MAP_INVALID", f"Saved map artifact is invalid: {exc}") from exc
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            raise AdapterError("MAP_SERVER_UNAVAILABLE", "Map Server load service unavailable")
        previous_path = self.active_map_path
        previous_identity = (self.map_id, self.map_version)
        previous_grid = self.saved_map
        previous_localized = self.localized
        previous_localization_state = self.localization_state

        def rollback_map() -> None:
            self._stop_localization_rotation()
            if previous_path is None or not previous_path.is_file():
                self.map_id = ""
                self.map_version = 0
                self.saved_map = None
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
            self.failed_segments = []
            self._publish_failed_segments()
            self.active_map_path = map_yaml
            self.localized = False
            self.initial_pose_requested = False
            self.current_mission_id = ""
            self.latest_global_path = []
            self.latest_dynamic_obstacles = []
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
        self._begin_auto_localization(payload.get("last_known_pose"))
        return {
            "status": "completed",
            "current_state": self.current_state,
            "progress_percent": 100,
            "state": self._state(),
        }

    def _set_initial_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = dict(payload["pose"])
        self._validate_command_map(payload)
        if self.saved_map is None or self.saved_map.world_to_cell(float(pose["x"]), float(pose["y"])) is None:
            raise AdapterError("POSE_OUTSIDE_MAP", "Vị trí gần đúng nằm ngoài bản đồ")
        # Never let confidence accumulated around a false, locally stable AMCL
        # hypothesis immediately validate an operator-supplied correction.
        # Require fresh AMCL samples around the new pose before returning READY.
        self._reset_localization_evidence()
        self.localization_rotation_authorized = False
        self.localization_started_monotonic = time.monotonic()
        self.localization_phase_started_monotonic = self.localization_started_monotonic
        # This is deliberately a broad search hint, not the robot pose. AMCL
        # must move/refine the estimate using current LiDAR data. If the hint is
        # wrong it falls back to global localization after the bounded hint
        # phase instead of pinning the marker to the operator's click.
        operator_pose = dict(pose)
        self.localization_seed_pose = operator_pose
        self.localization_seed_approximate = True
        self.global_search_requires_rotation = False
        self._publish_initial_pose(operator_pose, approximate=True)
        self.localization_state = "LOCALIZING_APPROXIMATE_POSE"
        self._set_state("LOCALIZING_APPROXIMATE_POSE", "operator_pose_hint")
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
                footprint_half_length=self.footprint_half_length,
                footprint_half_width=self.footprint_half_width,
                footprint_padding=self.planning_footprint_padding,
            )
            if snapped is None:
                raise
            resolved = dict(goal)
            resolved["x"], resolved["y"] = snapped
            self._validate_goal(resolved)
            return resolved, True

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
        self._wait(
            self.clear_global_costmap_client.call_async(
                ClearEntireCostmap.Request()
            ),
            3.0,
            "GLOBAL_COSTMAP_RESET_TIMEOUT",
        )
        self.global_costmap_update.clear()
        if not self.global_costmap_update.wait(2.0):
            raise AdapterError(
                "COSTMAP_NOT_READY",
                "Global costmap did not publish an update after reset",
            )

    def _request_path_once(self, goal: dict[str, Any]) -> list[dict[str, float]]:
        request = ComputePathToPose.Goal()
        request.goal = self._goal_pose(goal)
        request.use_start = False
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

    def _compute_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        planning_started = time.monotonic()
        self._validate_command_map(payload)
        requested_goal = dict(payload["goal"])
        resolved_goal = dict(requested_goal)

        def mark_plan_failed(reason: str) -> None:
            # A preview failure occurs before motion and therefore leaves the
            # robot READY. Only an exhausted active NavigateToPose recovery may
            # enter BLOCKED.
            with self.state_lock:
                self._set_state("READY", reason)
                if self.latest_global_path:
                    self.latest_global_path = []
                    self.visualization_revision += 1

        try:
            resolved_goal, goal_adjusted = self._resolve_planning_goal(
                requested_goal
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
        # Do not reject the current pose by testing it against the saved map.
        # A mapped wall can overlap the robot by one 5 cm cell because of SLAM
        # quantization or a small localization offset, even though the current
        # LiDAR and Nav2 costmap have a valid escape corridor. Nav2 owns start
        # validity using its live, footprint-cleared costmap; goal preflight
        # above remains strict because the robot does not already occupy it.
        if not self.compute_path_client.wait_for_server(timeout_sec=5.0):
            error = AdapterError(
                "PLANNER_UNAVAILABLE",
                "ComputePathToPose action unavailable",
            )
            mark_plan_failed("planner_unavailable")
            self._record_planning_failure(
                error,
                requested_goal,
                resolved_goal,
                planning_started,
            )
            raise error
        with self.state_lock:
            # Planning creates a new Center mission only after Nav2 returns a
            # path. Do not let interim READY telemetry mutate the prior goal.
            self.current_mission_id = ""
            self._set_state("PLANNING", "manual_plan_request")
        self._nav_debug(
            "PLAN_REQUEST",
            start=dict(self.pose or {}),
            goal=resolved_goal,
            planner="ThetaStar",
            retry=0,
        )
        try:
            points = self._request_path_once(resolved_goal)
        except AdapterError as exc:
            mark_plan_failed(f"plan_failed:{exc.code}")
            self._record_planning_failure(
                exc,
                requested_goal,
                resolved_goal,
                planning_started,
            )
            raise
        if not points:
            first_error = self._classify_empty_path(resolved_goal)
            if first_error.code in {"START_BLOCKED", "COSTMAP_NOT_READY"}:
                self._nav_debug(
                    "PLAN_RETRY",
                    reason=first_error.code,
                    action="CLEAR_GLOBAL_COSTMAP_ONCE",
                )
                try:
                    self._refresh_global_costmap_for_planning()
                    self._nav_debug(
                        "PLAN_REQUEST",
                        start=dict(self.pose or {}),
                        goal=resolved_goal,
                        planner="ThetaStar",
                        retry=1,
                    )
                    points = self._request_path_once(resolved_goal)
                except AdapterError as exc:
                    mark_plan_failed(f"plan_retry_failed:{exc.code}")
                    self._record_planning_failure(
                        exc,
                        requested_goal,
                        resolved_goal,
                        planning_started,
                    )
                    raise
            if not points:
                error = self._classify_empty_path(resolved_goal)
                mark_plan_failed(f"plan_failed:{error.code}")
                self._record_planning_failure(
                    error,
                    requested_goal,
                    resolved_goal,
                    planning_started,
                )
                raise error
        distance = sum(
            math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            for a, b in zip(points, points[1:])
        )
        with self.state_lock:
            self._set_state("READY", "plan_success")
            self.latest_global_path = points
            self.visualization_revision += 1
            self.planner_latency_ms = round(
                (time.monotonic() - planning_started) * 1000.0, 3
            )
            self.last_planning_failure = {}
        self._nav_debug(
            "PLAN_RESULT",
            status="SUCCESS",
            points=len(points),
            length=round(distance, 3),
            duration_ms=self.planner_latency_ms,
            goal_adjusted=goal_adjusted,
        )
        return {
            "status": "completed",
            "current_state": "READY",
            "points": points,
            "distance_m": round(distance, 3),
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
        self._validate_goal(goal_payload)
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            raise AdapterError("NAV2_UNAVAILABLE", "NavigateToPose action unavailable")
        goal = NavigateToPose.Goal()
        goal.pose = self._goal_pose(goal_payload)
        # Every goal selects the generated tree for the current profile. This
        # changes replan/recovery timing without restarting bt_navigator.
        goal.behavior_tree = str(self.behavior_tree_paths[self.auto_speed_mode])
        with self.state_lock:
            self.navigation_goal_generation += 1
            goal_generation = self.navigation_goal_generation
            # A new Nav2 action owns a fresh, mission-local recovery count.
            # Keeping feedback from the previous goal could misclassify an
            # unrelated planner/controller failure as an obstacle blockage.
            self.latest_feedback = {"recoveries": 0}
            self.replan_timestamps = []
            self.last_slowdown_obstacle_distance = math.inf
            self.last_replan_obstacle_distance = math.inf
            if not recovery_attempt:
                self.navigation_recovery_attempts = 0
                self.navigation_corridor_clear_retried = False
                self.navigation_original_path_length = self._path_length(
                    self.latest_global_path
                )
                self.corridor_samples.clear()
        future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=(
                lambda feedback, generation=goal_generation:
                self._navigation_feedback(feedback, generation)
            ),
        )
        handle = self._wait(future, 5, "NAVIGATION_TIMEOUT")
        if not handle.accepted:
            raise AdapterError("GOAL_REJECTED", "Nav2 rejected goal")
        with self.state_lock:
            superseded = goal_generation != self.navigation_goal_generation
            if not superseded:
                self.current_goal_handle = handle
                self.paused_goal = dict(goal_payload)
                self._set_state("NAVIGATING", "navigation_goal_accepted")
                self.motion_owner = "NAVIGATION"
        if superseded:
            # Manual takeover/cancel won the race while Nav2 was accepting the
            # goal. Cancel the late handle; never resurrect autonomous motion.
            handle.cancel_goal_async()
            raise AdapterError(
                "NAVIGATION_CANCELED",
                "Navigation was canceled while Nav2 was accepting the goal",
            )
        handle.get_result_async().add_done_callback(
            lambda result_future, generation=goal_generation: self._navigation_result(
                result_future, generation
            )
        )
        return {"status": "accepted", "current_state": "NAVIGATING", "state": self._state()}

    def _navigation_feedback(self, feedback: Any, goal_generation: int) -> None:
        data = feedback.feedback
        with self.state_lock:
            if goal_generation != self.navigation_goal_generation:
                return
            self.latest_feedback = {
                "distance_remaining": float(data.distance_remaining),
                "navigation_time_seconds": data.navigation_time.sec + data.navigation_time.nanosec / 1e9,
                "recoveries": int(data.number_of_recoveries),
            }

    def _navigation_result(self, future: Any, goal_generation: int) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose result failed: {exc}")
            status = GoalStatus.STATUS_ABORTED
        retry: tuple[str, dict[str, Any], Any, list[dict[str, float]]] | None = None
        with self.state_lock:
            # Pause/resume can start a replacement action before the canceled
            # action's result callback arrives. Never let that stale callback
            # cancel or fail the newer mission attempt.
            if goal_generation != self.navigation_goal_generation:
                return
            self.current_goal_handle = None
            self.motion_owner = "NONE"
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._set_state("SUCCEEDED", "goal_reached")
                self.paused_goal = None
                self.latest_global_path = []
                self.visualization_revision += 1
            elif status == GoalStatus.STATUS_CANCELED and self.current_state == "PAUSED":
                pass
            elif status == GoalStatus.STATUS_CANCELED:
                self._set_state("CANCELED", "nav2_goal_canceled")
                self.paused_goal = None
            else:
                recoveries = int(self.latest_feedback.get("recoveries", 0) or 0)
                requested = self.pipeline_samples.get("controller_requested")
                self._nav_debug(
                    "STOP",
                    reason="NO_PROGRESS",
                    source=(
                        self.safety_stop_source
                        if self.safety_stop_source not in {"NONE", "UNKNOWN"}
                        else "CONTROLLER_COLLISION"
                    ),
                    recoveries=recoveries,
                    direction_mask=self.safety_direction_mask,
                    linear_cmd=None if requested is None else requested[0],
                    angular_cmd=None if requested is None else requested[1],
                )
                evidence_reason, corridor = self._corridor_failure_evidence()
                retry_goal = dict(self.paused_goal or {})
                old_path = list(self.latest_global_path)
                if (
                    recoveries > 0
                    and retry_goal
                    and evidence_reason == "CORRIDOR_CLEAR"
                    and not self.navigation_corridor_clear_retried
                ):
                    self.navigation_corridor_clear_retried = True
                    self._set_state("RECOVERING", "corridor_clear_retry_current_path")
                    retry = (evidence_reason, retry_goal, corridor, old_path)
                elif (
                    recoveries > 0
                    and retry_goal
                    and evidence_reason in {
                        "INSUFFICIENT_CLEARANCE", "CONFIRMED_FRONT_OBSTACLE",
                    }
                    and self.navigation_recovery_attempts
                    < self.failed_segment_max_replans
                ):
                    self.navigation_recovery_attempts += 1
                    self._set_state("RECOVERING", "confirmed_failed_segment")
                    retry = (evidence_reason, retry_goal, corridor, old_path)
                else:
                    # Unconfirmed sensor/localization/controller failures are
                    # not route evidence. BLOCKED is reserved for a planner
                    # proving no route after a confirmed keepout below.
                    terminal_reason = (
                        "NAV2_ABORTED"
                        if recoveries == 0
                        else "CORRIDOR_EVIDENCE_UNCONFIRMED"
                        if evidence_reason == "UNCONFIRMED"
                        else "CORRIDOR_CLEAR_CONTROLLER_FAILURE"
                    )
                    self._set_state("FAILED", terminal_reason.lower())
                    self.latest_feedback["terminal_reason"] = terminal_reason
                    self.paused_goal = None
                    self.latest_global_path = []
                    self.visualization_revision += 1
        self.profile_limiter.reset()
        self.motion_owner = "NONE"
        self.navigation_velocity.publish(Twist())
        if retry is not None:
            evidence_reason, retry_goal, corridor, old_path = retry
            segment = None
            if evidence_reason != "CORRIDOR_CLEAR":
                segment = self._mark_failed_segment(evidence_reason, corridor)
            else:
                self._nav_debug(
                    "RECOVERY",
                    reason="CORRIDOR_CLEAR",
                    action="RETRY_CURRENT_PATH",
                    can_go_straight=True,
                    can_rotate=corridor.can_rotate,
                )
            threading.Thread(
                target=self._recover_navigation,
                args=(
                    retry_goal,
                    evidence_reason,
                    segment,
                    goal_generation,
                    old_path,
                ),
                daemon=True,
            ).start()

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
            self.visualization_revision += 1

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
        if segment is not None:
            self.global_costmap_update.clear()
            self._publish_failed_segments()
            if not self.global_costmap_update.wait(2.0):
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
                {"map_id": self.map_id, "version": self.map_version},
                recovery_attempt=True,
            )
        except AdapterError as exc:
            self._set_recovery_terminal("FAILED", exc.code, expected_generation)

    def _cancel_navigation(self, target: str) -> dict[str, Any]:
        with self.state_lock:
            handle = self.current_goal_handle
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
            self.current_goal_handle = None
            self.navigation_goal_generation += 1
            if target != "PAUSED":
                self.paused_goal = None
                self.latest_global_path = []
                self.visualization_revision += 1
        self.profile_limiter.reset()
        self.rotation_metric_active = False
        self.obstacle_slowdown_active = False
        self.navigation_velocity.publish(Twist())
        return {"status": "completed", "current_state": target, "state": self._state()}

    def _pause_navigation(self) -> dict[str, Any]:
        if self.current_goal_handle is None:
            raise AdapterError("STATE_CONFLICT", "Navigation is not active")
        return self._cancel_navigation("PAUSED")

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
            posegraph_path = str(payload.get("posegraph_path") or "")
            if posegraph_path:
                self._load_mapping_posegraph(posegraph_path, payload.get("initial_pose"))
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
        self._call_empty_like(
            self.slam_deserialize_client,
            request,
            "POSEGRAPH_LOAD_FAILED",
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
        image = Image.open(pgm_path)
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
