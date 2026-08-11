from __future__ import annotations

from copy import deepcopy
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
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
from PIL import Image
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from navigation_core import SavedOccupancyMap, compact_lethal_cells, localization_confidence


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
        self.state_lock = threading.Lock()
        self.current_state = "IDLE" if self.mode == "MAPPING" else "STARTING"
        self.localization_state = "IDLE"
        self.localized = False
        self.initial_pose_requested = False
        self.map_id = ""
        self.map_version = 0
        self.mapping_payload: dict[str, Any] = {}
        self.current_goal_handle: Any = None
        self.paused_goal: dict[str, float] | None = None
        self.current_mission_id = ""
        self.latest_feedback: dict[str, Any] = {}
        self.saved_map: SavedOccupancyMap | None = None
        self.active_map_path: Path | None = None
        self.map_received_monotonic = 0.0
        self.last_scan_monotonic = 0.0
        self.scan_clock_skew_seconds = 0.0
        self.last_scan_clock_warning_monotonic = 0.0
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
        self.amcl_stable_samples = 0
        self.last_amcl_pose: tuple[float, float, float] | None = None
        self.last_amcl_covariance: list[float] = []
        self.low_confidence_since: float | None = None
        self.trusted_last_pose = False
        self.last_nomotion_request_monotonic = 0.0
        self.rotation_active = False
        self.rotation_angle = 0.0
        self.rotation_last_monotonic = 0.0
        self.localization_confidence_threshold = float(
            os.getenv("LOCALIZATION_CONFIDENCE_THRESHOLD", "0.72")
        )
        self.localization_low_threshold = float(
            os.getenv("LOCALIZATION_LOW_CONFIDENCE_THRESHOLD", "0.30")
        )
        self.localization_low_grace = float(
            os.getenv("LOCALIZATION_LOW_CONFIDENCE_GRACE_SECONDS", "5")
        )
        self.last_pose_timeout = float(os.getenv("LAST_POSE_TIMEOUT_SECONDS", "12"))
        self.global_rotate_delay = float(
            os.getenv("GLOBAL_LOCALIZATION_ROTATE_DELAY_SECONDS", "5")
        )
        self.localization_timeout = float(
            os.getenv("AUTO_LOCALIZATION_TIMEOUT_SECONDS", "45")
        )
        self.rotation_speed = math.radians(
            float(os.getenv("AUTO_LOCALIZATION_ROTATION_DEG_S", "20"))
        )
        self.rotation_max_angle = math.radians(
            float(os.getenv("AUTO_LOCALIZATION_MAX_ANGLE_DEG", "360"))
        )
        self.footprint = [
            {"x": 0.15, "y": 0.05}, {"x": 0.15, "y": -0.05},
            {"x": -0.15, "y": -0.05}, {"x": -0.15, "y": 0.05},
        ]
        self.footprint_clearance = float(os.getenv("GOAL_CLEARANCE_METERS", "0.15"))
        self.goal_snap_max_distance = float(
            os.getenv("GOAL_SNAP_MAX_DISTANCE_METERS", "0.45")
        )
        self.latest_global_path: list[dict[str, float]] = []
        self.latest_dynamic_obstacles: list[dict[str, float]] = []
        self.visualization_revision = 0
        self.mapping_started_monotonic = 0.0

        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map_load_client = self.create_client(LoadMap, "/map_server/load_map")
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
        self.mapping_scan = self.create_publisher(
            LaserScan, "/scan_mapping", qos_profile_sensor_data
        )
        self.navigation_scan = self.create_publisher(
            LaserScan, "/scan_navigation", qos_profile_sensor_data
        )
        self.initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        # This is a low-priority twist_mux input. It is smoothed and checked by
        # motion-safety before /cmd_vel; the adapter never bypasses safety.
        self.localization_velocity = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 1)
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
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(String, "/safety/health", self._safety_callback, 1)
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
        self.get_logger().info(
            f"navigation adapter ready mode={self.mode} socket={self.socket_path}"
        )

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
                "nav2": nav2_state,
                "feedback": dict(self.latest_feedback),
                "scan_fresh": time.monotonic() - self.last_scan_monotonic <= 0.30,
                "scan_clock_skew_seconds": round(self.scan_clock_skew_seconds, 3),
                "odometry_ready": self.tf_buffer.can_transform(
                    "odom", "base_footprint", Time()
                ),
                "lidar_tf_ready": self.tf_buffer.can_transform(
                    "base_link", "laser_frame", Time()
                ),
                "safety": "HEALTHY" if self.safety_health.startswith("HEALTHY") else self.safety_health,
                "estop": self.estop_active,
                "mission_id": self.current_mission_id,
                "footprint": list(self.footprint),
                "mapping": {
                    "state": self.current_state,
                    "scanHealthy": time.monotonic() - self.last_scan_monotonic <= 0.30,
                    "odomHealthy": self.tf_buffer.can_transform("odom", "base_footprint", Time()),
                    "tfHealthy": self.tf_buffer.can_transform("base_link", "laser_frame", Time()),
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
        unconditional_safety_command = command in {"navigation.cancel", "map.deactivate"}
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
            self.map_received_monotonic = time.monotonic()

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
        pose = message.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        sample = (float(pose.position.x), float(pose.position.y), yaw)
        if self.last_amcl_pose is not None:
            jump = math.hypot(
                sample[0] - self.last_amcl_pose[0], sample[1] - self.last_amcl_pose[1]
            )
            yaw_jump = abs(self._yaw_delta(sample[2], self.last_amcl_pose[2]))
            self.amcl_stable_samples = self.amcl_stable_samples + 1 if jump <= 0.20 and yaw_jump <= 0.30 else 0
        else:
            self.amcl_stable_samples = 1
        self.last_amcl_pose = sample
        self.last_amcl_monotonic = time.monotonic()
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

    def _refresh_localization_confidence(self) -> None:
        if not self.last_amcl_covariance:
            self.localization_confidence = 0.0
            return
        self.localization_confidence = localization_confidence(
            self.last_amcl_covariance,
            stable_samples=self.amcl_stable_samples,
            scan_fresh=time.monotonic() - self.last_scan_monotonic <= 0.30,
            tf_stable=time.monotonic() - self.last_map_tf_monotonic <= 0.60,
        )

    def _path_callback(self, message: NavigationPath) -> None:
        path = [
            {"x": round(float(item.pose.position.x), 3), "y": round(float(item.pose.position.y), 3)}
            for item in message.poses
        ]
        if path != self.latest_global_path:
            self.latest_global_path = path
            self.visualization_revision += 1

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
        if obstacles != self.latest_dynamic_obstacles:
            self.latest_dynamic_obstacles = obstacles
            self.visualization_revision += 1

    def _scan_callback(self, message: LaserScan) -> None:
        self.last_scan_monotonic = time.monotonic()
        # The Yahboom micro-ROS firmware can retain an old epoch offset after
        # the Agent reconnects without an MCU reboot. Messages then arrive at
        # the correct rate but are minutes behind the host TF clock, causing
        # SLAM Toolbox and AMCL to discard or misplace every scan. Normalize
        # only Rovera's private per-mode copy. The shared /scan topic remains
        # byte-for-byte untouched for legacy programs on this ROS domain.
        now = self.get_clock().now()
        source_stamp = Time.from_msg(message.header.stamp)
        self.scan_clock_skew_seconds = (
            now.nanoseconds - source_stamp.nanoseconds
        ) / 1_000_000_000
        normalized_message = message
        if (
            source_stamp.nanoseconds == 0
            or abs(self.scan_clock_skew_seconds) > 0.5
        ):
            normalized_message = deepcopy(message)
            normalized_message.header.stamp = now.to_msg()
            monotonic_now = time.monotonic()
            if monotonic_now - self.last_scan_clock_warning_monotonic >= 60.0:
                destination = (
                    "/scan_mapping" if self.mode == "MAPPING" else "/scan_navigation"
                )
                self.get_logger().warning(
                    f"normalizing stale LiDAR timestamp for {destination} "
                    f"skew={self.scan_clock_skew_seconds:.3f}s"
                )
                self.last_scan_clock_warning_monotonic = monotonic_now
        if self.mode == "MAPPING" and self.current_state in {"MAPPING", "MAPPING_RUNNING"}:
            self.mapping_scan.publish(normalized_message)
        elif self.mode == "NAVIGATION":
            self.navigation_scan.publish(normalized_message)
    def _safety_callback(self, message: String) -> None:
        self.safety_health = message.data

    def _estop_callback(self, message: Bool) -> None:
        self.estop_active = bool(message.data)
        if self.estop_active:
            self._stop_localization_rotation()

    def _direction_mask_callback(self, message: UInt8) -> None:
        self.safety_direction_mask = int(message.data)

    def _manual_takeover_callback(self, message: Bool) -> None:
        if message.data:
            self.last_manual_takeover_monotonic = time.monotonic()
            self._stop_localization_rotation()
        if (
            message.data
            and self.mode == "NAVIGATION"
            and self.current_state in {"NAVIGATING", "PAUSED", "BLOCKED"}
        ):
            self.get_logger().warning("manual takeover: canceling Nav2 goal")
            try:
                self._cancel_navigation("CANCELED")
            except AdapterError as exc:
                self.get_logger().error(f"manual takeover cancel failed: {exc}")

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
                self.current_state = "NO_ACTIVE_MAP"
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

    def _publish_initial_pose(self, pose: dict[str, Any], *, approximate: bool) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        # Yahboom odometry reaches ROS tens of milliseconds behind wall time.
        # Stamping an initial pose at ``now`` therefore asks AMCL for a future
        # odom transform; AMCL logs the error and can retain a stale map->odom
        # offset. ROS AMCL replaces a zero stamp with ``now``, so use a small,
        # bounded look-back that is guaranteed to be inside the TF cache.
        now = self.get_clock().now()
        message.header.stamp = Time(
            nanoseconds=max(0, now.nanoseconds - 200_000_000)
        ).to_msg()
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        message.pose.pose.orientation = quaternion_from_yaw(float(pose.get("yaw", 0)))
        position_variance = 1.0 if approximate else max(0.04, float(pose.get("covariance", 0.25)))
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[35] = 0.274 if approximate else 0.0685
        self.initial_pose.publish(message)
        self.initial_pose_requested = True

    def _begin_auto_localization(self, last_pose: Any) -> None:
        now = time.monotonic()
        self.localized = False
        self.localization_confidence = 0.0
        self.amcl_stable_samples = 0
        self.low_confidence_since = None
        self.trusted_last_pose = False
        self.last_nomotion_request_monotonic = 0.0
        self.localization_started_monotonic = now
        self.localization_phase_started_monotonic = now
        self.rotation_angle = 0.0
        self.rotation_active = False
        self.last_amcl_pose = None
        self.last_amcl_covariance = []
        if isinstance(last_pose, dict) and all(
            math.isfinite(float(last_pose.get(axis, 0.0))) for axis in ("x", "y", "yaw")
        ):
            self.trusted_last_pose = float(last_pose.get("covariance", 1.0)) <= 0.25
            self._publish_initial_pose(last_pose, approximate=False)
            self.localization_state = "LOCALIZING_LAST_POSE"
            self.current_state = "LOCALIZING_LAST_POSE"
        else:
            self._start_global_localization()

    def _start_global_localization(self) -> None:
        self._stop_localization_rotation()
        self.trusted_last_pose = False
        if not self.global_localization_client.wait_for_service(timeout_sec=3.0):
            self.localization_state = "LOCALIZATION_FAILED"
            self.current_state = "LOCALIZATION_FAILED"
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
        self.current_state = "LOCALIZING_GLOBAL"
        self.initial_pose_requested = False
        future = self.global_localization_client.call_async(Empty.Request())

        def completed(response_future: Any) -> None:
            try:
                response_future.result()
            except Exception as exc:
                self.get_logger().error(f"global localization failed: {exc}")
                self._stop_localization_rotation()
                self.localization_state = "LOCALIZATION_FAILED"
                self.current_state = "LOCALIZATION_FAILED"

        future.add_done_callback(completed)

    def _safe_to_rotate(self) -> bool:
        return (
            time.monotonic() - self.last_scan_monotonic <= 0.30
            and self.safety_health.startswith("HEALTHY")
            and not self.estop_active
            and self.safety_direction_mask == 0
            and time.monotonic() - self.last_manual_takeover_monotonic > 0.5
        )

    def _stop_localization_rotation(self) -> None:
        if self.rotation_active:
            self.localization_velocity.publish(Twist())
        self.rotation_active = False
        self.rotation_last_monotonic = 0.0

    def _localization_lost(self, reason: str) -> None:
        if not self.localized:
            return
        self.get_logger().error(f"localization lost; stopping Nav2: {reason}")
        self.localized = False
        self.localization_state = "LOCALIZATION_LOST"
        self.current_state = "LOCALIZATION_LOST"
        handle = self.current_goal_handle
        self.current_goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()
        self.localization_velocity.publish(Twist())
        self.latest_global_path = []
        self.visualization_revision += 1
        try:
            self._start_global_localization()
        except AdapterError as exc:
            self.get_logger().error(str(exc))

    def _localization_tick(self) -> None:
        if self.mode != "NAVIGATION":
            return
        localizing_states = {
            "LOCALIZATION_INITIALIZING", "LOCALIZING_LAST_POSE", "LOCALIZING_GLOBAL",
            "LOCALIZING_ROTATING", "LOW_CONFIDENCE", "LOCALIZATION_LOST",
        }
        if self.localization_state not in localizing_states:
            return
        now = time.monotonic()
        # AMCL may publish its only stationary sample just before map->base is
        # visible. Re-evaluate it against current TF freshness on every tick.
        self._refresh_localization_confidence()
        # A stationary robot normally produces only one AMCL estimate because
        # update_min_d/a suppresses further filter updates. Ask AMCL for
        # bounded no-motion updates so a recent persisted pose is verified by
        # multiple LiDAR scans instead of either trusting its first covariance
        # spike or timing out without enough samples.
        if (
            self.localization_state == "LOCALIZING_LAST_POSE"
            and now - self.last_nomotion_request_monotonic >= 0.5
            and self.nomotion_update_client.service_is_ready()
        ):
            self.last_nomotion_request_monotonic = now
            self.nomotion_update_client.call_async(Empty.Request())
        required_confidence = (
            self.localization_low_threshold
            if self.localization_state == "LOCALIZING_LAST_POSE" and self.trusted_last_pose
            else self.localization_confidence_threshold
        )
        if (
            self.localization_confidence >= required_confidence
            and self.amcl_stable_samples >= 5
            and now - self.last_scan_monotonic <= 0.30
            and now - self.last_map_tf_monotonic <= 0.60
        ):
            self._stop_localization_rotation()
            self.localized = True
            self.localization_state = "READY"
            self.current_state = "READY"
            self.low_confidence_since = None
            return
        if now - self.localization_started_monotonic >= self.localization_timeout:
            self._stop_localization_rotation()
            self.localized = False
            self.localization_state = "LOCALIZATION_FAILED"
            self.current_state = "LOCALIZATION_FAILED"
            return
        if (
            self.localization_state == "LOCALIZING_LAST_POSE"
            and now - self.localization_phase_started_monotonic >= self.last_pose_timeout
        ):
            try:
                self._start_global_localization()
            except AdapterError as exc:
                self.get_logger().error(str(exc))
            return
        if self.localization_state in {"LOCALIZING_GLOBAL", "LOW_CONFIDENCE", "LOCALIZATION_LOST"}:
            if now - self.localization_phase_started_monotonic < self.global_rotate_delay:
                return
            self.localization_state = "LOCALIZING_ROTATING"
            self.current_state = "LOCALIZING_ROTATING"
        if self.localization_state == "LOCALIZING_ROTATING":
            if self.rotation_angle >= self.rotation_max_angle:
                self._stop_localization_rotation()
                self.localization_state = "LOCALIZATION_FAILED"
                self.current_state = "LOCALIZATION_FAILED"
                return
            if not self._safe_to_rotate():
                self._stop_localization_rotation()
                return
            delta = now - self.rotation_last_monotonic if self.rotation_last_monotonic else 0.0
            self.rotation_angle += abs(self.rotation_speed) * max(0.0, min(delta, 0.5))
            command = Twist()
            command.angular.z = self.rotation_speed
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
                self.active_map_path = None
                self.localized = False
                self.localization_state = "IDLE"
                self.current_state = "NO_ACTIVE_MAP"
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
            self.current_state = "READY" if previous_localized else previous_localization_state
        with self.state_lock:
            self.current_state = "MAP_LOADING"
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
                self.current_state = "FAILED"
            raise AdapterError("MAP_LOAD_FAILED", f"Map Server returned {response.result}")
        with self.state_lock:
            self.map_id = str(payload["map_id"])
            self.map_version = int(payload["version"])
            self.saved_map = candidate_grid
            self.active_map_path = map_yaml
            self.localized = False
            self.initial_pose_requested = False
            self.current_mission_id = ""
            self.latest_global_path = []
            self.latest_dynamic_obstacles = []
            self.visualization_revision += 1
            self.current_state = "LOCALIZATION_INITIALIZING"
            self.localization_state = "LOCALIZATION_INITIALIZING"
        map_deadline = time.monotonic() + 3.0
        while self.map_received_monotonic < load_started and time.monotonic() < map_deadline:
            time.sleep(0.05)
        if self.map_received_monotonic < load_started:
            try:
                rollback_map()
            except AdapterError:
                self.current_state = "FAILED"
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
        self._publish_initial_pose(pose, approximate=True)
        self.localized = False
        self.localization_started_monotonic = time.monotonic()
        self.localization_phase_started_monotonic = self.localization_started_monotonic
        self.localization_state = "LOCALIZING_GLOBAL"
        self.current_state = "LOCALIZING_GLOBAL"
        return {
            "status": "accepted",
            "current_state": "LOCALIZING_GLOBAL",
            "localized": False,
            "state": self._state(),
        }

    def _deactivate_map(self) -> dict[str, Any]:
        self._cancel_navigation("CANCELED")
        self._stop_localization_rotation()
        self.map_id = ""
        self.map_version = 0
        self.saved_map = None
        self.active_map_path = None
        self.localized = False
        self.localization_state = "IDLE"
        self.current_state = "NO_ACTIVE_MAP"
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
        validation = self.saved_map.validate_goal(
            float(goal["x"]),
            float(goal["y"]),
            clearance_m=self.footprint_clearance,
            allow_unknown=os.getenv("NAVIGATION_ALLOW_UNKNOWN_GOAL", "0").lower() in {"1", "true", "yes"},
            lethal_world_cells=(
                (float(item["x"]), float(item["y"])) for item in self.latest_dynamic_obstacles
            ),
        )
        if not validation.valid:
            raise AdapterError(validation.code, validation.message)

    def _resolve_planning_goal(self, goal: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        """Validate a click and, when needed, move it to a nearby safe cell."""
        try:
            self._validate_goal(goal)
            return goal, False
        except AdapterError as exc:
            if exc.code not in {"GOAL_UNKNOWN", "GOAL_OCCUPIED", "GOAL_CLEARANCE", "GOAL_LETHAL"}:
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
            )
            if snapped is None:
                raise
            resolved = dict(goal)
            resolved["x"], resolved["y"] = snapped
            self._validate_goal(resolved)
            return resolved, True

    def _compute_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._validate_command_map(payload)
        requested_goal = dict(payload["goal"])
        resolved_goal, goal_adjusted = self._resolve_planning_goal(requested_goal)
        if not self.compute_path_client.wait_for_server(timeout_sec=5.0):
            raise AdapterError("PLANNER_UNAVAILABLE", "ComputePathToPose action unavailable")
        with self.state_lock:
            # Planning creates a new Center mission only after Nav2 returns a
            # path. Do not let interim READY telemetry mutate the prior goal.
            self.current_mission_id = ""
            self.current_state = "PLANNING"
        goal = ComputePathToPose.Goal()
        goal.goal = self._goal_pose(resolved_goal)
        goal.use_start = False
        handle = self._wait(
            self.compute_path_client.send_goal_async(goal), 5, "PLANNER_TIMEOUT"
        )
        if not handle.accepted:
            raise AdapterError("PLAN_REJECTED", "Planner rejected goal")
        result = self._wait(handle.get_result_async(), 15, "PLANNER_TIMEOUT").result
        points = [
            {"x": pose.pose.position.x, "y": pose.pose.position.y}
            for pose in result.path.poses
        ]
        if not points:
            with self.state_lock:
                self.current_state = "BLOCKED"
            raise AdapterError("NO_PATH", "Nav2 returned an empty path")
        distance = sum(
            math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            for a, b in zip(points, points[1:])
        )
        with self.state_lock:
            self.current_state = "READY"
            self.latest_global_path = points
            self.visualization_revision += 1
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

    def _navigate(self, goal_payload: dict[str, Any], command_payload: dict[str, Any]) -> dict[str, Any]:
        self._validate_command_map(command_payload)
        self._validate_goal(goal_payload)
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            raise AdapterError("NAV2_UNAVAILABLE", "NavigateToPose action unavailable")
        goal = NavigateToPose.Goal()
        goal.pose = self._goal_pose(goal_payload)
        future = self.navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback
        )
        handle = self._wait(future, 5, "NAVIGATION_TIMEOUT")
        if not handle.accepted:
            raise AdapterError("GOAL_REJECTED", "Nav2 rejected goal")
        with self.state_lock:
            self.current_goal_handle = handle
            self.paused_goal = dict(goal_payload)
            self.current_state = "NAVIGATING"
        handle.get_result_async().add_done_callback(self._navigation_result)
        return {"status": "accepted", "current_state": "NAVIGATING", "state": self._state()}

    def _navigation_feedback(self, feedback: Any) -> None:
        data = feedback.feedback
        with self.state_lock:
            self.latest_feedback = {
                "distance_remaining": float(data.distance_remaining),
                "navigation_time_seconds": data.navigation_time.sec + data.navigation_time.nanosec / 1e9,
                "recoveries": int(data.number_of_recoveries),
            }

    def _navigation_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose result failed: {exc}")
            status = GoalStatus.STATUS_ABORTED
        with self.state_lock:
            self.current_goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.current_state = "SUCCEEDED"
                self.paused_goal = None
                self.latest_global_path = []
                self.visualization_revision += 1
            elif status == GoalStatus.STATUS_CANCELED and self.current_state == "PAUSED":
                pass
            elif status == GoalStatus.STATUS_CANCELED:
                self.current_state = "CANCELED"
                self.paused_goal = None
            else:
                self.current_state = "FAILED"

    def _cancel_navigation(self, target: str) -> dict[str, Any]:
        with self.state_lock:
            handle = self.current_goal_handle
            self.current_state = target
        if handle is not None:
            self._wait(handle.cancel_goal_async(), 3, "CANCEL_TIMEOUT")
        if target != "PAUSED":
            self.paused_goal = None
            self.latest_global_path = []
            self.visualization_revision += 1
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
                    "TF base_link -> laser_frame is unavailable",
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
