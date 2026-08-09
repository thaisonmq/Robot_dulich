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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.localized = False
        self.initial_pose_requested = False
        self.map_id = ""
        self.map_version = 0
        self.mapping_payload: dict[str, Any] = {}
        self.current_goal_handle: Any = None
        self.paused_goal: dict[str, float] | None = None
        self.current_mission_id = ""
        self.latest_feedback: dict[str, Any] = {}
        self.latest_map_snapshot: dict[str, Any] | None = None
        self.latest_scan_points: list[dict[str, float]] = []
        self.map_revision = 0
        self.map_preview_max_cells = max(
            1_000, int(os.getenv("MAPPING_PREVIEW_MAX_CELLS", "20000"))
        )
        self.last_scan_monotonic = 0.0
        self.scan_clock_skew_seconds = 0.0
        self.last_scan_clock_warning_monotonic = 0.0
        self.safety_health = "UNKNOWN"
        self.pose: dict[str, float] | None = None
        self.trail: list[dict[str, float]] = []

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
        self.mapping_scan = self.create_publisher(
            LaserScan, "/scan_mapping", qos_profile_sensor_data
        )
        self.navigation_scan = self.create_publisher(
            LaserScan, "/scan_navigation", qos_profile_sensor_data
        )
        self.initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 1)
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(String, "/safety/health", self._safety_callback, 1)
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
                "mission_id": self.current_mission_id,
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
            and self.current_state in {"CANCELED", "FINISHED", "FAULT"}
        )
        if expected_state and expected_state != self.current_state and not restartable_mapping_start:
            raise AdapterError(
                "STATE_CONFLICT",
                f"Expected {expected_state}, robot is {self.current_state}",
            )
        if command == "map.load":
            return self._load_map(payload)
        if command == "map.set_initial_pose":
            return self._set_initial_pose(payload)
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
            return self._navigate(goal)
        if command == "navigation.pause":
            return self._pause_navigation()
        if command == "navigation.resume":
            if self.paused_goal is None:
                raise AdapterError("STATE_CONFLICT", "No paused goal to resume")
            return self._navigate(self.paused_goal)
        if command == "navigation.cancel":
            return self._cancel_navigation("CANCELED")
        if command.startswith("mapping."):
            return self._mapping_command(command, payload)
        if command == "system.status":
            return {
                "status": "completed",
                "current_state": self.current_state,
                "state": self._state(),
                "mapping_snapshot": self.latest_map_snapshot if self.mode == "MAPPING" else None,
                "scan": list(self.latest_scan_points) if self.mode == "MAPPING" else None,
                "pose": self.pose,
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
        width = int(message.info.width)
        height = int(message.info.height)
        if width <= 0 or height <= 0:
            return
        step = max(
            1,
            math.ceil(math.sqrt(width * height / self.map_preview_max_cells)),
        )
        sampled: list[int] = []
        for row in range(0, height, step):
            for column in range(0, width, step):
                occupied = -1
                probability = -1
                has_free = False
                for source_row in range(row, min(row + step, height)):
                    base = source_row * width
                    for source_column in range(column, min(column + step, width)):
                        value = int(message.data[base + source_column])
                        if value >= 65:
                            occupied = max(occupied, value)
                        elif value > 0:
                            probability = max(probability, value)
                        elif value == 0:
                            has_free = True
                # Conservative block reduction keeps thin walls visible. A
                # stride sample discarded obstacles whenever it happened to
                # land on a neighbouring unknown/free cell.
                sampled.append(
                    occupied
                    if occupied >= 0
                    else probability
                    if probability >= 0
                    else 0
                    if has_free
                    else -1
                )
        rle: list[int] = []
        for value in sampled:
            if rle and rle[-2] == value:
                rle[-1] += 1
            else:
                rle.extend((value, 1))
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.map_revision += 1
        self.latest_map_snapshot = {
            "width": math.ceil(width / step),
            "height": math.ceil(height / step),
            "resolution": float(message.info.resolution) * step,
            "origin": {
                "x": message.info.origin.position.x,
                "y": message.info.origin.position.y,
                "yaw": yaw,
            },
            "rle": rle,
            "revision": self.map_revision,
            "source_width": width,
            "source_height": height,
            "downsample_step": step,
            "scan": list(self.latest_scan_points),
            "trail": list(self.trail[-500:]),
        }

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
        if self.mode == "MAPPING" and self.current_state == "MAPPING":
            self.mapping_scan.publish(normalized_message)
        elif self.mode == "NAVIGATION":
            self.navigation_scan.publish(normalized_message)
        points: list[dict[str, float]] = []
        stride = max(1, len(message.ranges) // 90)
        for index in range(0, len(message.ranges), stride):
            distance = float(message.ranges[index])
            if (
                not math.isfinite(distance)
                or distance < message.range_min
                or distance > message.range_max
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            points.append(
                {"x": distance * math.cos(angle), "y": distance * math.sin(angle)}
            )
        self.latest_scan_points = points

    def _safety_callback(self, message: String) -> None:
        self.safety_health = message.data

    def _manual_takeover_callback(self, message: Bool) -> None:
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
                self.current_state = "READY"
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
        except TransformException:
            return
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        pose = {
            "x": transform.transform.translation.x,
            "y": transform.transform.translation.y,
            "yaw": yaw,
        }
        if (
            (self.mode != "MAPPING" or self.current_state == "MAPPING")
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
            and self.current_state == "LOCALIZING"
            and self.initial_pose_requested
        ):
            with self.state_lock:
                self.localized = True
                self.current_state = "READY"

    def _load_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_yaml = Path(str(payload["map_path"])) / "map.yaml"
        if not map_yaml.is_file():
            raise AdapterError("MAP_MISSING", "map.yaml is missing from verified cache")
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            raise AdapterError("MAP_SERVER_UNAVAILABLE", "Map Server load service unavailable")
        request = LoadMap.Request()
        request.map_url = str(map_yaml)
        response = self._wait(
            self.map_load_client.call_async(request), 10, "MAP_LOAD_TIMEOUT"
        )
        if int(response.result) != 0:
            raise AdapterError("MAP_LOAD_FAILED", f"Map Server returned {response.result}")
        with self.state_lock:
            self.map_id = str(payload["map_id"])
            self.map_version = int(payload["version"])
            self.localized = False
            self.initial_pose_requested = False
            self.current_state = "LOCALIZING"
        return {
            "status": "completed",
            "current_state": "LOCALIZING",
            "progress_percent": 100,
            "state": self._state(),
        }

    def _set_initial_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = dict(payload["pose"])
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        message.pose.pose.orientation = quaternion_from_yaw(float(pose.get("yaw", 0)))
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.initial_pose.publish(message)
        self.initial_pose_requested = True
        # Do not claim localization from publish success. _update_pose changes
        # to READY only after map -> base_footprint is actually available.
        return {
            "status": "accepted",
            "current_state": "LOCALIZING",
            "localized": False,
            "state": self._state(),
        }

    def _goal_pose(self, goal: dict[str, Any]) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(goal["x"])
        message.pose.position.y = float(goal["y"])
        message.pose.orientation = quaternion_from_yaw(float(goal.get("yaw", 0)))
        return message

    def _compute_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.compute_path_client.wait_for_server(timeout_sec=5.0):
            raise AdapterError("PLANNER_UNAVAILABLE", "ComputePathToPose action unavailable")
        with self.state_lock:
            self.current_state = "PLANNING"
        goal = ComputePathToPose.Goal()
        goal.goal = self._goal_pose(dict(payload["goal"]))
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
        return {
            "status": "completed",
            "current_state": "READY",
            "points": points,
            "distance_m": round(distance, 3),
            "state": self._state(),
        }

    def _navigate(self, goal_payload: dict[str, Any]) -> dict[str, Any]:
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
                self.current_state = "ARRIVED"
                self.paused_goal = None
            elif status == GoalStatus.STATUS_CANCELED and self.current_state == "PAUSED":
                pass
            elif status == GoalStatus.STATUS_CANCELED:
                self.current_state = "CANCELED"
                self.paused_goal = None
            else:
                self.current_state = "FAULT"

    def _cancel_navigation(self, target: str) -> dict[str, Any]:
        with self.state_lock:
            handle = self.current_goal_handle
            self.current_state = target
        if handle is not None:
            self._wait(handle.cancel_goal_async(), 3, "CANCEL_TIMEOUT")
        if target != "PAUSED":
            self.paused_goal = None
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
            if self.current_state in {"CANCELED", "FINISHED", "FAULT"}:
                self._restart_slam_runtime()
            with self.state_lock:
                self.mapping_payload = dict(payload)
                self.map_id = str(payload.get("map_id") or "")
                self.map_version = int(payload.get("version") or 1)
                self.latest_map_snapshot = None
                self.map_revision = 0
                self.trail = []
                self.pose = None
            posegraph_path = str(payload.get("posegraph_path") or "")
            if posegraph_path:
                self._load_mapping_posegraph(posegraph_path, payload.get("initial_pose"))
            with self.state_lock:
                self.current_state = "MAPPING"
            return {"status": "completed", "current_state": "MAPPING", "state": self._state()}
        if command == "mapping.pause":
            self._call_empty_like(self.slam_pause_client, Pause.Request(), "SLAM_PAUSE_FAILED")
            self.current_state = "PAUSED"
            return {"status": "completed", "current_state": "PAUSED", "state": self._state()}
        if command == "mapping.resume":
            self._call_empty_like(self.slam_pause_client, Pause.Request(), "SLAM_RESUME_FAILED")
            self.current_state = "MAPPING"
            return {"status": "completed", "current_state": "MAPPING", "state": self._state()}
        if command in {"mapping.save_draft", "mapping.finish"}:
            state_before_save = self.current_state
            bundle = self._save_mapping_bundle(payload)
            self.current_state = state_before_save if command.endswith("save_draft") else "FINISHED"
            return {
                "status": "completed",
                "current_state": self.current_state,
                "bundle_path": str(bundle),
                "draft_saved": command.endswith("save_draft"),
                "state": self._state(),
            }
        if command == "mapping.cancel":
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
            "version": version,
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
            "terminal_pose": dict(self.pose) if self.pose else None,
            "files": {},
            "poi": [],
            "keepout_zones": [],
            "speed_zones": [],
        }
        for path in staging.iterdir():
            if path.is_file() and path.name != "metadata.json":
                metadata["files"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        (staging / "metadata.json").write_text(json.dumps(metadata, indent=2))
        bundle = staging / "map-bundle.tar.gz"
        with tarfile.open(bundle, "w:gz") as archive:
            for path in staging.iterdir():
                if path.is_file() and path != bundle:
                    archive.add(path, arcname=path.name)
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
        if self.mode != "MAPPING" or self.current_state not in {"MAPPING", "PAUSED"}:
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
