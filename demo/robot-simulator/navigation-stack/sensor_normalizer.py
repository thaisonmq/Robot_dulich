from __future__ import annotations

import copy
import json
import math
import time
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from navigation_core import SensorClockEstimator, mask_scan_self_returns


class SensorNormalizer(Node):
    """Normalize the shared Yahboom sensor clock before TF/EKF/AMCL use."""

    SENSOR_NAMES = ("scan", "odom", "imu")

    def __init__(self) -> None:
        super().__init__("rovera_sensor_normalizer")
        self.declare_parameter("minimum_sync_samples", 5)
        self.declare_parameter("window_samples", 120)
        self.declare_parameter("sync_spread_seconds", 0.08)
        self.declare_parameter("jump_tolerance_seconds", 0.50)
        self.declare_parameter("maximum_corrected_age_seconds", 0.25)
        self.declare_parameter("future_tolerance_seconds", 0.02)
        self.declare_parameter("invalid_debounce_samples", 3)
        self.declare_parameter("scan_arrival_timeout_seconds", 0.30)
        self.declare_parameter("odom_arrival_timeout_seconds", 0.25)
        self.declare_parameter("imu_arrival_timeout_seconds", 0.25)
        self.declare_parameter("scan_self_filter_half_length", 0.20)
        self.declare_parameter("scan_self_filter_half_width", 0.18)
        self.declare_parameter("scan_laser_x", -0.0046412)
        self.declare_parameter("scan_laser_y", 0.0)
        self.declare_parameter("scan_laser_yaw", 0.0)
        self.clock = SensorClockEstimator(
            minimum_sync_samples=int(self.get_parameter("minimum_sync_samples").value),
            window_samples=int(self.get_parameter("window_samples").value),
            sync_spread_seconds=float(self.get_parameter("sync_spread_seconds").value),
            jump_tolerance_seconds=float(self.get_parameter("jump_tolerance_seconds").value),
            maximum_corrected_age_seconds=float(
                self.get_parameter("maximum_corrected_age_seconds").value
            ),
            future_tolerance_seconds=float(
                self.get_parameter("future_tolerance_seconds").value
            ),
            invalid_debounce_samples=int(
                self.get_parameter("invalid_debounce_samples").value
            ),
        )
        self.arrival_timeouts = {
            "scan": float(self.get_parameter("scan_arrival_timeout_seconds").value),
            "odom": float(self.get_parameter("odom_arrival_timeout_seconds").value),
            "imu": float(self.get_parameter("imu_arrival_timeout_seconds").value),
        }
        self.last_arrival_monotonic = {name: 0.0 for name in self.SENSOR_NAMES}
        self.last_valid_monotonic = {name: 0.0 for name in self.SENSOR_NAMES}
        self.frame_valid = {name: False for name in self.SENSOR_NAMES}
        self.last_corrected_age_ms: dict[str, float | None] = {
            name: None for name in self.SENSOR_NAMES
        }
        self.last_rejection = {name: "" for name in self.SENSOR_NAMES}
        self.sensor_rejections = {name: 0 for name in self.SENSOR_NAMES}
        self.scan_self_filter = {
            "half_length": float(
                self.get_parameter("scan_self_filter_half_length").value
            ),
            "half_width": float(
                self.get_parameter("scan_self_filter_half_width").value
            ),
            "laser_x": float(self.get_parameter("scan_laser_x").value),
            "laser_y": float(self.get_parameter("scan_laser_y").value),
            "laser_yaw": float(self.get_parameter("scan_laser_yaw").value),
        }
        self.last_scan_self_filtered_beams = 0

        self.scan_pub = self.create_publisher(
            LaserScan, "/scan/normalized", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry/normalized", 10)
        self.imu_pub = self.create_publisher(
            Imu, "/imu/normalized", qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, "/sensors/time_status", 1)
        self.create_subscription(
            LaserScan, "/scan", self._scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom_raw", self._odom, 10)
        self.create_subscription(Imu, "/imu", self._imu, qos_profile_sensor_data)
        self.create_timer(0.2, self._publish_status)

    @staticmethod
    def _stamp_nanoseconds(message: Any) -> int:
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _correct_stamp(self, sensor: str, message: Any) -> Any | None:
        arrival = self.get_clock().now()
        monotonic_now = time.monotonic()
        self.last_arrival_monotonic[sensor] = monotonic_now
        correction = self.clock.observe(
            sensor,
            source_nanoseconds=self._stamp_nanoseconds(message),
            arrival_nanoseconds=arrival.nanoseconds,
        )
        self.last_rejection[sensor] = correction.reason
        if not correction.accepted or correction.corrected_nanoseconds is None:
            self.sensor_rejections[sensor] += 1
            return None
        normalized = copy.deepcopy(message)
        normalized.header.stamp = Time(
            nanoseconds=correction.corrected_nanoseconds
        ).to_msg()
        self.last_valid_monotonic[sensor] = monotonic_now
        self.last_corrected_age_ms[sensor] = round(
            (arrival.nanoseconds - correction.corrected_nanoseconds) / 1e6, 3
        )
        return normalized

    def _scan(self, source: LaserScan) -> None:
        self.frame_valid["scan"] = source.header.frame_id == "laser_frame"
        if not self.frame_valid["scan"]:
            self.last_arrival_monotonic["scan"] = time.monotonic()
            self.last_rejection["scan"] = "UNEXPECTED_SCAN_FRAME"
            self.sensor_rejections["scan"] += 1
            return
        message = self._correct_stamp("scan", source)
        if message is not None:
            message.ranges, self.last_scan_self_filtered_beams = mask_scan_self_returns(
                message.ranges,
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
                laser_x=self.scan_self_filter["laser_x"],
                laser_y=self.scan_self_filter["laser_y"],
                laser_yaw=self.scan_self_filter["laser_yaw"],
                half_length=self.scan_self_filter["half_length"],
                half_width=self.scan_self_filter["half_width"],
            )
            self.scan_pub.publish(message)

    def _odom(self, source: Odometry) -> None:
        # The vendor's ``odom_frame`` is the planar wheel-odometry coordinate
        # basis, not a physically different frame. It publishes no TF of its
        # own; this explicit alias makes EKF the sole canonical odom authority.
        self.frame_valid["odom"] = (
            source.header.frame_id in {"odom", "odom_frame"}
            and source.child_frame_id == "base_footprint"
        )
        if not self.frame_valid["odom"]:
            self.last_arrival_monotonic["odom"] = time.monotonic()
            self.last_rejection["odom"] = "UNEXPECTED_ODOM_FRAME"
            self.sensor_rejections["odom"] += 1
            return
        message = self._correct_stamp("odom", source)
        if message is None:
            return
        message.header.frame_id = "odom"
        # Vendor covariance is overly optimistic. Preserve measured planar
        # axes while keeping unobserved Z/roll/pitch unavailable to the EKF.
        message.pose.covariance[0] = max(message.pose.covariance[0], 0.01)
        message.pose.covariance[7] = max(message.pose.covariance[7], 0.01)
        message.pose.covariance[35] = max(message.pose.covariance[35], 0.03)
        message.twist.covariance[0] = max(message.twist.covariance[0], 0.02)
        message.twist.covariance[35] = max(message.twist.covariance[35], 0.04)
        self.odom_pub.publish(message)

    def _imu(self, source: Imu) -> None:
        self.frame_valid["imu"] = source.header.frame_id == "imu_frame"
        if not self.frame_valid["imu"]:
            self.last_arrival_monotonic["imu"] = time.monotonic()
            self.last_rejection["imu"] = "UNEXPECTED_IMU_FRAME"
            self.sensor_rejections["imu"] += 1
            return
        message = self._correct_stamp("imu", source)
        if message is None:
            return
        # Live inspection found identity orientation, zero covariance and a
        # stationary gyro Z of exactly zero. Mark orientation unavailable and
        # keep gyro data diagnostic-only until a physical axis/sign test.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.08,
        ]
        message.linear_acceleration_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.36,
        ]
        self.imu_pub.publish(message)

    def _publish_status(self) -> None:
        now = time.monotonic()
        sensors: dict[str, dict[str, Any]] = {}
        for sensor in self.SENSOR_NAMES:
            arrival_age = (
                math.inf
                if self.last_arrival_monotonic[sensor] <= 0
                else now - self.last_arrival_monotonic[sensor]
            )
            valid_age = (
                math.inf
                if self.last_valid_monotonic[sensor] <= 0
                else now - self.last_valid_monotonic[sensor]
            )
            arrival_fresh = arrival_age <= self.arrival_timeouts[sensor]
            timestamp_valid = (
                self.clock.state == "SYNCED"
                and valid_age <= self.arrival_timeouts[sensor]
                and self.frame_valid[sensor]
                and self.clock.invalid_streak < self.clock.invalid_debounce_samples
            )
            sensors[sensor] = {
                "arrival_fresh": arrival_fresh,
                "timestamp_valid": timestamp_valid,
                "frame_valid": self.frame_valid[sensor],
                "arrival_age_ms": None if not math.isfinite(arrival_age) else round(arrival_age * 1000, 1),
                "corrected_age_ms": self.last_corrected_age_ms[sensor],
                "clock_skew_ms": round(
                    self.clock.last_raw_skew_seconds.get(sensor, 0.0) * 1000, 1
                ),
                "rejected_packets": self.sensor_rejections[sensor],
                "last_rejection": self.last_rejection[sensor],
            }
        status = {
            "clock_state": self.clock.state,
            "estimated_clock_offset_ms": (
                None
                if self.clock.offset_nanoseconds is None
                else round(self.clock.offset_nanoseconds / 1e6, 3)
            ),
            "invalid_streak": self.clock.invalid_streak,
            "scan_self_filtered_beams": self.last_scan_self_filtered_beams,
            "sensors": sensors,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))


def main() -> None:
    rclpy.init()
    node = SensorNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
