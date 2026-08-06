from __future__ import annotations

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class SensorNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("rovera_sensor_normalizer")
        self.odom_pub = self.create_publisher(Odometry, "/odometry/normalized", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/normalized", qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom_raw", self._odom, 10)
        self.create_subscription(Imu, "/imu", self._imu, qos_profile_sensor_data)

    def _odom(self, source: Odometry) -> None:
        message = copy.deepcopy(source)
        message.header.frame_id = "odom"
        message.child_frame_id = "base_footprint"
        # Vendor covariance is overly optimistic. Preserve measured axes but
        # keep unobserved Z/roll/pitch explicitly high for a planar EKF.
        message.pose.covariance[0] = max(message.pose.covariance[0], 0.01)
        message.pose.covariance[7] = max(message.pose.covariance[7], 0.01)
        message.pose.covariance[35] = max(message.pose.covariance[35], 0.03)
        message.twist.covariance[0] = max(message.twist.covariance[0], 0.02)
        message.twist.covariance[35] = max(message.twist.covariance[35], 0.04)
        self.odom_pub.publish(message)

    def _imu(self, source: Imu) -> None:
        message = copy.deepcopy(source)
        message.header.frame_id = "imu_frame"
        # Orientation is identity with zero covariance on current hardware and
        # is therefore marked unavailable. Angular/linear samples get bounded
        # covariance; EKF yaw fusion remains disabled until an axis test.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.08]
        message.linear_acceleration_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.36]
        self.imu_pub.publish(message)


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
