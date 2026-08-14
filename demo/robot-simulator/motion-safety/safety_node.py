from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String, UInt8

from safety_core import (
    Direction,
    SafetyConfig,
    ScanSample,
    StopHysteresis,
    evaluate_scan,
    motion_blocked_by_mask,
)


class MotionSafetyNode(Node):
    """Final fail-closed producer of /cmd_vel for every motion source."""

    def __init__(self) -> None:
        super().__init__("rovera_motion_safety")
        self.declare_parameter("scan_timeout", 0.28)
        self.declare_parameter("clear_hysteresis", 0.40)
        self.declare_parameter("lidar_obstacle_avoidance_enabled", True)
        self.declare_parameter("half_length", 0.20)
        self.declare_parameter("half_width", 0.18)
        self.declare_parameter("clearance", 0.10)
        self.declare_parameter("side_margin", 0.06)
        self.declare_parameter("rotation_margin", 0.04)
        self.declare_parameter("slow_extra", 0.20)
        self.declare_parameter("latency", 0.12)
        self.declare_parameter("braking_acceleration", 0.35)
        self.config = SafetyConfig(
            half_length=float(self.get_parameter("half_length").value),
            half_width=float(self.get_parameter("half_width").value),
            clearance=float(self.get_parameter("clearance").value),
            side_margin=float(self.get_parameter("side_margin").value),
            rotation_margin=float(self.get_parameter("rotation_margin").value),
            slow_extra=float(self.get_parameter("slow_extra").value),
            latency_seconds=float(self.get_parameter("latency").value),
            braking_acceleration=float(self.get_parameter("braking_acceleration").value),
            clear_hysteresis_seconds=float(self.get_parameter("clear_hysteresis").value),
            scan_timeout_seconds=float(self.get_parameter("scan_timeout").value),
        )
        self.lidar_obstacle_avoidance_enabled = bool(
            self.get_parameter("lidar_obstacle_avoidance_enabled").value
        )
        self.output = self.create_publisher(Twist, "/cmd_vel", 1)
        self.stop_state = self.create_publisher(Bool, "/safety/stop", 1)
        self.direction_state = self.create_publisher(UInt8, "/safety/directional_mask", 1)
        self.health = self.create_publisher(String, "/safety/health", 1)
        self.stop_source = self.create_publisher(String, "/safety/stop_source", 1)
        self.manual_takeover = self.create_publisher(Bool, "/safety/manual_takeover", 1)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_command, 1)
        self.create_subscription(Twist, "/cmd_vel_joy", self._on_joy, 1)
        self.create_subscription(Twist, "/cmd_vel_web", self._on_web, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "/safety/estop", self._on_estop, 1)
        self.create_subscription(Bool, "/safety/cliff", self._on_cliff, 1)
        self.create_subscription(Bool, "/safety/bumper", self._on_bumper, 1)
        self.create_subscription(Range, "/safety/range", self._on_range, qos_profile_sensor_data)
        self.create_subscription(
            UInt8,
            "/rovera/obstacle_directions",
            self._on_obstacle_directions,
            1,
        )
        self.create_subscription(
            Bool,
            "/rovera/obstacle_stop",
            self._on_obstacle_stop,
            1,
        )

        self.command = Twist()
        self.last_command = 0.0
        self.scan: ScanSample | None = None
        self.last_scan = 0.0
        self.estop = False
        self.cliff = False
        self.bumper = False
        self.range_stop = False
        self.external_stop = False
        self.external_directions = Direction.NONE
        self.hysteresis = StopHysteresis(self.config.clear_hysteresis_seconds)
        self.last_manual = 0.0
        self.create_timer(0.02, self._tick)
        if not self.lidar_obstacle_avoidance_enabled:
            self.get_logger().warning(
                "LiDAR obstacle avoidance is disabled by safety.yaml; "
                "external safety topics and hard-stop inputs remain active"
            )

    def _on_command(self, message: Twist) -> None:
        self.command = message
        self.last_command = time.monotonic()

    def _manual_seen(self) -> None:
        now = time.monotonic()
        if now - self.last_manual > 0.25:
            self.manual_takeover.publish(Bool(data=True))
        self.last_manual = now

    def _on_joy(self, message: Twist) -> None:
        if abs(message.linear.x) > 1e-4 or abs(message.angular.z) > 1e-4:
            self._manual_seen()

    def _on_web(self, message: Twist) -> None:
        if abs(message.linear.x) > 1e-4 or abs(message.angular.z) > 1e-4:
            self._manual_seen()

    def _on_scan(self, message: LaserScan) -> None:
        self.scan = ScanSample(
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            ranges=tuple(message.ranges),
        )
        self.last_scan = time.monotonic()

    def _on_estop(self, message: Bool) -> None:
        self.estop = bool(message.data)
        if self.estop:
            self._trip_immediately("estop")

    def _on_cliff(self, message: Bool) -> None:
        self.cliff = bool(message.data)
        if self.cliff:
            self._trip_immediately("cliff")

    def _on_bumper(self, message: Bool) -> None:
        self.bumper = bool(message.data)
        if self.bumper:
            self._trip_immediately("bumper")

    def _on_range(self, message: Range) -> None:
        self.range_stop = math.isfinite(message.range) and message.min_range <= message.range <= 0.20
        if self.range_stop:
            self._trip_immediately("range")

    def _on_obstacle_stop(self, message: Bool) -> None:
        self.external_stop = bool(message.data)
        if self.external_stop:
            self._trip_immediately("external_obstacle")

    def _on_obstacle_directions(self, message: UInt8) -> None:
        value = int(message.data)
        if value & ~int(
            Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT
        ):
            self.get_logger().warning(
                f"dropping invalid obstacle direction mask={value}"
            )
            return
        self.external_directions = Direction(value)
        if motion_blocked_by_mask(
            self.command.linear.x,
            self.command.angular.z,
            self.external_directions,
        ):
            self._trip_immediately(
                "external_direction",
                blocked=self.external_directions,
            )

    def _trip_immediately(
        self,
        reason: str,
        *,
        blocked: Direction = (
            Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT
        ),
    ) -> None:
        now = time.monotonic()
        self.hysteresis.update(True, now)
        self._publish_zero(reason, blocked)

    def _publish_zero(self, reason: str, blocked: Direction, *, healthy_idle: bool = False) -> None:
        self.output.publish(Twist())
        self.stop_state.publish(Bool(data=True))
        self.direction_state.publish(UInt8(data=int(blocked)))
        status = (
            "HEALTHY:IDLE"
            if healthy_idle
            else "BLOCKED:obstacle"
            if reason == "obstacle"
            else f"FAULT:{reason}"
        )
        self.health.publish(String(data=status))
        sources = {
            "obstacle": "MOTION_SAFETY",
            "external_obstacle": "EXTERNAL_OBSTACLE_STOP",
            "external_direction": "EXTERNAL_OBSTACLE_DIRECTION",
            "estop": "EMERGENCY_STOP",
            "cliff": "CLIFF",
            "bumper": "BUMPER",
            "range": "RANGE_SENSOR",
            "scan_timeout": "SCAN_TIMEOUT",
            "command_timeout": "COMMAND_TIMEOUT",
            "clear_hysteresis": "MOTION_SAFETY_HYSTERESIS",
        }
        self.stop_source.publish(String(data=sources.get(reason, reason.upper())))

    def _tick(self) -> None:
        now = time.monotonic()
        if self.estop or self.cliff or self.bumper or self.range_stop or self.external_stop:
            reason = (
                "estop"
                if self.estop
                else "cliff"
                if self.cliff
                else "bumper"
                if self.bumper
                else "range"
                if self.range_stop
                else "external_obstacle"
            )
            self.hysteresis.update(True, now)
            self._publish_zero(reason, Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT)
            return
        if now - self.last_command > 0.35:
            # A stationary robot with no active source is normal and safe. The
            # output still goes to zero, but preflight may remain healthy.
            self._publish_zero("command_timeout", Direction.NONE, healthy_idle=True)
            return
        if motion_blocked_by_mask(
            self.command.linear.x,
            self.command.angular.z,
            self.external_directions,
        ):
            self.hysteresis.update(True, now)
            self._publish_zero(
                "external_direction",
                self.external_directions,
            )
            return
        if not self.lidar_obstacle_avoidance_enabled:
            if self.hysteresis.update(False, now):
                self._publish_zero("clear_hysteresis", self.external_directions)
                return
            self.output.publish(self.command)
            self.stop_state.publish(Bool(data=False))
            self.direction_state.publish(UInt8(data=int(self.external_directions)))
            self.health.publish(String(data="HEALTHY:LIDAR_AVOIDANCE_DISABLED"))
            self.stop_source.publish(String(data="NONE"))
            return
        if self.scan is None or now - self.last_scan > self.config.scan_timeout_seconds:
            self.hysteresis.update(True, now)
            self._publish_zero("scan_timeout", Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT)
            return
        decision = evaluate_scan(
            self.scan,
            linear_x=self.command.linear.x,
            angular_z=self.command.angular.z,
            config=self.config,
        )
        if self.hysteresis.update(decision.stop, now):
            self._publish_zero(decision.reason or "clear_hysteresis", decision.blocked)
            return
        safe = Twist()
        safe.linear.x = self.command.linear.x * decision.speed_scale
        safe.linear.y = self.command.linear.y * decision.speed_scale
        safe.angular.z = self.command.angular.z * decision.speed_scale
        self.output.publish(safe)
        self.stop_state.publish(Bool(data=False))
        self.direction_state.publish(
            UInt8(data=int(decision.blocked | self.external_directions))
        )
        self.health.publish(String(data="HEALTHY"))
        self.stop_source.publish(String(data="NONE"))


def main() -> None:
    rclpy.init()
    node = MotionSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
