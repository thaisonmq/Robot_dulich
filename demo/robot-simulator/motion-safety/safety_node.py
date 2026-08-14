from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String, UInt8

from safety_core import (
    Direction,
    SafetyConfig,
    ScanSample,
    StopHysteresis,
    clip_motion_by_mask,
    evaluate_scan,
)


class MotionSafetyNode(Node):
    """Final fail-closed producer of /cmd_vel for every motion source."""

    def __init__(self) -> None:
        super().__init__("rovera_motion_safety")
        self.declare_parameter("scan_timeout", 0.28)
        self.declare_parameter("clear_hysteresis", 0.20)
        self.declare_parameter("lidar_obstacle_avoidance_enabled", True)
        self.declare_parameter("half_length", 0.15)
        self.declare_parameter("half_width", 0.10)
        self.declare_parameter("clearance", 0.04)
        self.declare_parameter("side_margin", 0.04)
        self.declare_parameter("rotation_margin", 0.03)
        self.declare_parameter("slow_extra", 0.10)
        self.declare_parameter("latency", 0.12)
        self.declare_parameter("braking_acceleration", 0.60)
        self.declare_parameter("angular_braking_acceleration", 2.0)
        self.declare_parameter("rotation_preview_angle", 0.30)
        self.declare_parameter("trajectory_samples", 6)
        self.config = SafetyConfig(
            half_length=float(self.get_parameter("half_length").value),
            half_width=float(self.get_parameter("half_width").value),
            clearance=float(self.get_parameter("clearance").value),
            side_margin=float(self.get_parameter("side_margin").value),
            rotation_margin=float(self.get_parameter("rotation_margin").value),
            slow_extra=float(self.get_parameter("slow_extra").value),
            latency_seconds=float(self.get_parameter("latency").value),
            braking_acceleration=float(self.get_parameter("braking_acceleration").value),
            angular_braking_acceleration=float(
                self.get_parameter("angular_braking_acceleration").value
            ),
            rotation_preview_angle=float(
                self.get_parameter("rotation_preview_angle").value
            ),
            trajectory_samples=int(self.get_parameter("trajectory_samples").value),
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
        self.create_subscription(Twist, "/cmd_vel_muxed", self._on_muxed, 1)
        self.create_subscription(Twist, "/cmd_vel_joy", self._on_joy, 1)
        self.create_subscription(Twist, "/cmd_vel_web", self._on_web, 1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self._on_odometry,
            qos_profile_sensor_data,
        )
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
        self.measured_linear_x = 0.0
        self.last_odometry = 0.0
        self.estop = False
        self.cliff = False
        self.bumper = False
        self.range_stop = False
        self.external_stop = False
        self.external_directions = Direction.NONE
        self.fault_hysteresis = StopHysteresis(
            self.config.clear_hysteresis_seconds
        )
        self.direction_hysteresis = {
            direction: StopHysteresis(self.config.clear_hysteresis_seconds)
            for direction in (
                Direction.FRONT,
                Direction.REAR,
                Direction.LEFT,
                Direction.RIGHT,
            )
        }
        self.manual_input = (0.0, 0.0)
        self.muxed_input = (0.0, 0.0)
        self.last_manual = 0.0
        self.debug_enabled = os.getenv(
            "NAVIGATION_DEBUG_LOG", "true"
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.last_safety_log_monotonic = 0.0
        self.last_safety_log_signature: tuple[object, ...] | None = None
        self.create_timer(0.02, self._tick)
        if not self.lidar_obstacle_avoidance_enabled:
            self.get_logger().warning(
                "LiDAR obstacle avoidance is disabled by safety.yaml; "
                "external safety topics and hard-stop inputs remain active"
            )

    def _on_command(self, message: Twist) -> None:
        self.command = message
        self.last_command = time.monotonic()

    def _on_muxed(self, message: Twist) -> None:
        self.muxed_input = (float(message.linear.x), float(message.angular.z))

    def _manual_seen(self) -> None:
        now = time.monotonic()
        if now - self.last_manual > 0.25:
            self.manual_takeover.publish(Bool(data=True))
        self.last_manual = now

    def _on_joy(self, message: Twist) -> None:
        self.manual_input = (float(message.linear.x), float(message.angular.z))
        if abs(message.linear.x) > 1e-4 or abs(message.angular.z) > 1e-4:
            self._manual_seen()

    def _on_web(self, message: Twist) -> None:
        self.manual_input = (float(message.linear.x), float(message.angular.z))
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

    def _on_odometry(self, message: Odometry) -> None:
        self.measured_linear_x = float(message.twist.twist.linear.x)
        self.last_odometry = time.monotonic()

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
        # Compatibility masks are directional geometry, not hard faults. They
        # are clipped component-wise in _tick and never arm full-stop hold.

    def _log_safety(
        self,
        *,
        input_linear: float,
        input_angular: float,
        output_linear: float,
        output_angular: float,
        blocked: Direction,
        source: str,
        decision: object | None = None,
    ) -> None:
        if not self.debug_enabled:
            return
        signature = (
            round(input_linear, 3), round(input_angular, 3),
            round(output_linear, 3), round(output_angular, 3),
            int(blocked), source,
        )
        now = time.monotonic()
        if (
            signature == self.last_safety_log_signature
            and now - self.last_safety_log_monotonic < 0.5
        ):
            return
        fields = {
            "manual_input_v": self.manual_input[0],
            "manual_input_w": self.manual_input[1],
            "after_mux_v": self.muxed_input[0],
            "after_mux_w": self.muxed_input[1],
            "after_smoother_v": input_linear,
            "after_smoother_w": input_angular,
            "input_v": input_linear,
            "input_w": input_angular,
            "output_v": output_linear,
            "output_w": output_angular,
            "blocked_directions": int(blocked),
            "source": source,
        }
        if decision is not None:
            fields.update({
                "front_clearance": getattr(decision, "front_clearance", math.inf),
                "rear_clearance": getattr(decision, "rear_clearance", math.inf),
                "left_clearance": getattr(decision, "left_clearance", math.inf),
                "right_clearance": getattr(decision, "right_clearance", math.inf),
                "forward_blocked": bool(blocked & Direction.FRONT),
                "reverse_blocked": bool(blocked & Direction.REAR),
                "turn_left_blocked": bool(blocked & Direction.LEFT),
                "turn_right_blocked": bool(blocked & Direction.RIGHT),
                "hard_stop": bool(getattr(decision, "hard_stop", False)),
                "slowdown_scale": getattr(decision, "speed_scale", 1.0),
                "required_stop_distance": getattr(
                    decision, "required_stop_distance", 0.0
                ),
                "reason": getattr(decision, "reason", source),
            })
        rendered = " ".join(f"{key}={value}" for key, value in fields.items())
        self.get_logger().info(f"[NAV][SAFETY] {rendered}")
        self.last_safety_log_signature = signature
        self.last_safety_log_monotonic = now

    def _trip_immediately(
        self,
        reason: str,
        *,
        blocked: Direction = (
            Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT
        ),
    ) -> None:
        now = time.monotonic()
        self.fault_hysteresis.update(True, now)
        self._publish_zero(reason, blocked)

    def _publish_zero(self, reason: str, blocked: Direction, *, healthy_idle: bool = False) -> None:
        self.output.publish(Twist())
        self.stop_state.publish(Bool(data=True))
        self.direction_state.publish(UInt8(data=int(blocked)))
        lidar_reasons = {
            "front_sweep_collision",
            "rear_sweep_collision",
            "rotation_sweep_collision",
            "empty_scan",
        }
        status = (
            "HEALTHY:IDLE"
            if healthy_idle
            else f"BLOCKED:{reason}"
            if reason in lidar_reasons
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
            "fault_hysteresis": "MOTION_SAFETY_FAULT_HYSTERESIS",
        }
        source = "MOTION_SAFETY" if reason in lidar_reasons else sources.get(
            reason, reason.upper()
        )
        self.stop_source.publish(String(data=source))

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
            self.fault_hysteresis.update(True, now)
            self._publish_zero(reason, Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT)
            return
        if self.fault_hysteresis.update(False, now):
            self._publish_zero(
                "fault_hysteresis",
                Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT,
            )
            return
        if now - self.last_command > 0.35:
            # A stationary robot with no active source is normal and safe. The
            # output still goes to zero, but preflight may remain healthy.
            self._publish_zero("command_timeout", Direction.NONE, healthy_idle=True)
            return
        masked_linear, masked_angular = clip_motion_by_mask(
            self.command.linear.x,
            self.command.angular.z,
            self.external_directions,
        )
        if (
            (abs(self.command.linear.x) > 1e-4 or abs(self.command.angular.z) > 1e-4)
            and abs(masked_linear) <= 1e-4
            and abs(masked_angular) <= 1e-4
        ):
            self._publish_zero(
                "external_direction",
                self.external_directions,
            )
            self._log_safety(
                input_linear=self.command.linear.x,
                input_angular=self.command.angular.z,
                output_linear=0.0,
                output_angular=0.0,
                blocked=self.external_directions,
                source="EXTERNAL_OBSTACLE_DIRECTION",
            )
            return
        candidate = Twist()
        candidate.linear.x = masked_linear
        candidate.linear.y = self.command.linear.y
        candidate.angular.z = masked_angular
        if not self.lidar_obstacle_avoidance_enabled:
            self.output.publish(candidate)
            self.stop_state.publish(Bool(data=False))
            self.direction_state.publish(UInt8(data=int(self.external_directions)))
            self.health.publish(String(data="HEALTHY:LIDAR_AVOIDANCE_DISABLED"))
            self.stop_source.publish(String(data="NONE"))
            if masked_linear != self.command.linear.x or masked_angular != self.command.angular.z:
                self._log_safety(
                    input_linear=self.command.linear.x,
                    input_angular=self.command.angular.z,
                    output_linear=candidate.linear.x,
                    output_angular=candidate.angular.z,
                    blocked=self.external_directions,
                    source="EXTERNAL_OBSTACLE_DIRECTION",
                )
            return
        if self.scan is None or now - self.last_scan > self.config.scan_timeout_seconds:
            self.fault_hysteresis.update(True, now)
            self._publish_zero("scan_timeout", Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT)
            return
        decision = evaluate_scan(
            self.scan,
            linear_x=candidate.linear.x,
            angular_z=candidate.angular.z,
            config=self.config,
            measured_linear_x=(
                self.measured_linear_x
                if now - self.last_odometry <= 0.30
                else candidate.linear.x
            ),
        )
        if decision.reason == "empty_scan":
            self.fault_hysteresis.update(True, now)
            self._publish_zero(
                "empty_scan",
                Direction.FRONT | Direction.REAR | Direction.LEFT | Direction.RIGHT,
            )
            return
        raw_blocked = decision.blocked | self.external_directions
        held_blocked = Direction.NONE
        for direction, gate in self.direction_hysteresis.items():
            if gate.update(bool(raw_blocked & direction), now):
                held_blocked |= direction
        safe_linear, safe_angular = clip_motion_by_mask(
            candidate.linear.x,
            candidate.angular.z,
            held_blocked,
        )
        safe = Twist()
        safe.linear.x = safe_linear * decision.speed_scale
        safe.linear.y = candidate.linear.y * decision.speed_scale
        safe.angular.z = safe_angular
        self.output.publish(safe)
        all_requested_components_blocked = (
            (abs(candidate.linear.x) > 1e-4 or abs(candidate.angular.z) > 1e-4)
            and abs(safe.linear.x) <= 1e-4
            and abs(safe.angular.z) <= 1e-4
        )
        self.stop_state.publish(Bool(data=all_requested_components_blocked))
        self.direction_state.publish(
            UInt8(data=int(held_blocked))
        )
        modified = (
            abs(safe.linear.x - self.command.linear.x) > 1e-4
            or abs(safe.angular.z - self.command.angular.z) > 1e-4
        )
        self.health.publish(String(data=(
            f"BLOCKED:{decision.reason or 'direction_hysteresis'}"
            if all_requested_components_blocked
            else "HEALTHY:CLIPPED"
            if modified
            else "HEALTHY"
        )))
        self.stop_source.publish(String(data=(
            "MOTION_SAFETY" if all_requested_components_blocked else "NONE"
        )))
        if modified:
            self._log_safety(
                input_linear=self.command.linear.x,
                input_angular=self.command.angular.z,
                output_linear=safe.linear.x,
                output_angular=safe.angular.z,
                blocked=held_blocked,
                source=(
                    decision.reason
                    or (
                        "EXTERNAL_OBSTACLE_DIRECTION"
                        if self.external_directions
                        else "BRAKING_SLOWDOWN"
                    )
                ),
                decision=decision,
            )


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
