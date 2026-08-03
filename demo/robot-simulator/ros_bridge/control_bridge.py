from __future__ import annotations

import logging
import math
import os
import socket
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from simulator.control_protocol import (
    LatestMotionSlot,
    MAX_MOTION_DATAGRAM_BYTES,
    MotionDatagram,
    decode_motion_datagram,
    joy_input_active,
)


logger = logging.getLogger("rovera.ros-control-bridge")


def _float_env(name: str, default: float) -> float:
    try:
        value = float(os.getenv(name, str(default)))
    except ValueError as exc:
        raise RuntimeError(f"{name} must be numeric") from exc
    if not math.isfinite(value) or value <= 0:
        raise RuntimeError(f"{name} must be positive and finite")
    return value


def _int_env(name: str, default: int) -> int:
    try:
        value = int(os.getenv(name, str(default)))
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer") from exc
    if value <= 0:
        raise RuntimeError(f"{name} must be positive")
    return value


def _axis_indices_env(name: str, default: str) -> tuple[int, ...]:
    try:
        indices = tuple(
            int(item.strip())
            for item in os.getenv(name, default).split(",")
            if item.strip()
        )
    except ValueError as exc:
        raise RuntimeError(f"{name} must be comma-separated integers") from exc
    if not indices or any(index < 0 for index in indices):
        raise RuntimeError(f"{name} must contain non-negative axis indices")
    return indices


class RosControlBridge(Node):
    """Latest-only local IPC bridge. It never queues stale movement commands."""

    def __init__(self) -> None:
        super().__init__("rovera_control_bridge")
        self.socket_path = Path(
            os.getenv(
                "MOTION_SOCKET_PATH", "/var/lib/rovera/control/motion.sock"
            )
        )
        self.watchdog_ms = _int_env("MOTION_WATCHDOG_MS", 250)
        self.command_rate_hz = _int_env("ROS_COMMAND_RATE_HZ", 30)
        self.max_forward = _float_env("ROS_MAX_FORWARD_SPEED", 0.33)
        self.max_reverse = _float_env("ROS_MAX_REVERSE_SPEED", 0.25)
        self.max_angular = _float_env("ROS_MAX_ANGULAR_SPEED", 0.8)
        self.legacy_joy_deadzone = _float_env("ROS_LEGACY_JOY_DEADZONE", 0.12)
        self.legacy_joy_axes = _axis_indices_env("ROS_LEGACY_JOY_AXES", "1,2")
        self.legacy_override_ms = _int_env("ROS_LEGACY_OVERRIDE_MS", 350)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.velocity_publisher = self.create_publisher(
            Twist, os.getenv("ROS_WEB_CMD_VEL_TOPIC", "/cmd_vel_web"), qos
        )
        self.estop_publisher = self.create_publisher(
            Bool, os.getenv("ROS_ESTOP_TOPIC", "/rovera/emergency_stop"), qos
        )
        self.legacy_joy_subscription = self.create_subscription(
            Joy,
            os.getenv("ROS_LEGACY_JOY_TOPIC", "/joy"),
            self._on_legacy_joy,
            qos_profile_sensor_data,
        )
        self.legacy_state_subscription = self.create_subscription(
            Bool,
            os.getenv("ROS_LEGACY_JOY_STATE_TOPIC", "/JoyState"),
            self._on_legacy_state,
            10,
        )

        self._lock = threading.Lock()
        self._pending = LatestMotionSlot()
        self._active_command: MotionDatagram | None = None
        self._last_receive_monotonic = 0.0
        self._last_publish_monotonic = 0.0
        self._last_estop_publish_monotonic = 0.0
        self._legacy_override_until = 0.0
        self._legacy_mode_active = False
        self._legacy_override_logged = False
        self._stop_burst_remaining = 0
        self._estop_latched = False
        self._closing = threading.Event()
        self._guard = self.create_guard_condition(self._apply_pending)
        self._socket = self._open_socket()
        self._receiver = threading.Thread(
            target=self._receive_loop,
            name="rovera-motion-ipc",
            daemon=True,
        )
        self._receiver.start()
        self._timer = self.create_timer(0.01, self._tick)
        self._publish_estop_state()
        self.get_logger().info(
            "motion bridge ready "
            f"path={self.socket_path} watchdog_ms={self.watchdog_ms} "
            f"output_hz={self.command_rate_hz}"
        )

    def _open_socket(self) -> socket.socket:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        server.bind(str(self.socket_path))
        self.socket_path.chmod(0o660)
        server.settimeout(0.1)
        return server

    def _receive_loop(self) -> None:
        while not self._closing.is_set():
            try:
                payload = self._socket.recv(MAX_MOTION_DATAGRAM_BYTES + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._closing.is_set():
                    return
                logger.exception("motion IPC receive failed")
                continue

            newest = self._decode(payload)
            try:
                self._socket.setblocking(False)
                while True:
                    try:
                        candidate = self._decode(
                            self._socket.recv(MAX_MOTION_DATAGRAM_BYTES + 1)
                        )
                    except BlockingIOError:
                        break
                    if candidate is not None:
                        newest = candidate
            except OSError:
                if self._closing.is_set():
                    return
                logger.exception("motion IPC drain failed")
                continue
            finally:
                try:
                    self._socket.settimeout(0.1)
                except OSError:
                    if self._closing.is_set():
                        return
                    logger.exception("motion IPC timeout restore failed")
                    return

            if newest is not None and self._stage(newest):
                self._guard.trigger()

    def _decode(self, payload: bytes) -> MotionDatagram | None:
        try:
            return decode_motion_datagram(payload)
        except ValueError as exc:
            self.get_logger().warning(f"dropping motion datagram: {exc}")
            return None

    def _stage(self, command: MotionDatagram) -> bool:
        with self._lock:
            return self._pending.stage(command)

    def _apply_pending(self) -> None:
        with self._lock:
            command = self._pending.take()
        if command is None:
            return
        if command.expired():
            self.get_logger().warning(
                f"dropping expired staged command sequence={command.sequence}"
            )
            return

        now = time.monotonic()
        self._last_receive_monotonic = now
        if command.message_type == "stop":
            self._active_command = None
            if command.reason == "emergency_stop":
                self._set_estop(True)
            if self._legacy_override_active(now):
                self._stop_burst_remaining = 0
                return
            self._publish_zero_burst()
            return

        if self._legacy_override_active(now):
            # The unchanged Yahboom stack still publishes directly to
            # /cmd_vel. Do not interleave web Twist messages while its physical
            # joystick is being operated.
            self._active_command = None
            if not self._legacy_override_logged:
                self.get_logger().info(
                    "physical joystick active; suppressing web velocity"
                )
                self._legacy_override_logged = True
            return

        if self._estop_latched:
            # A fresh deliberate movement command re-arms the software stop.
            # A physical emergency-stop circuit is still required on the car.
            self._set_estop(False)
        self._active_command = command
        self._stop_burst_remaining = 0
        self._publish_velocity(command)
        self._last_publish_monotonic = now

    def _tick(self) -> None:
        now = time.monotonic()
        # Republish the lock state so a restarted twist_mux cannot miss an
        # already-latched software emergency stop on the volatile ROS topic.
        if now - self._last_estop_publish_monotonic >= 0.5:
            self._publish_estop_state(now)
        command = self._active_command
        if command is not None:
            if self._legacy_override_active(now):
                self._active_command = None
                return
            stale_ms = (now - self._last_receive_monotonic) * 1000
            if stale_ms > self.watchdog_ms:
                self._active_command = None
                self.get_logger().warning(
                    f"motion watchdog stopped output stale_ms={stale_ms:.1f}"
                )
                self._publish_zero_burst()
                return
            if now - self._last_publish_monotonic >= 1 / self.command_rate_hz:
                self._publish_velocity(command)
                self._last_publish_monotonic = now
                return
        if self._stop_burst_remaining > 0:
            self._publish_zero()
            self._stop_burst_remaining -= 1

    def _on_legacy_joy(self, message: Joy) -> None:
        if not joy_input_active(
            message.axes,
            message.buttons,
            deadzone=self.legacy_joy_deadzone,
            axis_indices=self.legacy_joy_axes,
        ):
            return
        now = time.monotonic()
        self._legacy_override_until = max(
            self._legacy_override_until,
            now + self.legacy_override_ms / 1000,
        )
        self._active_command = None
        self._stop_burst_remaining = 0

    def _on_legacy_state(self, message: Bool) -> None:
        self._legacy_mode_active = bool(message.data)
        if self._legacy_mode_active:
            self._active_command = None
            self._stop_burst_remaining = 0
        else:
            self._legacy_override_until = max(
                self._legacy_override_until,
                time.monotonic() + self.legacy_override_ms / 1000,
            )

    def _legacy_override_active(self, now: float) -> bool:
        active = self._legacy_mode_active or now < self._legacy_override_until
        if not active:
            self._legacy_override_logged = False
        return active

    def _publish_velocity(self, command: MotionDatagram) -> None:
        message = Twist()
        message.linear.x = max(
            -self.max_reverse, min(self.max_forward, command.linear_x)
        )
        message.angular.z = max(
            -self.max_angular, min(self.max_angular, command.angular_z)
        )
        self.velocity_publisher.publish(message)

    def _publish_zero(self) -> None:
        self.velocity_publisher.publish(Twist())

    def _publish_zero_burst(self) -> None:
        self._publish_zero()
        self._stop_burst_remaining = 2

    def _set_estop(self, active: bool) -> None:
        if active == self._estop_latched:
            return
        self._estop_latched = active
        self._publish_estop_state()

    def _publish_estop_state(self, now: float | None = None) -> None:
        self.estop_publisher.publish(Bool(data=self._estop_latched))
        self._last_estop_publish_monotonic = (
            time.monotonic() if now is None else now
        )

    def close(self) -> None:
        if self._closing.is_set():
            return
        self._active_command = None
        self._publish_zero()
        self._publish_zero()
        self._publish_zero()
        self._closing.set()
        self._socket.close()
        self._receiver.join(timeout=1.0)
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s service=ros-control-bridge %(name)s %(message)s",
    )
    rclpy.init()
    bridge = RosControlBridge()
    try:
        rclpy.spin(bridge)
    finally:
        bridge.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
