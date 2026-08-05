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
from std_msgs.msg import Bool, UInt8

from simulator.control_protocol import (
    LatestMotionSlot,
    MAX_MOTION_DATAGRAM_BYTES,
    MotionDatagram,
    SafetyInterlock,
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


def _nonnegative_int_env(name: str, default: int) -> int:
    try:
        value = int(os.getenv(name, str(default)))
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer") from exc
    if value < 0:
        raise RuntimeError(f"{name} must not be negative")
    return value


def _bool_env(name: str, default: bool) -> bool:
    raw = os.getenv(name, str(default)).strip().lower()
    if raw in {"1", "true", "yes", "on"}:
        return True
    if raw in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be a boolean")


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
        self.use_twist_mux = _bool_env("ROS_USE_TWIST_MUX", False)
        self.obstacle_watchdog_ms = _nonnegative_int_env(
            "ROS_OBSTACLE_WATCHDOG_MS", 0
        )
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
        self.safety_velocity_publisher = self.create_publisher(
            Twist,
            os.getenv("ROS_SAFETY_CMD_VEL_TOPIC", "/cmd_vel_safety"),
            qos,
        )
        self.estop_publisher = self.create_publisher(
            Bool, os.getenv("ROS_ESTOP_TOPIC", "/rovera/emergency_stop"), qos
        )
        self.obstacle_stop_subscription = self.create_subscription(
            Bool,
            os.getenv("ROS_OBSTACLE_STOP_TOPIC", "/rovera/obstacle_stop"),
            self._on_obstacle_stop,
            qos,
        )
        self.obstacle_directions_subscription = self.create_subscription(
            UInt8,
            os.getenv(
                "ROS_OBSTACLE_DIRECTIONS_TOPIC",
                "/rovera/obstacle_directions",
            ),
            self._on_obstacle_directions,
            qos,
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
        self._last_latency_log_monotonic = 0.0
        self._legacy_override_until = 0.0
        self._legacy_mode_active = False
        self._legacy_override_logged = False
        self._stop_burst_remaining = 0
        self._estop_latched = False
        self._obstacle_interlock = SafetyInterlock(self.obstacle_watchdog_ms)
        # Avoid a misleading "released" transition on startup when the
        # optional heartbeat watchdog is disabled. A positive watchdog still
        # starts fail-closed and logs the initial lock transition.
        self._obstacle_lock_reported = False
        self._obstacle_directions_reported = 0
        self._last_safety_zero_monotonic = 0.0
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
        self._sync_obstacle_lock_state(time.monotonic())
        self.get_logger().info(
            "motion bridge ready "
            f"path={self.socket_path} watchdog_ms={self.watchdog_ms} "
            f"output_hz={self.command_rate_hz} "
            f"obstacle_watchdog_ms={self.obstacle_watchdog_ms}"
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
            if self._estop_latched or self._sync_obstacle_lock_state(now):
                self._publish_safety_zero(now)
                return
            if self._legacy_override_active(now):
                self._stop_burst_remaining = 0
                return
            self._publish_zero_burst()
            return

        if self._sync_obstacle_lock_state(now):
            self._active_command = None
            self._stop_burst_remaining = 0
            self._publish_safety_zero(now)
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
        if now - self._last_latency_log_monotonic >= 1.0:
            ipc_age_ms = max(
                0.0,
                (time.monotonic_ns() - command.sent_monotonic_ns) / 1_000_000,
            )
            self.get_logger().info(
                "control latency edge_to_ros_ms="
                f"{ipc_age_ms:.3f} sequence={command.sequence}"
            )
            self._last_latency_log_monotonic = now

    def _tick(self) -> None:
        now = time.monotonic()
        # Republish the lock state so a restarted twist_mux cannot miss an
        # already-latched software emergency stop on the volatile ROS topic.
        if now - self._last_estop_publish_monotonic >= 0.5:
            self._publish_estop_state(now)
        if self._estop_latched or self._sync_obstacle_lock_state(now):
            self._active_command = None
            self._stop_burst_remaining = 0
            if now - self._last_safety_zero_monotonic >= 1 / self.command_rate_hz:
                self._publish_safety_zero(now)
            return
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

    def _on_obstacle_stop(self, message: Bool) -> None:
        now = time.monotonic()
        self._obstacle_interlock.update(bool(message.data), now)
        if self._sync_obstacle_lock_state(now):
            self._active_command = None
            self._stop_burst_remaining = 0
            # Do not wait for the 100 Hz timer before issuing the first stop.
            self._publish_safety_zero(now)

    def _on_obstacle_directions(self, message: UInt8) -> None:
        now = time.monotonic()
        try:
            self._obstacle_interlock.update_directions(int(message.data), now)
        except ValueError as exc:
            self.get_logger().warning(f"dropping obstacle directions: {exc}")
            return
        if message.data != self._obstacle_directions_reported:
            self._obstacle_directions_reported = int(message.data)
            self.get_logger().info(
                f"obstacle direction mask changed mask={message.data}"
            )
        if self._sync_obstacle_lock_state(now):
            self._active_command = None
            self._stop_burst_remaining = 0
            self._publish_safety_zero(now)
            return
        # Apply a newly blocked direction to the current command immediately;
        # never wait for the next browser packet to stop that component.
        if self._active_command is not None:
            self._publish_velocity(self._active_command, now=now)
            self._last_publish_monotonic = now

    def _sync_obstacle_lock_state(self, now: float) -> bool:
        locked = self._obstacle_interlock.locked(now)
        if locked == self._obstacle_lock_reported:
            return locked
        self._obstacle_lock_reported = locked
        if locked:
            self._active_command = None
            self._stop_burst_remaining = 0
            self.get_logger().warning(
                "obstacle safety stop engaged "
                f"reason={self._obstacle_interlock.reason(now)}"
            )
        else:
            self.get_logger().info(
                "obstacle safety stop released; waiting for fresh velocity"
            )
        return locked

    def _legacy_override_active(self, now: float) -> bool:
        active = self._legacy_mode_active or now < self._legacy_override_until
        if not active:
            self._legacy_override_logged = False
        return active

    def _publish_velocity(
        self, command: MotionDatagram, *, now: float | None = None
    ) -> None:
        message = Twist()
        linear_x = max(
            -self.max_reverse, min(self.max_forward, command.linear_x)
        )
        angular_z = max(
            -self.max_angular, min(self.max_angular, command.angular_z)
        )
        message.linear.x, message.angular.z = (
            self._obstacle_interlock.filter_velocity(linear_x, angular_z, now)
        )
        self.velocity_publisher.publish(message)

    def _publish_zero(self) -> None:
        self.velocity_publisher.publish(Twist())

    def _publish_safety_zero(self, now: float | None = None) -> None:
        message = Twist()
        # In mux mode this priority-255 input is the only velocity allowed
        # through while a safety lock is active. In parallel legacy mode the
        # regular web output is /cmd_vel, so publish zero there as well.
        self.safety_velocity_publisher.publish(message)
        if not self.use_twist_mux:
            self.velocity_publisher.publish(message)
        self._last_safety_zero_monotonic = (
            time.monotonic() if now is None else now
        )

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
        for _ in range(3):
            self._publish_zero()
            self._publish_safety_zero()
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
