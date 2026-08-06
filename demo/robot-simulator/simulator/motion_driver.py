from __future__ import annotations

import errno
import logging
import socket
import time
from typing import Protocol
from uuid import uuid4

from simulator.config import SimulatorConfig
from simulator.control_protocol import (
    MOTION_PROTOCOL_VERSION,
    MotionDatagram,
    encode_motion_datagram,
)
from simulator.motion import MotionSimulator


logger = logging.getLogger("simulator.motion-driver")


class MotionDisabledError(RuntimeError):
    """Raised when this deployment deliberately has no chassis output path."""


class MotionDriver(Protocol):
    def set_velocity(self, linear_x: float, angular_z: float) -> None: ...

    def stop(self, reason: str = "") -> None: ...

    def watchdog(self, now: float | None = None) -> bool: ...

    def close(self) -> None: ...


class DisabledMotionDriver:
    """Read-only driver for legacy coexistence and mapping observation.

    Dropping a Web command while ACKing it would be dangerously misleading,
    so velocity requests fail explicitly. Stop remains an idempotent no-op
    because this process does not own the chassis output in this mode.
    """

    def set_velocity(self, _linear_x: float, _angular_z: float) -> None:
        raise MotionDisabledError(
            "Web motion is disabled while the legacy /cmd_vel owner is active"
        )

    def stop(self, _reason: str = "") -> None:
        return

    def watchdog(self, _now: float | None = None) -> bool:
        return False

    def close(self) -> None:
        return


class UnixMotionDriver:
    """Non-blocking latest-command transport to the local ROS 2 bridge."""

    def __init__(
        self,
        config: SimulatorConfig,
        *,
        transport: socket.socket | None = None,
        clock=time.monotonic,
    ) -> None:
        self.config = config
        self.socket = transport or socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.clock = clock
        self.boot_id = str(uuid4())
        self.sequence = 0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_command_monotonic = 0.0
        self._last_warning_monotonic = 0.0
        self._closed = False

    def set_velocity(self, linear_x: float, angular_z: float) -> None:
        self.linear_x = max(
            -self.config.ros_max_reverse_speed,
            min(self.config.ros_max_forward_speed, linear_x),
        )
        self.angular_z = max(
            -self.config.ros_max_angular_speed,
            min(self.config.ros_max_angular_speed, angular_z),
        )
        self.last_command_monotonic = self.clock()
        self._send("velocity", now=self.last_command_monotonic)

    def stop(self, reason: str = "") -> None:
        if self._closed:
            return
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_command_monotonic = 0.0
        # The bridge also has an independent watchdog. Repeating a small local
        # datagram is cheap and covers a transient full receive buffer.
        for _ in range(3):
            self._send("stop", reason=reason, now=self.clock())

    def watchdog(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else now
        if self.last_command_monotonic and (
            now - self.last_command_monotonic
        ) * 1000 > self.config.motion_watchdog_ms:
            was_moving = self.linear_x != 0.0 or self.angular_z != 0.0
            self.stop("edge_watchdog")
            return was_moving
        return False

    def close(self) -> None:
        if self._closed:
            return
        self.stop("edge_shutdown")
        self._closed = True
        self.socket.close()

    def _send(
        self,
        message_type: str,
        *,
        reason: str = "",
        now: float,
    ) -> bool:
        if self._closed:
            return False
        self.sequence += 1
        command = MotionDatagram(
            protocol_version=MOTION_PROTOCOL_VERSION,
            boot_id=self.boot_id,
            sequence=self.sequence,
            message_type=message_type,  # type: ignore[arg-type]
            sent_monotonic_ns=int(now * 1_000_000_000),
            ttl_ms=self.config.motion_watchdog_ms,
            linear_x=self.linear_x,
            angular_z=self.angular_z,
            reason=reason,
        )
        try:
            self.socket.sendto(
                encode_motion_datagram(command), self.config.motion_socket_path
            )
            return True
        except OSError as exc:
            if exc.errno not in {
                errno.EAGAIN,
                errno.EWOULDBLOCK,
                errno.ENOENT,
                errno.ECONNREFUSED,
            }:
                raise
            if now - self._last_warning_monotonic >= 5.0:
                logger.warning(
                    "ROS motion bridge unavailable path=%s error=%s",
                    self.config.motion_socket_path,
                    exc,
                )
                self._last_warning_monotonic = now
            return False


def build_motion_driver(
    config: SimulatorConfig,
    simulator: MotionSimulator,
) -> MotionDriver:
    if config.motion_backend == "simulator":
        return simulator
    if config.motion_backend == "disabled":
        return DisabledMotionDriver()
    return UnixMotionDriver(config)
