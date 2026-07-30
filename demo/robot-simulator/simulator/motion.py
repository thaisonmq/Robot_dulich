import math
import time
from dataclasses import dataclass

from simulator.config import SimulatorConfig


def normalize_yaw(yaw: float) -> float:
    return (yaw + math.pi) % (2 * math.pi) - math.pi


@dataclass(slots=True)
class Pose:
    x: float
    y: float
    yaw: float
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


class MotionSimulator:
    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.pose = Pose(config.initial_x, config.initial_y, config.initial_yaw)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_command_monotonic = 0.0

    def set_velocity(self, linear_x: float, angular_z: float) -> None:
        self.linear_x = max(-self.config.max_reverse_speed, min(self.config.max_forward_speed, linear_x))
        self.angular_z = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, angular_z))
        self.last_command_monotonic = time.monotonic()

    def stop(self) -> None:
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.pose.linear_velocity = 0.0
        self.pose.angular_velocity = 0.0

    def watchdog(self, now: float | None = None) -> bool:
        now = now if now is not None else time.monotonic()
        if self.last_command_monotonic and (
            now - self.last_command_monotonic
        ) * 1000 > self.config.command_watchdog_ms:
            was_moving = self.linear_x != 0 or self.angular_z != 0
            self.stop()
            self.last_command_monotonic = 0
            return was_moving
        return False

    def step(self, dt: float) -> Pose:
        self.pose.yaw = normalize_yaw(self.pose.yaw + self.angular_z * dt)
        next_x = self.pose.x + self.linear_x * math.cos(self.pose.yaw) * dt
        next_y = self.pose.y + self.linear_x * math.sin(self.pose.yaw) * dt
        margin = 0.25
        if margin <= next_x <= self.config.map_width_m - margin:
            self.pose.x = next_x
        else:
            self.linear_x = 0
        if margin <= next_y <= self.config.map_height_m - margin:
            self.pose.y = next_y
        else:
            self.linear_x = 0
        self.pose.linear_velocity = self.linear_x
        self.pose.angular_velocity = self.angular_z
        return self.pose

    def as_payload(self) -> dict:
        return {
            "map_id": self.config.map_id,
            "x": round(self.pose.x, 4),
            "y": round(self.pose.y, 4),
            "yaw": round(self.pose.yaw, 4),
            "linear_velocity": round(self.pose.linear_velocity, 3),
            "angular_velocity": round(self.pose.angular_velocity, 3),
        }

