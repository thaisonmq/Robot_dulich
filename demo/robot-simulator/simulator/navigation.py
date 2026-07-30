import math
from dataclasses import dataclass, field

from simulator.motion import MotionSimulator, normalize_yaw


@dataclass(slots=True)
class NavigationSimulator:
    motion: MotionSimulator
    status: str = "idle"
    route_id: str = ""
    points: list[dict[str, float]] = field(default_factory=list)
    point_index: int = 0

    def start(self, route_id: str, points: list[dict[str, float]]) -> None:
        self.route_id = route_id
        self.points = points
        self.point_index = 1 if len(points) > 1 else 0
        self.status = "moving" if points else "failed"

    def cancel(self) -> None:
        self.points = []
        self.status = "cancelled"
        self.motion.stop()

    def update(self) -> bool:
        if self.status != "moving" or self.point_index >= len(self.points):
            return False
        target = self.points[self.point_index]
        dx = target["x"] - self.motion.pose.x
        dy = target["y"] - self.motion.pose.y
        distance = math.hypot(dx, dy)
        if distance < 0.12:
            self.point_index += 1
            if self.point_index >= len(self.points):
                self.status = "arrived"
                self.motion.stop()
                return True
            return False
        target_yaw = math.atan2(dy, dx)
        error = normalize_yaw(target_yaw - self.motion.pose.yaw)
        angular = max(-0.8, min(0.8, error * 2.2))
        linear = min(0.35, distance) if abs(error) < 0.6 else 0.0
        self.motion.set_velocity(linear, angular)
        return False

