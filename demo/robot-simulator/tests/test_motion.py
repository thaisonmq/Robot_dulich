import math

from simulator.config import SimulatorConfig
from simulator.motion import MotionSimulator, normalize_yaw


def test_normalize_yaw() -> None:
    assert -math.pi <= normalize_yaw(9 * math.pi) <= math.pi
    assert normalize_yaw(0) == 0


def test_pose_moves_forward() -> None:
    motion = MotionSimulator(SimulatorConfig(initial_x=5, initial_y=5, initial_yaw=0))
    motion.set_velocity(0.4, 0)
    pose = motion.step(1.0)
    assert pose.x == 5.4
    assert pose.y == 5


def test_watchdog_stops() -> None:
    motion = MotionSimulator(SimulatorConfig())
    motion.set_velocity(0.4, 0)
    assert motion.watchdog(motion.last_command_monotonic + 0.5)
    assert motion.linear_x == 0

