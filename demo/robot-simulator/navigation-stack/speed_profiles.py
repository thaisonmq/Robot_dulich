from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


SPEED_MODES = ("SLOW", "NORMAL", "FAST")


class SpeedProfileError(ValueError):
    """Raised when a speed profile or requested mode is unsafe/invalid."""


def normalize_speed_mode(value: object) -> str:
    mode = str(value).strip().upper()
    if mode not in SPEED_MODES:
        raise SpeedProfileError(
            f"Invalid auto navigation speed mode {value!r}; expected SLOW, NORMAL or FAST"
        )
    return mode


def _positive(mapping: Mapping[str, Any], name: str) -> float:
    try:
        value = float(mapping[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise SpeedProfileError(f"{name} must be numeric") from exc
    if not math.isfinite(value) or value <= 0:
        raise SpeedProfileError(f"{name} must be positive and finite")
    return value


@dataclass(frozen=True, slots=True)
class HardwareLimits:
    linear_max: float
    reverse_max: float
    angular_max: float
    linear_accel_max: float
    linear_decel_max: float
    angular_accel_max: float
    angular_decel_max: float

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "HardwareLimits":
        return cls(
            linear_max=_positive(data, "linear_max"),
            reverse_max=_positive(data, "reverse_max"),
            angular_max=_positive(data, "angular_max"),
            linear_accel_max=_positive(data, "linear_accel_max"),
            linear_decel_max=_positive(data, "linear_decel_max"),
            angular_accel_max=_positive(data, "angular_accel_max"),
            angular_decel_max=_positive(data, "angular_decel_max"),
        )


@dataclass(frozen=True, slots=True)
class AutoNavigationSpeedProfile:
    mode: str
    linear_max: float
    angular_max: float
    linear_accel: float
    linear_decel: float
    angular_accel: float
    angular_decel: float
    regulated_min_radius: float
    regulated_min_speed: float
    collision_horizon: float
    lookahead_dist: float
    min_lookahead_dist: float
    max_lookahead_dist: float
    lookahead_time: float
    recovery_wait: float
    backup_distance: float
    backup_speed: float
    replan_frequency: float

    @classmethod
    def from_mapping(
        cls,
        mode: str,
        data: Mapping[str, Any],
        hardware: HardwareLimits,
    ) -> "AutoNavigationSpeedProfile":
        normalized = normalize_speed_mode(mode)
        profile = cls(
            mode=normalized,
            linear_max=_positive(data, "linear_max"),
            angular_max=_positive(data, "angular_max"),
            linear_accel=_positive(data, "linear_accel"),
            linear_decel=_positive(data, "linear_decel"),
            angular_accel=_positive(data, "angular_accel"),
            angular_decel=_positive(data, "angular_decel"),
            regulated_min_radius=_positive(data, "regulated_min_radius"),
            regulated_min_speed=_positive(data, "regulated_min_speed"),
            collision_horizon=_positive(data, "collision_horizon"),
            lookahead_dist=_positive(data, "lookahead_dist"),
            min_lookahead_dist=_positive(data, "min_lookahead_dist"),
            max_lookahead_dist=_positive(data, "max_lookahead_dist"),
            lookahead_time=_positive(data, "lookahead_time"),
            recovery_wait=_positive(data, "recovery_wait"),
            backup_distance=_positive(data, "backup_distance"),
            backup_speed=_positive(data, "backup_speed"),
            replan_frequency=_positive(data, "replan_frequency"),
        )
        limits = {
            "linear_max": hardware.linear_max,
            "angular_max": hardware.angular_max,
            "linear_accel": hardware.linear_accel_max,
            "linear_decel": hardware.linear_decel_max,
            "angular_accel": hardware.angular_accel_max,
            "angular_decel": hardware.angular_decel_max,
            "backup_speed": hardware.reverse_max,
        }
        for name, maximum in limits.items():
            if getattr(profile, name) > maximum + 1e-9:
                raise SpeedProfileError(
                    f"{normalized}.{name}={getattr(profile, name)} exceeds proven hardware limit {maximum}"
                )
        if profile.regulated_min_speed > profile.linear_max:
            raise SpeedProfileError(
                f"{normalized}.regulated_min_speed must not exceed linear_max"
            )
        if not (
            profile.min_lookahead_dist
            <= profile.lookahead_dist
            <= profile.max_lookahead_dist
        ):
            raise SpeedProfileError(
                f"{normalized} lookahead must satisfy min <= nominal <= max"
            )
        return profile

    def controller_parameters(self) -> dict[str, float]:
        return {
            "FollowPath.desired_linear_vel": self.linear_max,
            "FollowPath.rotate_to_heading_angular_vel": self.angular_max,
            "FollowPath.max_angular_accel": self.angular_accel,
            "FollowPath.regulated_linear_scaling_min_radius": self.regulated_min_radius,
            "FollowPath.regulated_linear_scaling_min_speed": self.regulated_min_speed,
            "FollowPath.max_allowed_time_to_collision_up_to_carrot": self.collision_horizon,
            "FollowPath.lookahead_dist": self.lookahead_dist,
            "FollowPath.min_lookahead_dist": self.min_lookahead_dist,
            "FollowPath.max_lookahead_dist": self.max_lookahead_dist,
            "FollowPath.lookahead_time": self.lookahead_time,
        }

    def behavior_parameters(self) -> dict[str, float]:
        return {
            "max_rotational_vel": self.angular_max,
            "rotational_acc_lim": self.angular_accel,
        }


@dataclass(frozen=True, slots=True)
class AutoNavigationProfiles:
    default_mode: str
    hardware: HardwareLimits
    profiles: dict[str, AutoNavigationSpeedProfile]
    debug_enabled: bool = False
    debug_throttle_seconds: float = 1.0

    @classmethod
    def load(cls, path: str | Path) -> "AutoNavigationProfiles":
        source = Path(path)
        try:
            data = yaml.safe_load(source.read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise SpeedProfileError(f"Cannot read speed profiles from {source}: {exc}") from exc
        if not isinstance(data, dict):
            raise SpeedProfileError("Speed profile YAML must contain an object")
        root = data.get("auto_navigation_speed_profiles")
        if not isinstance(root, dict):
            raise SpeedProfileError("auto_navigation_speed_profiles is required")
        hardware_data = root.get("hardware_limits")
        profile_data = root.get("profiles")
        if not isinstance(hardware_data, dict) or not isinstance(profile_data, dict):
            raise SpeedProfileError("hardware_limits and profiles are required")
        hardware = HardwareLimits.from_mapping(hardware_data)
        profiles = {
            mode: AutoNavigationSpeedProfile.from_mapping(
                mode,
                profile_data.get(mode.lower(), {}),
                hardware,
            )
            for mode in SPEED_MODES
        }
        default_mode = normalize_speed_mode(root.get("default_mode", "NORMAL"))
        debug = root.get("debug") if isinstance(root.get("debug"), dict) else {}
        throttle = float(debug.get("throttle_seconds", 1.0))
        if not math.isfinite(throttle) or throttle < 0.2:
            raise SpeedProfileError("debug.throttle_seconds must be at least 0.2")
        return cls(
            default_mode=default_mode,
            hardware=hardware,
            profiles=profiles,
            debug_enabled=bool(debug.get("enabled", False)),
            debug_throttle_seconds=throttle,
        )

    def get(self, mode: object) -> AutoNavigationSpeedProfile:
        return self.profiles[normalize_speed_mode(mode)]

    def smoother_parameters(self) -> dict[str, list[float]]:
        """Return the shared manual-safe envelope, never a per-Auto clamp.

        The smoother is downstream of twist_mux, so lowering it for SLOW Auto
        would also slow manual Fast. Per-Auto acceleration is enforced before
        the mux; this envelope stays at the proven manual/hardware maximum.
        """

        hardware = self.hardware
        return {
            "max_velocity": [hardware.linear_max, 0.0, hardware.angular_max],
            "min_velocity": [-hardware.reverse_max, 0.0, -hardware.angular_max],
            "max_accel": [hardware.linear_accel_max, 0.0, hardware.angular_accel_max],
            "max_decel": [-hardware.linear_decel_max, 0.0, -hardware.angular_decel_max],
        }

    def write_behavior_trees(self, directory: str | Path) -> dict[str, Path]:
        destination = Path(directory)
        destination.mkdir(parents=True, exist_ok=True)
        paths: dict[str, Path] = {}
        for mode, profile in self.profiles.items():
            path = destination / f"navigate_to_pose_{mode.lower()}.xml"
            content = render_behavior_tree(profile)
            temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
            temporary.write_text(content)
            os.replace(temporary, path)
            paths[mode] = path
        return paths


class SpeedModeStore:
    def __init__(self, path: str | Path, default_mode: str = "NORMAL") -> None:
        self.path = Path(path)
        self.default_mode = normalize_speed_mode(default_mode)

    def load(self) -> str:
        try:
            data = json.loads(self.path.read_text())
            return normalize_speed_mode(data.get("mode"))
        except (FileNotFoundError, OSError, TypeError, ValueError, json.JSONDecodeError):
            return self.default_mode

    def save(self, mode: object) -> str:
        normalized = normalize_speed_mode(mode)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps({"mode": normalized}, separators=(",", ":")))
        os.replace(temporary, self.path)
        return normalized


class ProfileVelocityLimiter:
    """Per-Auto envelope before twist_mux; manual commands never pass here."""

    def __init__(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.last_time: float | None = None

    @staticmethod
    def _slew(current: float, target: float, accel: float, decel: float, dt: float) -> tuple[float, bool]:
        same_direction = current == 0.0 or target == 0.0 or current * target > 0
        speeding_up = same_direction and abs(target) > abs(current)
        rate = accel if speeding_up else decel
        change = target - current
        maximum_change = rate * dt
        if abs(change) <= maximum_change:
            return target, False
        return current + math.copysign(maximum_change, change), True

    def apply(
        self,
        linear: float,
        angular: float,
        profile: AutoNavigationSpeedProfile,
        now: float,
    ) -> tuple[float, float, tuple[str, ...]]:
        reasons: list[str] = []
        target_linear = max(-profile.linear_max, min(profile.linear_max, float(linear)))
        target_angular = max(-profile.angular_max, min(profile.angular_max, float(angular)))
        if not math.isclose(target_linear, linear, abs_tol=1e-9):
            reasons.append("PROFILE_LINEAR_MAX")
        if not math.isclose(target_angular, angular, abs_tol=1e-9):
            reasons.append("PROFILE_ANGULAR_MAX")
        dt = 0.05 if self.last_time is None else max(0.001, min(0.25, now - self.last_time))
        self.linear, linear_limited = self._slew(
            self.linear,
            target_linear,
            profile.linear_accel,
            profile.linear_decel,
            dt,
        )
        self.angular, angular_limited = self._slew(
            self.angular,
            target_angular,
            profile.angular_accel,
            profile.angular_decel,
            dt,
        )
        self.last_time = now
        if linear_limited:
            reasons.append("PROFILE_LINEAR_ACCEL")
        if angular_limited:
            reasons.append("PROFILE_ANGULAR_ACCEL")
        return self.linear, self.angular, tuple(reasons)

    def reset(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.last_time = None


def render_behavior_tree(profile: AutoNavigationSpeedProfile) -> str:
    """Build the bounded recovery tree from the central speed profile."""

    # Nav2 Humble's Wait BT port is an integer number of seconds. Fractional
    # strings such as 0.50 are truncated to zero and the recovery returns
    # immediately, so round upward to a real, bounded wait.
    recovery_wait_seconds = max(1, math.ceil(profile.recovery_wait))
    return f'''<!-- Generated from auto_navigation_speed_profiles.yaml ({profile.mode}). -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="5" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithEarlyReplanning">
        <RateController hz="{profile.replan_frequency:.2f}">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{{goal}}" path="{{path}}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{{path}}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="BoundedRecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Wait wait_duration="{recovery_wait_seconds}"/>
          <!-- Two individually bounded retreats let a robot that stopped too
               close to an obstacle regain enough planning clearance. A new
               plan is attempted between them, so the second one only runs if
               the first short retreat was insufficient. -->
          <BackUp name="BackUp-First" backup_dist="{profile.backup_distance:.2f}" backup_speed="{profile.backup_speed:.2f}"/>
          <BackUp name="BackUp-Second" backup_dist="{profile.backup_distance:.2f}" backup_speed="{profile.backup_speed:.2f}"/>
          <Spin spin_dist="1.57"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
'''
