"""ROS-independent deadman state for keyboard teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class DeadmanCommand:
    timeout_s: float = 0.18
    linear_speed: float = 0.55
    angular_speed: float = 1.00
    linear: float = 0.0
    angular: float = 0.0
    last_motion_key_s: float | None = None

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.timeout_s, self.linear_speed, self.angular_speed)
        ):
            raise ValueError("teleop limits must be finite and positive")

    def apply(self, key: str, now_s: float) -> bool:
        commands = {
            "w": (self.linear_speed, 0.0),
            "s": (-self.linear_speed, 0.0),
            "a": (0.0, self.angular_speed),
            "d": (0.0, -self.angular_speed),
        }
        if key in commands:
            self.linear, self.angular = commands[key]
            self.last_motion_key_s = now_s
            return True
        if key == " ":
            self.stop()
            return True
        return False

    def set_speeds(self, linear_speed: float, angular_speed: float) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (linear_speed, angular_speed)
        ):
            raise ValueError("teleop speeds must be finite and positive")
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def update(self, now_s: float) -> bool:
        if self.last_motion_key_s is None:
            return False
        if now_s - self.last_motion_key_s <= self.timeout_s:
            return False
        changed = self.linear != 0.0 or self.angular != 0.0
        self.stop()
        return changed

    def stop(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key_s = None


@dataclass
class MappingProgress:
    """Accumulate meaningful planar odometry while rejecting stationary jitter."""

    minimum_path_m: float = 2.0
    minimum_increment_m: float = 0.01
    maximum_increment_m: float = 1.0
    path_length_m: float = 0.0
    last_counted_xy: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.minimum_path_m,
                self.minimum_increment_m,
                self.maximum_increment_m,
            )
        ):
            raise ValueError("mapping progress limits must be finite and positive")
        if self.minimum_increment_m >= self.maximum_increment_m:
            raise ValueError("mapping progress increment bounds are invalid")

    def update(self, x: float, y: float) -> bool:
        if not all(math.isfinite(value) for value in (x, y)):
            return False
        if self.last_counted_xy is None:
            self.last_counted_xy = (x, y)
            return False
        increment = math.hypot(x - self.last_counted_xy[0], y - self.last_counted_xy[1])
        if increment < self.minimum_increment_m:
            return False
        self.last_counted_xy = (x, y)
        if increment > self.maximum_increment_m:
            return False
        self.path_length_m += increment
        return True

    @property
    def can_finish(self) -> bool:
        return self.path_length_m >= self.minimum_path_m
