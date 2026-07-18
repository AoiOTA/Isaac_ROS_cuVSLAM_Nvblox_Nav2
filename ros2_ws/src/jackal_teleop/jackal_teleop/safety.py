"""ROS-independent deadman state for keyboard teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class DeadmanCommand:
    timeout_s: float = 0.18
    linear_speed: float = 0.35
    angular_speed: float = 0.80
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
