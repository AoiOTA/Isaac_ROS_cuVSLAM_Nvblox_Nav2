"""Small, ROS-independent differential-drive math helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math


def body_twist_from_wheels(
    left_radps: float,
    right_radps: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
) -> tuple[float, float]:
    linear = 0.5 * wheel_radius_m * (left_radps + right_radps)
    angular = wheel_radius_m * (right_radps - left_radps) / wheel_separation_m
    return linear, angular


def wheel_speeds_from_body_twist(
    linear_mps: float,
    angular_radps: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
) -> tuple[float, float]:
    half_turn = 0.5 * wheel_separation_m * angular_radps
    return (
        (linear_mps - half_turn) / wheel_radius_m,
        (linear_mps + half_turn) / wheel_radius_m,
    )


def integrate_pose(
    x: float, y: float, yaw: float, linear_mps: float, angular_radps: float, dt: float
) -> tuple[float, float, float]:
    if dt <= 0.0:
        return x, y, yaw
    midpoint = yaw + 0.5 * angular_radps * dt
    return (
        x + linear_mps * math.cos(midpoint) * dt,
        y + linear_mps * math.sin(midpoint) * dt,
        yaw + angular_radps * dt,
    )


@dataclass
class SlewAxis:
    value: float = 0.0
    acceleration: float = 0.0

    def reset(self) -> None:
        self.value = 0.0
        self.acceleration = 0.0

    def update(
        self,
        target: float,
        dt: float,
        acceleration_limit: float,
        deceleration_limit: float,
        jerk_limit: float,
    ) -> float:
        if dt <= 0.0:
            return self.value
        error = target - self.value
        if abs(error) < 1.0e-9:
            self.value = target
            self.acceleration = 0.0
            return self.value

        speeding_up = self.value == 0.0 or math.copysign(1.0, error) == math.copysign(
            1.0, self.value
        )
        rate_limit = acceleration_limit if speeding_up else deceleration_limit
        desired_acceleration = max(-rate_limit, min(rate_limit, error / dt))
        max_acceleration_change = jerk_limit * dt
        acceleration_delta = max(
            -max_acceleration_change,
            min(max_acceleration_change, desired_acceleration - self.acceleration),
        )
        self.acceleration += acceleration_delta
        candidate = self.value + self.acceleration * dt
        if (target - self.value) * (target - candidate) <= 0.0:
            self.value = target
            self.acceleration = 0.0
        else:
            self.value = candidate
        return self.value
