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
        response_rate: float = 8.0,
    ) -> float:
        if dt <= 0.0:
            return self.value
        error = target - self.value
        if abs(error) < 1.0e-6 and abs(self.acceleration) < 1.0e-5:
            self.value = target
            self.acceleration = 0.0
            return self.value

        # Critically damped second-order tracking in command-velocity space.
        # The old target snap reset acceleration at every small MPPI command
        # change, creating an output jerk spike even though the configured
        # acceleration limit was respected.  Integrating a bounded jerk keeps
        # every normal navigation transition continuous.  Emergency health
        # stops intentionally bypass this filter in CommandGuard.
        desired_jerk = (
            response_rate * response_rate * error
            - 2.0 * response_rate * self.acceleration
        )
        applied_jerk = max(-jerk_limit, min(jerk_limit, desired_jerk))
        candidate_acceleration = self.acceleration + applied_jerk * dt

        if abs(self.value) < 1.0e-9:
            speeding_up = candidate_acceleration * target >= 0.0
        else:
            speeding_up = candidate_acceleration * self.value >= 0.0
        rate_limit = acceleration_limit if speeding_up else deceleration_limit
        # When a target reversal changes which rate limit applies, approach
        # the new bound with the same jerk limit instead of clipping the
        # existing acceleration instantaneously.
        if (
            abs(candidate_acceleration) > rate_limit
            and abs(candidate_acceleration) > abs(self.acceleration)
        ):
            candidate_acceleration = math.copysign(
                max(rate_limit, abs(self.acceleration)), candidate_acceleration
            )
        self.acceleration = candidate_acceleration
        self.value += self.acceleration * dt
        return self.value
