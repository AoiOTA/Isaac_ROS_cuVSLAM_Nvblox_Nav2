import math

from jackal_control.kinematics import (
    SlewAxis,
    body_twist_from_wheels,
    integrate_pose,
    wheel_speeds_from_body_twist,
)


def test_forward_inverse_kinematics_round_trip() -> None:
    for linear, angular in ((0.5, 0.0), (0.0, 1.0), (0.4, -0.8), (-0.3, 0.5)):
        left, right = wheel_speeds_from_body_twist(linear, angular, 0.14, 0.4132)
        actual_linear, actual_angular = body_twist_from_wheels(left, right, 0.14, 0.4132)
        assert math.isclose(actual_linear, linear, abs_tol=1.0e-12)
        assert math.isclose(actual_angular, angular, abs_tol=1.0e-12)


def test_midpoint_arc_integration() -> None:
    x, y, yaw = 0.0, 0.0, 0.0
    dt = 0.001
    for _ in range(1000):
        x, y, yaw = integrate_pose(x, y, yaw, 1.0, 1.0, dt)
    assert math.isclose(yaw, 1.0, abs_tol=1.0e-9)
    assert math.isclose(x, math.sin(1.0), rel_tol=1.0e-6)
    assert math.isclose(y, 1.0 - math.cos(1.0), rel_tol=1.0e-6)


def test_slew_axis_respects_acceleration_and_reaches_target() -> None:
    axis = SlewAxis()
    values = [axis.update(1.0, 0.01, 1.2, 1.6, 6.0) for _ in range(300)]
    assert all(b >= a for a, b in zip(values, values[1:]))
    assert math.isclose(values[-1], 1.0, abs_tol=1.0e-9)


def test_slew_axis_normal_tracking_strictly_respects_jerk() -> None:
    axis = SlewAxis()
    dt = 0.01
    values: list[float] = []
    for target in (0.8, -0.4, 0.3, 0.0):
        values.extend(axis.update(target, dt, 1.0, 1.6, 5.0) for _ in range(100))
    accelerations = [
        (current - previous) / dt
        for previous, current in zip(values, values[1:])
    ]
    jerks = [
        (current - previous) / dt
        for previous, current in zip(accelerations, accelerations[1:])
    ]
    assert max(abs(value) for value in accelerations) <= 1.6 + 1.0e-9
    assert max(abs(value) for value in jerks) <= 5.0 + 1.0e-7
