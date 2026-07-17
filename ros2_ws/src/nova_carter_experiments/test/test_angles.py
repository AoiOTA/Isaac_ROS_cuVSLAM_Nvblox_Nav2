import math

from nova_carter_experiments.motion_test_runner import angle_delta


def test_angle_delta_crosses_wrap_continuously() -> None:
    assert math.isclose(angle_delta(-math.pi + 0.1, math.pi - 0.1), 0.2, abs_tol=1.0e-12)
