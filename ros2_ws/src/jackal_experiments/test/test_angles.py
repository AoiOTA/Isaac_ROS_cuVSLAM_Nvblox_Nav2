import math

from sensor_msgs.msg import JointState

from jackal_experiments.motion_test_runner import (
    LEFT_JOINTS,
    RIGHT_JOINTS,
    MotionTestRunner,
    angle_delta,
)


def test_angle_delta_crosses_wrap_continuously() -> None:
    assert math.isclose(angle_delta(-math.pi + 0.1, math.pi - 0.1), 0.2, abs_tol=1.0e-12)


def test_jackal_wheel_velocity_averages_front_and_rear() -> None:
    message = JointState()
    message.name = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]
    message.velocity = [-3.0, 5.0, -5.0, 7.0]

    assert MotionTestRunner.average_joint_velocity(message, LEFT_JOINTS) == -4.0
    assert MotionTestRunner.average_joint_velocity(message, RIGHT_JOINTS) == 6.0
