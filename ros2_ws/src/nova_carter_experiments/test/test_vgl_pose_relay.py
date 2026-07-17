import math

from geometry_msgs.msg import Pose, Quaternion

from nova_carter_experiments.vgl_pose_relay import (
    angle_difference,
    map_to_odom_2d,
)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def test_map_to_odom_anchor_uses_current_odom_pose() -> None:
    map_pose = Pose()
    map_pose.position.x = 2.0
    map_pose.position.y = 3.0
    map_pose.orientation = quaternion_from_yaw(math.pi / 2.0)
    odom_pose = Pose()
    odom_pose.position.x = 1.0
    odom_pose.orientation = Quaternion(w=1.0)

    x, y, yaw = map_to_odom_2d(map_pose, odom_pose)
    assert math.isclose(x, 2.0, abs_tol=1.0e-12)
    assert math.isclose(y, 2.0, abs_tol=1.0e-12)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1.0e-12)


def test_reanchor_angle_innovation_wraps_at_pi() -> None:
    assert math.isclose(
        angle_difference(-math.pi + 0.1, math.pi - 0.1),
        0.2,
        abs_tol=1.0e-12,
    )
