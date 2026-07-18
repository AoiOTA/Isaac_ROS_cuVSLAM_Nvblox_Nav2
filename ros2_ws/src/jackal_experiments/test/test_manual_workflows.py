from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from jackal_experiments.manual_goal_bridge import ManualGoalBridge
from jackal_experiments.manual_navigation_ready import ManualNavigationReady


def test_manual_goal_bridge_rejects_wrong_frame_and_waits_for_localization() -> None:
    rclpy.init()
    node = ManualGoalBridge()
    try:
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.orientation.w = 1.0
        assert node.valid_pose(pose) is False
        pose.header.frame_id = "map"
        assert node.valid_pose(pose) is True
        assert node.ready is False
        node.on_goal_pose(pose)
        assert node.pending is pose
        assert node.active_handle is None
        node.on_ready(Bool(data=True))
        assert node.ready is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_readiness_requires_valid_map_and_map_to_odom() -> None:
    rclpy.init()
    node = ManualNavigationReady()
    try:
        node.on_localization_ready(Bool(data=True))
        occupancy = OccupancyGrid()
        occupancy.header.frame_id = "map"
        occupancy.info.width = 2
        occupancy.info.height = 2
        occupancy.data = [0, 0, 100, -1]
        node.on_map(occupancy)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        node.on_tf(TFMessage(transforms=[transform]))
        status = node.status()
        assert status["localization"] is True
        assert status["map"] is True
        assert status["map_to_odom"] is True
        assert status["nav2_action"] is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
