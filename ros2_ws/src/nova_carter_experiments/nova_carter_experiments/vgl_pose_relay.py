"""Validate cuVGL poses and relay them to cuVSLAM's initial-pose input."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class VglPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("vgl_pose_relay")
        self.declare_parameter("cuvslam_map_dir", "")
        self.map_dir = Path(str(self.get_parameter("cuvslam_map_dir").value))
        if not self.map_dir.is_dir():
            raise RuntimeError(f"cuVSLAM map directory does not exist: {self.map_dir}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/visual_slam/initial_pose", qos
        )
        self.accepted_publisher = self.create_publisher(Bool, "/vgl_pose_relay/accepted", qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/visual_localization/pose",
            self.on_pose,
            qos,
        )

    def on_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            *message.pose.covariance,
        ]
        accepted = message.header.frame_id == "map" and all(math.isfinite(v) for v in values)
        status = Bool(data=accepted)
        self.accepted_publisher.publish(status)
        if not accepted:
            self.get_logger().error(
                f"Rejected cuVGL pose: frame={message.header.frame_id!r} finite="
                f"{all(math.isfinite(v) for v in values)}"
            )
            return
        message.header.frame_id = "map"
        self.publisher.publish(message)
        self.get_logger().info("Relayed cuVGL map pose to /visual_slam/initial_pose")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VglPoseRelay()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
