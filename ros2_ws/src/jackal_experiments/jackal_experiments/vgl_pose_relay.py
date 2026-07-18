"""Validate cuVGL poses and relay them to cuVSLAM's initial-pose input."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def yaw_from_quaternion(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def map_to_odom_2d(
    map_pose: object, odom_pose: object
) -> tuple[float, float, float]:
    """Return the planar map->odom transform implied by two base poses."""
    yaw = angle_difference(
        yaw_from_quaternion(map_pose.orientation),
        yaw_from_quaternion(odom_pose.orientation),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotated_x = cosine * odom_pose.position.x - sine * odom_pose.position.y
    rotated_y = sine * odom_pose.position.x + cosine * odom_pose.position.y
    return (
        map_pose.position.x - rotated_x,
        map_pose.position.y - rotated_y,
        yaw,
    )


class VglPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("vgl_pose_relay")
        self.declare_parameter("cuvslam_map_dir", "")
        self.declare_parameter(
            "odometry_topic", "/visual_slam/tracking/odometry"
        )
        self.declare_parameter("max_reanchor_translation_m", 0.35)
        self.declare_parameter("max_reanchor_yaw_rad", 0.60)
        self.map_dir = Path(str(self.get_parameter("cuvslam_map_dir").value))
        if not self.map_dir.is_dir():
            raise RuntimeError(f"cuVSLAM map directory does not exist: {self.map_dir}")
        self.max_reanchor_translation = float(
            self.get_parameter("max_reanchor_translation_m").value
        )
        self.max_reanchor_yaw = float(
            self.get_parameter("max_reanchor_yaw_rad").value
        )
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        if min(self.max_reanchor_translation, self.max_reanchor_yaw) <= 0.0:
            raise ValueError("VGL re-anchor innovation limits must be positive")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/visual_slam/initial_pose", qos
        )
        self.accepted_publisher = self.create_publisher(Bool, "/vgl_pose_relay/accepted", qos)
        self.accepted_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/vgl_pose_relay/pose", qos
        )
        self.latest_odom_pose = None
        self.anchor: tuple[float, float, float] | None = None
        self.last_accepted_map_pose = None
        self.localization_ready = False
        self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.on_odometry,
            qos,
        )
        self.create_subscription(
            Bool,
            "/localization/ready",
            self.on_localization_ready,
            latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/visual_localization/pose",
            self.on_pose,
            qos,
        )

    def on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id == "odom":
            self.latest_odom_pose = message.pose.pose

    def on_localization_ready(self, message: Bool) -> None:
        was_ready = self.localization_ready
        self.localization_ready = message.data
        if (
            message.data
            and not was_ready
            and self.last_accepted_map_pose is not None
            and self.latest_odom_pose is not None
        ):
            # cuVSLAM may rebase odom while asynchronously loading its map.
            # Recompute the innovation anchor only after stable tracking so
            # future VGL results are compared in the post-localization frame.
            self.anchor = map_to_odom_2d(
                self.last_accepted_map_pose, self.latest_odom_pose
            )
            self.get_logger().info(
                "Rebased VGL innovation anchor on tracking-recovery edge"
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
        accepted = (
            message.header.frame_id == "map"
            and all(math.isfinite(v) for v in values)
            and self.latest_odom_pose is not None
        )
        candidate = None
        innovation = (0.0, 0.0)
        if accepted:
            candidate = map_to_odom_2d(pose, self.latest_odom_pose)
            if self.anchor is not None:
                innovation = (
                    math.hypot(
                        candidate[0] - self.anchor[0],
                        candidate[1] - self.anchor[1],
                    ),
                    abs(angle_difference(candidate[2], self.anchor[2])),
                )
                accepted = (
                    innovation[0] <= self.max_reanchor_translation
                    and innovation[1] <= self.max_reanchor_yaw
                )
        status = Bool(data=accepted)
        self.accepted_publisher.publish(status)
        if not accepted:
            self.get_logger().error(
                f"Rejected cuVGL pose: frame={message.header.frame_id!r} finite="
                f"{all(math.isfinite(v) for v in values)} odom={self.latest_odom_pose is not None} "
                f"anchor_innovation=(translation={innovation[0]:.3f}m, "
                f"yaw={innovation[1]:.3f}rad)"
            )
            return
        assert candidate is not None
        self.anchor = candidate
        self.last_accepted_map_pose = pose
        message.header.frame_id = "map"
        self.publisher.publish(message)
        self.accepted_pose_publisher.publish(message)
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
