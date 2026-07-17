"""Publish a current-time map->odom transform derived only from cuVSLAM outputs."""

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def quaternion_multiply(a: object, b: object) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def quaternion_conjugate(q: object) -> Quaternion:
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm <= 1.0e-12:
        raise ValueError("cannot invert a zero quaternion")
    return Quaternion(x=-q.x / norm, y=-q.y / norm, z=-q.z / norm, w=q.w / norm)


def rotate(q: object, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    vector = Quaternion(x=xyz[0], y=xyz[1], z=xyz[2], w=0.0)
    rotated = quaternion_multiply(quaternion_multiply(q, vector), quaternion_conjugate(q))
    return rotated.x, rotated.y, rotated.z


def map_to_odom_transform(map_pose: object, odom_pose: object) -> tuple[tuple[float, float, float], Quaternion]:
    """Solve T_map_odom = T_map_base * inverse(T_odom_base)."""
    q_map_odom = quaternion_multiply(
        map_pose.orientation, quaternion_conjugate(odom_pose.orientation)
    )
    rotated_odom = rotate(
        q_map_odom,
        (odom_pose.position.x, odom_pose.position.y, odom_pose.position.z),
    )
    translation = (
        map_pose.position.x - rotated_odom[0],
        map_pose.position.y - rotated_odom[1],
        map_pose.position.z - rotated_odom[2],
    )
    norm = math.sqrt(
        q_map_odom.x**2 + q_map_odom.y**2 + q_map_odom.z**2 + q_map_odom.w**2
    )
    q_map_odom.x /= norm
    q_map_odom.y /= norm
    q_map_odom.z /= norm
    q_map_odom.w /= norm
    return translation, q_map_odom


class NavigationTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("navigation_tf_bridge")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_source_age_s", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.max_age_ns = int(
            float(self.get_parameter("max_source_age_s").value) * 1_000_000_000
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        if rate <= 0.0 or self.max_age_ns <= 0:
            raise ValueError("publish_rate_hz and max_source_age_s must be positive")
        self.map_pose = None
        self.map_stamp_ns = -1
        self.odom_pose = None
        self.odom_stamp_ns = -1
        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, "/visual_slam/tracking/slam_path", self.on_path, qos)
        self.create_subscription(
            Odometry, "/visual_slam/tracking/odometry", self.on_odometry, qos
        )
        self.create_timer(1.0 / rate, self.publish_transform)

    @staticmethod
    def stamp_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def on_path(self, message: Path) -> None:
        if message.header.frame_id == self.map_frame and message.poses:
            latest = message.poses[-1]
            self.map_pose = latest.pose
            self.map_stamp_ns = self.stamp_ns(latest.header.stamp)
            if self.map_stamp_ns == 0:
                self.map_stamp_ns = self.stamp_ns(message.header.stamp)

    def on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id == self.odom_frame:
            self.odom_pose = message.pose.pose
            self.odom_stamp_ns = self.stamp_ns(message.header.stamp)

    def publish_transform(self) -> None:
        if self.map_pose is None or self.odom_pose is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if now_ns - min(self.map_stamp_ns, self.odom_stamp_ns) > self.max_age_ns:
            return
        translation, rotation = map_to_odom_transform(self.map_pose, self.odom_pose)
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation = rotation
        self.broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationTfBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
