"""Release depth scans only when the matching cuVSLAM transform exists.

Rendering and visual SLAM are separate GPU pipelines.  Under load, a rendered
depth image can be several frames ahead of cuVSLAM's ``odom -> base_link`` TF.
Nav2 must never consume that future-dated observation.  This node buffers scans,
selects the newest scan no newer than the observed cuVSLAM transform, and stamps
it at that exact transform time.  If localization stalls, output stalls as well;
the downstream safety timeouts then stop the robot by design.
"""

from __future__ import annotations

from collections import deque
import math
import time

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def stamp_to_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp_ns(stamp: object, value: int) -> None:
    value = max(0, int(value))
    stamp.sec = value // 1_000_000_000
    stamp.nanosec = value % 1_000_000_000


def rotate_vector(q: object, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a vector by a geometry_msgs quaternion."""
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + (q.y * tz - q.z * ty),
        y + q.w * ty + (q.z * tx - q.x * tz),
        z + q.w * tz + (q.x * ty - q.y * tx),
    )


class ScanTimestampRelay(Node):
    def __init__(self) -> None:
        super().__init__("scan_timestamp_relay")
        self.declare_parameter("input_topic", "/front_depth/scan_raw")
        self.declare_parameter("output_topic", "/front_depth/scan")
        self.declare_parameter("cloud_output_topic", "/front_depth/points_odom")
        self.declare_parameter("tf_parent_frame", "odom")
        self.declare_parameter("tf_child_frame", "base_link")
        self.declare_parameter("tf_release_delay_s", 0.02)
        self.declare_parameter("max_scan_age_s", 0.5)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        cloud_output_topic = str(self.get_parameter("cloud_output_topic").value)
        self.parent_frame = str(self.get_parameter("tf_parent_frame").value)
        self.child_frame = str(self.get_parameter("tf_child_frame").value)
        self.release_delay = float(self.get_parameter("tf_release_delay_s").value)
        self.max_age_ns = int(
            float(self.get_parameter("max_scan_age_s").value) * 1_000_000_000
        )
        if not 0.0 <= self.release_delay <= 0.2:
            raise ValueError("tf_release_delay_s must be in [0.0, 0.2]")
        if self.max_age_ns <= 0:
            raise ValueError("max_scan_age_s must be positive")

        self.scans: deque[LaserScan] = deque(maxlen=180)
        self.ready_tf_ns = -1
        self.ready_transform: TransformStamped | None = None
        self.ready_tf_wall = 0.0
        self.last_published_tf_ns = -1
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, cloud_output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, input_topic, self.on_scan, qos_profile_sensor_data
        )
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_timer(0.01, self.release_ready_scan)
        self.get_logger().info(
            f"Synchronizing {input_topic} -> {output_topic} against "
            f"{self.parent_frame}->{self.child_frame} TF"
        )

    def on_scan(self, message: LaserScan) -> None:
        self.scans.append(message)

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == self.parent_frame
                and transform.child_frame_id == self.child_frame
            ):
                value = stamp_to_ns(transform.header.stamp)
                if value > self.ready_tf_ns:
                    self.ready_tf_ns = value
                    self.ready_transform = transform
                    self.ready_tf_wall = time.monotonic()

    def to_odom_cloud(
        self, scan: LaserScan, transform: TransformStamped
    ) -> PointCloud2:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = []
        angle = float(scan.angle_min)
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                x, y, z = rotate_vector(
                    rotation,
                    float(distance) * math.cos(angle),
                    float(distance) * math.sin(angle),
                    0.0,
                )
                points.append((x + translation.x, y + translation.y, z + translation.z))
            angle += float(scan.angle_increment)
        header = Header(stamp=transform.header.stamp, frame_id=self.parent_frame)
        return point_cloud2.create_cloud_xyz32(header, points)

    def release_ready_scan(self) -> None:
        if (
            self.ready_tf_ns <= self.last_published_tf_ns
            or self.ready_transform is None
            or time.monotonic() - self.ready_tf_wall < self.release_delay
        ):
            return
        selected = None
        while self.scans and stamp_to_ns(self.scans[0].header.stamp) <= self.ready_tf_ns:
            selected = self.scans.popleft()
        if selected is None:
            return
        age_ns = self.ready_tf_ns - stamp_to_ns(selected.header.stamp)
        if age_ns > self.max_age_ns:
            self.get_logger().warn(
                f"Discarding scan {age_ns / 1e9:.3f} s older than cuVSLAM TF",
                throttle_duration_sec=2.0,
            )
            return
        set_stamp_ns(selected.header.stamp, self.ready_tf_ns)
        self.publisher.publish(selected)
        self.cloud_publisher.publish(self.to_odom_cloud(selected, self.ready_transform))
        self.last_published_tf_ns = self.ready_tf_ns


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanTimestampRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
