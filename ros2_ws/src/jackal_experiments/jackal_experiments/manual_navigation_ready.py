"""Wait until manual RViz navigation is safe to accept a goal."""

from __future__ import annotations

import time

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class ManualNavigationReady(Node):
    """Gate the operator on localization, map, TF, and Nav2 action readiness."""

    def __init__(self) -> None:
        super().__init__("manual_navigation_ready")
        self.declare_parameter("action_topic", "/navigate_to_pose")
        self.declare_parameter("timeout_s", 180.0)
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        if self.timeout <= 0.0:
            raise ValueError("manual navigation readiness timeout must be positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.localization_ready = False
        self.map_ready = False
        self.map_to_odom_ready = False
        self.client = ActionClient(self, NavigateToPose, self.action_topic)
        self.create_subscription(
            Bool, "/localization/ready", self.on_localization_ready, latched
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(TFMessage, "/tf", self.on_tf, reliable)

    def on_localization_ready(self, message: Bool) -> None:
        self.localization_ready = bool(message.data)

    def on_map(self, message: OccupancyGrid) -> None:
        self.map_ready = (
            message.header.frame_id.lstrip("/") == "map"
            and message.info.width > 0
            and message.info.height > 0
            and len(message.data) == message.info.width * message.info.height
        )

    def on_tf(self, message: TFMessage) -> None:
        self.map_to_odom_ready = self.map_to_odom_ready or any(
            transform.header.frame_id.lstrip("/") == "map"
            and transform.child_frame_id.lstrip("/") == "odom"
            for transform in message.transforms
        )

    def status(self) -> dict[str, bool]:
        return {
            "localization": self.localization_ready,
            "map": self.map_ready,
            "map_to_odom": self.map_to_odom_ready,
            "nav2_action": self.client.server_is_ready(),
        }

    def wait(self) -> bool:
        deadline = time.monotonic() + self.timeout
        next_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.20)
            status = self.status()
            if all(status.values()):
                self.get_logger().info(
                    "MANUAL_NAVIGATION_READY "
                    f"action={self.action_topic} fixed_frame=map"
                )
                return True
            now = time.monotonic()
            if now >= next_log:
                waiting = ", ".join(
                    name for name, ready in status.items() if not ready
                )
                self.get_logger().info(f"Waiting for manual navigation: {waiting}")
                next_log = now + 10.0
        self.get_logger().error(
            f"Manual navigation readiness timed out: {self.status()}"
        )
        return False

    def destroy_node(self) -> None:
        self.client.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManualNavigationReady()
    success = False
    try:
        success = node.wait()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
