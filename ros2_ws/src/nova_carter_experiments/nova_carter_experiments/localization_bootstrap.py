"""Trigger cuVGL and expose a latched navigation-ready health signal."""

from __future__ import annotations

import time

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class LocalizationBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("localization_bootstrap")
        self.declare_parameter("max_trigger_attempts", 5)
        self.declare_parameter("trigger_period_s", 1.0)
        self.declare_parameter("tracking_samples_required", 20)
        self.max_attempts = int(self.get_parameter("max_trigger_attempts").value)
        self.trigger_period = float(self.get_parameter("trigger_period_s").value)
        self.required_samples = int(
            self.get_parameter("tracking_samples_required").value
        )
        if min(self.max_attempts, self.required_samples) <= 0 or self.trigger_period <= 0.0:
            raise ValueError("localization bootstrap limits must be positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.ready_publisher = self.create_publisher(Bool, "/localization/ready", latched)
        self.create_subscription(
            Bool, "/vgl_pose_relay/accepted", self.on_relay, reliable
        )
        self.create_subscription(
            VisualSlamStatus, "/visual_slam/status", self.on_status, reliable
        )
        self.trigger = self.create_client(
            Trigger, "/visual_localization/trigger_localization"
        )
        self.relay_accepted = False
        self.consecutive_tracking = 0
        self.attempts = 0
        self.ready = False
        self.pending = None
        self.next_trigger_wall = 0.0
        self.ready_publisher.publish(Bool(data=False))
        self.timer = self.create_timer(
            0.1, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def set_ready(self, value: bool) -> None:
        if value == self.ready:
            return
        self.ready = value
        self.ready_publisher.publish(Bool(data=value))
        self.get_logger().info(f"Localization navigation-ready={value}")

    def on_relay(self, message: Bool) -> None:
        self.relay_accepted = self.relay_accepted or message.data

    def on_status(self, message: VisualSlamStatus) -> None:
        if int(message.vo_state) == 1:
            self.consecutive_tracking += 1
        else:
            self.consecutive_tracking = 0
            self.set_ready(False)
        if self.relay_accepted and self.consecutive_tracking >= self.required_samples:
            self.set_ready(True)

    def on_timer(self) -> None:
        if self.ready or self.relay_accepted:
            return
        if self.pending is not None:
            if self.pending.done():
                response = self.pending.result()
                if response is None or not response.success:
                    self.get_logger().warning("cuVGL trigger request was not accepted")
                self.pending = None
            return
        now = time.monotonic()
        if now < self.next_trigger_wall or self.attempts >= self.max_attempts:
            return
        if not self.trigger.wait_for_service(timeout_sec=0.05):
            return
        self.attempts += 1
        self.pending = self.trigger.call_async(Trigger.Request())
        self.next_trigger_wall = now + self.trigger_period
        self.get_logger().info(
            f"Triggered cuVGL localization attempt {self.attempts}/{self.max_attempts}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalizationBootstrap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
