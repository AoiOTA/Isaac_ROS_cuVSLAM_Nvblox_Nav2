"""Final command safety envelope before Isaac Sim's differential controller."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from .kinematics import SlewAxis


class CommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("command_guard")
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("output_topic", "/cmd_vel_sim")
        self.declare_parameter("status_topic", "/control/guard_status")
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("max_linear_acceleration", 1.2)
        self.declare_parameter("max_linear_deceleration", 1.6)
        self.declare_parameter("max_angular_acceleration", 2.4)
        self.declare_parameter("max_angular_deceleration", 3.0)
        self.declare_parameter("max_linear_jerk", 6.0)
        self.declare_parameter("max_angular_jerk", 12.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("update_rate", 100.0)

        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.linear_accel = float(self.get_parameter("max_linear_acceleration").value)
        self.linear_decel = float(self.get_parameter("max_linear_deceleration").value)
        self.angular_accel = float(self.get_parameter("max_angular_acceleration").value)
        self.angular_decel = float(self.get_parameter("max_angular_deceleration").value)
        self.linear_jerk = float(self.get_parameter("max_linear_jerk").value)
        self.angular_jerk = float(self.get_parameter("max_angular_jerk").value)
        self.timeout_s = float(self.get_parameter("command_timeout").value)
        update_rate = float(self.get_parameter("update_rate").value)
        if min(
            self.max_linear,
            self.max_angular,
            self.linear_accel,
            self.linear_decel,
            self.angular_accel,
            self.angular_decel,
            self.linear_jerk,
            self.angular_jerk,
            self.timeout_s,
            update_rate,
        ) <= 0.0:
            raise ValueError("all command guard limits must be positive")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), qos
        )
        self.subscription = self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self.on_command, qos
        )

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_command_ns: int | None = None
        self.last_update_ns: int | None = None
        self.linear_axis = SlewAxis()
        self.angular_axis = SlewAxis()
        self.state = "waiting"
        self.rejected_commands = 0
        self.clamped_commands = 0
        self.timer = self.create_timer(1.0 / update_rate, self.on_timer)

    def publish_status(self, state: str) -> None:
        if state == self.state:
            return
        self.state = state
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "rejected_commands": self.rejected_commands,
                "clamped_commands": self.clamped_commands,
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def publish_zero_immediately(self, state: str) -> None:
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.linear_axis.reset()
        self.angular_axis.reset()
        self.publisher.publish(Twist())
        self.publish_status(state)

    def on_command(self, message: Twist) -> None:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.rejected_commands += 1
            self.last_command_ns = None
            self.publish_zero_immediately("invalid_command")
            return

        linear = max(-self.max_linear, min(self.max_linear, message.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, message.angular.z))
        if linear != message.linear.x or angular != message.angular.z:
            self.clamped_commands += 1
        self.target_linear = linear
        self.target_angular = angular
        self.last_command_ns = self.get_clock().now().nanoseconds
        self.publish_status("active")

    def on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_update_ns is None or now_ns <= self.last_update_ns:
            dt = 0.01
        else:
            dt = min(0.05, (now_ns - self.last_update_ns) * 1.0e-9)
        self.last_update_ns = now_ns

        stale = self.last_command_ns is None or (now_ns - self.last_command_ns) * 1.0e-9 > self.timeout_s
        if stale:
            self.publish_zero_immediately("timeout" if self.last_command_ns is not None else "waiting")
            return

        output = Twist()
        output.linear.x = self.linear_axis.update(
            self.target_linear,
            dt,
            self.linear_accel,
            self.linear_decel,
            self.linear_jerk,
        )
        output.angular.z = self.angular_axis.update(
            self.target_angular,
            dt,
            self.angular_accel,
            self.angular_decel,
            self.angular_jerk,
        )
        self.publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_zero_immediately("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
