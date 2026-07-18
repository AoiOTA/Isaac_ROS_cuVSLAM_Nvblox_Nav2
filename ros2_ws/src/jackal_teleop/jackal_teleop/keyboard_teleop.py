"""Interactive WASD teleop with a 0.18-second no-key stop."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .safety import DeadmanCommand


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("jackal_keyboard_teleop")
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("deadman_timeout_s", 0.18)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.state = DeadmanCommand(
            timeout_s=float(self.get_parameter("deadman_timeout_s").value),
            linear_speed=float(self.get_parameter("linear_speed").value),
            angular_speed=float(self.get_parameter("angular_speed").value),
        )
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self.publish)

    def publish(self) -> None:
        self.state.update(time.monotonic())
        message = Twist()
        message.linear.x = self.state.linear
        message.angular.z = self.state.angular
        self.publisher.publish(message)

    def stop_now(self) -> None:
        self.state.stop()
        self.publish()


def main(args: list[str] | None = None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit("keyboard teleop requires an interactive terminal")
    rclpy.init(args=args)
    node = KeyboardTeleop()
    descriptor = sys.stdin.fileno()
    original = termios.tcgetattr(descriptor)
    print("W/S 前后，A/D 转向，Space 急停，Q 保存并退出；松键 0.18 秒自动停车。")
    try:
        tty.setcbreak(descriptor)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not readable:
                continue
            key = os.read(descriptor, 1).decode(errors="ignore").lower()
            if key == "q":
                break
            node.state.apply(key, time.monotonic())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_now()
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
