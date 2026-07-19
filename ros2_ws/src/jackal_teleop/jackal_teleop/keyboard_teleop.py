"""Interactive WASD teleop with deadman stop and safe speed presets."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .safety import DeadmanCommand, MappingProgress


SPEED_SCALES = (0.50, 0.75, 1.00, 1.20, 1.35)


def speed_for_level(
    base_linear_speed: float, base_angular_speed: float, level: int
) -> tuple[float, float]:
    if level < 1 or level > len(SPEED_SCALES):
        raise ValueError(f"speed_level must be in 1..{len(SPEED_SCALES)}")
    scale = SPEED_SCALES[level - 1]
    return base_linear_speed * scale, base_angular_speed * scale


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("jackal_keyboard_teleop")
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("linear_speed", 0.55)
        self.declare_parameter("angular_speed", 1.00)
        self.declare_parameter("speed_level", 3)
        self.declare_parameter("deadman_timeout_s", 0.18)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("minimum_save_path_m", 2.0)
        self.base_linear_speed = float(self.get_parameter("linear_speed").value)
        self.base_angular_speed = float(self.get_parameter("angular_speed").value)
        self.speed_level = int(self.get_parameter("speed_level").value)
        linear_speed, angular_speed = speed_for_level(
            self.base_linear_speed, self.base_angular_speed, self.speed_level
        )
        self.state = DeadmanCommand(
            timeout_s=float(self.get_parameter("deadman_timeout_s").value),
            linear_speed=linear_speed,
            angular_speed=angular_speed,
        )
        self.progress = MappingProgress(
            minimum_path_m=float(self.get_parameter("minimum_save_path_m").value)
        )
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            qos_profile_sensor_data,
        )

    def on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.progress.update(float(position.x), float(position.y))

    def report_progress(self, blocked: bool = False) -> None:
        prefix = "Cannot save yet. " if blocked else ""
        self.get_logger().info(
            f"{prefix}Travelled {self.progress.path_length_m:.2f} m / "
            f"minimum {self.progress.minimum_path_m:.2f} m."
        )

    def set_speed_level(self, level: int) -> bool:
        level = max(1, min(len(SPEED_SCALES), level))
        if level == self.speed_level:
            return False
        linear_speed, angular_speed = speed_for_level(
            self.base_linear_speed, self.base_angular_speed, level
        )
        self.state.stop()
        self.state.set_speeds(linear_speed, angular_speed)
        self.speed_level = level
        self.get_logger().info(
            "Speed level %d/%d: %.2f m/s, %.2f rad/s (stopped before change)"
            % (level, len(SPEED_SCALES), linear_speed, angular_speed)
        )
        return True

    def handle_key(self, key: str) -> bool:
        if key in "12345":
            return self.set_speed_level(int(key))
        if key in ("+", "="):
            return self.set_speed_level(self.speed_level + 1)
        if key in ("-", "_"):
            return self.set_speed_level(self.speed_level - 1)
        return self.state.apply(key, time.monotonic())

    def publish(self) -> None:
        self.state.update(time.monotonic())
        message = Twist()
        message.linear.x = self.state.linear
        message.angular.z = self.state.angular
        self.publisher.publish(message)

    def stop_now(self) -> None:
        self.state.stop()
        # SIGINT can invalidate the rcl context before the Python finally
        # block runs. The simulator-side command timeout already guarantees
        # a stop in that case; avoid publishing through an invalid context.
        if rclpy.ok():
            self.publish()


def main(args: list[str] | None = None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit("keyboard teleop requires an interactive terminal")
    rclpy.init(args=args)
    node = KeyboardTeleop()
    descriptor = sys.stdin.fileno()
    original = termios.tcgetattr(descriptor)
    print(
        "W/S 前后，A/D 转向，1-5 或 +/- 调速度，Space 急停，P 查看距离，"
        "Q 保存并退出；松键 0.18 秒自动停车。键盘必须聚焦此终端。"
    )
    try:
        tty.setcbreak(descriptor)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not readable:
                continue
            key = os.read(descriptor, 1).decode(errors="ignore").lower()
            if key == "q":
                if node.progress.can_finish:
                    break
                node.report_progress(blocked=True)
                continue
            if key == "p":
                node.report_progress()
                continue
            node.handle_key(key)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_now()
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
