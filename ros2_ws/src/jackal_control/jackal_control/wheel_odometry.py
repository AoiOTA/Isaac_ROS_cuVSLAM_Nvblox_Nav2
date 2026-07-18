"""Integrate Jackal odometry by averaging the front/rear wheels per side."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

from .kinematics import body_twist_from_wheels, integrate_pose


class WheelOdometry(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odometry")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odometry_topic", "/wheel/odometry")
        self.declare_parameter(
            "left_wheel_joints", ["front_left_wheel_joint", "rear_left_wheel_joint"]
        )
        self.declare_parameter(
            "right_wheel_joints", ["front_right_wheel_joint", "rear_right_wheel_joint"]
        )
        self.declare_parameter("wheel_radius", 0.098)
        self.declare_parameter("wheel_separation", 0.800)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.left_joints = [str(value) for value in self.get_parameter("left_wheel_joints").value]
        self.right_joints = [str(value) for value in self.get_parameter("right_wheel_joints").value]
        self.radius = float(self.get_parameter("wheel_radius").value)
        self.separation = float(self.get_parameter("wheel_separation").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        if (
            self.radius <= 0.0
            or self.separation <= 0.0
            or len(self.left_joints) != 2
            or len(self.right_joints) != 2
        ):
            raise ValueError("wheel geometry must be positive")

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odometry_topic").value), qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp_ns: int | None = None
        self.missing_joint_messages = 0

    @staticmethod
    def _find_joint(names: list[str], desired: str) -> int:
        for index, name in enumerate(names):
            if name == desired or name.rsplit("/", 1)[-1] == desired:
                return index
        raise ValueError(desired)

    def on_joint_state(self, message: JointState) -> None:
        try:
            left = sum(
                float(message.velocity[self._find_joint(message.name, name)])
                for name in self.left_joints
            ) / 2.0
            right = sum(
                float(message.velocity[self._find_joint(message.name, name)])
                for name in self.right_joints
            ) / 2.0
        except (ValueError, IndexError):
            self.missing_joint_messages += 1
            if self.missing_joint_messages in (1, 100):
                self.get_logger().error(
                    f"joint state lacks active wheel velocities: {message.name}"
                )
            return
        if not math.isfinite(left) or not math.isfinite(right):
            self.get_logger().error("non-finite wheel velocity rejected")
            return

        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if self.last_stamp_ns is None or stamp_ns <= self.last_stamp_ns:
            self.last_stamp_ns = stamp_ns
            return
        dt = (stamp_ns - self.last_stamp_ns) * 1.0e-9
        self.last_stamp_ns = stamp_ns
        if dt > 0.25:
            self.get_logger().warning(f"joint state gap {dt:.3f}s; integration skipped")
            return

        linear, angular = body_twist_from_wheels(left, right, self.radius, self.separation)
        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw, linear, angular, dt
        )

        odometry = Odometry()
        odometry.header = message.header
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        odometry.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        odometry.twist.twist.linear.x = linear
        odometry.twist.twist.angular.z = angular
        odometry.pose.covariance[0] = 0.01
        odometry.pose.covariance[7] = 0.01
        odometry.pose.covariance[35] = 0.02
        odometry.twist.covariance[0] = 0.02
        odometry.twist.covariance[35] = 0.04
        self.publisher.publish(odometry)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
