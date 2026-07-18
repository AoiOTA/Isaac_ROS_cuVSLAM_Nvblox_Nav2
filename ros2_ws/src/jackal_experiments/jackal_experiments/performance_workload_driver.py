"""Keep mapping and navigation performance observations under active load."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(quaternion: object) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_difference(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


class PerformanceWorkloadDriver(Node):
    """Drive a safe in-place mapping load or cycle through real Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("performance_workload_driver")
        self.declare_parameter("mode", "mapping")
        self.declare_parameter("ready_file", "")
        self.declare_parameter("stop_file", "")
        self.declare_parameter("report_path", "")
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("observed_command_topic", "/cmd_vel_sim")
        self.declare_parameter("mapping_angular_speed_radps", 0.35)
        self.declare_parameter("mapping_direction_period_s", 8.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("minimum_active_command", 0.03)
        self.declare_parameter("action_topic", "/navigate_to_pose")
        self.declare_parameter("goal_timeout_s", 180.0)
        self.declare_parameter(
            "goal_poses",
            [
                -2.10,
                2.85,
                1.570796327,
                0.40,
                -5.10,
                -1.570796327,
                0.98,
                4.93,
                1.570796327,
            ],
        )

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {"mapping", "navigation"}:
            raise ValueError("mode must be mapping or navigation")
        path_values = [
            str(self.get_parameter(name).value).strip()
            for name in ("ready_file", "stop_file", "report_path")
        ]
        if not all(path_values):
            raise ValueError("ready_file, stop_file, and report_path are required")
        self.ready_file, self.stop_file, self.report_path = (
            Path(value).resolve() for value in path_values
        )
        self.angular_speed = float(
            self.get_parameter("mapping_angular_speed_radps").value
        )
        self.direction_period = float(
            self.get_parameter("mapping_direction_period_s").value
        )
        self.minimum_active_command = float(
            self.get_parameter("minimum_active_command").value
        )
        self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if min(
            self.angular_speed,
            self.direction_period,
            self.minimum_active_command,
            self.goal_timeout,
            publish_rate,
        ) <= 0.0:
            raise ValueError("workload speeds, timing, and thresholds must be positive")

        flat_goals = [float(value) for value in self.get_parameter("goal_poses").value]
        if self.mode == "navigation" and (not flat_goals or len(flat_goals) % 3):
            raise ValueError("navigation goal_poses must contain x,y,yaw triples")
        self.goals = [
            tuple(flat_goals[index : index + 3])
            for index in range(0, len(flat_goals), 3)
        ]

        self.started_monotonic = time.monotonic()
        self.started_unix_s = time.time()
        self.ready_unix_s: float | None = None
        self.ready = False
        self.stop_requested = False
        self.fatal_error = ""
        self.command_samples = 0
        self.nonzero_command_samples = 0
        self.maximum_linear_command = 0.0
        self.maximum_angular_command = 0.0
        self.first_odom: tuple[float, float, float] | None = None
        self.last_odom: tuple[float, float, float] | None = None
        self.ground_truth_path_m = 0.0
        self.maximum_yaw_change_rad = 0.0
        self.goals_requested = 0
        self.goals_accepted = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.goal_index = 0
        self.goal_state = "idle"
        self.goal_started_monotonic: float | None = None
        self.active_goal_handle = None

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("observed_command_topic").value),
            self.on_command,
            20,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("action_topic").value),
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.tick)

    def mark_ready(self) -> None:
        if self.ready:
            return
        self.ready_file.parent.mkdir(parents=True, exist_ok=True)
        self.ready_file.touch()
        self.ready = True
        self.ready_unix_s = time.time()
        self.get_logger().info(
            f"Active {self.mode} workload confirmed; adaptive warmup may start"
        )

    def on_command(self, message: Twist) -> None:
        linear = abs(float(message.linear.x))
        angular = abs(float(message.angular.z))
        self.command_samples += 1
        self.maximum_linear_command = max(self.maximum_linear_command, linear)
        self.maximum_angular_command = max(self.maximum_angular_command, angular)
        if max(linear, angular) < self.minimum_active_command:
            return
        self.nonzero_command_samples += 1
        if (
            self.physical_motion_confirmed()
            and (self.mode == "mapping" or self.goal_state == "active")
        ):
            self.mark_ready()

    def physical_motion_confirmed(self) -> bool:
        return self.ground_truth_path_m >= 0.02 or self.maximum_yaw_change_rad >= 0.03

    def on_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        sample = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        if self.first_odom is None:
            self.first_odom = sample
        if self.last_odom is not None:
            self.ground_truth_path_m += math.dist(sample[:2], self.last_odom[:2])
        self.last_odom = sample
        self.maximum_yaw_change_rad = max(
            self.maximum_yaw_change_rad,
            abs(angle_difference(sample[2], self.first_odom[2])),
        )
        if (
            self.nonzero_command_samples > 0
            and self.physical_motion_confirmed()
            and (self.mode == "mapping" or self.goal_state == "active")
        ):
            self.mark_ready()

    def publish_mapping_command(self) -> None:
        elapsed = time.monotonic() - self.started_monotonic
        direction = 1.0 if int(elapsed / self.direction_period) % 2 == 0 else -1.0
        message = Twist()
        message.angular.z = direction * self.angular_speed
        self.command_publisher.publish(message)

    def publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def begin_navigation_goal(self) -> None:
        if not self.action.wait_for_server(timeout_sec=0.0):
            return
        x, y, yaw = self.goals[self.goal_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_state = "requesting"
        self.goals_requested += 1
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future: object) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # rclpy delivers action transport failures here.
            self.fatal_error = f"NavigateToPose request failed: {exc}"
            return
        if handle is None or not handle.accepted:
            self.goals_failed += 1
            self.fatal_error = "NavigateToPose rejected performance goal"
            return
        self.goals_accepted += 1
        self.goal_state = "active"
        self.goal_started_monotonic = time.monotonic()
        self.active_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future: object) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # rclpy delivers action transport failures here.
            self.fatal_error = f"NavigateToPose result failed: {exc}"
            return
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.goals_failed += 1
            status = None if wrapped is None else int(wrapped.status)
            self.fatal_error = f"NavigateToPose performance goal failed: status={status}"
            return
        self.goals_succeeded += 1
        self.goal_index = (self.goal_index + 1) % len(self.goals)
        self.goal_state = "idle"
        self.goal_started_monotonic = None
        self.active_goal_handle = None

    def tick(self) -> None:
        if self.stop_file.exists():
            self.stop_requested = True
            self.publish_stop()
            return
        if self.mode == "mapping":
            self.publish_mapping_command()
            return
        if self.goal_state == "active" and self.goal_started_monotonic is not None:
            if time.monotonic() - self.goal_started_monotonic > self.goal_timeout:
                self.goals_failed += 1
                self.fatal_error = "NavigateToPose performance goal timed out"
            return
        if self.goal_state == "idle" and not self.fatal_error:
            self.begin_navigation_goal()

    def report(self) -> dict[str, object]:
        passed = (
            self.ready
            and not self.fatal_error
            and self.nonzero_command_samples > 0
            and self.physical_motion_confirmed()
        )
        if self.mode == "navigation":
            passed = passed and self.goals_accepted > 0
        return {
            "schema_version": 1,
            "status": "passed" if passed else "failed",
            "mode": self.mode,
            "started_unix_s": self.started_unix_s,
            "ready_unix_s": self.ready_unix_s,
            "ended_unix_s": time.time(),
            "active_workload_confirmed": self.ready,
            "physical_motion_confirmed": self.physical_motion_confirmed(),
            "fatal_error": self.fatal_error or None,
            "commands": {
                "samples": self.command_samples,
                "nonzero_samples": self.nonzero_command_samples,
                "maximum_linear_mps": self.maximum_linear_command,
                "maximum_angular_radps": self.maximum_angular_command,
            },
            "ground_truth": {
                "samples_present": self.first_odom is not None,
                "path_length_m": self.ground_truth_path_m,
                "maximum_yaw_change_rad": self.maximum_yaw_change_rad,
            },
            "navigation": {
                "goals_requested": self.goals_requested,
                "goals_accepted": self.goals_accepted,
                "goals_succeeded": self.goals_succeeded,
                "goals_failed": self.goals_failed,
                "goal_state_at_stop": self.goal_state,
            },
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerformanceWorkloadDriver()
    exit_code = 0
    try:
        while rclpy.ok() and not node.stop_requested and not node.fatal_error:
            if node.stop_file.exists():
                node.stop_requested = True
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.active_goal_handle is not None:
            node.active_goal_handle.cancel_goal_async()
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        report = node.report()
        exit_code = 0 if report["status"] == "passed" else 1
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
