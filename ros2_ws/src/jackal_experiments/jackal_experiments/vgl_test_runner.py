"""Trigger cuVGL once and verify pose accuracy plus cuVSLAM tracking recovery."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def yaw(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class VglTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("vgl_test_runner")
        self.declare_parameter("result_path", "data/reports/phase7/vgl.json")
        self.declare_parameter("attempt_name", "pose_0")
        self.result_path = Path(str(self.get_parameter("result_path").value)).resolve()
        self.attempt_name = str(self.get_parameter("attempt_name").value)
        self.vgl_poses: list[PoseWithCovarianceStamped] = []
        self.gt: Odometry | None = None
        self.status: list[tuple[float, int]] = []
        self.relay_accepted = False
        self.failure_hints = 0
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/visual_localization/pose",
            self.vgl_poses.append,
            10,
        )
        self.create_subscription(
            Odometry, "/ground_truth/odometry", self.on_gt, qos_profile_sensor_data
        )
        self.create_subscription(VisualSlamStatus, "/visual_slam/status", self.on_status, 20)
        self.create_subscription(Bool, "/vgl_pose_relay/accepted", self.on_relay, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/visual_slam/trigger_hint",
            self.on_failure_hint,
            10,
        )
        self.trigger = self.create_client(Trigger, "/visual_localization/trigger_localization")

    def on_gt(self, message: Odometry) -> None:
        self.gt = message

    def on_status(self, message: VisualSlamStatus) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.status.append((stamp, int(message.vo_state)))

    def on_relay(self, message: Bool) -> None:
        self.relay_accepted = self.relay_accepted or message.data

    def on_failure_hint(self, _message: PoseWithCovarianceStamped) -> None:
        if self.relay_accepted:
            self.failure_hints += 1

    def run(self) -> dict[str, object]:
        if not self.trigger.wait_for_service(timeout_sec=90.0):
            raise RuntimeError("cuVGL trigger service unavailable")
        deadline = time.monotonic() + 150.0
        triggers = 0
        next_trigger = 0.0
        while time.monotonic() < deadline and not self.vgl_poses:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() >= next_trigger:
                future = self.trigger.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                triggers += 1
                next_trigger = time.monotonic() + 1.0
        if not self.vgl_poses:
            raise RuntimeError(f"cuVGL returned no pose after {triggers} triggers")
        pose = self.vgl_poses[-1]
        recovery_deadline = time.monotonic() + 45.0
        consecutive_tracking = 0
        while time.monotonic() < recovery_deadline and consecutive_tracking < 20:
            before = len(self.status)
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.status) > before:
                consecutive_tracking = consecutive_tracking + 1 if self.status[-1][1] == 1 else 0
        if self.gt is None:
            raise RuntimeError("ground-truth pose unavailable")
        p = pose.pose.pose
        g = self.gt.pose.pose
        translation_error = math.hypot(p.position.x - g.position.x, p.position.y - g.position.y)
        yaw_error = abs(math.atan2(math.sin(yaw(p.orientation) - yaw(g.orientation)),
                                   math.cos(yaw(p.orientation) - yaw(g.orientation))))
        pose_values = [p.position.x, p.position.y, p.position.z, yaw(p.orientation)]
        checks = {
            "pose_frame_map": pose.header.frame_id == "map",
            "pose_relay_accepted": self.relay_accepted,
            "pose_reasonable": all(math.isfinite(value) for value in pose_values)
            and abs(p.position.x) <= 50.0
            and abs(p.position.y) <= 50.0
            and abs(p.position.z) <= 5.0,
            "cuvslam_tracking_recovered": consecutive_tracking >= 20,
            "no_localization_failure_hint": self.failure_hints == 0,
        }
        return {
            "status": "passed" if all(checks.values()) else "failed",
            "attempt": self.attempt_name,
            "checks": checks,
            "trigger_count": triggers,
            "translation_error_m": translation_error,
            "yaw_error_deg": math.degrees(yaw_error),
            "vgl_pose": [p.position.x, p.position.y, p.position.z, yaw(p.orientation)],
            "ground_truth_pose": [g.position.x, g.position.y, g.position.z, yaw(g.orientation)],
            "tracking_success_samples": consecutive_tracking,
            "localization_failure_hints": self.failure_hints,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VglTestRunner()
    try:
        report = node.run()
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        node.result_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        if report["status"] != "passed":
            raise RuntimeError(f"VGL acceptance failed: {report}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
