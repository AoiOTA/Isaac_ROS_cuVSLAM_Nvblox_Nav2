"""Run the Phase 5 two-minute cuVSLAM tracking and map-service acceptance test."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import shutil
import statistics
import time

from geometry_msgs.msg import TransformStamped, Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import FilePath, GetAllPoses
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def message_time(message: object) -> float:
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1.0e-9


def yaw_from_quaternion(quaternion: object) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    x: float
    y: float
    yaw: float


class VisualSlamTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("phase5_visual_slam_test_runner")
        self.declare_parameter("result_path", "data/reports/phase5/latest.json")
        self.declare_parameter("map_path", "data/maps/phase5_validation/cuvslam")
        self.declare_parameter("tracking_duration_sim_seconds", 122.0)
        self.result_path = Path(str(self.get_parameter("result_path").value)).resolve()
        self.map_path = Path(str(self.get_parameter("map_path").value)).resolve()
        self.tracking_duration = float(
            self.get_parameter("tracking_duration_sim_seconds").value
        )

        self.clock_time = 0.0
        self.clock_regressions = 0
        self._last_clock = -math.inf
        self.statuses: list[dict[str, float | int]] = []
        self.visual_poses: list[PoseSample] = []
        self.ground_truth: list[PoseSample] = []
        self._last_visual_yaw: float | None = None
        self._visual_yaw = 0.0
        self._last_ground_truth_yaw: float | None = None
        self._ground_truth_yaw = 0.0
        self.tf_edges: dict[tuple[str, str], list[float]] = {}
        self.parents_by_child: dict[str, set[str]] = {}
        self._last_tf_stamp: dict[tuple[str, str], float] = {}
        self.tf_regressions = 0
        self.command_phases: dict[str, int] = {}

        command_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        clock_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        tf_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_safe", command_qos)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(
            VisualSlamStatus, "/visual_slam/status", self.on_status, 20
        )
        self.create_subscription(
            Odometry, "/visual_slam/tracking/odometry", self.on_visual_odometry, 50
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.save_client = self.create_client(FilePath, "/visual_slam/save_map")
        self.load_client = self.create_client(FilePath, "/visual_slam/load_map")
        self.poses_client = self.create_client(GetAllPoses, "/visual_slam/get_all_poses")

    def on_clock(self, message: Clock) -> None:
        current = message.clock.sec + message.clock.nanosec * 1.0e-9
        if current + 1.0e-9 < self._last_clock:
            self.clock_regressions += 1
        self._last_clock = current
        self.clock_time = current

    def on_status(self, message: VisualSlamStatus) -> None:
        self.statuses.append(
            {
                "time_s": message_time(message),
                "state": int(message.vo_state),
                "callback_s": float(message.node_callback_execution_time),
                "track_s": float(message.track_execution_time),
            }
        )

    @staticmethod
    def _append_pose(
        samples: list[PoseSample],
        message: Odometry,
        last_yaw: float | None,
        unwrapped_yaw: float,
    ) -> tuple[float, float]:
        raw_yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if last_yaw is None:
            unwrapped_yaw = raw_yaw
        else:
            unwrapped_yaw += angle_delta(raw_yaw, last_yaw)
        samples.append(
            PoseSample(
                message_time(message),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                unwrapped_yaw,
            )
        )
        return raw_yaw, unwrapped_yaw

    def on_visual_odometry(self, message: Odometry) -> None:
        self._last_visual_yaw, self._visual_yaw = self._append_pose(
            self.visual_poses, message, self._last_visual_yaw, self._visual_yaw
        )

    def on_ground_truth(self, message: Odometry) -> None:
        self._last_ground_truth_yaw, self._ground_truth_yaw = self._append_pose(
            self.ground_truth,
            message,
            self._last_ground_truth_yaw,
            self._ground_truth_yaw,
        )

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            edge = (parent, child)
            stamp = message_time(transform)
            if stamp + 1.0e-9 < self._last_tf_stamp.get(edge, -math.inf):
                self.tf_regressions += 1
            self._last_tf_stamp[edge] = stamp
            self.tf_edges.setdefault(edge, []).append(stamp)
            self.parents_by_child.setdefault(child, set()).add(parent)

    def publish_command(self, linear: float, angular: float, phase: str) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.command_publisher.publish(command)
        self.command_phases[phase] = self.command_phases.get(phase, 0) + 1

    def wait_ready(self, wall_timeout_s: float = 90.0) -> float:
        deadline = time.monotonic() + wall_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            successes = [item for item in self.statuses if item["state"] == 1]
            if successes and self.visual_poses and self.ground_truth:
                return float(successes[-1]["time_s"])
        raise RuntimeError(
            "cuVSLAM did not become ready: "
            f"status={len(self.statuses)} odom={len(self.visual_poses)} "
            f"ground_truth={len(self.ground_truth)}"
        )

    @staticmethod
    def course_command(elapsed: float) -> tuple[float, float, str]:
        cycle = elapsed % 13.0
        if cycle < 1.5:
            return 0.28, 0.65, "forward_left"
        if cycle < 4.5:
            return 0.28, -0.65, "forward_right"
        if cycle < 6.0:
            return 0.28, 0.65, "forward_left"
        if cycle < 6.5:
            return 0.0, 0.0, "settle_forward"
        if cycle < 8.0:
            return -0.28, -0.65, "reverse_right"
        if cycle < 11.0:
            return -0.28, 0.65, "reverse_left"
        if cycle < 12.5:
            return -0.28, -0.65, "reverse_right"
        return 0.0, 0.0, "settle_reverse"

    def run_tracking_course(self, first_success_time: float) -> None:
        wall_deadline = time.monotonic() + 360.0
        while time.monotonic() < wall_deadline:
            elapsed = self.clock_time - first_success_time
            if elapsed >= self.tracking_duration:
                break
            linear, angular, phase = self.course_command(max(0.0, elapsed))
            self.publish_command(linear, angular, phase)
            rclpy.spin_once(self, timeout_sec=0.01)
        else:
            raise RuntimeError("two-minute tracking course exceeded 360 wall seconds")
        settle_start = self.clock_time
        while self.clock_time - settle_start < 1.0:
            self.publish_command(0.0, 0.0, "final_settle")
            rclpy.spin_once(self, timeout_sec=0.01)

    def call_file_service(self, client: object, path: Path) -> bool:
        if not client.wait_for_service(timeout_sec=15.0):
            return False
        request = FilePath.Request()
        request.file_path = str(path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        return future.done() and future.exception() is None and bool(future.result().success)

    def exercise_map_services(self) -> dict[str, object]:
        if self.map_path.exists():
            shutil.rmtree(self.map_path)
        self.map_path.parent.mkdir(parents=True, exist_ok=True)
        save_success = self.call_file_service(self.save_client, self.map_path)
        files = sorted(
            str(path.relative_to(self.map_path))
            for path in self.map_path.rglob("*")
            if path.is_file()
        ) if self.map_path.is_dir() else []

        poses_success = False
        pose_count = 0
        if self.poses_client.wait_for_service(timeout_sec=15.0):
            request = GetAllPoses.Request()
            request.max_count = 10000
            future = self.poses_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            if future.done() and future.exception() is None:
                poses_success = bool(future.result().success)
                pose_count = len(future.result().poses)

        load_success = self.call_file_service(self.load_client, self.map_path)
        return {
            "save_success": save_success,
            "load_success": load_success,
            "get_all_poses_success": poses_success,
            "optimized_pose_count": pose_count,
            "map_files": files,
            "map_path": str(self.map_path),
        }

    @staticmethod
    def path_length(samples: list[PoseSample], minimum_dt: float = 0.08) -> float:
        if not samples:
            return 0.0
        selected = [samples[0]]
        for sample in samples[1:]:
            if sample.time_s - selected[-1].time_s >= minimum_dt:
                selected.append(sample)
        return sum(
            math.hypot(current.x - previous.x, current.y - previous.y)
            for previous, current in zip(selected, selected[1:])
        )

    @staticmethod
    def nearest(samples: list[PoseSample], stamp: float, start_index: int) -> tuple[int, PoseSample]:
        index = start_index
        while index + 1 < len(samples) and abs(samples[index + 1].time_s - stamp) <= abs(
            samples[index].time_s - stamp
        ):
            index += 1
        return index, samples[index]

    def trajectory_metrics(self) -> dict[str, float | int]:
        if len(self.visual_poses) < 2 or len(self.ground_truth) < 2:
            return {
                "pair_count": 0,
                "window_count": 0,
                "median_translation_cosine": -1.0,
                "median_scale_ratio": math.inf,
                "windowed_path_ratio": math.inf,
                "yaw_sign_fraction": 0.0,
            }
        pairs: list[tuple[PoseSample, PoseSample]] = []
        gt_index = 0
        for visual in self.visual_poses:
            gt_index, ground_truth = self.nearest(self.ground_truth, visual.time_s, gt_index)
            if abs(ground_truth.time_s - visual.time_s) <= 0.1:
                pairs.append((visual, ground_truth))
        # map/odom is intentionally independent from the simulator's sim_world
        # frame. Compare one-second displacement vectors after estimating the
        # single planar rotation between those coordinate systems. The longer
        # window also prevents frame-to-frame estimator noise from being counted
        # as physical path length.
        windows: list[tuple[float, float, float, float]] = []
        end_index = 0
        for start_index, (visual_a, gt_a) in enumerate(pairs):
            end_index = max(end_index, start_index + 1)
            while (
                end_index < len(pairs)
                and pairs[end_index][0].time_s - visual_a.time_s < 1.0
            ):
                end_index += 1
            if end_index >= len(pairs):
                break
            visual_b, gt_b = pairs[end_index]
            if visual_b.time_s - visual_a.time_s <= 1.5:
                windows.append(
                    (
                        visual_b.x - visual_a.x,
                        visual_b.y - visual_a.y,
                        gt_b.x - gt_a.x,
                        gt_b.y - gt_a.y,
                    )
                )

        rotation_a = sum(vx * gx + vy * gy for vx, vy, gx, gy in windows)
        rotation_b = sum(-vy * gx + vx * gy for vx, vy, gx, gy in windows)
        alignment_yaw = math.atan2(rotation_b, rotation_a)
        cosine_yaw = math.cos(alignment_yaw)
        sine_yaw = math.sin(alignment_yaw)
        translation_cosines: list[float] = []
        scale_ratios: list[float] = []
        for visual_x, visual_y, gt_x, gt_y in windows:
            aligned_x = cosine_yaw * visual_x - sine_yaw * visual_y
            aligned_y = sine_yaw * visual_x + cosine_yaw * visual_y
            visual_norm = math.hypot(aligned_x, aligned_y)
            gt_norm = math.hypot(gt_x, gt_y)
            if visual_norm > 0.03 and gt_norm > 0.03:
                translation_cosines.append(
                    (aligned_x * gt_x + aligned_y * gt_y) / (visual_norm * gt_norm)
                )
                scale_ratios.append(visual_norm / gt_norm)

        yaw_signs: list[float] = []
        for (visual_a, gt_a), (visual_b, gt_b) in zip(pairs, pairs[1:]):
            vyaw, gyaw = visual_b.yaw - visual_a.yaw, gt_b.yaw - gt_a.yaw
            if abs(vyaw) > 0.003 and abs(gyaw) > 0.003:
                yaw_signs.append(1.0 if vyaw * gyaw > 0.0 else 0.0)
        visual_windowed_length = self.path_length(
            [visual for visual, _ in pairs], minimum_dt=1.0
        )
        gt_windowed_length = self.path_length(
            [ground_truth for _, ground_truth in pairs], minimum_dt=1.0
        )
        return {
            "pair_count": len(pairs),
            "window_count": len(windows),
            "translation_comparison_count": len(translation_cosines),
            "yaw_comparison_count": len(yaw_signs),
            "coordinate_alignment_yaw_rad": alignment_yaw,
            "median_translation_cosine": statistics.median(translation_cosines)
            if translation_cosines
            else -1.0,
            "median_scale_ratio": statistics.median(scale_ratios)
            if scale_ratios
            else math.inf,
            "visual_windowed_path_length_m": visual_windowed_length,
            "ground_truth_windowed_path_length_m": gt_windowed_length,
            "windowed_path_ratio": visual_windowed_length / gt_windowed_length
            if gt_windowed_length > 0.0
            else math.inf,
            "yaw_sign_fraction": statistics.mean(yaw_signs) if yaw_signs else 0.0,
        }

    def build_report(self, map_services: dict[str, object]) -> dict[str, object]:
        success_statuses = [item for item in self.statuses if item["state"] == 1]
        tracking_start = float(success_statuses[0]["time_s"]) if success_statuses else math.inf
        tracking_end = float(success_statuses[-1]["time_s"]) if success_statuses else -math.inf
        tracking_statuses = [
            item for item in self.statuses if float(item["time_s"]) >= tracking_start
        ]
        failed_after_lock = [item for item in tracking_statuses if item["state"] != 1]
        status_gaps = [
            float(right["time_s"]) - float(left["time_s"])
            for left, right in zip(tracking_statuses, tracking_statuses[1:])
        ]
        visual_length = self.path_length(self.visual_poses)
        ground_truth_length = self.path_length(self.ground_truth)
        trajectory = self.trajectory_metrics()
        required_edges = (("map", "odom"), ("odom", "base_link"))
        checks = {
            "tracking_two_minutes": tracking_end - tracking_start >= 120.0,
            "tracking_never_failed_after_lock": not failed_after_lock,
            "tracking_status_flowing": len(tracking_statuses) >= 300
            and (max(status_gaps) if status_gaps else math.inf) <= 1.0,
            "tracking_odometry_flowing": len(self.visual_poses) >= 300,
            "clock_monotonic": self.clock_regressions == 0,
            "tf_monotonic": self.tf_regressions == 0,
            "tf_chain_present": all(len(self.tf_edges.get(edge, [])) >= 300 for edge in required_edges),
            "tf_unique_parents": self.parents_by_child.get("odom") == {"map"}
            and self.parents_by_child.get("base_link") == {"odom"},
            "trajectory_scale": ground_truth_length > 5.0
            and int(trajectory["translation_comparison_count"]) >= 50
            and 0.65 <= float(trajectory["median_scale_ratio"]) <= 1.35
            and 0.65 <= float(trajectory["windowed_path_ratio"]) <= 1.35,
            "translation_direction": int(trajectory["translation_comparison_count"]) >= 50
            and float(trajectory["median_translation_cosine"]) >= 0.75,
            "rotation_direction": int(trajectory["yaw_comparison_count"]) >= 50
            and float(trajectory["yaw_sign_fraction"]) >= 0.85,
            "map_saved": bool(map_services["save_success"])
            and bool(map_services["map_files"]),
            "map_loaded": bool(map_services["load_success"]),
            "optimized_poses_available": bool(map_services["get_all_poses_success"])
            and int(map_services["optimized_pose_count"]) >= 10,
            "all_motion_phases": all(count >= 10 for count in self.command_phases.values()),
        }
        checks = {name: bool(value) for name, value in checks.items()}
        callback_times = [float(item["callback_s"]) for item in tracking_statuses]
        track_times = [float(item["track_s"]) for item in tracking_statuses]
        return {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "tracking": {
                "start_s": tracking_start,
                "end_s": tracking_end,
                "duration_s": tracking_end - tracking_start,
                "status_count": len(tracking_statuses),
                "failed_after_lock": len(failed_after_lock),
                "maximum_status_gap_s": max(status_gaps) if status_gaps else math.inf,
                "callback_time_mean_ms": 1000.0 * statistics.mean(callback_times),
                "callback_time_p99_ms": 1000.0 * sorted(callback_times)[
                    min(len(callback_times) - 1, int(0.99 * len(callback_times)))
                ],
                "track_time_mean_ms": 1000.0 * statistics.mean(track_times),
            },
            "trajectories": {
                "visual_sample_count": len(self.visual_poses),
                "ground_truth_sample_count": len(self.ground_truth),
                "visual_path_length_m": visual_length,
                "ground_truth_path_length_m": ground_truth_length,
                "path_length_ratio": visual_length / ground_truth_length
                if ground_truth_length > 0.0
                else math.inf,
                **trajectory,
            },
            "tf": {
                "edge_counts": {
                    f"{parent}->{child}": len(stamps)
                    for (parent, child), stamps in self.tf_edges.items()
                },
                "parents_by_child": {
                    child: sorted(parents) for child, parents in self.parents_by_child.items()
                },
                "regressions": self.tf_regressions,
            },
            "clock_regressions": self.clock_regressions,
            "map_services": map_services,
            "command_phases": self.command_phases,
        }

    def run(self) -> dict[str, object]:
        first_success = self.wait_ready()
        self.run_tracking_course(first_success)
        map_services = self.exercise_map_services()
        report = self.build_report(map_services)
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return report


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualSlamTestRunner()
    try:
        report = node.run()
        failed = [name for name, passed in report["checks"].items() if not passed]
        if failed:
            raise RuntimeError(f"Phase 5 checks failed: {', '.join(failed)}")
        node.get_logger().info(f"Phase 5 cuVSLAM checks passed: {node.result_path}")
    finally:
        node.publish_command(0.0, 0.0, "shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
