"""Run the Phase 3 straight, spin, arc, S-turn, sharp-turn, and safety suite."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


LEFT_JOINTS = ("front_left_wheel_joint", "rear_left_wheel_joint")
RIGHT_JOINTS = ("front_right_wheel_joint", "rear_right_wheel_joint")
WHEEL_JOINTS = (*LEFT_JOINTS, *RIGHT_JOINTS)
WHEEL_RADIUS = 0.098
WHEEL_SEPARATION = 0.800


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    x: float
    y: float
    yaw: float
    linear: float = 0.0
    angular: float = 0.0
    command_linear: float = 0.0
    command_angular: float = 0.0


@dataclass(frozen=True)
class CommandSample:
    time_s: float
    linear: float
    angular: float


class MotionTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("phase3_motion_test_runner")
        self.declare_parameter("result_path", "data/reports/phase3/latest.json")
        self.result_path = Path(str(self.get_parameter("result_path").value)).resolve()
        command_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_safe", command_qos)
        self.create_subscription(
            Twist, "/cmd_vel_sim", self.on_guard_command, command_qos
        )
        self.create_subscription(
            Odometry, "/ground_truth/odometry", self.on_ground_truth, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/wheel/odometry", self.on_wheel_odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, qos_profile_sensor_data
        )
        self.create_subscription(String, "/control/guard_status", self.on_guard_status, command_qos)

        self.ground_truth: list[PoseSample] = []
        self.wheel_odometry: list[PoseSample] = []
        self.commands: list[CommandSample] = []
        self.joint_samples: list[dict[str, float]] = []
        self.joint_names: set[str] = set()
        self.joint_peak_velocity: dict[str, float] = {}
        self.guard_states: list[str] = []
        self._last_ground_truth_yaw: float | None = None
        self._unwrapped_ground_truth_yaw = 0.0

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_ground_truth(self, message: Odometry) -> None:
        raw_yaw = yaw_from_odometry(message)
        if self._last_ground_truth_yaw is None:
            self._unwrapped_ground_truth_yaw = raw_yaw
        else:
            self._unwrapped_ground_truth_yaw += angle_delta(raw_yaw, self._last_ground_truth_yaw)
        self._last_ground_truth_yaw = raw_yaw
        latest_command = (
            self.commands[-1]
            if self.commands
            else CommandSample(self.now_s(), 0.0, 0.0)
        )
        self.ground_truth.append(
            PoseSample(
                self.now_s(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                self._unwrapped_ground_truth_yaw,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
                latest_command.linear,
                latest_command.angular,
            )
        )

    def on_wheel_odometry(self, message: Odometry) -> None:
        raw_yaw = yaw_from_odometry(message)
        if self.wheel_odometry:
            yaw = self.wheel_odometry[-1].yaw + angle_delta(raw_yaw, self.wheel_odometry[-1].yaw)
        else:
            yaw = raw_yaw
        latest_command = (
            self.commands[-1]
            if self.commands
            else CommandSample(self.now_s(), 0.0, 0.0)
        )
        self.wheel_odometry.append(
            PoseSample(
                self.now_s(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
                latest_command.linear,
                latest_command.angular,
            )
        )

    def on_guard_command(self, message: Twist) -> None:
        self.commands.append(CommandSample(self.now_s(), message.linear.x, message.angular.z))

    def on_guard_status(self, message: String) -> None:
        try:
            self.guard_states.append(str(json.loads(message.data)["state"]))
        except (json.JSONDecodeError, KeyError, TypeError):
            self.guard_states.append(message.data)

    @staticmethod
    def find_joint(names: list[str], desired: str) -> int:
        for index, name in enumerate(names):
            if name == desired or name.rsplit("/", 1)[-1] == desired:
                return index
        raise ValueError(desired)

    @classmethod
    def average_joint_velocity(
        cls,
        message: JointState,
        desired: tuple[str, str],
    ) -> float:
        indices = [cls.find_joint(message.name, name) for name in desired]
        return statistics.fmean(float(message.velocity[index]) for index in indices)

    def on_joint_state(self, message: JointState) -> None:
        self.joint_names.update(message.name)
        for index, name in enumerate(message.name):
            if index < len(message.velocity):
                self.joint_peak_velocity[name] = max(
                    self.joint_peak_velocity.get(name, 0.0), abs(float(message.velocity[index]))
                )
        try:
            left = self.average_joint_velocity(message, LEFT_JOINTS)
            right = self.average_joint_velocity(message, RIGHT_JOINTS)
        except (ValueError, IndexError):
            return
        latest_command = self.commands[-1] if self.commands else CommandSample(self.now_s(), 0.0, 0.0)
        self.joint_samples.append(
            {
                "time_s": self.now_s(),
                "left_radps": left,
                "right_radps": right,
                "command_linear": latest_command.linear,
                "command_angular": latest_command.angular,
            }
        )

    def spin_for_wall(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.now_s() > 0.0
                and self.ground_truth
                and self.wheel_odometry
                and self.joint_samples
                and self.commands
            ):
                return
        raise RuntimeError(
            "Phase 3 topics did not become ready: "
            f"gt={len(self.ground_truth)} wheel={len(self.wheel_odometry)} "
            f"joint={len(self.joint_samples)} command={len(self.commands)} clock={self.now_s():.3f}"
        )

    def publish_command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.command_publisher.publish(message)

    def drive_until(
        self,
        linear: float,
        angular: float,
        condition: callable,
        max_sim_duration_s: float,
    ) -> None:
        start = self.now_s()
        while self.now_s() - start < max_sim_duration_s:
            self.publish_command(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.01)
            if condition():
                return
        raise RuntimeError(
            f"motion condition timed out after {max_sim_duration_s}s: v={linear} w={angular}"
        )

    def drive_for(self, linear: float, angular: float, sim_duration_s: float) -> None:
        start = self.now_s()
        while self.now_s() - start < sim_duration_s:
            self.publish_command(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.01)

    def settle(self, sim_duration_s: float = 0.8) -> None:
        self.drive_for(0.0, 0.0, sim_duration_s)

    @staticmethod
    def relative_pose(start: PoseSample, end: PoseSample) -> tuple[float, float, float]:
        dx = end.x - start.x
        dy = end.y - start.y
        cosine = math.cos(start.yaw)
        sine = math.sin(start.yaw)
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            end.yaw - start.yaw,
        )

    def snapshot(self) -> tuple[PoseSample, PoseSample, int, int, int, int]:
        return (
            self.ground_truth[-1],
            self.wheel_odometry[-1],
            len(self.commands),
            len(self.joint_samples),
            len(self.ground_truth),
            len(self.wheel_odometry),
        )

    def segment_metrics(
        self,
        name: str,
        snapshot: tuple[PoseSample, PoseSample, int, int, int, int],
        *,
        expected_linear: float | None = None,
        expected_angular: float | None = None,
    ) -> dict[str, float | str | None]:
        (
            gt_start,
            wheel_start,
            command_start,
            joint_start,
            ground_truth_start,
            wheel_odometry_start,
        ) = snapshot
        gt = self.relative_pose(gt_start, self.ground_truth[-1])
        wheel = self.relative_pose(wheel_start, self.wheel_odometry[-1])
        gt_distance = math.hypot(gt[0], gt[1])
        position_error = math.hypot(wheel[0] - gt[0], wheel[1] - gt[1])
        yaw_error = abs(wheel[2] - gt[2])
        commands = self.commands[command_start:]
        joints = self.joint_samples[joint_start:]
        ground_truth = self.ground_truth[ground_truth_start:]
        wheel_odometry = self.wheel_odometry[wheel_odometry_start:]

        def steady(samples: list[PoseSample]) -> list[PoseSample]:
            if expected_linear is None or expected_angular is None:
                return []
            matching = [
                sample
                for sample in samples
                if abs(sample.command_linear - expected_linear) <= 0.02
                and abs(sample.command_angular - expected_angular) <= 0.03
            ]
            if not matching:
                return []
            # Mirror the reference benchmark's steady window: the output
            # command reaching its target is not the same instant that the
            # PhysX chassis has settled to that twist.
            steady_start = matching[0].time_s + 0.35
            steady_end = matching[-1].time_s - 0.10
            return [
                sample
                for sample in matching
                if steady_start <= sample.time_s <= steady_end
            ]

        steady_ground_truth = steady(ground_truth)
        steady_wheel_odometry = steady(wheel_odometry)
        mean_gt_linear = (
            statistics.fmean(sample.linear for sample in steady_ground_truth)
            if steady_ground_truth
            else None
        )
        mean_gt_angular = (
            statistics.fmean(sample.angular for sample in steady_ground_truth)
            if steady_ground_truth
            else None
        )
        mean_wheel_linear = (
            statistics.fmean(sample.linear for sample in steady_wheel_odometry)
            if steady_wheel_odometry
            else None
        )
        mean_wheel_angular = (
            statistics.fmean(sample.angular for sample in steady_wheel_odometry)
            if steady_wheel_odometry
            else None
        )
        return {
            "name": name,
            "ground_truth_x_m": gt[0],
            "ground_truth_y_m": gt[1],
            "ground_truth_yaw_rad": gt[2],
            "ground_truth_displacement_m": gt_distance,
            "wheel_odometry_position_error_m": position_error,
            "wheel_odometry_yaw_error_rad": yaw_error,
            "command_samples": len(commands),
            "joint_samples": len(joints),
            "steady_ground_truth_samples": len(steady_ground_truth),
            "steady_wheel_odometry_samples": len(steady_wheel_odometry),
            "mean_ground_truth_linear_mps": mean_gt_linear,
            "mean_ground_truth_angular_radps": mean_gt_angular,
            "mean_wheel_odometry_linear_mps": mean_wheel_linear,
            "mean_wheel_odometry_angular_radps": mean_wheel_angular,
            "steady_wheel_linear_error_mps": (
                abs(mean_wheel_linear - mean_gt_linear)
                if mean_wheel_linear is not None and mean_gt_linear is not None
                else None
            ),
            "steady_wheel_angular_error_radps": (
                abs(mean_wheel_angular - mean_gt_angular)
                if mean_wheel_angular is not None and mean_gt_angular is not None
                else None
            ),
        }

    def run_suite(self) -> dict[str, object]:
        self.wait_ready()
        self.settle(0.5)
        segments: dict[str, dict[str, float | str | None]] = {}

        start = self.snapshot()
        gt_start = start[0]
        self.drive_until(
            0.5,
            0.0,
            lambda: self.relative_pose(gt_start, self.ground_truth[-1])[0] >= 1.84,
            8.0,
        )
        self.settle()
        segments["straight_2m"] = self.segment_metrics(
            "straight_2m", start, expected_linear=0.5, expected_angular=0.0
        )

        start = self.snapshot()
        gt_start = start[0]
        self.drive_until(
            -0.5,
            0.0,
            lambda: self.relative_pose(gt_start, self.ground_truth[-1])[0] <= -1.84,
            8.0,
        )
        self.settle()
        segments["reverse_2m"] = self.segment_metrics(
            "reverse_2m", start, expected_linear=-0.5, expected_angular=0.0
        )

        start = self.snapshot()
        gt_start = start[0]
        self.drive_until(
            0.0,
            0.9,
            lambda: self.ground_truth[-1].yaw - gt_start.yaw >= math.pi - 0.27,
            7.0,
        )
        self.settle()
        segments["spin_180"] = self.segment_metrics(
            "spin_180", start, expected_linear=0.0, expected_angular=0.9
        )

        start = self.snapshot()
        gt_start = start[0]
        self.drive_until(
            0.32,
            0.8,
            lambda: self.ground_truth[-1].yaw - gt_start.yaw >= 0.5 * math.pi - 0.19,
            4.0,
        )
        self.settle()
        segments["arc_radius_040"] = self.segment_metrics(
            "arc_radius_040", start, expected_linear=0.32, expected_angular=0.8
        )
        arc_return_start = self.ground_truth[-1]
        self.drive_until(
            -0.32,
            -0.8,
            lambda: arc_return_start.yaw - self.ground_truth[-1].yaw >= 0.5 * math.pi - 0.19,
            4.0,
        )
        self.settle()

        start = self.snapshot()
        for angular, duration in ((0.95, 0.8), (-0.95, 1.6), (0.95, 0.8)):
            self.drive_for(0.30, angular, duration)
        self.settle()
        segments["s_curve"] = self.segment_metrics("s_curve", start)
        for angular, duration in ((-0.95, 0.8), (0.95, 1.6), (-0.95, 0.8)):
            self.drive_for(-0.30, angular, duration)
        self.settle()

        start = self.snapshot()
        for index in range(8):
            self.drive_for(0.20, 1.0 if index % 2 == 0 else -1.0, 0.45)
        self.settle()
        segments["continuous_sharp_turns"] = self.segment_metrics(
            "continuous_sharp_turns", start
        )

        limit_command_index = len(self.commands)
        self.drive_for(5.0, 5.0, 1.5)
        limited_commands = self.commands[limit_command_index:]
        limit_linear = max((abs(item.linear) for item in limited_commands), default=math.inf)
        limit_angular = max((abs(item.angular) for item in limited_commands), default=math.inf)
        self.settle()

        watchdog_index = len(self.commands)
        self.drive_for(0.35, 0.0, 0.8)
        watchdog_start = self.now_s()
        while self.now_s() - watchdog_start < 0.6:
            rclpy.spin_once(self, timeout_sec=0.01)
        watchdog_commands = self.commands[watchdog_index:]
        zero_times = [
            item.time_s
            for item in watchdog_commands
            if item.time_s >= watchdog_start
            and abs(item.linear) < 1.0e-4
            and abs(item.angular) < 1.0e-4
        ]
        watchdog_latency = min(zero_times) - watchdog_start if zero_times else math.inf

        self.drive_for(0.25, 0.2, 0.5)
        invalid_index = len(self.commands)
        invalid_start = self.now_s()
        invalid = Twist()
        invalid.linear.x = math.nan
        self.command_publisher.publish(invalid)
        while self.now_s() - invalid_start < 0.2:
            rclpy.spin_once(self, timeout_sec=0.01)
        invalid_zeros = [
            item.time_s
            for item in self.commands[invalid_index:]
            if abs(item.linear) < 1.0e-4 and abs(item.angular) < 1.0e-4
        ]
        invalid_latency = min(invalid_zeros) - invalid_start if invalid_zeros else math.inf

        self.publish_command(0.0, 0.0)
        self.settle()

        normal_commands = [item for item in self.commands if math.isfinite(item.linear)]
        linear_accelerations: list[float] = []
        angular_accelerations: list[float] = []
        for previous, current in zip(normal_commands, normal_commands[1:]):
            dt = current.time_s - previous.time_s
            if 0.002 <= dt <= 0.05:
                linear_accelerations.append(abs(current.linear - previous.linear) / dt)
                angular_accelerations.append(abs(current.angular - previous.angular) / dt)

        steady_wheel_errors: list[float] = []
        for sample in self.joint_samples:
            linear = sample["command_linear"]
            angular = sample["command_angular"]
            if abs(linear) < 0.2 and abs(angular) < 0.35:
                continue
            expected_left = (linear - 0.5 * WHEEL_SEPARATION * angular) / WHEEL_RADIUS
            expected_right = (linear + 0.5 * WHEEL_SEPARATION * angular) / WHEEL_RADIUS
            steady_wheel_errors.append(
                math.hypot(
                    sample["left_radps"] - expected_left,
                    sample["right_radps"] - expected_right,
                )
                / math.sqrt(2.0)
            )

        tracked_segments = [
            segments[name]
            for name in ("straight_2m", "reverse_2m", "spin_180", "arc_radius_040")
        ]
        wheel_linear_errors = [
            float(value)
            for segment in tracked_segments
            if (value := segment["steady_wheel_linear_error_mps"]) is not None
        ]
        wheel_angular_errors = [
            float(value)
            for segment in tracked_segments
            if (value := segment["steady_wheel_angular_error_radps"]) is not None
        ]
        arc_linear = segments["arc_radius_040"]["mean_ground_truth_linear_mps"]
        arc_angular = segments["arc_radius_040"]["mean_ground_truth_angular_radps"]
        steady_arc_radius = (
            abs(float(arc_linear) / float(arc_angular))
            if arc_linear is not None
            and arc_angular is not None
            and abs(float(arc_angular)) > 1.0e-9
            else math.inf
        )
        segments["arc_radius_040"]["steady_ground_truth_radius_m"] = (
            steady_arc_radius
        )

        checks = {
            "straight_distance": 1.95 <= float(segments["straight_2m"]["ground_truth_x_m"]) <= 2.15,
            "straight_lateral": abs(float(segments["straight_2m"]["ground_truth_y_m"])) <= 0.08,
            "straight_heading": abs(float(segments["straight_2m"]["ground_truth_yaw_rad"])) <= 0.08,
            "reverse_distance": float(segments["reverse_2m"]["ground_truth_x_m"]) <= -1.95,
            "spin_angle": abs(float(segments["spin_180"]["ground_truth_yaw_rad"]) - math.pi) <= 0.18,
            "spin_center_drift": float(segments["spin_180"]["ground_truth_displacement_m"]) <= 0.15,
            "arc_angle": abs(float(segments["arc_radius_040"]["ground_truth_yaw_rad"]) - 0.5 * math.pi) <= 0.18,
            "arc_radius": 0.30 <= steady_arc_radius <= 0.50,
            "steady_tracking_samples": all(
                int(segment["steady_ground_truth_samples"] or 0) >= 20
                and int(segment["steady_wheel_odometry_samples"] or 0) >= 20
                for segment in tracked_segments
            ),
            "wheel_odometry_linear_tracking": (
                len(wheel_linear_errors) == len(tracked_segments)
                and max(wheel_linear_errors, default=math.inf) <= 0.06
            ),
            "wheel_odometry_angular_tracking": (
                len(wheel_angular_errors) == len(tracked_segments)
                and max(wheel_angular_errors, default=math.inf) <= 0.12
            ),
            "hard_linear_limit": limit_linear <= 0.7501,
            "hard_angular_limit": limit_angular <= 1.2001,
            "watchdog": watchdog_latency <= 0.32,
            "invalid_rejected": invalid_latency <= 0.12,
            "four_wheel_skid_steer_joints_present": all(
                any(name.endswith(expected) for name in self.joint_names)
                for expected in WHEEL_JOINTS
            ),
        }
        result = {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "segments": segments,
            "safety": {
                "observed_max_linear_command_mps": limit_linear,
                "observed_max_angular_command_radps": limit_angular,
                "watchdog_zero_latency_s": watchdog_latency,
                "invalid_command_zero_latency_s": invalid_latency,
                "guard_states": sorted(set(self.guard_states)),
            },
            "smoothness": {
                "linear_acceleration_p99_mps2": statistics.quantiles(
                    linear_accelerations, n=100
                )[98]
                if len(linear_accelerations) >= 100
                else max(linear_accelerations, default=0.0),
                "angular_acceleration_p99_radps2": statistics.quantiles(
                    angular_accelerations, n=100
                )[98]
                if len(angular_accelerations) >= 100
                else max(angular_accelerations, default=0.0),
                "steady_wheel_speed_rmse_radps": math.sqrt(
                    statistics.fmean(value * value for value in steady_wheel_errors)
                )
                if steady_wheel_errors
                else math.inf,
            },
            "joint_state": {
                "names": sorted(self.joint_names),
                "commanded_wheel_names": list(WHEEL_JOINTS),
                "peak_velocity_radps": self.joint_peak_velocity,
            },
            "sample_counts": {
                "ground_truth": len(self.ground_truth),
                "wheel_odometry": len(self.wheel_odometry),
                "commands": len(self.commands),
                "joint_states": len(self.joint_samples),
            },
        }
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionTestRunner()
    exit_code = 1
    try:
        result = node.run_suite()
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = node.result_path.with_suffix(node.result_path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(node.result_path)
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)
        exit_code = 0 if result["status"] == "passed" else 1
    except Exception as exc:  # noqa: BLE001 - persist automation failure context
        failure = {"status": "failed", "error": str(exc)}
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        node.result_path.write_text(json.dumps(failure, indent=2) + "\n", encoding="utf-8")
        node.get_logger().error(str(exc))
    finally:
        node.publish_command(0.0, 0.0)
        node.spin_for_wall(0.1)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
