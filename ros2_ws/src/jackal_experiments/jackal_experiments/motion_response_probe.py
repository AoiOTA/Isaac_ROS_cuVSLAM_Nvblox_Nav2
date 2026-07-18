"""Measure a safe Jackal body-twist response against simulation Ground Truth."""

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


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    x: float
    y: float
    yaw: float
    angular_z: float
    linear_x: float = 0.0
    observed_linear: float = 0.0
    observed_angular: float = 0.0


def evaluate_spin(
    samples: list[PoseSample],
    *,
    command_angular: float,
    duration_s: float,
    maximum_observed_command: float,
) -> dict[str, object]:
    if len(samples) < 2:
        raise ValueError("spin response requires at least two Ground Truth samples")
    expected_yaw = command_angular * duration_s
    actual_yaw = samples[-1].yaw - samples[0].yaw
    tracking_ratio = abs(actual_yaw / expected_yaw)
    drift = math.hypot(samples[-1].x - samples[0].x, samples[-1].y - samples[0].y)
    effective_rate = actual_yaw / max(
        samples[-1].time_s - samples[0].time_s,
        1.0e-9,
    )
    checks = {
        "sample_count": len(samples) >= 30,
        "command_reached_simulator": maximum_observed_command
        >= 0.95 * abs(command_angular),
        "correct_direction": actual_yaw * command_angular > 0.0,
        "yaw_tracking": 0.85 <= tracking_ratio <= 1.15,
        "center_drift": drift <= 0.15,
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "command_angular_radps": command_angular,
        "configured_duration_s": duration_s,
        "observed_duration_s": samples[-1].time_s - samples[0].time_s,
        "expected_yaw_change_rad": expected_yaw,
        "actual_yaw_change_rad": actual_yaw,
        "yaw_tracking_ratio": tracking_ratio,
        "effective_yaw_rate_radps": effective_rate,
        "center_drift_m": drift,
        "maximum_observed_command_radps": maximum_observed_command,
        "ground_truth_sample_count": len(samples),
    }


def evaluate_arc(
    samples: list[PoseSample],
    *,
    command_linear: float,
    command_angular: float,
    duration_s: float,
    maximum_observed_linear: float,
    maximum_observed_angular: float,
) -> dict[str, object]:
    """Evaluate the steady portion of a direct body-twist response."""

    if len(samples) < 2:
        raise ValueError("arc response requires at least two Ground Truth samples")
    if abs(command_linear) < 1.0e-9 or abs(command_angular) < 1.0e-9:
        raise ValueError("arc response requires non-zero linear and angular commands")
    command_matching = [
        sample
        for sample in samples
        if abs(sample.observed_linear - command_linear) <= 0.02
        and abs(sample.observed_angular - command_angular) <= 0.03
    ]
    if command_matching:
        steady_start = command_matching[0].time_s + 0.35
        steady_end = command_matching[-1].time_s - 0.10
        steady = [
            sample
            for sample in command_matching
            if steady_start <= sample.time_s <= steady_end
        ]
    else:
        # Pure evaluator tests can omit observed-command fields.
        started = samples[0].time_s
        window = min(0.35, duration_s * 0.4)
        steady = [
            sample
            for sample in samples
            if window
            <= sample.time_s - started
            <= max(window, duration_s - 0.10)
        ]
    if not steady:
        raise ValueError("arc response has no steady Ground Truth samples")
    mean_linear = statistics.fmean(sample.linear_x for sample in steady)
    mean_angular = statistics.fmean(sample.angular_z for sample in steady)
    requested_radius = abs(command_linear / command_angular)
    actual_radius = (
        abs(mean_linear / mean_angular) if abs(mean_angular) > 1.0e-9 else math.inf
    )
    radius_error = abs(actual_radius - requested_radius) / requested_radius
    actual_yaw = samples[-1].yaw - samples[0].yaw
    checks = {
        "sample_count": len(samples) >= 30 and len(steady) >= 20,
        "command_reached_simulator": (
            maximum_observed_linear >= 0.95 * abs(command_linear)
            and maximum_observed_angular >= 0.95 * abs(command_angular)
        ),
        "correct_direction": (
            mean_linear * command_linear > 0.0
            and mean_angular * command_angular > 0.0
        ),
        "linear_tracking": abs(mean_linear - command_linear) <= 0.06,
        "angular_tracking": abs(mean_angular - command_angular) <= 0.12,
        "curvature_tracking": radius_error <= 0.20,
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "command_linear_mps": command_linear,
        "command_angular_radps": command_angular,
        "configured_duration_s": duration_s,
        "observed_duration_s": samples[-1].time_s - samples[0].time_s,
        "mean_steady_linear_mps": mean_linear,
        "mean_steady_angular_radps": mean_angular,
        "requested_radius_m": requested_radius,
        "actual_radius_m": actual_radius,
        "radius_relative_error_percent": 100.0 * radius_error,
        "actual_yaw_change_rad": actual_yaw,
        "net_displacement_m": math.hypot(
            samples[-1].x - samples[0].x,
            samples[-1].y - samples[0].y,
        ),
        "maximum_observed_linear_mps": maximum_observed_linear,
        "maximum_observed_angular_radps": maximum_observed_angular,
        "ground_truth_sample_count": len(samples),
        "steady_sample_count": len(steady),
        "command_matching_sample_count": len(command_matching),
    }
class MotionResponseProbe(Node):
    def __init__(self) -> None:
        super().__init__("jackal_motion_response_probe")
        self.declare_parameter("result_path", "/tmp/jackal-motion-response.json")
        self.declare_parameter("command_linear_mps", 0.0)
        self.declare_parameter("command_angular_radps", 0.8)
        self.declare_parameter("command_duration_s", 3.2)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("wall_timeout_s", 45.0)
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("observed_command_topic", "/cmd_vel_sim")
        self.declare_parameter("ground_truth_topic", "/ground_truth/odometry")
        self.result_path = Path(
            str(self.get_parameter("result_path").value)
        ).resolve()
        self.command_angular = float(
            self.get_parameter("command_angular_radps").value
        )
        self.command_linear = float(
            self.get_parameter("command_linear_mps").value
        )
        self.command_duration = float(
            self.get_parameter("command_duration_s").value
        )
        self.command_period = 1.0 / float(
            self.get_parameter("command_rate_hz").value
        )
        self.wall_timeout = float(self.get_parameter("wall_timeout_s").value)
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.command_duration,
                self.command_period,
                self.wall_timeout,
            )
        ):
            raise ValueError("motion response parameters must be finite and positive")
        if (
            not math.isfinite(self.command_linear)
            or not math.isfinite(self.command_angular)
            or (
                abs(self.command_linear) < 1.0e-9
                and abs(self.command_angular) < 1.0e-9
            )
        ):
            raise ValueError("motion response command must be finite and non-zero")

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        command_topic = str(self.get_parameter("command_topic").value)
        observed_command_topic = str(
            self.get_parameter("observed_command_topic").value
        )
        ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        if not all(
            topic.startswith("/")
            for topic in (
                command_topic,
                observed_command_topic,
                ground_truth_topic,
            )
        ):
            raise ValueError("motion response topics must be absolute")
        self.publisher = self.create_publisher(Twist, command_topic, reliable)
        self.create_subscription(
            Twist,
            observed_command_topic,
            self.on_command,
            reliable,
        )
        self.create_subscription(
            Odometry,
            ground_truth_topic,
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.samples: list[PoseSample] = []
        self.maximum_observed_linear = 0.0
        self.maximum_observed_command = 0.0
        self.observed_linear = 0.0
        self.observed_angular = 0.0
        self.last_raw_yaw: float | None = None
        self.unwrapped_yaw = 0.0

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_command(self, message: Twist) -> None:
        self.observed_linear = float(message.linear.x)
        self.observed_angular = float(message.angular.z)
        self.maximum_observed_linear = max(
            self.maximum_observed_linear,
            abs(float(message.linear.x)),
        )
        self.maximum_observed_command = max(
            self.maximum_observed_command,
            abs(float(message.angular.z)),
        )

    def on_odometry(self, message: Odometry) -> None:
        raw_yaw = yaw_from_odometry(message)
        if self.last_raw_yaw is None:
            self.unwrapped_yaw = raw_yaw
        else:
            self.unwrapped_yaw += angle_delta(raw_yaw, self.last_raw_yaw)
        self.last_raw_yaw = raw_yaw
        self.samples.append(
            PoseSample(
                time_s=self.now_s(),
                x=float(message.pose.pose.position.x),
                y=float(message.pose.pose.position.y),
                yaw=self.unwrapped_yaw,
                angular_z=float(message.twist.twist.angular.z),
                linear_x=float(message.twist.twist.linear.x),
                observed_linear=self.observed_linear,
                observed_angular=self.observed_angular,
            )
        )

    def publish(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def spin_until(self, condition, *, label: str) -> None:
        deadline = time.monotonic() + self.wall_timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for {label}")
            rclpy.spin_once(self, timeout_sec=min(self.command_period, 0.02))

    def publish_for_sim_time(
        self,
        linear: float,
        angular: float,
        duration_s: float,
    ) -> None:
        started = self.now_s()
        deadline = time.monotonic() + self.wall_timeout
        while self.now_s() - started < duration_s:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "motion command did not complete in wall-time budget"
                )
            self.publish(linear, angular)
            rclpy.spin_once(self, timeout_sec=self.command_period)

    def run_probe(self) -> dict[str, object]:
        self.spin_until(
            lambda: self.now_s() > 0.0 and len(self.samples) >= 5,
            label="simulation clock and Ground Truth",
        )
        self.publish_for_sim_time(0.0, 0.0, 0.5)
        self.samples.clear()
        self.maximum_observed_linear = 0.0
        self.maximum_observed_command = 0.0
        self.publish_for_sim_time(
            self.command_linear,
            self.command_angular,
            self.command_duration,
        )
        command_samples = list(self.samples)
        for _ in range(8):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        if abs(self.command_linear) > 1.0e-9:
            return evaluate_arc(
                command_samples,
                command_linear=self.command_linear,
                command_angular=self.command_angular,
                duration_s=self.command_duration,
                maximum_observed_linear=self.maximum_observed_linear,
                maximum_observed_angular=self.maximum_observed_command,
            )
        return evaluate_spin(
            command_samples,
            command_angular=self.command_angular,
            duration_s=self.command_duration,
            maximum_observed_command=self.maximum_observed_command,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionResponseProbe()
    exit_code = 1
    try:
        result = node.run_probe()
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = node.result_path.with_suffix(node.result_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(node.result_path)
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)
        exit_code = 0 if result["status"] == "passed" else 1
    finally:
        for _ in range(5):
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
