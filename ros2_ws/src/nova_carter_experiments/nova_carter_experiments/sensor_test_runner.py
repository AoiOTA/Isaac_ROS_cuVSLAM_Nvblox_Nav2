"""End-to-end Phase 4 visual sensor, timing, TF, and motion data-flow test."""

from __future__ import annotations

import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from tf2_msgs.msg import TFMessage


TOPICS = {
    "left_rgb": "/front_stereo_camera/left/image_raw_rgb",
    "right_rgb": "/front_stereo_camera/right/image_raw_rgb",
    "left_mono": "/front_stereo_camera/left/image_raw",
    "right_mono": "/front_stereo_camera/right/image_raw",
    "left_info": "/front_stereo_camera/left/camera_info",
    "right_info": "/front_stereo_camera/right/camera_info",
    "depth": "/front_stereo_camera/depth/image_raw",
    "depth_info": "/front_stereo_camera/depth/camera_info",
    "imu": "/front_stereo_imu/imu",
}


def stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def observed_rate(stamps: list[int]) -> float:
    unique = sorted(set(stamps))
    differences = [(right - left) * 1.0e-9 for left, right in zip(unique, unique[1:])]
    positive = [value for value in differences if value > 1.0e-6]
    return 1.0 / statistics.median(positive) if positive else 0.0


class SensorTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("phase4_sensor_test_runner")
        self.declare_parameter("result_path", "data/reports/phase4/latest.json")
        self.declare_parameter("payload_probe_path", "data/reports/phase4/payload.json")
        self.declare_parameter("test_duration_sim_seconds", 15.0)
        self._started_wall = time.monotonic()
        self._first_clock: float | None = None
        self._latest_clock = 0.0
        self._last_clock = -math.inf
        self._clock_stamps: list[int] = []
        self._clock_regressions = 0
        self._stamps = {key: [] for key in TOPICS}
        self._regressions = {key: 0 for key in TOPICS}
        probe_path = Path(str(self.get_parameter("payload_probe_path").value)).resolve()
        self._payload_probe = json.loads(probe_path.read_text(encoding="utf-8"))
        if self._payload_probe.get("status") != "passed":
            raise RuntimeError(f"sensor payload probe did not pass: {probe_path}")
        self._metadata: dict[str, dict[str, object]] = {
            key: value.copy() for key, value in self._payload_probe["metadata"].items()
        }
        self._image_samples: dict[str, list[float]] = {
            key: [] for key in ("left_rgb", "right_rgb", "left_mono", "right_mono")
        }
        for key in ("left_rgb", "right_rgb"):
            self._image_samples[key].append(self._payload_probe["image_sample_stddev"][key])
        self._depth_samples: list[dict[str, float]] = [self._payload_probe["depth_sample"]]
        self._imu_samples: list[dict[str, float]] = []
        self._joint_messages = 0
        self._tf_edges: set[tuple[str, str]] = set()
        self._tf_static_edges: set[tuple[str, str]] = set()
        self._commands_by_phase: dict[str, int] = {}
        self._done = False
        self._finalizing = False
        self._failure: str | None = None
        callbacks = ReentrantCallbackGroup()
        self._publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos, callback_group=callbacks
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            qos_profile_sensor_data,
            callback_group=callbacks,
        )
        for key in ("left_mono", "right_mono"):
            self.create_subscription(
                Image,
                TOPICS[key],
                lambda message, name=key: self._image_callback(name, message),
                qos_profile_sensor_data,
                callback_group=callbacks,
            )
        for key in ("left_info", "right_info", "depth_info"):
            self.create_subscription(
                CameraInfo,
                TOPICS[key],
                lambda message, name=key: self._info_callback(name, message),
                qos_profile_sensor_data,
                callback_group=callbacks,
            )
        self.create_subscription(
            Imu,
            TOPICS["imu"],
            self._imu_callback,
            qos_profile_sensor_data,
            callback_group=callbacks,
        )
        tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/tf", self._tf_callback, tf_qos, callback_group=callbacks
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            self._tf_static_callback,
            tf_static_qos,
            callback_group=callbacks,
        )
        self.create_timer(0.02, self._control_tick, callback_group=callbacks)

    def _record_stamp(self, key: str, message: object) -> None:
        value = stamp_ns(message)
        values = self._stamps[key]
        if values and value < values[-1]:
            self._regressions[key] += 1
        values.append(value)

    def _clock_callback(self, message: Clock) -> None:
        now = message.clock.sec + message.clock.nanosec * 1.0e-9
        if now < self._last_clock:
            self._clock_regressions += 1
        self._last_clock = now
        self._latest_clock = now
        self._clock_stamps.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)
        if self._first_clock is None:
            self._first_clock = now

    def _joint_callback(self, _message: JointState) -> None:
        self._joint_messages += 1

    def _image_callback(self, key: str, message: Image) -> None:
        self._record_stamp(key, message)
        self._metadata[key] = {
            "width": message.width,
            "height": message.height,
            "encoding": message.encoding,
            "frame_id": message.header.frame_id,
            "step": message.step,
        }
        if key == "depth":
            if len(self._stamps[key]) % 10 == 1 and message.encoding.upper() == "32FC1":
                rows = np.frombuffer(message.data, dtype=np.float32).reshape(
                    message.height, message.step // 4
                )
                values = rows[:, : message.width][::8, ::8]
                valid = values[np.isfinite(values) & (values > 0.0)]
                self._depth_samples.append(
                    {
                        "valid_fraction": float(valid.size / values.size),
                        "minimum_m": float(valid.min()) if valid.size else math.nan,
                        "maximum_m": float(valid.max()) if valid.size else math.nan,
                    }
                )
            return
        if len(self._stamps[key]) % 15 == 1:
            channels = 1 if message.encoding.lower() == "mono8" else 3
            rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
            sample = rows[::16, : message.width * channels : 16]
            self._image_samples[key].append(float(sample.std()))

    def _info_callback(self, key: str, message: CameraInfo) -> None:
        self._record_stamp(key, message)
        self._metadata[key] = {
            "width": message.width,
            "height": message.height,
            "frame_id": message.header.frame_id,
            "distortion_model": message.distortion_model,
            "fx": message.k[0],
            "fy": message.k[4],
            "cx": message.k[2],
            "cy": message.k[5],
            "projection_tx": message.p[3],
        }

    def _imu_callback(self, message: Imu) -> None:
        self._record_stamp("imu", message)
        self._metadata["imu"] = {"frame_id": message.header.frame_id}
        if len(self._stamps["imu"]) % 20 == 1:
            q = message.orientation
            a = message.linear_acceleration
            w = message.angular_velocity
            self._imu_samples.append(
                {
                    "orientation_norm": math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w),
                    "acceleration_norm": math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
                    "angular_speed": math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z),
                }
            )

    def _tf_callback(self, message: TFMessage) -> None:
        self._tf_edges.update((item.header.frame_id, item.child_frame_id) for item in message.transforms)

    def _tf_static_callback(self, message: TFMessage) -> None:
        self._tf_static_edges.update(
            (item.header.frame_id, item.child_frame_id) for item in message.transforms
        )

    def _control_tick(self) -> None:
        if self._done or self._first_clock is None:
            return
        elapsed = self._latest_clock - self._first_clock
        command = Twist()
        if elapsed < 3.0:
            phase = "stationary"
        elif elapsed < 7.0:
            phase = "arc_left"
            command.linear.x, command.angular.z = 0.35, 0.45
        elif elapsed < 11.0:
            phase = "arc_right"
            command.linear.x, command.angular.z = 0.35, -0.55
        elif elapsed < 13.0:
            phase = "spin"
            command.angular.z = 0.65
        else:
            phase = "settle"
        self._publisher.publish(command)
        self._commands_by_phase[phase] = self._commands_by_phase.get(phase, 0) + 1
        if elapsed >= float(self.get_parameter("test_duration_sim_seconds").value):
            self._finalize(elapsed)

    @staticmethod
    def _pair_fraction(left: list[int], right: list[int]) -> float:
        left_set, right_set = set(left), set(right)
        denominator = max(1, min(len(left_set), len(right_set)))
        return len(left_set & right_set) / denominator

    @staticmethod
    def _reference_fraction(samples: list[int], reference: list[int]) -> float:
        sample_set, reference_set = set(samples), set(reference)
        return len(sample_set & reference_set) / max(1, len(sample_set))

    def _finalize(self, elapsed: float) -> None:
        if self._done or self._finalizing:
            return
        self._finalizing = True
        zero = Twist()
        for _ in range(5):
            self._publisher.publish(zero)
        rates = {key: observed_rate(value) for key, value in self._stamps.items()}
        rates["clock"] = observed_rate(self._clock_stamps)
        rates["left_rgb"] = rates["left_info"]
        rates["right_rgb"] = rates["right_info"]
        rates["depth"] = rates["depth_info"]
        pair_fractions = {
            "stereo_camera_info": self._pair_fraction(
                self._stamps["left_info"], self._stamps["right_info"]
            ),
            "left_mono_to_info": self._reference_fraction(
                self._stamps["left_mono"], self._stamps["left_info"]
            ),
            "right_mono_to_info": self._reference_fraction(
                self._stamps["right_mono"], self._stamps["right_info"]
            ),
        }
        required_static = {
            ("base_link", "front_stereo_camera_link"),
            ("front_stereo_camera_link", "front_stereo_camera_left_optical"),
            ("front_stereo_camera_link", "front_stereo_camera_right_optical"),
            ("front_stereo_camera_link", "front_stereo_camera_imu"),
        }
        required_dynamic_children = {
            "wheel_left",
            "wheel_right",
            "caster_frame_base",
            "caster_swing_left",
            "caster_swing_right",
            "caster_wheel_left",
            "caster_wheel_right",
        }
        dynamic_children = {child for _, child in self._tf_edges}
        depth_valid = [sample["valid_fraction"] for sample in self._depth_samples]
        orientation_norms = [sample["orientation_norm"] for sample in self._imu_samples]
        checks = {
            "clock_120hz": 110.0 <= rates["clock"] <= 130.0,
            "stereo_30hz": all(
                27.0 <= rates[key] <= 33.0 for key in ("left_info", "right_info")
            ),
            # SensorData is intentionally Best-Effort, so this Python audit subscriber
            # samples the large mono payload while the render cadence is established by
            # the paired CameraInfo stream. Every sampled image must retain that cadence.
            "mono_sampled_flow": all(
                len(self._stamps[key]) >= 100 for key in ("left_mono", "right_mono")
            )
            and all(
                self.count_publishers(TOPICS[key]) == 1
                for key in ("left_mono", "right_mono")
            ),
            "depth_30hz": 27.0 <= rates["depth_info"] <= 33.0,
            "imu_120hz": 110.0 <= rates["imu"] <= 130.0,
            "enough_messages": all(
                len(self._stamps[key]) >= 200
                for key in ("left_info", "right_info", "depth_info", "imu")
            ),
            "timestamps_monotonic": not any(self._regressions.values()) and self._clock_regressions == 0,
            "synchronized": all(value >= 0.95 for value in pair_fractions.values()),
            "image_metadata": self._metadata.get("left_mono", {}).get("encoding") == "mono8"
            and self._metadata.get("right_mono", {}).get("encoding") == "mono8"
            and self._metadata.get("left_mono", {}).get("width") == 1280
            and self._metadata.get("left_mono", {}).get("height") == 800,
            "depth_metadata": self._metadata.get("depth", {}).get("encoding", "").upper()
            == "32FC1"
            and self._metadata.get("depth", {}).get("width") == 640
            and self._metadata.get("depth", {}).get("height") == 400,
            "camera_calibration": all(
                self._metadata.get(key, {}).get("fx", 0.0) > 0.0
                for key in ("left_info", "right_info", "depth_info")
            )
            and abs(
                -self._metadata["right_info"]["projection_tx"]
                / self._metadata["right_info"]["fx"]
                - 0.15
            )
            <= 0.01,
            "frame_ids": self._metadata.get("left_mono", {}).get("frame_id")
            == "front_stereo_camera_left_optical"
            and self._metadata.get("right_mono", {}).get("frame_id")
            == "front_stereo_camera_right_optical"
            and self._metadata.get("depth", {}).get("frame_id")
            == "front_stereo_camera_left_optical"
            and self._metadata.get("imu", {}).get("frame_id") == "front_stereo_camera_imu",
            "images_nonconstant": all(
                samples and statistics.median(samples) > 2.0
                for samples in self._image_samples.values()
            ),
            "depth_valid": bool(depth_valid) and statistics.median(depth_valid) >= 0.50,
            "imu_finite_normalized": bool(orientation_norms)
            and all(math.isfinite(value) and 0.95 <= value <= 1.05 for value in orientation_norms),
            "imu_dynamics_finite": all(
                math.isfinite(sample["acceleration_norm"])
                and math.isfinite(sample["angular_speed"])
                for sample in self._imu_samples
            ),
            "static_tf_complete": required_static <= self._tf_static_edges,
            "dynamic_tf_complete": required_dynamic_children <= dynamic_children,
            "joint_states_flowing": self._joint_messages >= 200,
            "motion_phases_exercised": all(
                self._commands_by_phase.get(name, 0) > 5
                for name in ("stationary", "arc_left", "arc_right", "spin", "settle")
            ),
            "tf_publisher_counts": self.count_publishers("/tf") == 1
            and self.count_publishers("/tf_static") == 1,
        }
        checks = {name: bool(passed) for name, passed in checks.items()}
        report = {
            "status": "passed" if all(checks.values()) else "failed",
            "elapsed_sim_seconds": elapsed,
            "elapsed_wall_seconds": time.monotonic() - self._started_wall,
            "checks": checks,
            "rates_hz": rates,
            "message_counts": {key: len(value) for key, value in self._stamps.items()},
            "clock_message_count": len(self._clock_stamps),
            "joint_message_count": self._joint_messages,
            "timestamp_regressions": {**self._regressions, "clock": self._clock_regressions},
            "pair_fractions": pair_fractions,
            "metadata": self._metadata,
            "image_sample_stddev": self._image_samples,
            "depth_samples": self._depth_samples,
            "imu_samples": self._imu_samples,
            "tf_edges": sorted([list(edge) for edge in self._tf_edges]),
            "tf_static_edges": sorted([list(edge) for edge in self._tf_static_edges]),
            "commands_by_phase": self._commands_by_phase,
            "publisher_counts": {
                "tf": self.count_publishers("/tf"),
                "tf_static": self.count_publishers("/tf_static"),
            },
            "payload_probe": self._payload_probe,
            "rate_basis": {
                "left_rgb": "paired left CameraInfo render cadence",
                "right_rgb": "paired right CameraInfo render cadence",
                "depth": "paired depth CameraInfo render cadence",
                "mono": "sampled Best-Effort payload delivery; stamps checked against CameraInfo",
            },
        }
        path = Path(str(self.get_parameter("result_path").value)).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            self._failure = f"Phase 4 checks failed: {', '.join(failed)}"
            self.get_logger().error(self._failure)
        else:
            self.get_logger().info(f"Phase 4 sensor checks passed: {path}")
        self._done = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SensorTestRunner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    wall_deadline = time.monotonic() + 240.0
    try:
        while rclpy.ok() and not node._done and time.monotonic() < wall_deadline:
            executor.spin_once(timeout_sec=0.1)
        if not node._done:
            raise RuntimeError("Phase 4 sensor test exceeded 240 wall seconds")
        if node._failure:
            raise RuntimeError(node._failure)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
