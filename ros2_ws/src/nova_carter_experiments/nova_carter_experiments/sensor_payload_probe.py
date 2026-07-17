"""Capture one synchronized stereo/depth payload before the sustained stream audit."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


TOPICS = {
    "left_rgb": "/front_stereo_camera/left/image_raw_rgb",
    "right_rgb": "/front_stereo_camera/right/image_raw_rgb",
    "depth": "/front_stereo_camera/depth/image_raw",
}


class PayloadProbe(Node):
    def __init__(self) -> None:
        super().__init__("phase4_sensor_payload_probe")
        self.declare_parameter("result_path", "data/reports/phase4/payload.json")
        self.messages: dict[str, dict[int, Image]] = {key: {} for key in TOPICS}
        self._probe_subscriptions = []
        for key, topic in TOPICS.items():
            self._probe_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda message, name=key: self._record(name, message),
                    qos_profile_sensor_data,
                )
            )

    def _record(self, key: str, message: Image) -> None:
        samples = self.messages[key]
        samples[self._stamp_ns(message)] = message
        while len(samples) > 5:
            samples.pop(min(samples))

    def ready(self) -> bool:
        stereo = set(self.messages["left_rgb"]) & set(self.messages["right_rgb"])
        return bool(stereo) and bool(self.messages["depth"])

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def write_result(self) -> Path:
        metadata: dict[str, dict[str, object]] = {}
        image_stddev: dict[str, float] = {}
        for key in ("left_rgb", "right_rgb"):
            common = set(self.messages["left_rgb"]) & set(self.messages["right_rgb"])
            message = self.messages[key][max(common)]
            rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
            sample = rows[::16, : message.width * 3 : 16]
            image_stddev[key] = float(sample.std())
            metadata[key] = {
                "width": message.width,
                "height": message.height,
                "encoding": message.encoding,
                "frame_id": message.header.frame_id,
                "step": message.step,
                "stamp_ns": self._stamp_ns(message),
            }
        depth = self.messages["depth"][max(self.messages["depth"])]
        rows = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.step // 4)
        values = rows[:, : depth.width][::8, ::8]
        valid = values[np.isfinite(values) & (values > 0.0)]
        depth_sample = {
            "valid_fraction": float(valid.size / values.size),
            "minimum_m": float(valid.min()) if valid.size else math.nan,
            "maximum_m": float(valid.max()) if valid.size else math.nan,
        }
        metadata["depth"] = {
            "width": depth.width,
            "height": depth.height,
            "encoding": depth.encoding,
            "frame_id": depth.header.frame_id,
            "step": depth.step,
            "stamp_ns": self._stamp_ns(depth),
        }
        checks = {
            "stereo_payload_shape": all(
                metadata[key]["width"] == 1280
                and metadata[key]["height"] == 800
                and str(metadata[key]["encoding"]).lower() == "rgb8"
                for key in ("left_rgb", "right_rgb")
            ),
            "stereo_payload_synchronized": metadata["left_rgb"]["stamp_ns"]
            == metadata["right_rgb"]["stamp_ns"],
            "stereo_payload_nonconstant": all(value > 2.0 for value in image_stddev.values()),
            "depth_payload_shape": metadata["depth"]["width"] == 640
            and metadata["depth"]["height"] == 400
            and str(metadata["depth"]["encoding"]).upper() == "32FC1",
            "depth_payload_valid": depth_sample["valid_fraction"] >= 0.50,
        }
        report = {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "metadata": metadata,
            "image_sample_stddev": image_stddev,
            "depth_sample": depth_sample,
        }
        path = Path(str(self.get_parameter("result_path").value)).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PayloadProbe()
    deadline = time.monotonic() + 30.0
    try:
        while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            received = {key: len(value) for key, value in node.messages.items()}
            raise RuntimeError(f"sensor payload probe timed out: received={received}")
        path = node.write_result()
        result = json.loads(path.read_text(encoding="utf-8"))
        if result["status"] != "passed":
            raise RuntimeError(f"sensor payload checks failed: {result['checks']}")
        node.get_logger().info(f"Sensor payload probe passed: {path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
