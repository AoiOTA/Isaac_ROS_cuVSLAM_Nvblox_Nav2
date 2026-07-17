#!/usr/bin/env python3
"""Subscribe to the three Phase 1 bridge topics and validate actual messages."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


class BridgeProbe(Node):
    def __init__(self) -> None:
        super().__init__("stage1_ros_bridge_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock: Clock | None = None
        self.image: Image | None = None
        self.info: CameraInfo | None = None
        self.create_subscription(Clock, "/clock", self._clock_callback, qos)
        self.create_subscription(Image, "/stage1/camera/image_raw", self._image_callback, qos)
        self.create_subscription(CameraInfo, "/stage1/camera/camera_info", self._info_callback, qos)

    def _clock_callback(self, message: Clock) -> None:
        self.clock = message

    def _image_callback(self, message: Image) -> None:
        self.image = message

    def _info_callback(self, message: CameraInfo) -> None:
        self.info = message

    def complete(self) -> bool:
        return self.clock is not None and self.image is not None and self.info is not None


def stamp_ns(message: Image | CameraInfo) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=75.0)
    args = parser.parse_args()

    rclpy.init()
    node = BridgeProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.complete() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.complete():
            missing = [name for name in ("clock", "image", "info") if getattr(node, name) is None]
            raise RuntimeError(f"Timed out waiting for: {', '.join(missing)}")

        assert node.clock is not None and node.image is not None and node.info is not None
        failures = []
        if node.image.width != 320 or node.image.height != 240:
            failures.append(f"unexpected image size {node.image.width}x{node.image.height}")
        if node.image.encoding not in {"rgb8", "rgba8"}:
            failures.append(f"unexpected image encoding {node.image.encoding}")
        if (node.info.width, node.info.height) != (node.image.width, node.image.height):
            failures.append("CameraInfo dimensions do not match Image")
        if node.image.header.frame_id != "stage1_camera_optical":
            failures.append(f"unexpected image frame {node.image.header.frame_id}")
        if node.info.header.frame_id != node.image.header.frame_id:
            failures.append("CameraInfo and Image frame IDs differ")
        clock_ns = node.clock.clock.sec * 1_000_000_000 + node.clock.clock.nanosec
        if clock_ns <= 0:
            failures.append("simulation clock did not advance")

        report = {
            "ok": not failures,
            "clock_ns": clock_ns,
            "image": {
                "width": node.image.width,
                "height": node.image.height,
                "encoding": node.image.encoding,
                "frame_id": node.image.header.frame_id,
                "stamp_ns": stamp_ns(node.image),
            },
            "camera_info": {
                "width": node.info.width,
                "height": node.info.height,
                "frame_id": node.info.header.frame_id,
                "stamp_ns": stamp_ns(node.info),
            },
            "failures": failures,
        }
        print(json.dumps(report, indent=2))
        return 0 if not failures else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
