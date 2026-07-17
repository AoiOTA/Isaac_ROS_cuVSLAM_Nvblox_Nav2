#!/usr/bin/env python3
"""Gate all four stereo pairs and measure their real ROS timestamp alignment."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


CAMERAS = (
    "front_left",
    "front_right",
    "left_left",
    "left_right",
    "right_left",
    "right_right",
    "back_left",
    "back_right",
)


def stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SurroundSyncProbe(Node):
    def __init__(self) -> None:
        super().__init__("surround_sync_probe")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gate = self.create_publisher(Bool, "/vgl/cameras_enabled", latched)
        self.images: defaultdict[str, list[int]] = defaultdict(list)
        self.infos: defaultdict[str, list[int]] = defaultdict(list)
        self.last_surround_message_wall = 0.0
        for label in CAMERAS:
            camera, side = label.split("_", 1)
            self.create_subscription(
                Image,
                f"/{camera}_stereo_camera/{side}/image_raw",
                lambda msg, name=label: self.on_message(self.images, name, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                f"/{camera}_stereo_camera/{side}/camera_info",
                lambda msg, name=label: self.on_message(self.infos, name, msg),
                qos_profile_sensor_data,
            )

    def on_message(
        self, destination: defaultdict[str, list[int]], label: str, message: object
    ) -> None:
        destination[label].append(stamp_ns(message))
        if len(destination[label]) > 1000:
            del destination[label][:-1000]
        if not label.startswith("front_"):
            self.last_surround_message_wall = time.monotonic()


def synchronization(stamps: dict[str, list[int]]) -> dict[str, object]:
    counts = {name: len(stamps.get(name, [])) for name in CAMERAS}
    if any(value == 0 for value in counts.values()):
        return {
            "counts": counts,
            "exact_common_stamps": 0,
            "minimum_spread_ms": None,
        }
    sets = [set(stamps[name]) for name in CAMERAS]
    exact = set.intersection(*sets)
    spreads_ns = []
    for reference in stamps[CAMERAS[0]]:
        nearest = [reference]
        for name in CAMERAS[1:]:
            nearest.append(min(stamps[name], key=lambda value: abs(value - reference)))
        spread = max(nearest) - min(nearest)
        spreads_ns.append(spread)
    spreads_ns.sort()
    percentile_index = min(len(spreads_ns) - 1, math.ceil(0.95 * len(spreads_ns)) - 1)
    return {
        "counts": counts,
        "exact_common_stamps": len(exact),
        "minimum_spread_ms": spreads_ns[0] * 1.0e-6,
        "p95_nearest_spread_ms": spreads_ns[percentile_index] * 1.0e-6,
        "reference_frames_within_threshold": {
            f"{threshold}_ms": sum(value <= threshold * 1_000_000 for value in spreads_ns)
            for threshold in (5, 10, 20, 40, 100)
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--output", type=Path, default=None)
    args, ros_args = parser.parse_known_args()
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    rclpy.init(args=ros_args)
    node = SurroundSyncProbe()
    try:
        node.gate.publish(Bool(data=True))
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.gate.publish(Bool(data=False))
        disabled_wall = time.monotonic()
        while time.monotonic() - disabled_wall < 3.0:
            # Mirror the recovery manager's latched heartbeat while allowing
            # already-negotiated GPU/DDS queues to drain.
            node.gate.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        image_sync = synchronization(node.images)
        info_sync = synchronization(node.infos)
        checks = {
            "all_image_streams_present": min(image_sync["counts"].values()) > 0,
            "all_camera_info_streams_present": min(info_sync["counts"].values()) > 0,
            "eight_images_have_common_stamp": image_sync["exact_common_stamps"] > 0,
            "eight_infos_have_common_stamp": info_sync["exact_common_stamps"] > 0,
            "publishers_stop_after_gate": time.monotonic()
            - node.last_surround_message_wall
            >= 0.5,
        }
        report = {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "images": image_sync,
            "camera_info": info_sync,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        return 0 if report["status"] == "passed" else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
