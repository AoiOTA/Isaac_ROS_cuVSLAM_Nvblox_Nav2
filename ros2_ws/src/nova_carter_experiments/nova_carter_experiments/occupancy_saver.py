"""Save the latest nvblox ESDF slice as a Nav2 PGM/YAML occupancy map."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from nvblox_msgs.msg import DistanceMapSlice
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OccupancySaver(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_saver")
        self.declare_parameter("output_dir", "data/maps/warehouse_v1/occupancy")
        self.declare_parameter("obstacle_distance_m", 0.28)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).resolve()
        self.obstacle_distance = float(self.get_parameter("obstacle_distance_m").value)
        self.latest: DistanceMapSlice | None = None
        self.create_subscription(
            DistanceMapSlice,
            "/nvblox_node/static_map_slice",
            self.on_slice,
            qos_profile_sensor_data,
        )

    def on_slice(self, message: DistanceMapSlice) -> None:
        if message.width and message.height and len(message.data) == message.width * message.height:
            self.latest = message

    def wait_and_save(self, timeout_s: float = 60.0) -> dict[str, object]:
        deadline = time.monotonic() + timeout_s
        while self.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError("no non-empty /nvblox_node/static_map_slice received")
        message = self.latest
        self.output_dir.mkdir(parents=True, exist_ok=True)
        pgm = self.output_dir / "map.pgm"
        yaml_path = self.output_dir / "map.yaml"
        pixels = bytearray()
        counts = {"occupied": 0, "free": 0, "unknown": 0}
        # PGM rows run top-to-bottom while OccupancyGrid/map_server uses a lower-left origin.
        for row in range(message.height - 1, -1, -1):
            start = row * message.width
            for value in message.data[start : start + message.width]:
                unknown = not math.isfinite(value) or abs(value - message.unknown_value) < 1.0e-5
                if unknown:
                    pixels.append(205)
                    counts["unknown"] += 1
                elif value <= self.obstacle_distance:
                    pixels.append(0)
                    counts["occupied"] += 1
                else:
                    pixels.append(254)
                    counts["free"] += 1
        pgm.write_bytes(
            f"P5\n# nvblox ESDF slice\n{message.width} {message.height}\n255\n".encode()
            + pixels
        )
        yaml_path.write_text(
            "image: map.pgm\n"
            "mode: trinary\n"
            f"resolution: {message.resolution:.9g}\n"
            f"origin: [{message.origin.x:.9g}, {message.origin.y:.9g}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        report = {
            "status": "passed" if counts["occupied"] and counts["free"] else "failed",
            "frame_id": message.header.frame_id,
            "width": message.width,
            "height": message.height,
            "resolution": message.resolution,
            "origin": [message.origin.x, message.origin.y, message.origin.z],
            "obstacle_distance_m": self.obstacle_distance,
            "counts": counts,
            "pgm": str(pgm),
            "yaml": str(yaml_path),
        }
        (self.output_dir / "save_report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        if report["status"] != "passed":
            raise RuntimeError(f"occupancy map lacks occupied/free cells: {counts}")
        return report


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OccupancySaver()
    try:
        report = node.wait_and_save()
        node.get_logger().info(f"Saved occupancy map: {report}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
