"""Persist the running nvblox map, mesh, rates, and timings without GUI interaction."""

from __future__ import annotations

import json
from pathlib import Path
import time

from nvblox_msgs.srv import FilePath
import rclpy
from rclpy.node import Node


class NvbloxMapSaver(Node):
    def __init__(self) -> None:
        super().__init__("nvblox_map_saver")
        self.declare_parameter("output_dir", "data/maps/warehouse_v1/nvblox")
        self.declare_parameter("stem", "warehouse")
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).resolve()
        self.stem = str(self.get_parameter("stem").value)

    def call(self, service: str, path: Path, timeout_s: float = 90.0) -> bool:
        client = self.create_client(FilePath, service)
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f"Service unavailable: {service}")
            return False
        request = FilePath.Request()
        request.file_path = str(path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return (
            future.done()
            and future.exception() is None
            and future.result() is not None
            and bool(future.result().success)
        )

    def save(self) -> dict[str, object]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        targets = {
            "map": self.output_dir / f"{self.stem}.nvblx",
            "mesh": self.output_dir / f"{self.stem}.ply",
            "rates": self.output_dir / "nvblox_rates.txt",
            "timings": self.output_dir / "nvblox_timings.txt",
        }
        services = {
            "map": "/nvblox_node/save_map",
            "mesh": "/nvblox_node/save_ply",
            "rates": "/nvblox_node/save_rates",
            "timings": "/nvblox_node/save_timings",
        }
        success = {name: self.call(services[name], path) for name, path in targets.items()}
        files = {
            name: {
                "path": str(path),
                "exists": path.is_file(),
                "size_bytes": path.stat().st_size if path.is_file() else 0,
            }
            for name, path in targets.items()
        }
        report = {
            "status": "passed"
            if all(success.values()) and all(item["size_bytes"] > 0 for item in files.values())
            else "failed",
            "created_unix_s": time.time(),
            "services": success,
            "files": files,
        }
        report_path = self.output_dir / "save_report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return report


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NvbloxMapSaver()
    try:
        report = node.save()
        if report["status"] != "passed":
            raise RuntimeError(f"nvblox save failed: {report}")
        node.get_logger().info(f"Saved nvblox artifacts to {node.output_dir}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
