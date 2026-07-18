"""Persist the live cuVSLAM map and its globally optimized pose history."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseStamped
from isaac_ros_visual_slam_interfaces.srv import FilePath, GetAllPoses
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class PoseRecord:
    timestamp_s: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str


def pose_record(message: PoseStamped) -> PoseRecord:
    stamp = message.header.stamp
    position = message.pose.position
    orientation = message.pose.orientation
    values = (
        stamp.sec + stamp.nanosec * 1.0e-9,
        float(position.x),
        float(position.y),
        float(position.z),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("cuVSLAM returned a non-finite optimized pose")
    quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
    if not 0.95 <= quaternion_norm <= 1.05:
        raise ValueError(f"invalid optimized-pose quaternion norm: {quaternion_norm}")
    return PoseRecord(*values, str(message.header.frame_id))


def trajectory_metrics(records: list[PoseRecord]) -> dict[str, object]:
    ordered = sorted(records, key=lambda item: item.timestamp_s)
    if not ordered:
        return {
            "pose_count": 0,
            "duration_s": 0.0,
            "planar_path_length_m": 0.0,
            "path_length_3d_m": 0.0,
            "path_length_3d_to_planar_ratio": math.inf,
            "closure_error_m": math.inf,
            "vertical_min_m": math.inf,
            "vertical_max_m": -math.inf,
            "vertical_range_m": math.inf,
            "vertical_stddev_m": math.inf,
            "frame_ids": [],
        }
    planar_path = sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(ordered, ordered[1:])
    )
    path_3d = sum(
        math.dist(
            (previous.x, previous.y, previous.z),
            (current.x, current.y, current.z),
        )
        for previous, current in zip(ordered, ordered[1:])
    )
    vertical = [item.z for item in ordered]
    return {
        "pose_count": len(ordered),
        "duration_s": ordered[-1].timestamp_s - ordered[0].timestamp_s,
        "first_timestamp_s": ordered[0].timestamp_s,
        "last_timestamp_s": ordered[-1].timestamp_s,
        "planar_path_length_m": planar_path,
        "path_length_3d_m": path_3d,
        "path_length_3d_to_planar_ratio": (
            path_3d / planar_path if planar_path > 0.0 else math.inf
        ),
        "closure_error_m": math.hypot(
            ordered[-1].x - ordered[0].x,
            ordered[-1].y - ordered[0].y,
        ),
        "vertical_min_m": min(vertical),
        "vertical_max_m": max(vertical),
        "vertical_range_m": max(vertical) - min(vertical),
        "vertical_stddev_m": statistics.pstdev(vertical),
        "frame_ids": sorted({item.frame_id for item in ordered}),
    }


def tum_line(record: PoseRecord) -> str:
    return (
        f"{record.timestamp_s:.9f} {record.x:.9f} {record.y:.9f} "
        f"{record.z:.9f} {record.qx:.9f} {record.qy:.9f} "
        f"{record.qz:.9f} {record.qw:.9f}"
    )


class VisualMapSaver(Node):
    def __init__(self) -> None:
        super().__init__("visual_map_saver")
        self.declare_parameter("output_dir", "data/maps/kujiale_jackal_8cam/cuvslam")
        self.declare_parameter("expected_path_length_m", 0.0)
        self.declare_parameter("minimum_path_ratio", 0.85)
        self.declare_parameter("maximum_path_ratio", 1.15)
        self.declare_parameter("minimum_pose_count", 100)
        self.declare_parameter("minimum_duration_s", 30.0)
        self.declare_parameter("maximum_closure_error_m", 0.50)
        self.declare_parameter("maximum_vertical_range_m", 0.10)
        self.declare_parameter("maximum_3d_to_planar_ratio", 1.01)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).resolve()

    def save_map(self, timeout_s: float = 120.0) -> bool:
        client = self.create_client(FilePath, "/visual_slam/save_map")
        if not client.wait_for_service(timeout_sec=20.0):
            return False
        request = FilePath.Request()
        request.file_path = str(self.output_dir)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return bool(
            future.done()
            and future.exception() is None
            and future.result() is not None
            and future.result().success
        )

    def get_optimized_poses(self, timeout_s: float = 120.0) -> list[PoseRecord]:
        client = self.create_client(GetAllPoses, "/visual_slam/get_all_poses")
        if not client.wait_for_service(timeout_sec=20.0):
            return []
        request = GetAllPoses.Request()
        request.max_count = 10000
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if (
            not future.done()
            or future.exception() is not None
            or future.result() is None
            or not future.result().success
        ):
            return []
        return sorted(
            (pose_record(message) for message in future.result().poses),
            key=lambda item: item.timestamp_s,
        )

    def save(self) -> dict[str, object]:
        if self.output_dir.exists():
            raise RuntimeError(f"cuVSLAM output already exists: {self.output_dir}")
        self.output_dir.parent.mkdir(parents=True, exist_ok=True)
        map_saved = self.save_map()
        records = self.get_optimized_poses() if map_saved else []
        metrics = trajectory_metrics(records)
        expected_path = float(self.get_parameter("expected_path_length_m").value)
        path_ratio = (
            float(metrics["planar_path_length_m"]) / expected_path
            if expected_path > 0.0
            else None
        )
        checks = {
            "map_service_succeeded": map_saved,
            "database_nonempty": (self.output_dir / "data.mdb").is_file()
            and (self.output_dir / "data.mdb").stat().st_size > 0,
            "optimized_pose_count": int(metrics["pose_count"])
            >= int(self.get_parameter("minimum_pose_count").value),
            "trajectory_duration": float(metrics["duration_s"])
            >= float(self.get_parameter("minimum_duration_s").value),
            # Isaac ROS 4.5 GetAllPoses documents these as globally optimized
            # SLAM poses but leaves PoseStamped.header.frame_id empty.  Accept
            # that version-specific encoding as well as an explicit map frame.
            "map_frame_semantics": metrics["frame_ids"] in ([""], ["map"]),
            "closed_loop": float(metrics["closure_error_m"])
            <= float(self.get_parameter("maximum_closure_error_m").value),
            "planar_vertical_range": float(metrics["vertical_range_m"])
            <= float(self.get_parameter("maximum_vertical_range_m").value),
            "planar_path_geometry": float(metrics["path_length_3d_to_planar_ratio"])
            <= float(self.get_parameter("maximum_3d_to_planar_ratio").value),
            "expected_path_length": path_ratio is None
            or (
                float(self.get_parameter("minimum_path_ratio").value)
                <= path_ratio
                <= float(self.get_parameter("maximum_path_ratio").value)
            ),
        }
        if records:
            (self.output_dir / "optimized_poses.tum").write_text(
                "\n".join(tum_line(item) for item in records) + "\n",
                encoding="utf-8",
            )
        result = {
            "schema_version": 1,
            "status": "passed" if all(checks.values()) else "failed",
            "created_unix_s": time.time(),
            "checks": checks,
            "failure_reasons": [name for name, passed in checks.items() if not passed],
            "expected_path_length_m": expected_path if expected_path > 0.0 else None,
            "path_length_ratio": path_ratio,
            "pose_frame_policy": (
                "isaac_ros_4_5_get_all_poses_empty_frame_is_global_slam_map"
                if metrics["frame_ids"] == [""]
                else "explicit_map_frame"
            ),
            "trajectory": metrics,
            "output_dir": str(self.output_dir),
        }
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "save_report.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualMapSaver()
    try:
        report = node.save()
        if report["status"] != "passed":
            raise RuntimeError(f"cuVSLAM map quality gate failed: {report['failure_reasons']}")
        node.get_logger().info(f"Saved qualified cuVSLAM map to {node.output_dir}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
