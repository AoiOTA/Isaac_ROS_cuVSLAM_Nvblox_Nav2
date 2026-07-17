"""Exercise the Phase 6 depth-to-TSDF/ESDF/mesh pipeline and persistence APIs."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice, Mesh, VoxelBlockLayer
from nvblox_msgs.srv import FilePath
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, PointCloud2


class NvbloxTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("phase6_nvblox_test_runner")
        self.declare_parameter("result_path", "data/reports/phase6/latest.json")
        self.declare_parameter("output_dir", "data/maps/phase6_validation/nvblox")
        self.declare_parameter("mapping_duration_sim_seconds", 60.0)
        self.result_path = Path(str(self.get_parameter("result_path").value)).resolve()
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).resolve()
        self.mapping_duration = float(
            self.get_parameter("mapping_duration_sim_seconds").value
        )

        self.clock_time = 0.0
        self.last_clock = -math.inf
        self.clock_regressions = 0
        self.depth_info_count = 0
        self.color_info_count = 0
        self.statuses: list[tuple[float, int]] = []
        self.ground_truth: list[tuple[float, float, float]] = []
        self.command_phases: dict[str, int] = {}
        self.frames: dict[str, set[str]] = {
            "tsdf": set(),
            "mesh": set(),
            "esdf": set(),
            "slice": set(),
        }
        self.output_counts = {key: 0 for key in self.frames}
        self.tsdf_blocks: set[tuple[int, int, int]] = set()
        self.tsdf_max_centers = 0
        self.mesh_blocks: set[tuple[int, int, int]] = set()
        self.mesh_max_vertices = 0
        self.mesh_max_triangles = 0
        self.esdf_max_points = 0
        self.slice_max_width = 0
        self.slice_max_height = 0
        self.slice_max_known_cells = 0
        self.slice_min_distance = math.inf
        self.slice_max_distance = -math.inf

        command_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        clock_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Reconstruction messages can be very large. A latest-only best-effort
        # observer must never back-pressure the production mapping pipeline.
        output_qos = qos_profile_sensor_data
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_safe", command_qos)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(
            CameraInfo,
            "/front_stereo_camera/depth/camera_info",
            self.on_depth_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/front_stereo_camera/left/camera_info",
            self.on_color_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(VisualSlamStatus, "/visual_slam/status", self.on_status, 20)
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.tsdf_subscription = self.create_subscription(
            VoxelBlockLayer, "/nvblox_node/tsdf_layer", self.on_tsdf, output_qos
        )
        self.mesh_subscription = self.create_subscription(
            Mesh, "/nvblox_node/mesh", self.on_mesh, output_qos
        )
        self.esdf_subscription = self.create_subscription(
            PointCloud2,
            "/nvblox_node/static_esdf_pointcloud",
            self.on_esdf,
            output_qos,
        )
        self.slice_subscription = self.create_subscription(
            DistanceMapSlice,
            "/nvblox_node/static_map_slice",
            self.on_slice,
            output_qos,
        )
        self.save_map_client = self.create_client(FilePath, "/nvblox_node/save_map")
        self.save_ply_client = self.create_client(FilePath, "/nvblox_node/save_ply")
        self.save_rates_client = self.create_client(FilePath, "/nvblox_node/save_rates")
        self.save_timings_client = self.create_client(FilePath, "/nvblox_node/save_timings")

    @staticmethod
    def stamp_s(message: object) -> float:
        stamp = message.header.stamp
        return stamp.sec + stamp.nanosec * 1.0e-9

    def on_clock(self, message: Clock) -> None:
        current = message.clock.sec + message.clock.nanosec * 1.0e-9
        if current + 1.0e-9 < self.last_clock:
            self.clock_regressions += 1
        self.last_clock = current
        self.clock_time = current

    def on_depth_info(self, _message: CameraInfo) -> None:
        self.depth_info_count += 1

    def on_color_info(self, _message: CameraInfo) -> None:
        self.color_info_count += 1

    def on_status(self, message: VisualSlamStatus) -> None:
        self.statuses.append((self.stamp_s(message), int(message.vo_state)))

    def on_ground_truth(self, message: Odometry) -> None:
        self.ground_truth.append(
            (
                self.stamp_s(message),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
        )

    def on_tsdf(self, message: VoxelBlockLayer) -> None:
        self.output_counts["tsdf"] += 1
        self.frames["tsdf"].add(message.header.frame_id)
        self.tsdf_blocks.update((index.x, index.y, index.z) for index in message.block_indices)
        self.tsdf_max_centers = max(
            self.tsdf_max_centers, sum(len(block.centers) for block in message.blocks)
        )
        if self.output_counts["tsdf"] >= 20 and self.tsdf_subscription is not None:
            self.destroy_subscription(self.tsdf_subscription)
            self.tsdf_subscription = None

    def on_mesh(self, message: Mesh) -> None:
        self.output_counts["mesh"] += 1
        self.frames["mesh"].add(message.header.frame_id)
        self.mesh_blocks.update((index.x, index.y, index.z) for index in message.block_indices)
        self.mesh_max_vertices = max(
            self.mesh_max_vertices, sum(len(block.vertices) for block in message.blocks)
        )
        self.mesh_max_triangles = max(
            self.mesh_max_triangles, sum(len(block.triangles) // 3 for block in message.blocks)
        )
        if self.output_counts["mesh"] >= 10 and self.mesh_subscription is not None:
            self.destroy_subscription(self.mesh_subscription)
            self.mesh_subscription = None

    def on_esdf(self, message: PointCloud2) -> None:
        self.output_counts["esdf"] += 1
        self.frames["esdf"].add(message.header.frame_id)
        self.esdf_max_points = max(self.esdf_max_points, message.width * message.height)
        if self.output_counts["esdf"] >= 50 and self.esdf_subscription is not None:
            self.destroy_subscription(self.esdf_subscription)
            self.esdf_subscription = None

    def on_slice(self, message: DistanceMapSlice) -> None:
        self.output_counts["slice"] += 1
        self.frames["slice"].add(message.header.frame_id)
        self.slice_max_width = max(self.slice_max_width, message.width)
        self.slice_max_height = max(self.slice_max_height, message.height)
        known = [
            float(value)
            for value in message.data
            if math.isfinite(value) and abs(float(value) - message.unknown_value) > 1.0e-5
        ]
        self.slice_max_known_cells = max(self.slice_max_known_cells, len(known))
        if known:
            self.slice_min_distance = min(self.slice_min_distance, min(known))
            self.slice_max_distance = max(self.slice_max_distance, max(known))
        if self.output_counts["slice"] >= 50 and self.slice_subscription is not None:
            self.destroy_subscription(self.slice_subscription)
            self.slice_subscription = None

    def publish_command(self, linear: float, angular: float, phase: str) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.command_publisher.publish(command)
        self.command_phases[phase] = self.command_phases.get(phase, 0) + 1

    @staticmethod
    def course_command(elapsed: float) -> tuple[float, float, str]:
        # A bounded bidirectional S course repeatedly changes viewpoint while
        # staying in the collision-free region validated in Phase 5.
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

    def wait_ready(self, wall_timeout_s: float = 120.0) -> float:
        deadline = time.monotonic() + wall_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            tracking = any(state == 1 for _, state in self.statuses)
            reconstruction = (
                self.output_counts["tsdf"] > 0
                and self.output_counts["mesh"] > 0
                and self.output_counts["slice"] > 0
            )
            if tracking and reconstruction and self.depth_info_count:
                return self.clock_time
        raise RuntimeError(
            "Phase 6 did not become ready: "
            f"status={len(self.statuses)} depth_info={self.depth_info_count} "
            f"outputs={self.output_counts}"
        )

    def run_course(self, start_s: float) -> None:
        deadline = time.monotonic() + 300.0
        next_command_s = start_s
        while time.monotonic() < deadline:
            elapsed = self.clock_time - start_s
            if elapsed >= self.mapping_duration:
                break
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.clock_time + 1.0e-9 >= next_command_s:
                linear, angular, phase = self.course_command(max(0.0, elapsed))
                self.publish_command(linear, angular, phase)
                next_command_s = self.clock_time + 0.05
        else:
            raise RuntimeError("Phase 6 mapping course exceeded its wall timeout")
        settle_start = self.clock_time
        next_command_s = settle_start
        while self.clock_time - settle_start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.clock_time + 1.0e-9 >= next_command_s:
                self.publish_command(0.0, 0.0, "final_settle")
                next_command_s = self.clock_time + 0.05

    def call_file_service(self, client: object, path: Path) -> bool:
        if not client.wait_for_service(timeout_sec=15.0):
            return False
        request = FilePath.Request()
        request.file_path = str(path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=90.0)
        return (
            future.done()
            and future.exception() is None
            and future.result() is not None
            and bool(future.result().success)
        )

    def save_outputs(self) -> dict[str, object]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        targets = {
            "map": self.output_dir / "warehouse.nvblx",
            "mesh": self.output_dir / "warehouse.ply",
            "rates": self.output_dir / "nvblox_rates.txt",
            "timings": self.output_dir / "nvblox_timings.txt",
        }
        clients = {
            "map": self.save_map_client,
            "mesh": self.save_ply_client,
            "rates": self.save_rates_client,
            "timings": self.save_timings_client,
        }
        success = {
            name: self.call_file_service(clients[name], path)
            for name, path in targets.items()
        }
        result = {
            "success": success,
            "files": {
                name: {
                    "path": str(path),
                    "exists": path.is_file(),
                    "size_bytes": path.stat().st_size if path.is_file() else 0,
                }
                for name, path in targets.items()
            },
        }
        result["reported_rates_hz"] = self.parse_rates(targets["rates"])
        return result

    @staticmethod
    def parse_rates(path: Path) -> dict[str, float]:
        rates: dict[str, float] = {}
        if not path.is_file():
            return rates
        for line in path.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if len(fields) == 3:
                try:
                    rates[fields[0]] = float(fields[2])
                except ValueError:
                    continue
        return rates

    @staticmethod
    def path_length(samples: list[tuple[float, float, float]]) -> float:
        selected: list[tuple[float, float, float]] = []
        for sample in samples:
            if not selected or sample[0] - selected[-1][0] >= 0.08:
                selected.append(sample)
        return sum(
            math.hypot(current[1] - previous[1], current[2] - previous[2])
            for previous, current in zip(selected, selected[1:])
        )

    def build_report(
        self, start_s: float, saved: dict[str, object]
    ) -> dict[str, object]:
        tracked = [(stamp, state) for stamp, state in self.statuses if stamp >= start_s]
        output_frames = {key: sorted(values) for key, values in self.frames.items()}
        files = saved["files"]
        rates = saved["reported_rates_hz"]
        checks = {
            "clock_monotonic": self.clock_regressions == 0,
            "camera_info_streams_sustained": self.depth_info_count >= int(
                20 * self.mapping_duration
            )
            and self.color_info_count >= int(20 * self.mapping_duration),
            # Configured rates are maxima. Under joint rendering, VSLAM, DDS
            # observation and map serialization, these lower bounds verify a
            # sustained production pipeline rather than a burst of messages.
            "nvblox_processing_rates": rates.get("ros/depth_image_callback", 0.0) >= 15.0
            and rates.get("ros/depth", 0.0) >= 12.0
            and rates.get("ros/color", 0.0) >= 3.0
            and rates.get("ros/update_esdf", 0.0) >= 7.0,
            "cuvslam_tracking_healthy": len(tracked) >= int(5 * self.mapping_duration)
            and all(state == 1 for _, state in tracked),
            "mapping_motion_completed": self.path_length(self.ground_truth) >= 8.0
            and len(self.command_phases) >= 7,
            "tsdf_nonempty": self.output_counts["tsdf"] >= 20
            and len(self.tsdf_blocks) >= 10
            and self.tsdf_max_centers >= 100,
            "mesh_nonempty": self.output_counts["mesh"] >= 10
            and len(self.mesh_blocks) >= 10
            and self.mesh_max_vertices >= 100
            and self.mesh_max_triangles >= 50,
            "esdf_nonempty": self.output_counts["esdf"] >= 20
            and self.esdf_max_points >= 100,
            "map_slice_nonempty": self.output_counts["slice"] >= 20
            and self.slice_max_known_cells >= 100
            and self.slice_max_width > 0
            and self.slice_max_height > 0,
            "outputs_in_odom": all(values == ["odom"] for values in output_frames.values()),
            "map_saved": bool(saved["success"]["map"])
            and int(files["map"]["size_bytes"]) > 1024,
            "mesh_saved": bool(saved["success"]["mesh"])
            and int(files["mesh"]["size_bytes"]) > 1024,
            "statistics_saved": bool(saved["success"]["rates"])
            and bool(saved["success"]["timings"])
            and int(files["rates"]["size_bytes"]) > 0
            and int(files["timings"]["size_bytes"]) > 0,
        }
        checks = {key: bool(value) for key, value in checks.items()}
        return {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "duration_sim_s": self.clock_time - start_s,
            "inputs": {
                "depth_camera_info_count": self.depth_info_count,
                "color_camera_info_count": self.color_info_count,
                "nvblox_reported_rates_hz": rates,
            },
            "tracking": {
                "status_count": len(tracked),
                "failure_count": sum(state != 1 for _, state in tracked),
            },
            "motion": {
                "ground_truth_path_length_m": self.path_length(self.ground_truth),
                "command_phases": self.command_phases,
            },
            "reconstruction": {
                "message_counts": self.output_counts,
                "frames": output_frames,
                "tsdf_unique_blocks": len(self.tsdf_blocks),
                "tsdf_max_voxel_centers_per_message": self.tsdf_max_centers,
                "mesh_unique_blocks": len(self.mesh_blocks),
                "mesh_max_vertices_per_message": self.mesh_max_vertices,
                "mesh_max_triangles_per_message": self.mesh_max_triangles,
                "esdf_max_points": self.esdf_max_points,
                "slice_max_width": self.slice_max_width,
                "slice_max_height": self.slice_max_height,
                "slice_max_known_cells": self.slice_max_known_cells,
                "slice_min_distance_m": self.slice_min_distance,
                "slice_max_distance_m": self.slice_max_distance,
            },
            "esdf_service_policy": {
                "called": False,
                "reason": "get_esdf_and_gradient is 3D-only in nvblox 4.5; Phase 6 uses 2D ESDF",
            },
            "saved": saved,
            "clock_regressions": self.clock_regressions,
        }

    def run(self) -> dict[str, object]:
        start_s = self.wait_ready()
        self.run_course(start_s)
        saved = self.save_outputs()
        report = self.build_report(start_s, saved)
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return report


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NvbloxTestRunner()
    try:
        report = node.run()
        failed = [name for name, passed in report["checks"].items() if not passed]
        if failed:
            raise RuntimeError(f"Phase 6 checks failed: {', '.join(failed)}")
        node.get_logger().info(f"Phase 6 nvblox checks passed: {node.result_path}")
    finally:
        node.publish_command(0.0, 0.0, "shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
