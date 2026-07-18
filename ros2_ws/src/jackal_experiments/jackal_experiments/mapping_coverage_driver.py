"""Follow a collision-clear closed loop while the live visual map is recorded."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import yaml


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    scan_turns: float = 0.0


@dataclass(frozen=True)
class ControllerConfig:
    publish_rate_hz: float
    maximum_linear_speed_mps: float
    minimum_linear_speed_mps: float
    maximum_angular_speed_radps: float
    minimum_angular_speed_radps: float
    heading_gain: float
    rotate_in_place_threshold_rad: float
    waypoint_tolerance_m: float
    maximum_cross_track_error_m: float
    progress_epsilon_m: float
    progress_timeout_sim_s: float
    visual_tracking_timeout_sim_s: float
    settle_sim_s: float
    maximum_sim_duration_s: float
    maximum_wall_duration_s: float


def local_pose(origin: Pose2D, absolute: Pose2D) -> Pose2D:
    delta_x = absolute.x - origin.x
    delta_y = absolute.y - origin.y
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    return Pose2D(
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        wrap_angle(absolute.yaw - origin.yaw),
    )


def point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator <= 1.0e-12:
        return math.dist(point, start)
    projection = (
        (point[0] - start[0]) * delta_x
        + (point[1] - start[1]) * delta_y
    ) / denominator
    projection = max(0.0, min(1.0, projection))
    closest = (start[0] + projection * delta_x, start[1] + projection * delta_y)
    return math.dist(point, closest)


def bounded_signed(
    value: float,
    minimum_magnitude: float,
    maximum_magnitude: float,
) -> float:
    if abs(value) < 1.0e-12:
        return 0.0
    magnitude = max(minimum_magnitude, min(maximum_magnitude, abs(value)))
    return math.copysign(magnitude, value)


def steering_command(
    pose: Pose2D,
    target: RoutePoint,
    config: ControllerConfig,
) -> tuple[float, float, float, float]:
    delta_x = target.x - pose.x
    delta_y = target.y - pose.y
    distance = math.hypot(delta_x, delta_y)
    desired_heading = math.atan2(delta_y, delta_x)
    heading_error = wrap_angle(desired_heading - pose.yaw)
    angular = bounded_signed(
        config.heading_gain * heading_error,
        config.minimum_angular_speed_radps,
        config.maximum_angular_speed_radps,
    )
    if abs(heading_error) >= config.rotate_in_place_threshold_rad:
        return 0.0, angular, distance, heading_error
    distance_speed = max(
        config.minimum_linear_speed_mps,
        min(config.maximum_linear_speed_mps, 0.65 * distance),
    )
    heading_scale = max(0.20, math.cos(heading_error))
    return distance_speed * heading_scale, angular, distance, heading_error


def _positive_float(mapping: dict[str, object], name: str) -> float:
    value = float(mapping[name])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"controller.{name} must be finite and positive")
    return value


def load_config(path: Path) -> tuple[ControllerConfig, list[RoutePoint], dict[str, object]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("unsupported mapping coverage schema")
    if document.get("coordinate_frame") != "initial_ground_truth_base":
        raise ValueError("coverage route must use initial_ground_truth_base")
    controller_document = document.get("controller")
    if not isinstance(controller_document, dict):
        raise ValueError("mapping coverage controller is missing")
    names = (
        "publish_rate_hz",
        "maximum_linear_speed_mps",
        "minimum_linear_speed_mps",
        "maximum_angular_speed_radps",
        "minimum_angular_speed_radps",
        "heading_gain",
        "rotate_in_place_threshold_rad",
        "waypoint_tolerance_m",
        "maximum_cross_track_error_m",
        "progress_epsilon_m",
        "progress_timeout_sim_s",
        "visual_tracking_timeout_sim_s",
        "settle_sim_s",
        "maximum_sim_duration_s",
        "maximum_wall_duration_s",
    )
    values = {name: _positive_float(controller_document, name) for name in names}
    config = ControllerConfig(**values)
    if config.minimum_linear_speed_mps > config.maximum_linear_speed_mps:
        raise ValueError("minimum linear speed exceeds maximum")
    if config.minimum_angular_speed_radps > config.maximum_angular_speed_radps:
        raise ValueError("minimum angular speed exceeds maximum")
    if config.maximum_linear_speed_mps > 0.35:
        raise ValueError("automated mapping speed must not exceed 0.35 m/s")
    if config.maximum_angular_speed_radps > 0.80:
        raise ValueError("automated mapping yaw rate must not exceed 0.80 rad/s")

    route_document = document.get("route")
    if not isinstance(route_document, list) or len(route_document) < 2:
        raise ValueError("mapping coverage route must contain at least two points")
    route: list[RoutePoint] = []
    for index, item in enumerate(route_document):
        if not isinstance(item, dict):
            raise ValueError(f"route[{index}] must be an object")
        xy = item.get("xy")
        if not isinstance(xy, list) or len(xy) != 2:
            raise ValueError(f"route[{index}].xy must contain x and y")
        x, y = (float(value) for value in xy)
        scan_turns = float(item.get("scan_turns", 0.0))
        if not all(math.isfinite(value) for value in (x, y, scan_turns)):
            raise ValueError(f"route[{index}] contains a non-finite value")
        if scan_turns < 0.0 or scan_turns > 1.0:
            raise ValueError(f"route[{index}].scan_turns must be within [0, 1]")
        route.append(RoutePoint(x, y, scan_turns))
    if math.hypot(route[0].x, route[0].y) > 0.15:
        raise ValueError("coverage route must begin at the mapping origin")
    if math.dist((route[0].x, route[0].y), (route[-1].x, route[-1].y)) > 0.15:
        raise ValueError("coverage route must close at its starting point")
    maximum_segment = max(
        math.dist((first.x, first.y), (second.x, second.y))
        for first, second in zip(route, route[1:])
    )
    if maximum_segment > 3.50:
        raise ValueError(f"coverage segment is too long: {maximum_segment:.3f} m")
    reference = document.get("planning_reference")
    if not isinstance(reference, dict) or reference.get("use") != "planning_only":
        raise ValueError("planning_reference must be explicitly planning_only")
    return config, route, reference


class MappingCoverageDriver(Node):
    def __init__(self) -> None:
        super().__init__("mapping_coverage_driver")
        self.declare_parameter("config_path", "")
        self.declare_parameter("report_path", "")
        self.declare_parameter("maximum_waypoints", 0)
        config_value = str(self.get_parameter("config_path").value).strip()
        report_value = str(self.get_parameter("report_path").value).strip()
        if not config_value or not report_value:
            raise ValueError("config_path and report_path are required")
        self.config_path = Path(config_value).resolve()
        self.report_path = Path(report_value).resolve()
        if not self.config_path.is_file():
            raise FileNotFoundError(self.config_path)
        self.config, full_route, self.planning_reference = load_config(self.config_path)
        maximum_waypoints = int(self.get_parameter("maximum_waypoints").value)
        if maximum_waypoints < 0:
            raise ValueError("maximum_waypoints must be non-negative")
        self.route = (
            full_route[:maximum_waypoints]
            if maximum_waypoints
            else full_route
        )
        if len(self.route) < 2:
            raise ValueError("selected coverage route must contain at least two points")
        self.partial_route = len(self.route) != len(full_route)

        command_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Twist, "/cmd_vel_safe", command_qos)
        self.create_subscription(
            Twist, "/cmd_vel_sim", self.on_observed_command, command_qos
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VisualSlamStatus,
            "/visual_slam/status",
            self.on_visual_slam_status,
            qos_profile_sensor_data,
        )

        self.started_wall = time.monotonic()
        self.started_unix_s = time.time()
        self.origin: Pose2D | None = None
        self.pose: Pose2D | None = None
        self.previous_absolute_yaw: float | None = None
        self.unwrapped_absolute_yaw = 0.0
        self.unwrapped_local_yaw = 0.0
        self.previous_local_position: tuple[float, float] | None = None
        self.path_length_m = 0.0
        self.visual_status_samples = 0
        self.visual_tracking_samples = 0
        self.visual_tracking = False
        self.tracking_lost_since_sim: float | None = None
        self.first_sim_s: float | None = None
        self.last_sim_s: float | None = None
        self.waypoint_index = 0
        self.completed_waypoints = 0
        self.scanned_waypoints: list[int] = []
        self.state = "waiting"
        self.settle_until_sim = 0.0
        self.settle_action = ""
        self.scan_start_yaw = 0.0
        self.best_waypoint_distance = math.inf
        self.last_progress_sim = 0.0
        self.maximum_cross_track_error_m = 0.0
        self.maximum_heading_error_rad = 0.0
        self.observed_command_samples = 0
        self.observed_negative_linear_samples = 0
        self.maximum_observed_linear_mps = 0.0
        self.maximum_observed_angular_radps = 0.0
        self.failure = ""
        self.done = False

    def simulation_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_ground_truth(self, message: Odometry) -> None:
        absolute_yaw = yaw_from_odometry(message)
        if self.previous_absolute_yaw is None:
            self.unwrapped_absolute_yaw = absolute_yaw
        else:
            self.unwrapped_absolute_yaw += wrap_angle(
                absolute_yaw - self.previous_absolute_yaw
            )
        self.previous_absolute_yaw = absolute_yaw
        absolute = Pose2D(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            absolute_yaw,
        )
        if self.origin is None:
            self.origin = absolute
        pose = local_pose(self.origin, absolute)
        self.unwrapped_local_yaw = self.unwrapped_absolute_yaw - self.origin.yaw
        self.pose = Pose2D(pose.x, pose.y, pose.yaw)
        position = (pose.x, pose.y)
        if self.previous_local_position is not None:
            self.path_length_m += math.dist(position, self.previous_local_position)
        self.previous_local_position = position
        now = self.simulation_time()
        if self.first_sim_s is None and now > 0.0:
            self.first_sim_s = now
        self.last_sim_s = now

    def on_visual_slam_status(self, message: VisualSlamStatus) -> None:
        self.visual_status_samples += 1
        self.visual_tracking = int(message.vo_state) == 1
        if self.visual_tracking:
            self.visual_tracking_samples += 1
            self.tracking_lost_since_sim = None
        elif self.tracking_lost_since_sim is None:
            self.tracking_lost_since_sim = self.simulation_time()

    def on_observed_command(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        self.observed_command_samples += 1
        self.maximum_observed_linear_mps = max(
            self.maximum_observed_linear_mps, abs(linear)
        )
        self.maximum_observed_angular_radps = max(
            self.maximum_observed_angular_radps, abs(angular)
        )
        if linear < -0.01:
            self.observed_negative_linear_samples += 1

    def publish(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = Twist()
        message.linear.x = max(0.0, float(linear))
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def fail(self, reason: str) -> None:
        if not self.failure:
            self.failure = reason
            self.get_logger().error(reason)
        self.publish()

    def begin_settle(self, action: str) -> None:
        self.state = "settling"
        self.settle_action = action
        self.settle_until_sim = self.simulation_time() + self.config.settle_sim_s
        self.publish()

    def advance_waypoint(self) -> None:
        self.completed_waypoints = max(
            self.completed_waypoints, self.waypoint_index + 1
        )
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.route):
            self.done = True
            self.state = "completed"
            self.publish()
            self.get_logger().info("Mapping coverage route completed")
            return
        self.state = "driving"
        self.best_waypoint_distance = math.inf
        self.last_progress_sim = self.simulation_time()
        target = self.route[self.waypoint_index]
        self.get_logger().info(
            f"Coverage waypoint {self.waypoint_index + 1}/{len(self.route)}: "
            f"({target.x:.2f}, {target.y:.2f})"
        )

    def step(self) -> None:
        now = self.simulation_time()
        if self.failure or self.done:
            self.publish()
            return
        if self.origin is None or self.pose is None or now <= 0.0:
            self.publish()
            return
        if not self.visual_tracking:
            self.publish()
            if self.tracking_lost_since_sim is None:
                self.tracking_lost_since_sim = now
            if (
                self.tracking_lost_since_sim is not None
                and now - self.tracking_lost_since_sim
                > self.config.visual_tracking_timeout_sim_s
            ):
                self.fail("cuVSLAM tracking was unavailable for too long")
            return
        if self.first_sim_s is None:
            self.first_sim_s = now
        if now - self.first_sim_s > self.config.maximum_sim_duration_s:
            self.fail("mapping coverage exceeded maximum simulation duration")
            return
        if self.state == "waiting":
            self.state = "driving"
            self.last_progress_sim = now
            self.get_logger().info(
                f"Starting {len(self.route)}-waypoint mapping coverage route"
            )

        if self.state == "settling":
            self.publish()
            if now < self.settle_until_sim:
                return
            if self.settle_action == "scan":
                self.scan_start_yaw = self.unwrapped_local_yaw
                self.state = "scanning"
            else:
                self.advance_waypoint()
            return

        target = self.route[self.waypoint_index]
        if self.state == "scanning":
            target_rotation = 2.0 * math.pi * target.scan_turns
            completed_rotation = self.unwrapped_local_yaw - self.scan_start_yaw
            remaining = target_rotation - completed_rotation
            if remaining <= 0.03:
                self.scanned_waypoints.append(self.waypoint_index)
                self.begin_settle("advance")
                return
            angular = min(
                self.config.maximum_angular_speed_radps,
                max(self.config.minimum_angular_speed_radps, 1.2 * remaining),
            )
            self.publish(0.0, angular)
            return

        linear, angular, distance, heading_error = steering_command(
            self.pose, target, self.config
        )
        self.maximum_heading_error_rad = max(
            self.maximum_heading_error_rad, abs(heading_error)
        )
        if self.waypoint_index > 0:
            previous = self.route[self.waypoint_index - 1]
            cross_track = point_to_segment_distance(
                (self.pose.x, self.pose.y),
                (previous.x, previous.y),
                (target.x, target.y),
            )
            self.maximum_cross_track_error_m = max(
                self.maximum_cross_track_error_m, cross_track
            )
            if cross_track > self.config.maximum_cross_track_error_m:
                self.fail(
                    f"coverage cross-track error {cross_track:.3f} m exceeds "
                    f"{self.config.maximum_cross_track_error_m:.3f} m"
                )
                return
        if distance <= self.config.waypoint_tolerance_m:
            self.publish()
            if target.scan_turns > 0.0:
                self.begin_settle("scan")
            else:
                self.begin_settle("advance")
            return
        if distance + self.config.progress_epsilon_m < self.best_waypoint_distance:
            self.best_waypoint_distance = distance
            self.last_progress_sim = now
        elif now - self.last_progress_sim > self.config.progress_timeout_sim_s:
            self.fail(
                f"no mapping-route progress for {self.config.progress_timeout_sim_s:.1f} s "
                f"at waypoint {self.waypoint_index}"
            )
            return
        self.publish(linear, angular)

    def report(self) -> dict[str, object]:
        config_hash = hashlib.sha256(self.config_path.read_bytes()).hexdigest()
        final_pose = None
        if self.pose is not None:
            final_pose = [self.pose.x, self.pose.y, self.pose.yaw]
        sim_duration = (
            None
            if self.first_sim_s is None or self.last_sim_s is None
            else self.last_sim_s - self.first_sim_s
        )
        passed = (
            self.done
            and not self.failure
            and self.completed_waypoints == len(self.route)
            and self.observed_negative_linear_samples == 0
            and self.visual_tracking_samples > 0
        )
        return {
            "schema_version": 1,
            "status": "passed" if passed else "failed",
            "error": self.failure or None,
            "started_unix_s": self.started_unix_s,
            "ended_unix_s": time.time(),
            "config_path": str(self.config_path),
            "config_sha256": config_hash,
            "planning_reference": self.planning_reference,
            "coordinate_frame": "initial_ground_truth_base",
            "partial_route": self.partial_route,
            "route": {
                "waypoint_count": len(self.route),
                "completed_waypoints": self.completed_waypoints,
                "scanned_waypoint_indices": self.scanned_waypoints,
                "closed_loop": math.dist(
                    (self.route[0].x, self.route[0].y),
                    (self.route[-1].x, self.route[-1].y),
                )
                <= 0.15,
                "final_local_pose": final_pose,
                "ground_truth_path_length_m": self.path_length_m,
                "maximum_cross_track_error_m": self.maximum_cross_track_error_m,
                "maximum_heading_error_rad": self.maximum_heading_error_rad,
            },
            "visual_slam": {
                "status_samples": self.visual_status_samples,
                "tracking_samples": self.visual_tracking_samples,
                "tracking_fraction": (
                    self.visual_tracking_samples / self.visual_status_samples
                    if self.visual_status_samples
                    else 0.0
                ),
            },
            "commands": {
                "observed_samples": self.observed_command_samples,
                "negative_linear_samples": self.observed_negative_linear_samples,
                "maximum_linear_mps": self.maximum_observed_linear_mps,
                "maximum_angular_radps": self.maximum_observed_angular_radps,
            },
            "simulation_duration_s": sim_duration,
            "wall_duration_s": time.monotonic() - self.started_wall,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingCoverageDriver()
    exit_code = 1
    try:
        period = 1.0 / node.config.publish_rate_hz
        while rclpy.ok() and not node.done and not node.failure:
            iteration_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(0.02, period))
            node.step()
            if (
                time.monotonic() - node.started_wall
                > node.config.maximum_wall_duration_s
            ):
                node.fail("mapping coverage exceeded maximum wall duration")
                break
            remaining = period - (time.monotonic() - iteration_started)
            if remaining > 0.0:
                time.sleep(remaining)
        for _ in range(10):
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.01)
        report = node.report()
        exit_code = 0 if report["status"] == "passed" else 1
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
