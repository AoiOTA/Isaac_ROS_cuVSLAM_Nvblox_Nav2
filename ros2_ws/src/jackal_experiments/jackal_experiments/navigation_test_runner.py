"""Run Stage 8-11 navigation goals and verify every critical data-flow edge."""

from __future__ import annotations

from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import time
import traceback

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nvblox_msgs.msg import DistanceMapSlice
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage


def angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_from_quaternion(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def percentile(values: list[float], quantile: float) -> float:
    """Return a linearly interpolated percentile without a numpy dependency."""

    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    rank = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class NavigationTestRunner(Node):
    def __init__(self) -> None:
        super().__init__("navigation_test_runner")
        self.declare_parameter("result_path", "data/reports/phase8/navigation.json")
        self.declare_parameter(
            "goal_poses", [2.0, 0.0, 0.0, 2.0, 1.5, 1.570796327, 0.5, 1.5, 3.141592654]
        )
        self.declare_parameter("goal_timeout_s", 180.0)
        self.declare_parameter("require_rviz", False)
        self.declare_parameter("action_topic", "/navigate_to_pose")
        self.declare_parameter(
            "nvblox_slice_topic", "/nvblox_node/static_map_slice"
        )
        self.declare_parameter("phase9_mode", False)
        self.declare_parameter("force_relocalization", False)
        self.declare_parameter("force_relocalization_delay_s", 45.0)
        self.declare_parameter("force_relocalization_distance_m", 0.50)
        self.declare_parameter("require_surround_cameras", False)
        self.declare_parameter("require_dynamic_outputs", False)
        self.declare_parameter("experiment_class", "stage8")
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("command_trace_path", "")
        self.declare_parameter("goal_xy_tolerance_m", 0.25)
        self.declare_parameter("goal_yaw_tolerance_deg", 12.0)
        self.declare_parameter("minimum_ground_truth_motion_m", 3.0)
        self.declare_parameter("fault_sequence", "")
        self.declare_parameter("fault_duration_s", 2.0)
        self.declare_parameter("fault_stop_grace_s", 1.5)
        self.declare_parameter("fault_injection_delay_s", 2.0)
        self.declare_parameter("fault_injection_distance_m", 0.15)
        self.declare_parameter("maximum_command_while_fault", 0.02)
        self.result_path = Path(str(self.get_parameter("result_path").value)).resolve()
        flat = [float(value) for value in self.get_parameter("goal_poses").value]
        if not flat or len(flat) % 3:
            raise ValueError("goal_poses must contain x,y,yaw triples")
        self.goals = [tuple(flat[i : i + 3]) for i in range(0, len(flat), 3)]
        self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
        self.require_rviz = bool(self.get_parameter("require_rviz").value)
        self.phase9_mode = bool(self.get_parameter("phase9_mode").value)
        self.force_relocalization = bool(
            self.get_parameter("force_relocalization").value
        )
        self.require_surround_cameras = bool(
            self.get_parameter("require_surround_cameras").value
        )
        self.require_dynamic_outputs = bool(
            self.get_parameter("require_dynamic_outputs").value
        )
        self.experiment_class = str(
            self.get_parameter("experiment_class").value
        )
        trajectory_value = str(self.get_parameter("trajectory_path").value)
        command_value = str(self.get_parameter("command_trace_path").value)
        self.trajectory_path = (
            Path(trajectory_value).resolve() if trajectory_value else None
        )
        self.command_trace_path = (
            Path(command_value).resolve() if command_value else None
        )
        self.goal_xy_tolerance = float(
            self.get_parameter("goal_xy_tolerance_m").value
        )
        self.goal_yaw_tolerance_deg = float(
            self.get_parameter("goal_yaw_tolerance_deg").value
        )
        self.minimum_ground_truth_motion = float(
            self.get_parameter("minimum_ground_truth_motion_m").value
        )
        fault_value = str(self.get_parameter("fault_sequence").value).strip()
        self.fault_sequence = [
            item.strip() for item in fault_value.split(",") if item.strip()
        ]
        valid_faults = {"depth_stale", "map_slice_stale", "relocalization"}
        invalid_faults = set(self.fault_sequence) - valid_faults
        if invalid_faults:
            raise ValueError(f"unsupported fault types: {sorted(invalid_faults)}")
        self.fault_duration = float(self.get_parameter("fault_duration_s").value)
        self.fault_stop_grace = float(
            self.get_parameter("fault_stop_grace_s").value
        )
        self.fault_injection_delay = float(
            self.get_parameter("fault_injection_delay_s").value
        )
        self.fault_injection_distance = float(
            self.get_parameter("fault_injection_distance_m").value
        )
        self.maximum_command_while_fault = float(
            self.get_parameter("maximum_command_while_fault").value
        )
        self.force_relocalization_delay = float(
            self.get_parameter("force_relocalization_delay_s").value
        )
        self.force_relocalization_distance = float(
            self.get_parameter("force_relocalization_distance_m").value
        )
        if min(
            self.force_relocalization_delay, self.force_relocalization_distance
        ) <= 0.0:
            raise ValueError("forced-relocalization delay and distance must be positive")
        if min(
            self.goal_xy_tolerance,
            self.goal_yaw_tolerance_deg,
            self.fault_duration,
            self.fault_stop_grace,
            self.fault_injection_delay,
            self.fault_injection_distance,
            self.maximum_command_while_fault,
        ) <= 0.0 or self.minimum_ground_truth_motion < 0.0:
            raise ValueError("Stage 10 tolerances and fault timings are invalid")
        self.counts: defaultdict[str, int] = defaultdict(int)
        self.first_wall: dict[str, float] = {}
        self.last_wall: dict[str, float] = {}
        self.localization_ready = False
        self.tracking_samples = 0
        self.tf_edges: set[tuple[str, str]] = set()
        self.tf_transforms: dict[
            tuple[str, str], tuple[float, float, float]
        ] = {}
        self.scan_finite = 0
        self.safety_scan_finite = 0
        self.depth_cloud_points = 0
        self.last_scan_stamp_ns = -1
        self.scan_stamp_regressions = 0
        self.map_cells = 0
        self.global_costmap_cells = 0
        self.local_costmap_cells = 0
        self.nvblox_slice_cells = 0
        self.latest_feedback_pose = None
        self.guard_states: set[str] = set()
        self.collision_actions: defaultdict[int, int] = defaultdict(int)
        self.ground_truth: list[tuple[float, float]] = []
        self.ground_truth_trace: list[dict[str, float]] = []
        self.command_trace: list[dict[str, float | str]] = []
        self.active_goal_index = -1
        self.active_goal_ground_truth_start = 0
        self.active_goal_command_start = 0
        self.active_global_plan_length = 0.0
        self.active_fault: dict[str, object] | None = None
        self.fault_results: list[dict[str, object]] = []
        self.command_max = defaultdict(float)
        self.command_latest: dict[str, tuple[float, float]] = {}
        self.command_receive_wall: dict[str, float] = {}
        self.command_latency_ms: defaultdict[str, list[float]] = defaultdict(list)
        self.data_age_ms: defaultdict[str, list[float]] = defaultdict(list)
        self.command_lateral_max = 0.0
        self.latest_guard_state = "unseen"
        self.latest_collision_action = -1
        self.feedback_log_wall = 0.0
        self.active_goal: tuple[float, float, float] | None = None
        self.camera_window_enabled = False
        self.camera_window_last_change_wall = time.monotonic()
        self.last_surround_message_wall = 0.0
        self.surround_counts: defaultdict[str, int] = defaultdict(int)
        self.recovery_states: set[str] = set()
        self.recovery_count = 0
        self.resilient_states: set[str] = set()
        self.resilient_resume_count = 0
        self.dynamic_slice_cells = 0
        self.dynamic_points = 0
        self.dynamic_esdf_points = 0
        self.combined_esdf_points = 0
        self.force_injected = False
        self.force_response_success = False
        self.unready_since_wall: float | None = None
        self.max_command_while_unready = 0.0

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/localization/ready", self.on_ready, latched)
        self.create_subscription(
            Bool, "/vgl/cameras_enabled", self.on_camera_window, latched
        )
        self.create_subscription(
            String,
            "/localization/recovery_state",
            self.on_recovery_state,
            latched,
        )
        self.create_subscription(
            String,
            "/navigation/resilient_status",
            self.on_resilient_status,
            latched,
        )
        self.create_subscription(
            VisualSlamStatus, "/visual_slam/status", self.on_status, reliable
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, reliable)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.on_global_costmap,
            reliable,
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.on_local_costmap,
            reliable,
        )
        self.create_subscription(
            LaserScan, "/front_depth/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            "/front_depth/scan_raw",
            self.on_safety_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/front_depth/points_odom",
            self.on_depth_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DistanceMapSlice,
            str(self.get_parameter("nvblox_slice_topic").value),
            self.on_nvblox_slice,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DistanceMapSlice,
            "/nvblox_node/dynamic_map_slice",
            self.on_dynamic_slice,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/nvblox_node/dynamic_points",
            self.on_dynamic_points,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/nvblox_node/dynamic_esdf_pointcloud",
            self.on_dynamic_esdf,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/nvblox_node/combined_esdf_pointcloud",
            self.on_combined_esdf,
            qos_profile_sensor_data,
        )
        for camera in ("left", "right", "back"):
            for side in ("left", "right"):
                image_topic = f"/{camera}_stereo_camera/{side}/image_raw"
                info_topic = f"/{camera}_stereo_camera/{side}/camera_info"
                self.create_subscription(
                    Image,
                    image_topic,
                    lambda msg, label=f"{camera}_{side}_image": self.on_surround(
                        label, msg
                    ),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    CameraInfo,
                    info_topic,
                    lambda msg, label=f"{camera}_{side}_info": self.on_surround(
                        label, msg
                    ),
                    qos_profile_sensor_data,
                )
        self.create_subscription(NavPath, "/plan", lambda msg: self.on_path("plan", msg), reliable)
        self.create_subscription(
            NavPath,
            "/transformed_global_plan",
            lambda msg: self.on_path("local_plan", msg),
            reliable,
        )
        for topic, name in (
            ("/cmd_vel_nav_raw", "cmd_nav_raw"),
            ("/cmd_vel_smoothed", "cmd_smoothed"),
            ("/cmd_vel_safe", "cmd_safe"),
            ("/cmd_vel_sim", "cmd_sim"),
        ):
            self.create_subscription(
                Twist, topic, lambda msg, label=name: self.on_command(label, msg), reliable
            )
        self.create_subscription(
            String, "/control/guard_status", self.on_guard_status, reliable
        )
        self.create_subscription(
            CollisionMonitorState,
            "/collision_monitor/state",
            self.on_collision_state,
            reliable,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("action_topic").value),
        )
        self.force_client = self.create_client(
            Trigger, "/localization/force_relocalization"
        )
        self.depth_fault_client = self.create_client(
            SetBool, "/control/fault_depth_stale"
        )
        self.map_slice_fault_client = self.create_client(
            SetBool, "/control/fault_map_slice_stale"
        )
        lifecycle_names = (
            "map_server",
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "velocity_smoother",
            "collision_monitor",
            "bt_navigator",
            "waypoint_follower",
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in lifecycle_names
        }

    def record(self, name: str) -> None:
        now = time.monotonic()
        self.counts[name] += 1
        self.first_wall.setdefault(name, now)
        self.last_wall[name] = now

    def record_message_age(self, name: str, message: object) -> None:
        """Record source-stamp age in simulation time when a header exists."""

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        if 0 <= age_ns <= 5_000_000_000:
            self.data_age_ms[name].append(age_ns * 1.0e-6)

    def on_ready(self, message: Bool) -> None:
        self.record("localization_ready")
        was_ready = self.localization_ready
        self.localization_ready = message.data
        if message.data:
            self.unready_since_wall = None
        elif was_ready or self.unready_since_wall is None:
            # Preserve the start of the unsafe window across the manager's
            # latched heartbeat so the command-stop assertion is meaningful.
            self.unready_since_wall = time.monotonic()

    def on_camera_window(self, message: Bool) -> None:
        self.record("camera_window")
        if message.data != self.camera_window_enabled:
            self.camera_window_last_change_wall = time.monotonic()
        self.camera_window_enabled = message.data

    def on_recovery_state(self, message: String) -> None:
        self.record("recovery_state")
        try:
            payload = json.loads(message.data)
            self.recovery_states.add(str(payload["state"]))
            self.recovery_count = max(
                self.recovery_count, int(payload.get("recovery_count", 0))
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.recovery_states.add("invalid_state")

    def on_resilient_status(self, message: String) -> None:
        self.record("resilient_status")
        try:
            payload = json.loads(message.data)
            self.resilient_states.add(str(payload["state"]))
            self.resilient_resume_count = max(
                self.resilient_resume_count, int(payload.get("resume_count", 0))
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.resilient_states.add("invalid_state")

    def on_surround(self, label: str, _message: object) -> None:
        self.record(label)
        self.surround_counts[label] += 1
        self.last_surround_message_wall = time.monotonic()

    def on_status(self, message: VisualSlamStatus) -> None:
        self.record("visual_slam_status")
        self.record_message_age("visual_slam_status", message)
        if int(message.vo_state) == 1:
            self.tracking_samples += 1

    def on_tf(self, message: TFMessage) -> None:
        self.record("tf")
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            self.tf_edges.add(edge)
            self.tf_transforms[edge] = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                yaw_from_quaternion(transform.transform.rotation),
            )

    def current_map_base_pose(self) -> tuple[float, float, float] | None:
        map_to_odom = self.tf_transforms.get(("map", "odom"))
        odom_to_base = self.tf_transforms.get(("odom", "base_link"))
        if map_to_odom is None or odom_to_base is None:
            return None
        x_map_odom, y_map_odom, yaw_map_odom = map_to_odom
        x_odom_base, y_odom_base, yaw_odom_base = odom_to_base
        cosine = math.cos(yaw_map_odom)
        sine = math.sin(yaw_map_odom)
        return (
            x_map_odom + cosine * x_odom_base - sine * y_odom_base,
            y_map_odom + sine * x_odom_base + cosine * y_odom_base,
            yaw_map_odom + yaw_odom_base,
        )

    def on_map(self, message: OccupancyGrid) -> None:
        self.record("map")
        if message.header.frame_id == "map":
            self.map_cells = max(self.map_cells, len(message.data))

    def on_global_costmap(self, message: OccupancyGrid) -> None:
        self.record("global_costmap")
        self.global_costmap_cells = max(self.global_costmap_cells, len(message.data))

    def on_local_costmap(self, message: OccupancyGrid) -> None:
        self.record("local_costmap")
        self.local_costmap_cells = max(self.local_costmap_cells, len(message.data))

    def on_scan(self, message: LaserScan) -> None:
        self.record("depth_scan")
        self.record_message_age("front_depth", message)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if self.last_scan_stamp_ns >= 0 and stamp_ns < self.last_scan_stamp_ns:
            self.scan_stamp_regressions += 1
        self.last_scan_stamp_ns = stamp_ns
        self.scan_finite += sum(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )

    def on_safety_scan(self, message: LaserScan) -> None:
        self.record("safety_scan_raw")
        self.record_message_age("front_depth_raw", message)
        self.safety_scan_finite += sum(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )

    def on_nvblox_slice(self, message: DistanceMapSlice) -> None:
        self.record("nvblox_slice")
        self.record_message_age("nvblox_slice", message)
        expected = int(message.width) * int(message.height)
        if expected > 0 and len(message.data) == expected:
            self.nvblox_slice_cells = max(self.nvblox_slice_cells, expected)

    def on_dynamic_slice(self, message: DistanceMapSlice) -> None:
        self.record("dynamic_slice")
        self.record_message_age("dynamic_slice", message)
        expected = int(message.width) * int(message.height)
        if expected > 0 and len(message.data) == expected:
            self.dynamic_slice_cells = max(self.dynamic_slice_cells, expected)

    def on_dynamic_points(self, message: PointCloud2) -> None:
        self.record("dynamic_points")
        self.dynamic_points = max(
            self.dynamic_points, int(message.width) * int(message.height)
        )

    def on_dynamic_esdf(self, message: PointCloud2) -> None:
        self.record("dynamic_esdf")
        self.dynamic_esdf_points = max(
            self.dynamic_esdf_points, int(message.width) * int(message.height)
        )

    def on_combined_esdf(self, message: PointCloud2) -> None:
        self.record("combined_esdf")
        self.combined_esdf_points = max(
            self.combined_esdf_points, int(message.width) * int(message.height)
        )

    def on_depth_cloud(self, message: PointCloud2) -> None:
        self.record("depth_cloud")
        self.record_message_age("depth_cloud", message)
        if message.header.frame_id == "odom":
            self.depth_cloud_points = max(self.depth_cloud_points, int(message.width))

    def on_path(self, name: str, message: NavPath) -> None:
        if message.poses:
            self.record(name)
            if name == "plan" and self.active_goal is not None:
                points = [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in message.poses
                ]
                length = path_length(points)
                if length > 0.0:
                    # Smac republishes a progressively shorter plan while the
                    # robot advances. Preserve the complete initial/reference
                    # route instead of the final few centimetres.
                    self.active_global_plan_length = max(
                        self.active_global_plan_length, length
                    )

    def on_command(self, name: str, message: Twist) -> None:
        self.record(name)
        now_wall = time.monotonic()
        predecessor = {
            "cmd_smoothed": "cmd_nav_raw",
            "cmd_safe": "cmd_smoothed",
            "cmd_sim": "cmd_safe",
        }.get(name)
        if predecessor in self.command_receive_wall:
            latency_ms = (now_wall - self.command_receive_wall[predecessor]) * 1000.0
            if 0.0 <= latency_ms <= 250.0:
                self.command_latency_ms[f"{predecessor}_to_{name}"].append(
                    latency_ms
                )
        if name == "cmd_sim" and "cmd_nav_raw" in self.command_receive_wall:
            freshness_ms = (now_wall - self.command_receive_wall["cmd_nav_raw"]) * 1000.0
            if self.active_goal is not None and 0.0 <= freshness_ms <= 250.0:
                self.command_latency_ms["cmd_nav_raw_to_cmd_sim_freshness"].append(
                    freshness_ms
                )
        self.command_receive_wall[name] = now_wall
        self.command_latest[name] = (float(message.linear.x), float(message.angular.z))
        self.command_max[f"{name}_linear"] = max(
            self.command_max[f"{name}_linear"], abs(message.linear.x)
        )
        self.command_max[f"{name}_angular"] = max(
            self.command_max[f"{name}_angular"], abs(message.angular.z)
        )
        self.command_lateral_max = max(self.command_lateral_max, abs(message.linear.y))
        if (
            name == "cmd_sim"
            and not self.localization_ready
            and self.unready_since_wall is not None
            and time.monotonic() - self.unready_since_wall >= 0.20
        ):
            self.max_command_while_unready = max(
                self.max_command_while_unready,
                abs(message.linear.x),
                abs(message.angular.z),
            )
        if name == "cmd_sim":
            self.command_trace.append(
                {
                    "wall_time_s": now_wall,
                    "sim_time_s": self.get_clock().now().nanoseconds * 1.0e-9,
                    "goal_index": self.active_goal_index,
                    "linear_x_mps": float(message.linear.x),
                    "angular_z_radps": float(message.angular.z),
                    "guard_state": self.latest_guard_state,
                }
            )
            if self.active_fault is not None:
                activated = float(self.active_fault["activated_wall_s"])
                if now_wall - activated >= self.fault_stop_grace:
                    self.active_fault["max_command_after_grace"] = max(
                        float(self.active_fault["max_command_after_grace"]),
                        abs(float(message.linear.x)),
                        abs(float(message.angular.z)),
                    )

    def on_guard_status(self, message: String) -> None:
        self.record("guard_status")
        try:
            self.latest_guard_state = str(json.loads(message.data)["state"])
            self.guard_states.add(self.latest_guard_state)
            if self.active_fault is not None:
                states = self.active_fault["guard_states"]
                if isinstance(states, set):
                    states.add(self.latest_guard_state)
        except (json.JSONDecodeError, KeyError, TypeError):
            self.guard_states.add("invalid_status")

    def on_collision_state(self, message: CollisionMonitorState) -> None:
        self.record("collision_state")
        self.latest_collision_action = int(message.action_type)
        self.collision_actions[self.latest_collision_action] += 1

    def on_ground_truth(self, message: Odometry) -> None:
        self.record("ground_truth")
        point = (message.pose.pose.position.x, message.pose.pose.position.y)
        if not self.ground_truth or math.dist(point, self.ground_truth[-1]) >= 0.005:
            self.ground_truth.append(point)
        stamp = message.header.stamp
        self.ground_truth_trace.append(
            {
                "sim_time_s": float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
                "goal_index": self.active_goal_index,
                "x_m": float(message.pose.pose.position.x),
                "y_m": float(message.pose.pose.position.y),
                "yaw_rad": yaw_from_quaternion(message.pose.pose.orientation),
                "linear_x_mps": float(message.twist.twist.linear.x),
                "angular_z_radps": float(message.twist.twist.angular.z),
            }
        )

    def call_bool_service(
        self, client: object, enabled: bool, timeout_s: float = 3.0
    ) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, "service unavailable"
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None:
            return False, "no response"
        return bool(response.success), str(response.message)

    def begin_fault(self, kind: str) -> None:
        target_state = {
            "depth_stale": "blocked_depth_stale",
            "map_slice_stale": "blocked_map_slice_stale",
            "relocalization": "blocked_localization_not_ready",
        }[kind]
        result: dict[str, object] = {
            "kind": kind,
            "injected": False,
            "restored": False,
            "target_guard_state": target_state,
            "guard_states": set(),
            "max_command_after_grace": 0.0,
            "activated_wall_s": time.monotonic(),
        }
        if kind == "depth_stale":
            success, message = self.call_bool_service(self.depth_fault_client, True)
        elif kind == "map_slice_stale":
            success, message = self.call_bool_service(
                self.map_slice_fault_client, True
            )
        else:
            if not self.force_client.wait_for_service(timeout_sec=3.0):
                success, message = False, "service unavailable"
            else:
                future = self.force_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                response = future.result()
                success = bool(response is not None and response.success)
                message = response.message if response is not None else "no response"
                self.force_injected = True
                self.force_response_success = success
        result["injected"] = success
        result["message"] = message
        result["activated_wall_s"] = time.monotonic()
        self.active_fault = result
        self.get_logger().info(f"Injected Stage 10 fault {kind}: {message}")

    def finish_fault(self) -> None:
        if self.active_fault is None:
            return
        kind = str(self.active_fault["kind"])
        if kind == "depth_stale":
            success, message = self.call_bool_service(self.depth_fault_client, False)
        elif kind == "map_slice_stale":
            success, message = self.call_bool_service(
                self.map_slice_fault_client, False
            )
        else:
            # The localization recovery manager owns relocalization restoration.
            success, message = True, "recovery manager owns restoration"
        states = self.active_fault["guard_states"]
        self.active_fault["guard_states"] = (
            sorted(states) if isinstance(states, set) else []
        )
        self.active_fault["restored"] = success
        self.active_fault["restore_message"] = message
        self.active_fault["duration_s"] = (
            time.monotonic() - float(self.active_fault["activated_wall_s"])
        )
        self.fault_results.append(self.active_fault)
        self.get_logger().info(f"Restored Stage 10 fault {kind}: {message}")
        self.active_fault = None

    @staticmethod
    def command_smoothness(samples: list[dict[str, float | str]]) -> dict[str, float]:
        accelerations_linear: list[float] = []
        accelerations_angular: list[float] = []
        jerks_linear: list[float] = []
        jerks_angular: list[float] = []
        previous_acceleration: tuple[float, float, float] | None = None
        for previous, current in zip(samples, samples[1:]):
            # Emergency health/watchdog stops deliberately bypass slew
            # limiting. Measure normal navigation smoothness only while the
            # final Guard reports active; fault-stop latency is audited by a
            # separate hardening check.
            if (
                previous.get("guard_state") != "active"
                or current.get("guard_state") != "active"
            ):
                previous_acceleration = None
                continue
            dt = float(current["sim_time_s"]) - float(previous["sim_time_s"])
            if dt <= 0.002 or dt > 0.10:
                continue
            linear = (
                float(current["linear_x_mps"])
                - float(previous["linear_x_mps"])
            ) / dt
            angular = (
                float(current["angular_z_radps"])
                - float(previous["angular_z_radps"])
            ) / dt
            accelerations_linear.append(abs(linear))
            accelerations_angular.append(abs(angular))
            if previous_acceleration is not None:
                last_time, last_linear, last_angular = previous_acceleration
                acceleration_dt = float(current["sim_time_s"]) - last_time
                if acceleration_dt > 0.002:
                    jerks_linear.append(abs(linear - last_linear) / acceleration_dt)
                    jerks_angular.append(abs(angular - last_angular) / acceleration_dt)
            previous_acceleration = (
                float(current["sim_time_s"]),
                linear,
                angular,
            )
        return {
            "linear_acceleration_p95_mps2": percentile(accelerations_linear, 0.95),
            "angular_acceleration_p95_radps2": percentile(accelerations_angular, 0.95),
            "linear_jerk_p95_mps3": percentile(jerks_linear, 0.95),
            "angular_jerk_p95_radps3": percentile(jerks_angular, 0.95),
        }

    def spin_until(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        next_diagnostic = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
            if time.monotonic() >= next_diagnostic:
                self.get_logger().info(
                    f"Waiting for {label}: ready={self.localization_ready} "
                    f"map={self.map_cells} global_costmap={self.global_costmap_cells} "
                    f"local_costmap={self.local_costmap_cells} scan_ranges={self.scan_finite} "
                    f"depth_cloud={self.depth_cloud_points} "
                    f"nvblox_slice={self.nvblox_slice_cells}"
                )
                next_diagnostic += 5.0
        raise TimeoutError(f"timeout waiting for {label}")

    def lifecycle_states(self) -> dict[str, str]:
        states = {}
        for name, client in self.lifecycle_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                states[name] = "service_unavailable"
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            states[name] = response.current_state.label if response else "no_response"
        return states

    def feedback(self, message) -> None:
        self.latest_feedback_pose = message.feedback.current_pose
        now = time.monotonic()
        if now - self.feedback_log_wall < 5.0:
            return
        self.feedback_log_wall = now
        pose = self.latest_feedback_pose.pose
        ground_truth = self.ground_truth[-1] if self.ground_truth else (math.nan, math.nan)
        command_text = ", ".join(
            f"{name}=({value[0]:.3f},{value[1]:.3f})"
            for name, value in sorted(self.command_latest.items())
        )
        self.get_logger().info(
            f"Goal progress target={self.active_goal} map_pose="
            f"({pose.position.x:.3f},{pose.position.y:.3f},"
            f"{yaw_from_quaternion(pose.orientation):.3f}) ground_truth="
            f"({ground_truth[0]:.3f},{ground_truth[1]:.3f}) guard="
            f"{self.latest_guard_state} collision_action={self.latest_collision_action} "
            f"commands=[{command_text}]"
        )

    def execute_goal(
        self,
        goal_index: int,
        x: float,
        y: float,
        heading: float,
        fault_kind: str | None = None,
    ) -> dict[str, object]:
        self.latest_feedback_pose = None
        self.active_goal = (x, y, heading)
        self.active_goal_index = goal_index
        self.active_goal_ground_truth_start = max(0, len(self.ground_truth) - 1)
        self.active_goal_command_start = len(self.command_trace)
        self.active_global_plan_length = 0.0
        self.feedback_log_wall = 0.0
        self.get_logger().info(f"Sending NavigateToPose goal {self.active_goal}")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(heading * 0.5)
        goal.pose.pose.orientation.w = math.cos(heading * 0.5)
        send = self.action.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"NavigateToPose rejected goal {(x, y, heading)}")
        self.get_logger().info(f"NavigateToPose accepted goal {self.active_goal}")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.goal_timeout
        goal_started_wall = time.monotonic()
        goal_start_point = self.ground_truth[-1] if self.ground_truth else None
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            travelled = (
                math.dist(goal_start_point, self.ground_truth[-1])
                if goal_start_point is not None and self.ground_truth
                else 0.0
            )
            if (
                fault_kind is not None
                and self.active_fault is None
                and not any(
                    result.get("kind") == fault_kind for result in self.fault_results
                )
                and (
                    travelled >= self.fault_injection_distance
                    or time.monotonic() - goal_started_wall
                    >= self.fault_injection_delay
                )
            ):
                self.begin_fault(fault_kind)
            if (
                self.active_fault is not None
                and time.monotonic()
                - float(self.active_fault["activated_wall_s"])
                >= self.fault_duration
            ):
                self.finish_fault()
            if (
                self.force_relocalization
                and not self.force_injected
                and self.ground_truth
                and (
                    math.dist(self.ground_truth[0], self.ground_truth[-1])
                    >= self.force_relocalization_distance
                    or time.monotonic() - goal_started_wall
                    >= self.force_relocalization_delay
                )
            ):
                if not self.force_client.wait_for_service(timeout_sec=0.05):
                    continue
                future = self.force_client.call_async(Trigger.Request())
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                response = future.result()
                self.force_injected = True
                self.force_response_success = bool(
                    response is not None and response.success
                )
                self.get_logger().info(
                    "Injected safe cuVGL relocalization during active goal"
                )
        if self.active_fault is not None:
            self.finish_fault()
        wrapped = result_future.result()
        if wrapped is None:
            handle.cancel_goal_async()
            raise TimeoutError(f"NavigateToPose timed out for {(x, y, heading)}")
        # Feedback can stop just before the final controller cycle. Sample the
        # authoritative TF chain after the action result for acceptance error.
        settle_deadline = time.monotonic() + 0.50
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        final_map_pose = self.current_map_base_pose()
        if final_map_pose is not None:
            pose_x, pose_y, pose_yaw = final_map_pose
            pose_source = "map_to_odom_to_base_link_tf"
        elif self.latest_feedback_pose is not None:
            pose = self.latest_feedback_pose.pose
            pose_x, pose_y = pose.position.x, pose.position.y
            pose_yaw = yaw_from_quaternion(pose.orientation)
            pose_source = "navigate_to_pose_feedback"
        else:
            pose_x = pose_y = pose_yaw = math.inf
            pose_source = "unavailable"
        xy_error = math.hypot(pose_x - x, pose_y - y)
        yaw_error = abs(angle_difference(pose_yaw, heading))
        self.get_logger().info(
            f"NavigateToPose result goal={self.active_goal} status={wrapped.status} "
            f"error_code={wrapped.result.error_code} xy_error={xy_error:.3f} "
            f"yaw_error_deg={math.degrees(yaw_error):.2f}"
        )
        goal_points = self.ground_truth[self.active_goal_ground_truth_start :]
        actual_length = path_length(goal_points)
        reference_length = self.active_global_plan_length
        stretch = (
            max(0.0, actual_length / reference_length - 1.0)
            if reference_length > 0.05
            else math.inf
        )
        goal_commands = self.command_trace[self.active_goal_command_start :]
        return {
            "requested": [x, y, heading],
            "status": int(wrapped.status),
            "error_code": int(wrapped.result.error_code),
            "error_msg": wrapped.result.error_msg,
            "xy_error_m": xy_error,
            "yaw_error_deg": math.degrees(yaw_error),
            "final_pose_source": pose_source,
            "final_map_pose": [pose_x, pose_y, pose_yaw],
            "elapsed_wall_s": time.monotonic() - goal_started_wall,
            "actual_path_length_m": actual_length,
            "reference_plan_length_m": reference_length,
            "path_stretch": stretch,
            "command_smoothness": self.command_smoothness(goal_commands),
        }

    def run(self) -> dict[str, object]:
        self.spin_until(lambda: self.localization_ready, 120.0, "cuVGL/cuvSLAM localization")
        self.spin_until(lambda: self.action.wait_for_server(timeout_sec=0.0), 120.0, "Nav2 action")
        self.spin_until(
            lambda: self.map_cells > 0
            and self.global_costmap_cells > 0
            and self.local_costmap_cells > 0
            and self.scan_finite > 0
            and self.counts["nvblox_slice"] > 0,
            90.0,
            "map, costmaps, depth scan, and nvblox slice",
        )
        states: dict[str, str] = {}

        def all_lifecycle_nodes_active() -> bool:
            nonlocal states
            states = self.lifecycle_states()
            return all(state == "active" for state in states.values())

        # The NavigateToPose action is created while bt_navigator is being
        # configured, slightly before the complete managed stack is active.
        self.spin_until(all_lifecycle_nodes_active, 90.0, "Nav2 lifecycle activation")
        node_names = sorted(
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        )
        goals = [
            self.execute_goal(
                index,
                *goal,
                fault_kind=(
                    self.fault_sequence[index]
                    if index < len(self.fault_sequence)
                    else None
                ),
            )
            for index, goal in enumerate(self.goals)
        ]
        self.active_goal = None
        self.active_goal_index = -1
        # Allow the zero command to propagate through every safety stage after completion.
        stop_deadline = time.monotonic() + (2.0 if self.phase9_mode else 1.0)
        while time.monotonic() < stop_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        ground_truth_path_length = path_length(self.ground_truth)
        required_counts = (
            "map",
            "global_costmap",
            "local_costmap",
            "depth_scan",
            "safety_scan_raw",
            "depth_cloud",
            "nvblox_slice",
            "plan",
            "local_plan",
            "cmd_nav_raw",
            "cmd_smoothed",
            "cmd_safe",
            "cmd_sim",
        )
        checks = {
            "all_lifecycle_nodes_active": all(v == "active" for v in states.values()),
            "all_goals_succeeded": all(
                goal["status"] == GoalStatus.STATUS_SUCCEEDED
                and goal["error_code"] == 0
                and goal["xy_error_m"] <= self.goal_xy_tolerance
                and goal["yaw_error_deg"] <= self.goal_yaw_tolerance_deg
                for goal in goals
            ),
            "critical_topics_nonempty": all(self.counts[name] > 0 for name in required_counts),
            "map_and_costmaps_nonempty": min(
                self.map_cells, self.global_costmap_cells, self.local_costmap_cells
            ) > 0,
            "depth_scan_contains_ranges": self.scan_finite > 0,
            "raw_safety_scan_contains_ranges": self.safety_scan_finite > 0,
            "depth_cloud_in_odom_nonempty": self.depth_cloud_points > 0,
            "depth_scan_stamps_monotonic": self.scan_stamp_regressions == 0,
            "nvblox_slice_nonempty": self.nvblox_slice_cells > 0,
            "main_tf_chain_seen": ("map", "odom") in self.tf_edges
            and ("odom", "base_link") in self.tf_edges,
            "cuvslam_tracking": self.tracking_samples >= 20,
            "command_chain_moved": self.command_max["cmd_sim_linear"] > 0.05,
            "differential_drive_no_lateral_command": self.command_lateral_max < 1.0e-9,
            "ground_truth_motion": (
                ground_truth_path_length >= self.minimum_ground_truth_motion
            ),
            "guard_became_active": "active" in self.guard_states,
            "rviz_running_when_requested": not self.require_rviz or "/rviz2" in node_names,
        }
        if self.phase9_mode:
            required_surround = {
                f"{camera}_{side}_{kind}"
                for camera in ("left", "right", "back")
                for side in ("left", "right")
                for kind in ("image", "info")
            }
            checks.update(
                {
                    "vgl_camera_mode_valid": (
                        (
                            required_surround <= set(self.surround_counts)
                            and all(
                                self.surround_counts[name] > 0
                                for name in required_surround
                            )
                        )
                        if self.require_surround_cameras
                        else (
                            not self.camera_window_enabled
                            and not any(self.surround_counts.values())
                        )
                    ),
                    "combined_slice_nonempty": self.nvblox_slice_cells > 0,
                }
            )
            if self.require_dynamic_outputs:
                checks.update(
                    {
                        "dynamic_slice_nonempty": self.dynamic_slice_cells > 0,
                        "dynamic_esdf_nonempty": self.dynamic_esdf_points > 0,
                        "combined_esdf_nonempty": self.combined_esdf_points > 0,
                    }
                )
            recovery_required = self.force_relocalization or (
                "relocalization" in self.fault_sequence
            )
            if recovery_required:
                checks.update(
                    {
                        "forced_relocalization_accepted": self.force_injected
                        and self.force_response_success,
                        "recovery_state_machine_completed": self.recovery_count >= 1
                        and "camera_warmup" in self.recovery_states
                        and "waiting_for_vgl_pose" in self.recovery_states
                        and "waiting_for_tracking" in self.recovery_states
                        and "navigation_ready" in self.recovery_states,
                        "resilient_action_paused_and_resumed": (
                            self.resilient_resume_count >= 1
                            and "waiting_for_localization" in self.resilient_states
                            and "succeeded" in self.resilient_states
                        ),
                        "guard_stopped_during_relocalization": (
                            self.max_command_while_unready
                            <= self.maximum_command_while_fault
                        ),
                    }
                )
        if self.experiment_class.startswith("stage10"):
            checks["path_metrics_available"] = all(
                math.isfinite(float(goal["path_stretch"]))
                and float(goal["reference_plan_length_m"]) > 0.05
                for goal in goals
            )
        if self.fault_sequence:
            checks["all_faults_injected_and_restored"] = (
                len(self.fault_results) == len(self.fault_sequence)
                and all(
                    bool(result["injected"]) and bool(result["restored"])
                    for result in self.fault_results
                )
            )
            checks["guard_blocked_each_fault"] = all(
                str(result["target_guard_state"])
                in set(result.get("guard_states", []))
                for result in self.fault_results
            )
            checks["commands_zero_during_faults"] = all(
                float(result["max_command_after_grace"])
                <= self.maximum_command_while_fault
                for result in self.fault_results
            )
        rates = {}
        for name, count in self.counts.items():
            if name not in self.first_wall or name not in self.last_wall:
                rates[name] = 0.0
                continue
            span = self.last_wall[name] - self.first_wall[name]
            rates[name] = count / span if count > 1 and span > 0.0 else 0.0
        latency_metrics = {
            name: {
                "sample_count": len(values),
                "p50_ms": percentile(values, 0.50),
                "p95_ms": percentile(values, 0.95),
                "maximum_ms": max(values, default=0.0),
            }
            for name, values in sorted(self.command_latency_ms.items())
        }
        data_age_metrics = {
            name: {
                "sample_count": len(values),
                "p50_ms": percentile(values, 0.50),
                "p95_ms": percentile(values, 0.95),
                "maximum_ms": max(values, default=0.0),
            }
            for name, values in sorted(self.data_age_ms.items())
        }
        return {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "goals": goals,
            "lifecycle_states": states,
            "message_counts": dict(self.counts),
            "observed_rates_hz": rates,
            "command_latency_metrics": latency_metrics,
            "data_age_metrics": data_age_metrics,
            "tf_edges": sorted([list(edge) for edge in self.tf_edges]),
            "scan_finite_ranges": self.scan_finite,
            "safety_scan_finite_ranges": self.safety_scan_finite,
            "depth_cloud_points": self.depth_cloud_points,
            "scan_stamp_regressions": self.scan_stamp_regressions,
            "map_cells": self.map_cells,
            "global_costmap_cells": self.global_costmap_cells,
            "local_costmap_cells": self.local_costmap_cells,
            "nvblox_slice_cells": self.nvblox_slice_cells,
            "dynamic_slice_cells": self.dynamic_slice_cells,
            "dynamic_points": self.dynamic_points,
            "dynamic_esdf_points": self.dynamic_esdf_points,
            "combined_esdf_points": self.combined_esdf_points,
            "surround_counts": dict(self.surround_counts),
            "vgl_camera_mode": (
                "four_way" if self.require_surround_cameras else "front_stereo"
            ),
            "recovery_states": sorted(self.recovery_states),
            "recovery_count": self.recovery_count,
            "resilient_states": sorted(self.resilient_states),
            "resilient_resume_count": self.resilient_resume_count,
            "force_relocalization_injected": self.force_injected,
            "fault_results": self.fault_results,
            "max_command_while_unready_after_grace": self.max_command_while_unready,
            "command_max": dict(self.command_max),
            "command_lateral_max": self.command_lateral_max,
            "guard_states": sorted(self.guard_states),
            "collision_actions": dict(self.collision_actions),
            "ground_truth_path_length_m": ground_truth_path_length,
            "experiment_class": self.experiment_class,
            "automation": {
                "goal_dispatch": "navigation_test_runner",
                "manual_intervention": False,
            },
            "goal_tolerances": {
                "position_m": self.goal_xy_tolerance,
                "yaw_deg": self.goal_yaw_tolerance_deg,
            },
            "node_names": node_names,
        }

    def write_traces(self) -> None:
        if self.trajectory_path is not None:
            self.trajectory_path.parent.mkdir(parents=True, exist_ok=True)
            with self.trajectory_path.open("w", newline="", encoding="utf-8") as stream:
                fields = [
                    "sim_time_s",
                    "goal_index",
                    "x_m",
                    "y_m",
                    "yaw_rad",
                    "linear_x_mps",
                    "angular_z_radps",
                ]
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self.ground_truth_trace)
        if self.command_trace_path is not None:
            self.command_trace_path.parent.mkdir(parents=True, exist_ok=True)
            with self.command_trace_path.open(
                "w", newline="", encoding="utf-8"
            ) as stream:
                fields = [
                    "wall_time_s",
                    "sim_time_s",
                    "goal_index",
                    "linear_x_mps",
                    "angular_z_radps",
                    "guard_state",
                ]
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self.command_trace)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationTestRunner()
    try:
        try:
            report = node.run()
        except Exception as error:
            # Preserve partial trajectories and an actionable machine-readable
            # report even when Nav2 aborts or a goal reaches its timeout.
            partial_checks = {
                "main_tf_chain_seen": ("map", "odom") in node.tf_edges
                and ("odom", "base_link") in node.tf_edges,
                "cuvslam_tracking": node.tracking_samples >= 20,
                "guard_became_active": "active" in node.guard_states,
            }
            report = {
                "status": "failed",
                "experiment_class": node.experiment_class,
                "automation": {
                    "goal_dispatch": "navigation_test_runner",
                    "manual_intervention": False,
                },
                "error": str(error),
                "error_type": type(error).__name__,
                "traceback": traceback.format_exc(),
                "checks": partial_checks,
                "message_counts": dict(node.counts),
                "tracking_samples": node.tracking_samples,
                "tf_edges": sorted([list(edge) for edge in node.tf_edges]),
                "guard_states": sorted(node.guard_states),
                "collision_actions": dict(node.collision_actions),
                "ground_truth_path_length_m": path_length(node.ground_truth),
            }
            node.write_traces()
            node.result_path.parent.mkdir(parents=True, exist_ok=True)
            node.result_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n"
            )
            raise
        node.write_traces()
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        node.result_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        if report["status"] != "passed":
            stage = (
                11
                if node.experiment_class.startswith("stage11")
                else 10
                if node.experiment_class.startswith("stage10")
                else 9
                if node.phase9_mode
                else 8
            )
            raise RuntimeError(
                f"Stage {stage} navigation acceptance failed: {report['checks']}"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
