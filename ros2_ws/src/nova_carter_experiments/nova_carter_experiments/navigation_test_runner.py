"""Run Stage 8/9 navigation goals and verify every critical data-flow edge."""

from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path
import time

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
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_from_quaternion(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


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
        self.counts: defaultdict[str, int] = defaultdict(int)
        self.first_wall: dict[str, float] = {}
        self.last_wall: dict[str, float] = {}
        self.localization_ready = False
        self.tracking_samples = 0
        self.tf_edges: set[tuple[str, str]] = set()
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
        self.command_max = defaultdict(float)
        self.command_latest: dict[str, tuple[float, float]] = {}
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
        if int(message.vo_state) == 1:
            self.tracking_samples += 1

    def on_tf(self, message: TFMessage) -> None:
        self.record("tf")
        for transform in message.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))

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
        self.safety_scan_finite += sum(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )

    def on_nvblox_slice(self, message: DistanceMapSlice) -> None:
        self.record("nvblox_slice")
        expected = int(message.width) * int(message.height)
        if expected > 0 and len(message.data) == expected:
            self.nvblox_slice_cells = max(self.nvblox_slice_cells, expected)

    def on_dynamic_slice(self, message: DistanceMapSlice) -> None:
        self.record("dynamic_slice")
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
        if message.header.frame_id == "odom":
            self.depth_cloud_points = max(self.depth_cloud_points, int(message.width))

    def on_path(self, name: str, message: NavPath) -> None:
        if message.poses:
            self.record(name)

    def on_command(self, name: str, message: Twist) -> None:
        self.record(name)
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

    def on_guard_status(self, message: String) -> None:
        self.record("guard_status")
        try:
            self.latest_guard_state = str(json.loads(message.data)["state"])
            self.guard_states.add(self.latest_guard_state)
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

    def execute_goal(self, x: float, y: float, heading: float) -> dict[str, object]:
        self.latest_feedback_pose = None
        self.active_goal = (x, y, heading)
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
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
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
        wrapped = result_future.result()
        if wrapped is None:
            handle.cancel_goal_async()
            raise TimeoutError(f"NavigateToPose timed out for {(x, y, heading)}")
        pose = self.latest_feedback_pose.pose if self.latest_feedback_pose else None
        xy_error = math.hypot(pose.position.x - x, pose.position.y - y) if pose else math.inf
        yaw_error = (
            abs(angle_difference(yaw_from_quaternion(pose.orientation), heading))
            if pose
            else math.inf
        )
        self.get_logger().info(
            f"NavigateToPose result goal={self.active_goal} status={wrapped.status} "
            f"error_code={wrapped.result.error_code} xy_error={xy_error:.3f} "
            f"yaw_error_deg={math.degrees(yaw_error):.2f}"
        )
        return {
            "requested": [x, y, heading],
            "status": int(wrapped.status),
            "error_code": int(wrapped.result.error_code),
            "error_msg": wrapped.result.error_msg,
            "xy_error_m": xy_error,
            "yaw_error_deg": math.degrees(yaw_error),
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
        goals = [self.execute_goal(*goal) for goal in self.goals]
        # Allow the zero command to propagate through every safety stage after completion.
        stop_deadline = time.monotonic() + (2.0 if self.phase9_mode else 1.0)
        while time.monotonic() < stop_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        path_length = sum(
            math.dist(a, b) for a, b in zip(self.ground_truth, self.ground_truth[1:])
        )
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
                and goal["xy_error_m"] <= 0.25
                and goal["yaw_error_deg"] <= 12.0
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
            "ground_truth_motion": path_length >= 3.0,
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
                    "dynamic_slice_nonempty": self.dynamic_slice_cells > 0,
                    "dynamic_esdf_nonempty": self.dynamic_esdf_points > 0,
                    "combined_esdf_nonempty": self.combined_esdf_points > 0,
                    "guard_stopped_during_relocalization": (
                        self.max_command_while_unready <= 0.02
                    ),
                }
            )
        rates = {}
        for name, count in self.counts.items():
            if name not in self.first_wall or name not in self.last_wall:
                rates[name] = 0.0
                continue
            span = self.last_wall[name] - self.first_wall[name]
            rates[name] = count / span if count > 1 and span > 0.0 else 0.0
        return {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "goals": goals,
            "lifecycle_states": states,
            "message_counts": dict(self.counts),
            "observed_rates_hz": rates,
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
            "max_command_while_unready_after_grace": self.max_command_while_unready,
            "command_max": dict(self.command_max),
            "command_lateral_max": self.command_lateral_max,
            "guard_states": sorted(self.guard_states),
            "collision_actions": dict(self.collision_actions),
            "ground_truth_path_length_m": path_length,
            "node_names": node_names,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationTestRunner()
    try:
        report = node.run()
        node.result_path.parent.mkdir(parents=True, exist_ok=True)
        node.result_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        if report["status"] != "passed":
            stage = 9 if node.phase9_mode else 8
            raise RuntimeError(
                f"Stage {stage} navigation acceptance failed: {report['checks']}"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
