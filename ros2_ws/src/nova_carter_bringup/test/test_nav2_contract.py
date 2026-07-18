from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
BRINGUP = ROOT / "ros2_ws/src/nova_carter_bringup"


def params() -> dict:
    return yaml.safe_load((BRINGUP / "config/nav2.yaml").read_text())


def collect_values(value: object) -> set[str]:
    found = set()
    if isinstance(value, dict):
        for key, child in value.items():
            if key == "Value" and isinstance(child, str) and child.startswith("/"):
                found.add(child)
            found.update(collect_values(child))
    elif isinstance(value, list):
        for child in value:
            found.update(collect_values(child))
    return found


def test_nav2_uses_diff_drive_mppi_and_smac_2d() -> None:
    config = params()
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "DiffDrive"
    assert controller["vx_max"] == 1.10
    assert controller["wz_max"] == 1.40
    assert controller["ax_max"] == 1.60
    assert controller["az_max"] == 3.20
    assert controller["temperature"] <= 0.2
    assert controller["GoalCritic"]["cost_weight"] >= 10.0
    assert controller["VelocityDeadbandCritic"]["deadband_velocities"] == [0.08, 0.0, 0.08]
    progress = config["controller_server"]["ros__parameters"]["progress_checker"]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_angle"] > 0.0
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["allow_unknown"] is False


def test_costmap_and_visual_safety_sources_are_wired() -> None:
    config = params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert local["plugins"] == ["nvblox_layer", "obstacle_layer", "inflation_layer"]
    assert local["nvblox_layer"]["plugin"] == "nvblox::nav2::NvbloxCostmapLayer"
    assert local["nvblox_layer"]["nvblox_map_slice_topic"] == "/nvblox_node/static_map_slice"
    assert local["obstacle_layer"]["depth_scan"]["topic"] == "/front_depth/scan"
    assert local["obstacle_layer"]["depth_scan"]["data_type"] == "LaserScan"
    assert global_map["global_frame"] == "map"
    assert global_map["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    assert global_map["obstacle_layer"]["depth_scan"]["topic"] == "/front_depth/scan"
    assert local["update_frequency"] == 12.0
    assert global_map["update_frequency"] == 5.0


def test_scan_is_shifted_behind_visual_slam_tf_before_safety_consumers() -> None:
    launch = (BRINGUP / "launch/depth_scan.launch.py").read_text()
    assert '("scan", "/front_depth/scan_raw")' in launch
    assert '"output_topic": "/front_depth/scan"' in launch
    assert '"cloud_output_topic": "/front_depth/points_odom"' in launch
    assert '"tf_parent_frame": "odom"' in launch
    assert '"tf_child_frame": "base_link"' in launch
    assert '"tf_release_delay_s": 0.02' in launch


def test_scan_stamp_helpers_handle_second_boundary_and_startup() -> None:
    from builtin_interfaces.msg import Time
    from nova_carter_bringup.scan_timestamp_relay import set_stamp_ns, stamp_to_ns

    stamp = Time(sec=7, nanosec=970_000_000)
    assert stamp_to_ns(stamp) == 7_970_000_000
    set_stamp_ns(stamp, -1)
    assert (stamp.sec, stamp.nanosec) == (0, 0)


def test_depth_scan_point_rotation_uses_cuvslam_odom_transform() -> None:
    from geometry_msgs.msg import Quaternion
    from nova_carter_bringup.scan_timestamp_relay import rotate_vector

    half = 2.0**-0.5
    x, y, z = rotate_vector(Quaternion(z=half, w=half), 1.0, 0.0, 0.0)
    assert abs(x) < 1.0e-6
    assert abs(y - 1.0) < 1.0e-6
    assert abs(z) < 1.0e-6


def test_command_chain_and_collision_zones_are_fixed() -> None:
    config = params()
    collision = config["collision_monitor"]["ros__parameters"]
    assert collision["cmd_vel_in_topic"] == "/cmd_vel_smoothed"
    assert collision["cmd_vel_out_topic"] == "/cmd_vel_safe"
    assert collision["scan"]["topic"] == "/front_depth/scan_raw"
    assert collision["base_shift_correction"] is False
    assert collision["StopZone"]["action_type"] == "stop"
    assert collision["SlowdownZone"]["action_type"] == "slowdown"
    velocity = config["velocity_smoother"]["ros__parameters"]
    assert velocity["max_velocity"] == [1.10, 0.0, 1.40]
    assert velocity["max_accel"] == [1.40, 0.0, 2.80]
    assert collision["SlowdownZone"]["slowdown_ratio"] == 0.65
    assert collision["source_timeout"] == 0.75


def test_rviz_contains_every_stage8_display_source() -> None:
    rviz = yaml.safe_load((BRINGUP / "rviz/navigation.rviz").read_text())
    topics = collect_values(rviz)
    required = {
        "/robot_description",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/plan",
        "/transformed_global_plan",
        "/visual_slam/tracking/vo_path",
        "/nvblox_node/mesh",
        "/nvblox_node/static_esdf_pointcloud",
        "/front_stereo_camera/left/image_raw",
        "/front_stereo_camera/depth/image_raw",
        "/front_depth/scan",
        "/front_depth/points_odom",
        "/collision_monitor/stop_zone",
    }
    assert required <= topics


def test_phase8_launch_enables_health_gate_and_runtime_components() -> None:
    launch = (BRINGUP / "launch/phase8.launch.py").read_text()
    for token in (
        '"require_navigation_health": "true"',
        '"override_publishing_stamp": "true"',
        '"publish_map_to_odom_tf": "false"',
        '"nvblox.launch.py"',
        '"depth_scan.launch.py"',
        '"nav2.launch.py"',
        'executable="localization_bootstrap"',
        'executable="navigation_tf_bridge"',
        'executable="rviz2"',
        "TimerAction",
        'DeclareLaunchArgument("nav2_start_delay", default_value="12.0")',
    ):
        assert token in launch


def test_nav2_activation_is_staggered_after_map_server() -> None:
    launch = (BRINGUP / "launch/nav2.launch.py").read_text()
    assert "TimerAction" in launch
    assert "period=8.0" in launch
    assert '"bond_timeout": 15.0' in launch


def test_navigation_tf_bridge_composes_map_and_odom_poses() -> None:
    from geometry_msgs.msg import Pose, Quaternion
    from nova_carter_bringup.navigation_tf_bridge import map_to_odom_transform

    map_pose = Pose()
    map_pose.position.x = 5.0
    map_pose.position.y = 3.0
    map_pose.orientation = Quaternion(w=1.0)
    odom_pose = Pose()
    odom_pose.position.x = 2.0
    odom_pose.position.y = 1.0
    odom_pose.orientation = Quaternion(w=1.0)
    translation, rotation = map_to_odom_transform(map_pose, odom_pose)
    assert translation == (3.0, 2.0, 0.0)
    assert (rotation.x, rotation.y, rotation.z, rotation.w) == (0.0, 0.0, 0.0, 1.0)
