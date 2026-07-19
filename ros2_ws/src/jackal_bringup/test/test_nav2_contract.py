from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
BRINGUP = ROOT / "ros2_ws/src/jackal_bringup"


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
    controller_params = config["controller_server"]["ros__parameters"]
    assert controller_params["odom_topic"] == "/wheel/odometry"
    controller = controller_params["FollowPath"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "DiffDrive"
    assert controller["time_steps"] == 20
    assert controller["model_dt"] == 0.1
    assert controller["batch_size"] == 500
    assert controller["vx_max"] == 0.75
    assert controller["vx_min"] == 0.0
    assert controller["wz_max"] == 1.20
    assert controller["ax_max"] == 1.10
    assert controller["az_max"] == 3.00
    assert controller["temperature"] == 0.30
    assert controller["GoalCritic"]["cost_weight"] >= 5.0
    assert controller["PreferForwardCritic"]["enabled"] is True
    assert controller["PathAlignCritic"]["offset_from_furthest"] == 8
    assert controller["PathAlignCritic"]["max_path_occupancy_ratio"] == 0.40
    assert controller["PathFollowCritic"]["offset_from_furthest"] == 10
    assert controller["PathAngleCritic"]["offset_from_furthest"] == 8
    assert controller["PathAngleCritic"]["max_angle_to_furthest"] == 0.45
    progress = config["controller_server"]["ros__parameters"]["progress_checker"]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_angle"] > 0.0
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["allow_unknown"] is True


def test_nav2_launch_keeps_mppi_velocity_feedback_on_wheel_odometry() -> None:
    config = params()
    assert config["bt_navigator"]["ros__parameters"]["odom_topic"] == (
        "/wheel/odometry"
    )
    assert config["velocity_smoother"]["ros__parameters"]["odom_topic"] == (
        "/wheel/odometry"
    )
    launch = (BRINGUP / "launch/nav2.launch.py").read_text()
    assert '"odom_topic", default_value="/wheel/odometry"' in launch
    assert '"odom_topic", default_value="/visual_slam/tracking/odometry"' not in launch


def test_costmap_and_visual_safety_sources_are_wired() -> None:
    config = params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert local["plugins"] == ["nvblox_layer", "inflation_layer"]
    assert "static_layer" not in local
    assert local["nvblox_layer"]["plugin"] == "nvblox::nav2::NvbloxCostmapLayer"
    assert local["nvblox_layer"]["nvblox_map_slice_topic"] == "/nvblox_node/static_map_slice"
    assert "obstacle_layer" not in local
    assert local["footprint_padding"] == 0.005
    assert global_map["global_frame"] == "map"
    assert global_map["plugins"] == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" not in global_map
    assert global_map["footprint_padding"] == 0.005
    # Match the validated reference branch's costmap cadence and rolling size;
    # only the no-lidar observation plugin differs.
    assert local["update_frequency"] == 10.0
    assert local["publish_frequency"] == 5.0
    assert local["width"] == 4
    assert local["height"] == 4
    assert local["inflation_layer"]["inflation_radius"] == 0.40
    assert local["inflation_layer"]["cost_scaling_factor"] == 8.0
    assert global_map["update_frequency"] == 2.0
    assert global_map["publish_frequency"] == 1.0
    assert global_map["track_unknown_space"] is True


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
    from jackal_bringup.scan_timestamp_relay import set_stamp_ns, stamp_to_ns

    stamp = Time(sec=7, nanosec=970_000_000)
    assert stamp_to_ns(stamp) == 7_970_000_000
    set_stamp_ns(stamp, -1)
    assert (stamp.sec, stamp.nanosec) == (0, 0)


def test_depth_scan_point_rotation_uses_cuvslam_odom_transform() -> None:
    from geometry_msgs.msg import Quaternion
    from jackal_bringup.scan_timestamp_relay import rotate_vector

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
    assert collision["base_shift_correction"] is True
    assert collision["StopZone"]["action_type"] == "stop"
    assert collision["StopZone"]["min_points"] == 3
    assert collision["SlowdownZone"]["action_type"] == "slowdown"
    assert collision["SlowdownZone"]["min_points"] == 4
    assert collision["FootprintApproach"]["min_points"] == 3
    velocity = config["velocity_smoother"]["ros__parameters"]
    assert velocity["max_velocity"] == [0.75, 0.0, 1.20]
    assert velocity["min_velocity"][0] == 0.0
    assert velocity["max_accel"] == [1.10, 0.0, 3.00]
    assert velocity["scale_velocities"] is True
    assert velocity["smoothing_frequency"] == 20.0
    assert collision["SlowdownZone"]["slowdown_ratio"] == 0.85
    assert collision["source_timeout"] == 1.25


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
    assert "/goal_pose" in topics


def test_navigation_rviz_keeps_heavy_sensor_geometry_on_demand() -> None:
    rviz = yaml.safe_load((BRINGUP / "rviz/navigation.rviz").read_text())
    displays = rviz["Visualization Manager"]["Displays"]
    nvblox = next(display for display in displays if display.get("Name") == "Nvblox")
    assert all(not display["Enabled"] for display in nvblox["Displays"])
    sensors = next(
        display
        for display in displays
        if display.get("Name") == "Visual sensors and safety"
    )
    expensive = {
        "Front left image",
        "Front depth",
        "Left Hawk on demand",
        "Right Hawk on demand",
        "Back Hawk on demand",
        "Depth obstacles in odom",
    }
    assert all(
        not display["Enabled"]
        for display in sensors["Displays"]
        if display["Name"] in expensive
    )
    assert next(
        display
        for display in sensors["Displays"]
        if display["Name"] == "Depth safety scan"
    )["Enabled"]


def test_mapping_rviz_shows_geometry_without_default_camera_decoding() -> None:
    rviz = yaml.safe_load((BRINGUP / "rviz/mapping.rviz").read_text())
    topics = collect_values(rviz)
    assert {
        "/robot_description",
        "/visual_slam/tracking/slam_path",
        "/nvblox_node/mesh",
        "/nvblox_node/static_esdf_pointcloud",
        "/front_stereo_camera/left/image_raw",
        "/front_stereo_camera/depth/image_raw",
    } <= topics
    displays = rviz["Visualization Manager"]["Displays"]
    sensor_group = next(
        display
        for display in displays
        if display.get("Name") == "Sensor views (disabled by default)"
    )
    assert all(not display["Enabled"] for display in sensor_group["Displays"])


def test_phase8_launch_enables_health_gate_and_runtime_components() -> None:
    launch = (BRINGUP / "launch/phase8.launch.py").read_text()
    for token in (
        '"require_navigation_health": "true"',
        '"depth_timeout": LaunchConfiguration("depth_timeout")',
        '"source_timeout": LaunchConfiguration(',
        '"override_publishing_stamp": "true"',
        '"publish_map_to_odom_tf": "false"',
        '"nvblox.launch.py"',
        '"depth_scan.launch.py"',
        '"nav2.launch.py"',
        'executable="localization_bootstrap"',
        'executable="navigation_tf_bridge"',
        'executable="manual_goal_bridge"',
        '"anchor_pose_topic": "/vgl_pose_relay/pose"',
        '"anchor_ready_topic": "/localization/ready"',
        '"action_topic": "/navigate_to_pose"',
        'executable="rviz2"',
        "TimerAction",
        'DeclareLaunchArgument("nav2_start_delay", default_value="20.0")',
        'DeclareLaunchArgument("depth_timeout", default_value="1.25")',
        'DeclareLaunchArgument("source_timeout", default_value="1.25")',
    ):
        assert token in launch


def test_phase7_forwards_depth_health_timeout_to_command_guard() -> None:
    launch = (BRINGUP / "launch/phase7_localization.launch.py").read_text()
    assert 'DeclareLaunchArgument("depth_timeout", default_value="0.5")' in launch
    assert '"depth_timeout": LaunchConfiguration("depth_timeout")' in launch


def test_manual_gui_rviz_entrypoints_are_guarded() -> None:
    mapping = (ROOT / "scripts/run_mapping.sh").read_text()
    manual_mapping = (ROOT / "scripts/run_manual_mapping.sh").read_text()
    manual_navigation = (ROOT / "scripts/run_manual_navigation.sh").read_text()
    manual_check = (ROOT / "scripts/check_manual_navigation.sh").read_text()
    run_all = (ROOT / "scripts/run_all.sh").read_text()
    for token in (
        '--interactive --gui --rviz',
        'run_mapping.sh',
    ):
        assert token in manual_mapping
    assert 'rviz/mapping.rviz' in mapping
    assert 'RVIZ="true"' in mapping
    assert 'Map was not promoted' in mapping
    assert 'mapping-validation.json' in mapping
    assert '--manual --gui --rviz' in manual_navigation
    assert 'manual_navigation_ready' in manual_check
    assert 'fastdds-super-client.xml' in manual_check
    assert '/back_stereo_camera/left/image_raw' in manual_check
    assert 'manual_navigation_ready' in run_all
    assert 'fastdds discovery' in run_all
    assert 'unset ROS_LOCALHOST_ONLY' in run_all
    navigation_script = (ROOT / "scripts/run_navigation.sh").read_text()
    assert "CALLER_ROS_DISCOVERY_SERVER" in navigation_script
    assert 'manual navigation requires --gui' in run_all
    assert 'manual navigation requires --rviz' in run_all


def test_nav2_activation_is_staggered_after_map_server() -> None:
    launch = (BRINGUP / "launch/nav2.launch.py").read_text()
    assert "TimerAction" in launch
    assert "NAVIGATION_START_SCHEDULE" in launch
    assert "NAVIGATION_ACTIVATION_DELAY_S = 25.0" in launch
    assert "OpaqueFunction" in launch
    for delay in (6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0):
        assert f"({delay}," in launch
    assert '"bond_timeout": 15.0' in launch


def test_navigation_behavior_tree_and_recoveries_never_command_reverse() -> None:
    config = params()
    bt = (BRINGUP / "behavior_trees/navigate_forward_only.xml").read_text()
    behaviors = config["behavior_server"]["ros__parameters"]
    assert behaviors["behavior_plugins"] == ["spin", "wait"]
    assert "BackUp" not in bt
    assert "DriveOnHeading" not in bt
    assert "<Spin " in bt and "<Wait " in bt
    assert config["bt_navigator"]["ros__parameters"]["navigators"] == [
        "navigate_to_pose"
    ]
    launch = (BRINGUP / "launch/nav2.launch.py").read_text()
    assert "navigate_forward_only.xml" in launch


def test_navigation_tf_bridge_composes_map_and_odom_poses() -> None:
    from geometry_msgs.msg import Pose, Quaternion
    from jackal_bringup.navigation_tf_bridge import map_to_odom_transform

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
