from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(name: str, arguments: dict[str, object] | None = None):
    share = Path(get_package_share_directory("jackal_bringup"))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / name)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("jackal_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            DeclareLaunchArgument(
                "camera_profile",
                default_value="navigation_6cam",
                choices=["navigation_6cam"],
            ),
            DeclareLaunchArgument("image_qos", default_value="DEFAULT"),
            DeclareLaunchArgument(
                "visual_slam_params",
                default_value=str(share / "config/visual_slam_navigation_6cam.yaml"),
            ),
            DeclareLaunchArgument(
                "vgl_params",
                default_value=str(share / "config/vgl_navigation_6cam.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params", default_value=str(share / "config/nav2.yaml")
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("nvblox_start_delay", default_value="3.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="20.0"),
            # The front depth stream is 10 Hz in simulation, but under the
            # measured GUI + RViz GPU load callback gaps can approach 1.0 s.
            DeclareLaunchArgument("depth_timeout", default_value="1.25"),
            DeclareLaunchArgument("source_timeout", default_value="1.50"),
            DeclareLaunchArgument(
                "rviz_config", default_value=str(share / "rviz/navigation.rviz")
            ),
            include(
                "phase7_localization.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                    "require_navigation_health": "true",
                    "depth_timeout": LaunchConfiguration("depth_timeout"),
                    "override_publishing_stamp": "true",
                    "publish_map_to_odom_tf": "false",
                    "camera_profile": LaunchConfiguration("camera_profile"),
                    "image_qos": LaunchConfiguration("image_qos"),
                    "visual_slam_params": LaunchConfiguration(
                        "visual_slam_params"
                    ),
                    "vgl_params": LaunchConfiguration("vgl_params"),
                },
            ),
            include("depth_scan.launch.py"),
            # cuVGL, cuVSLAM and nvblox allocate substantial GPU resources at
            # startup. Staggering them keeps lifecycle services responsive.
            TimerAction(
                period=LaunchConfiguration("nvblox_start_delay"),
                actions=[include("nvblox.launch.py")],
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay"),
                actions=[
                    include(
                        "nav2.launch.py",
                        {
                            "map": LaunchConfiguration("map"),
                            "params_file": LaunchConfiguration("nav2_params"),
                            "source_timeout": LaunchConfiguration(
                                "source_timeout"
                            ),
                        },
                    )
                ],
            ),
            Node(
                package="jackal_bringup",
                executable="navigation_tf_bridge",
                name="navigation_tf_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_rate_hz": 30.0,
                        "max_source_age_s": 1.0,
                        # cuVGL supplies the global map pose.  Anchor it once
                        # cuVSLAM has accepted the initial pose and recovered
                        # stable tracking; no RViz 2D Pose Estimate is needed.
                        "anchor_pose_topic": "/vgl_pose_relay/pose",
                        "anchor_ready_topic": "/localization/ready",
                        "odometry_topic": "/visual_slam/tracking/odometry",
                    }
                ],
            ),
            Node(
                package="jackal_experiments",
                executable="localization_bootstrap",
                name="localization_bootstrap",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "max_trigger_attempts": 60,
                        "trigger_period_s": 2.0,
                    }
                ],
            ),
            Node(
                package="jackal_experiments",
                executable="manual_goal_bridge",
                name="manual_goal_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "goal_topic": "/goal_pose",
                        "action_topic": "/navigate_to_pose",
                        "required_frame": "map",
                        "ready_topic": "/localization/ready",
                        "require_ready": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
