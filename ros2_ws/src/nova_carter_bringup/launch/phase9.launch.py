from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(name: str, arguments: dict[str, object] | None = None):
    share = Path(get_package_share_directory("nova_carter_bringup"))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / name)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            DeclareLaunchArgument(
                "nav2_params", default_value=str(share / "config/nav2.yaml")
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("nvblox_start_delay", default_value="15.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="25.0"),
            DeclareLaunchArgument("visual_slam_timeout", default_value="2.0"),
            DeclareLaunchArgument("depth_timeout", default_value="2.0"),
            DeclareLaunchArgument("map_slice_timeout", default_value="2.0"),
            DeclareLaunchArgument("enable_fault_injection", default_value="false"),
            DeclareLaunchArgument(
                "nvblox_dynamic_params",
                default_value=str(share / "config/nvblox_dynamic.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config", default_value=str(share / "rviz/navigation.rviz")
            ),
            include(
                "phase9_localization.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                    "visual_slam_timeout": LaunchConfiguration(
                        "visual_slam_timeout"
                    ),
                    "depth_timeout": LaunchConfiguration("depth_timeout"),
                    "map_slice_timeout": LaunchConfiguration(
                        "map_slice_timeout"
                    ),
                    "enable_fault_injection": LaunchConfiguration(
                        "enable_fault_injection"
                    ),
                },
            ),
            include("depth_scan.launch.py"),
            TimerAction(
                period=LaunchConfiguration("nvblox_start_delay"),
                actions=[
                    include(
                        "nvblox_dynamic.launch.py",
                        {
                            "dynamic_params": LaunchConfiguration(
                                "nvblox_dynamic_params"
                            )
                        },
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay"),
                actions=[
                    include(
                        "nav2.launch.py",
                        {
                            "map": LaunchConfiguration("map"),
                            "params_file": LaunchConfiguration("nav2_params"),
                            "nvblox_map_slice_topic": (
                                "/nvblox_node/combined_map_slice"
                            ),
                            "odom_topic": "/odometry/filtered",
                            "movement_time_allowance": "60.0",
                            "source_timeout": "1.50",
                        },
                    )
                ],
            ),
            Node(
                package="nova_carter_bringup",
                executable="navigation_tf_bridge",
                name="navigation_tf_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_rate_hz": 30.0,
                        "max_source_age_s": 1.0,
                        "anchor_pose_topic": "/vgl_pose_relay/pose",
                        "anchor_ready_topic": "/localization/ready",
                        "odometry_topic": "/odometry/filtered",
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
