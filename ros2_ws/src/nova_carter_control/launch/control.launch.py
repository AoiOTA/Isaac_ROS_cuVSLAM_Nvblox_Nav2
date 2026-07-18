from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("nova_carter_control")) / "config/control_params.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("require_navigation_health", default_value="false"),
            DeclareLaunchArgument(
                "map_slice_topic", default_value="/nvblox_node/static_map_slice"
            ),
            DeclareLaunchArgument("visual_slam_timeout", default_value="1.0"),
            DeclareLaunchArgument("depth_timeout", default_value="0.5"),
            DeclareLaunchArgument("map_slice_timeout", default_value="1.0"),
            DeclareLaunchArgument("enable_fault_injection", default_value="false"),
            Node(
                package="nova_carter_control",
                executable="command_guard",
                name="command_guard",
                output="screen",
                parameters=[
                    str(config),
                    {
                        "require_navigation_health": LaunchConfiguration(
                            "require_navigation_health"
                        ),
                        "map_slice_topic": LaunchConfiguration("map_slice_topic"),
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
                ],
            ),
            Node(
                package="nova_carter_control",
                executable="wheel_odometry",
                name="wheel_odometry",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
