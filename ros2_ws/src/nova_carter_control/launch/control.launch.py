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
                        )
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
