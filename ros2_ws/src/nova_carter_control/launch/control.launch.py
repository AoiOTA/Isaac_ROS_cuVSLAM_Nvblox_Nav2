from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("nova_carter_control")) / "config/control_params.yaml"
    return LaunchDescription(
        [
            Node(
                package="nova_carter_control",
                executable="command_guard",
                name="command_guard",
                output="screen",
                parameters=[str(config)],
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
