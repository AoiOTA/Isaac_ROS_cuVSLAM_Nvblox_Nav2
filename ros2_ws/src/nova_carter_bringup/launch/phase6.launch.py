from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(
    package: str, launch_file: str, arguments: dict[str, object] | None = None
) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("front_image_width", default_value="1280"),
            DeclareLaunchArgument("front_image_height", default_value="800"),
            DeclareLaunchArgument("image_qos", default_value="SENSOR_DATA"),
            DeclareLaunchArgument("enable_surround_cameras", default_value="false"),
            _include(
                "nova_carter_bringup",
                "phase5.launch.py",
                {
                    "front_image_width": LaunchConfiguration("front_image_width"),
                    "front_image_height": LaunchConfiguration("front_image_height"),
                    "image_qos": LaunchConfiguration("image_qos"),
                    "enable_surround_cameras": LaunchConfiguration(
                        "enable_surround_cameras"
                    ),
                },
            ),
            _include("nova_carter_bringup", "nvblox.launch.py"),
        ]
    )
