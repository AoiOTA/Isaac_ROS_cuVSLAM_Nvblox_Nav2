from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package: str, launch_file: str) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            _include("nova_carter_control", "control.launch.py"),
            _include("nova_carter_bringup", "visual_slam.launch.py"),
        ]
    )
