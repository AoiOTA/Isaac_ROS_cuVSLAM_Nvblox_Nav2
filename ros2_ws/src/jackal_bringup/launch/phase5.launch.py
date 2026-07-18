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
    share = Path(get_package_share_directory("jackal_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("front_image_width", default_value="1280"),
            DeclareLaunchArgument("front_image_height", default_value="800"),
            DeclareLaunchArgument("image_qos", default_value="SENSOR_DATA"),
            DeclareLaunchArgument("camera_profile", default_value="navigation_6cam"),
            DeclareLaunchArgument(
                "visual_slam_params",
                default_value=str(share / "config/visual_slam_navigation_6cam.yaml"),
            ),
            _include("jackal_control", "control.launch.py"),
            _include(
                "jackal_bringup",
                "visual_slam.launch.py",
                {
                    "front_image_width": LaunchConfiguration("front_image_width"),
                    "front_image_height": LaunchConfiguration("front_image_height"),
                    "image_qos": LaunchConfiguration("image_qos"),
                    "camera_profile": LaunchConfiguration("camera_profile"),
                    "visual_slam_params": LaunchConfiguration("visual_slam_params"),
                },
            ),
        ]
    )
