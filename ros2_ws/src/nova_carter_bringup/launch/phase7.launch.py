from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(name: str, arguments: dict[str, object] | None = None) -> IncludeLaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / name)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            _include(
                "phase7_localization.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                },
            ),
            _include("nvblox.launch.py"),
        ]
    )
