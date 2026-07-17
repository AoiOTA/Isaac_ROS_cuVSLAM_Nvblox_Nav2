from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package: str, name: str, arguments: dict[str, object] | None = None):
    path = Path(get_package_share_directory(package)) / "launch" / name
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            include("nova_carter_control", "control.launch.py"),
            include(
                "nova_carter_bringup",
                "visual_slam.launch.py",
                {
                    "load_map_folder_path": LaunchConfiguration("cuvslam_map_dir"),
                    "localize_on_startup": "false",
                },
            ),
            include(
                "nova_carter_bringup",
                "vgl.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                },
            ),
        ]
    )
