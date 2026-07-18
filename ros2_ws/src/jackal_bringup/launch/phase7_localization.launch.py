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
    share = Path(get_package_share_directory("jackal_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            DeclareLaunchArgument("require_navigation_health", default_value="false"),
            DeclareLaunchArgument("camera_profile", default_value="navigation_6cam"),
            DeclareLaunchArgument(
                "visual_slam_params",
                default_value=str(share / "config/visual_slam_navigation_6cam.yaml"),
            ),
            DeclareLaunchArgument(
                "vgl_params",
                default_value=str(share / "config/vgl_navigation_6cam.yaml"),
            ),
            DeclareLaunchArgument(
                "override_publishing_stamp", default_value="false"
            ),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            include(
                "jackal_control",
                "control.launch.py",
                {
                    "require_navigation_health": LaunchConfiguration(
                        "require_navigation_health"
                    )
                },
            ),
            include(
                "jackal_bringup",
                "visual_slam.launch.py",
                {
                    "load_map_folder_path": LaunchConfiguration("cuvslam_map_dir"),
                    "localize_on_startup": "false",
                    "override_publishing_stamp": LaunchConfiguration(
                        "override_publishing_stamp"
                    ),
                    "publish_map_to_odom_tf": LaunchConfiguration(
                        "publish_map_to_odom_tf"
                    ),
                    "camera_profile": LaunchConfiguration("camera_profile"),
                    "visual_slam_params": LaunchConfiguration("visual_slam_params"),
                },
            ),
            include(
                "jackal_bringup",
                "vgl.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                    "vgl_params": LaunchConfiguration("vgl_params"),
                },
            ),
        ]
    )
