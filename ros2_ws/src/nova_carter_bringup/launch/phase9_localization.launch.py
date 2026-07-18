from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package: str, name: str, arguments: dict[str, object] | None = None):
    path = Path(get_package_share_directory(package)) / "launch" / name
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            DeclareLaunchArgument(
                "vgl_params", default_value=str(share / "config/vgl.yaml")
            ),
            DeclareLaunchArgument("visual_slam_timeout", default_value="2.0"),
            DeclareLaunchArgument("depth_timeout", default_value="2.0"),
            DeclareLaunchArgument("map_slice_timeout", default_value="2.0"),
            DeclareLaunchArgument("enable_fault_injection", default_value="false"),
            include(
                "nova_carter_control",
                "control.launch.py",
                {
                    "require_navigation_health": "true",
                    "map_slice_topic": "/nvblox_node/combined_map_slice",
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
            include(
                "nova_carter_bringup",
                "visual_slam.launch.py",
                {
                    "load_map_folder_path": LaunchConfiguration("cuvslam_map_dir"),
                    "visual_slam_params": str(
                        share / "config/visual_slam_phase9.yaml"
                    ),
                    "localize_on_startup": "false",
                    "override_publishing_stamp": "true",
                    "publish_map_to_odom_tf": "false",
                    "publish_odom_to_base_tf": "false",
                    # Stage 9 deliberately uses only the validated front Hawk
                    # pair. The optional four-way assets remain available for
                    # later experiments without burdening dynamic navigation.
                    "front_image_width": "1280",
                    "front_image_height": "800",
                    "image_qos": "DEFAULT",
                },
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="visual_wheel_ekf",
                output="screen",
                parameters=[str(share / "config/visual_wheel_ekf.yaml")],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            include(
                "nova_carter_bringup",
                "vgl.launch.py",
                {
                    "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
                    "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
                    "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
                    "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
                    "vgl_params": LaunchConfiguration("vgl_params"),
                    "odometry_topic": "/odometry/filtered",
                },
            ),
            Node(
                package="nova_carter_experiments",
                executable="localization_recovery_manager",
                name="localization_recovery_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_warmup_s": 1.5,
                        "obstruction_clearance_s": 5.0,
                        "retry_period_s": 3.0,
                        "vgl_pose_timeout_s": 20.0,
                        "tracking_timeout_s": 15.0,
                        "status_timeout_s": 3.0,
                        "enable_surround_cameras": False,
                    }
                ],
            ),
            Node(
                package="nova_carter_experiments",
                executable="resilient_navigation",
                name="resilient_navigation",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
