from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    vgl = ComposableNode(
        package="isaac_ros_visual_global_localization",
        plugin=(
            "nvidia::isaac_ros::visual_global_localization::"
            "VisualGlobalLocalizationNode"
        ),
        name="visual_global_localization_node",
        parameters=[
            str(share / "config/vgl.yaml"),
            {
                "map_dir": LaunchConfiguration("vgl_map_dir"),
                "config_dir": LaunchConfiguration("vgl_config_dir"),
                "model_dir": LaunchConfiguration("vgl_model_dir"),
            },
        ],
        remappings=[
            ("visual_localization/image_0", "/front_stereo_camera/left/image_raw"),
            (
                "visual_localization/camera_info_0",
                "/front_stereo_camera/left/camera_info",
            ),
            ("visual_localization/image_1", "/front_stereo_camera/right/image_raw"),
            (
                "visual_localization/camera_info_1",
                "/front_stereo_camera/right/camera_info",
            ),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument(
                "vgl_config_dir",
                default_value=(
                    "/opt/ros/jazzy/share/isaac_ros_visual_mapping/configs/isaac"
                ),
            ),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            ComposableNodeContainer(
                name="vgl_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[vgl],
            ),
            Node(
                package="nova_carter_experiments",
                executable="vgl_pose_relay",
                name="vgl_pose_relay",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir")},
                ],
            ),
        ]
    )
