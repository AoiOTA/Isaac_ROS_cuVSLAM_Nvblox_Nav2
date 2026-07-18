from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("jackal_bringup"))
    config = LaunchConfiguration("nvblox_config")
    nvblox = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        name="nvblox_node",
        parameters=[config],
        remappings=[
            ("camera_0/depth/image", "/front_stereo_camera/depth/image_raw"),
            (
                "camera_0/depth/camera_info",
                "/front_stereo_camera/depth/camera_info",
            ),
            ("camera_0/color/image", "/front_stereo_camera/left/image_raw_rgb"),
            (
                "camera_0/color/camera_info",
                "/front_stereo_camera/left/camera_info",
            ),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "nvblox_config",
                default_value=str(share / "config/nvblox.yaml"),
                description="Absolute path to the nvblox parameter file.",
            ),
            ComposableNodeContainer(
                name="nvblox_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[nvblox],
            ),
        ]
    )
