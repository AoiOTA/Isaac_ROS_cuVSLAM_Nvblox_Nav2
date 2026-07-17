from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    node = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        name="nvblox_node",
        parameters=[
            str(share / "config/nvblox.yaml"),
            str(share / "config/nvblox_dynamic.yaml"),
        ],
        remappings=[
            ("camera_0/depth/image", "/front_stereo_camera/depth/image_raw"),
            ("camera_0/depth/camera_info", "/front_stereo_camera/depth/camera_info"),
            ("camera_0/color/image", "/front_stereo_camera/left/image_raw_rgb"),
            ("camera_0/color/camera_info", "/front_stereo_camera/left/camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="nvblox_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[node],
            )
        ]
    )
