from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("jackal_bringup"))
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="depth_scan_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depthimage_to_laserscan",
                        plugin="depthimage_to_laserscan::DepthImageToLaserScanROS",
                        name="depthimage_to_laserscan",
                        parameters=[str(share / "config/depth_to_scan.yaml")],
                        remappings=[
                            ("depth", "/front_stereo_camera/depth/image_raw"),
                            (
                                "depth_camera_info",
                                "/front_stereo_camera/depth/camera_info",
                            ),
                            ("scan", "/front_depth/scan_raw"),
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
            ),
            Node(
                package="jackal_bringup",
                executable="scan_timestamp_relay",
                name="scan_timestamp_relay",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/front_depth/scan_raw",
                        "output_topic": "/front_depth/scan",
                        "cloud_output_topic": "/front_depth/points_odom",
                        "tf_parent_frame": "odom",
                        "tf_child_frame": "base_link",
                        "tf_release_delay_s": 0.02,
                        "max_scan_age_s": 0.5,
                    }
                ],
            ),
        ]
    )
