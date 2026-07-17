from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def normalizer(
    name: str,
    source: str,
    destination: str,
    width: int = 1280,
    height: int = 800,
    *,
    optional_surround: bool = False,
) -> ComposableNode:
    return ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
        name=name,
        parameters=[
            {
                "encoding_desired": "mono8",
                "image_width": width,
                "image_height": height,
                "input_qos": "SENSOR_DATA",
                "output_qos": "SENSOR_DATA",
            }
        ],
        remappings=[("image_raw", source), ("image", destination)],
        condition=(
            IfCondition(LaunchConfiguration("enable_surround_cameras"))
            if optional_surround
            else None
        ),
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", str(share / "urdf/nova_carter.urdf.xacro")]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_surround_cameras", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            ),
            ComposableNodeContainer(
                name="stereo_image_pipeline",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    normalizer(
                        "front_left_image_normalizer",
                        "/front_stereo_camera/left/image_raw_rgb",
                        "/front_stereo_camera/left/image_raw",
                    ),
                    normalizer(
                        "front_right_image_normalizer",
                        "/front_stereo_camera/right/image_raw_rgb",
                        "/front_stereo_camera/right/image_raw",
                    ),
                    normalizer(
                        "left_left_image_normalizer",
                        "/left_stereo_camera/left/image_raw_rgb",
                        "/left_stereo_camera/left/image_raw",
                        optional_surround=True,
                    ),
                    normalizer(
                        "left_right_image_normalizer",
                        "/left_stereo_camera/right/image_raw_rgb",
                        "/left_stereo_camera/right/image_raw",
                        optional_surround=True,
                    ),
                    normalizer(
                        "right_left_image_normalizer",
                        "/right_stereo_camera/left/image_raw_rgb",
                        "/right_stereo_camera/left/image_raw",
                        optional_surround=True,
                    ),
                    normalizer(
                        "right_right_image_normalizer",
                        "/right_stereo_camera/right/image_raw_rgb",
                        "/right_stereo_camera/right/image_raw",
                        optional_surround=True,
                    ),
                    normalizer(
                        "back_left_image_normalizer",
                        "/back_stereo_camera/left/image_raw_rgb",
                        "/back_stereo_camera/left/image_raw",
                        optional_surround=True,
                    ),
                    normalizer(
                        "back_right_image_normalizer",
                        "/back_stereo_camera/right/image_raw_rgb",
                        "/back_stereo_camera/right/image_raw",
                        optional_surround=True,
                    ),
                ],
            ),
        ]
    )
