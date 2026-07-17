from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", str(share / "urdf/nova_carter.urdf.xacro")]
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            ),
            ComposableNodeContainer(
                name="front_stereo_image_pipeline",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="isaac_ros_image_proc",
                        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                        name="left_image_normalizer",
                        parameters=[
                            {
                                "encoding_desired": "mono8",
                                "image_width": 1280,
                                "image_height": 800,
                                "input_qos": "SENSOR_DATA",
                                "output_qos": "SENSOR_DATA",
                            }
                        ],
                        remappings=[
                            ("image_raw", "/front_stereo_camera/left/image_raw_rgb"),
                            ("image", "/front_stereo_camera/left/image_raw"),
                        ],
                    ),
                    ComposableNode(
                        package="isaac_ros_image_proc",
                        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                        name="right_image_normalizer",
                        parameters=[
                            {
                                "encoding_desired": "mono8",
                                "image_width": 1280,
                                "image_height": 800,
                                "input_qos": "SENSOR_DATA",
                                "output_qos": "SENSOR_DATA",
                            }
                        ],
                        remappings=[
                            ("image_raw", "/front_stereo_camera/right/image_raw_rgb"),
                            ("image", "/front_stereo_camera/right/image_raw"),
                        ],
                    ),
                ],
            ),
        ]
    )
