from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _mono_converter(
    name: str,
    camera: str,
    side: str,
    width: object,
    height: object,
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
                "input_qos": LaunchConfiguration("image_qos"),
                "output_qos": LaunchConfiguration("image_qos"),
            }
        ],
        remappings=[
            ("image_raw", f"/{camera}_stereo_camera/{side}/image_raw_rgb"),
            ("image", f"/{camera}_stereo_camera/{side}/image_raw"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
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
    visual_slam = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam_node",
        parameters=[
            LaunchConfiguration("visual_slam_params"),
            {
                "load_map_folder_path": LaunchConfiguration("load_map_folder_path"),
                "localize_on_startup": LaunchConfiguration("localize_on_startup"),
                "override_publishing_stamp": LaunchConfiguration(
                    "override_publishing_stamp"
                ),
                "publish_map_to_odom_tf": LaunchConfiguration(
                    "publish_map_to_odom_tf"
                ),
                "publish_odom_to_base_tf": LaunchConfiguration(
                    "publish_odom_to_base_tf"
                ),
            },
        ],
        remappings=[
            ("/visual_slam/image_0", "/front_stereo_camera/left/image_raw"),
            ("/visual_slam/camera_info_0", "/front_stereo_camera/left/camera_info"),
            ("/visual_slam/image_1", "/front_stereo_camera/right/image_raw"),
            ("/visual_slam/camera_info_1", "/front_stereo_camera/right/camera_info"),
            ("/visual_slam/image_2", "/left_stereo_camera/left/image_raw"),
            ("/visual_slam/camera_info_2", "/left_stereo_camera/left/camera_info"),
            ("/visual_slam/image_3", "/left_stereo_camera/right/image_raw"),
            ("/visual_slam/camera_info_3", "/left_stereo_camera/right/camera_info"),
            ("/visual_slam/image_4", "/right_stereo_camera/left/image_raw"),
            ("/visual_slam/camera_info_4", "/right_stereo_camera/left/camera_info"),
            ("/visual_slam/image_5", "/right_stereo_camera/right/image_raw"),
            ("/visual_slam/camera_info_5", "/right_stereo_camera/right/camera_info"),
            ("/visual_slam/image_6", "/back_stereo_camera/left/image_raw"),
            ("/visual_slam/camera_info_6", "/back_stereo_camera/left/camera_info"),
            ("/visual_slam/image_7", "/back_stereo_camera/right/image_raw"),
            ("/visual_slam/camera_info_7", "/back_stereo_camera/right/camera_info"),
            ("/visual_slam/imu", "/front_stereo_imu/imu"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("load_map_folder_path", default_value=""),
            DeclareLaunchArgument(
                "visual_slam_params",
                default_value=str(share / "config/visual_slam.yaml"),
            ),
            DeclareLaunchArgument("front_image_width", default_value="1280"),
            DeclareLaunchArgument("front_image_height", default_value="800"),
            DeclareLaunchArgument("image_qos", default_value="SENSOR_DATA"),
            DeclareLaunchArgument("enable_surround_cameras", default_value="false"),
            DeclareLaunchArgument("localize_on_startup", default_value="false"),
            DeclareLaunchArgument(
                "override_publishing_stamp",
                default_value="false",
                description="Stamp cuVSLAM output at current ROS time for live Nav2",
            ),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom_to_base_tf", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            ),
            ComposableNodeContainer(
                name="front_stereo_vslam_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    _mono_converter(
                        "left_image_normalizer",
                        "front",
                        "left",
                        LaunchConfiguration("front_image_width"),
                        LaunchConfiguration("front_image_height"),
                    ),
                    _mono_converter(
                        "right_image_normalizer",
                        "front",
                        "right",
                        LaunchConfiguration("front_image_width"),
                        LaunchConfiguration("front_image_height"),
                    ),
                    _mono_converter(
                        "left_left_image_normalizer",
                        "left",
                        "left",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    _mono_converter(
                        "left_right_image_normalizer",
                        "left",
                        "right",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    _mono_converter(
                        "right_left_image_normalizer",
                        "right",
                        "left",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    _mono_converter(
                        "right_right_image_normalizer",
                        "right",
                        "right",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    _mono_converter(
                        "back_left_image_normalizer",
                        "back",
                        "left",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    _mono_converter(
                        "back_right_image_normalizer",
                        "back",
                        "right",
                        1280,
                        800,
                        optional_surround=True,
                    ),
                    visual_slam,
                ],
            ),
        ]
    )
