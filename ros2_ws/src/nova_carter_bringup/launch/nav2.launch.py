from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "nvblox_map_slice_topic": LaunchConfiguration(
                "nvblox_map_slice_topic"
            ),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "movement_time_allowance": LaunchConfiguration(
                "movement_time_allowance"
            ),
            "source_timeout": LaunchConfiguration("source_timeout"),
        },
        convert_types=True,
    )
    map_yaml = LaunchConfiguration("map")
    common = {"output": "screen", "parameters": [params]}
    navigation_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config/nav2.yaml")
            ),
            DeclareLaunchArgument(
                "nvblox_map_slice_topic",
                default_value="/nvblox_node/static_map_slice",
            ),
            DeclareLaunchArgument(
                "odom_topic", default_value="/visual_slam/tracking/odometry"
            ),
            DeclareLaunchArgument("movement_time_allowance", default_value="25.0"),
            DeclareLaunchArgument("source_timeout", default_value="0.75"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params, {"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
                **common,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                **common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                **common,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
                **common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=[
                    ("cmd_vel", "/cmd_vel_nav_raw"),
                    ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
                ],
                **common,
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                **common,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                **common,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                **common,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                        "bond_timeout": 15.0,
                    }
                ],
            ),
            TimerAction(
                # This launch is included with a scoped launch context.  Use a
                # literal wall-clock delay so the value remains valid after
                # the include action returns.
                period=8.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            {
                                "autostart": True,
                                "node_names": navigation_nodes,
                                "bond_timeout": 15.0,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
