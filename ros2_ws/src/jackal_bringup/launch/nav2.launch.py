from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


NAVIGATION_START_SCHEDULE = (
    (6.0, "smoother_server"),
    (8.0, "planner_server"),
    (10.0, "behavior_server"),
    (12.0, "velocity_smoother"),
    (14.0, "collision_monitor"),
    (16.0, "bt_navigator"),
    (18.0, "waypoint_follower"),
)
NAVIGATION_ACTIVATION_DELAY_S = 25.0


def _launch_setup(context):
    share = Path(get_package_share_directory("jackal_bringup"))
    resolved = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "map",
            "params_file",
            "nvblox_map_slice_topic",
            "odom_topic",
            "movement_time_allowance",
            "source_timeout",
        )
    }
    params = RewrittenYaml(
        source_file=resolved["params_file"],
        param_rewrites={
            "nvblox_map_slice_topic": resolved["nvblox_map_slice_topic"],
            "odom_topic": resolved["odom_topic"],
            "movement_time_allowance": resolved["movement_time_allowance"],
            "source_timeout": resolved["source_timeout"],
        },
        convert_types=True,
    )
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
    delayed_nodes = {
        "smoother_server": Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            **common,
        ),
        "planner_server": Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            **common,
        ),
        "behavior_server": Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
            **common,
        ),
        "velocity_smoother": Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[
                ("cmd_vel", "/cmd_vel_nav_raw"),
                ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
            ],
            **common,
        ),
        "collision_monitor": Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            **common,
        ),
        "bt_navigator": Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params,
                {
                    "default_nav_to_pose_bt_xml": str(
                        share / "behavior_trees/navigate_forward_only.xml"
                    )
                },
            ],
        ),
        "waypoint_follower": Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
    }
    delayed_actions = [
        TimerAction(period=delay, actions=[delayed_nodes[name]])
        for delay, name in NAVIGATION_START_SCHEDULE
    ]
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, {"yaml_filename": resolved["map"]}],
        ),
        # MPPI and the nvblox local-costmap layer are the most expensive Nav2
        # constructors. Give the controller an uncontended startup window.
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
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
        *delayed_actions,
        TimerAction(
            period=NAVIGATION_ACTIVATION_DELAY_S,
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


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("jackal_bringup"))
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
            # Resolve substitutions now.  This launch is itself included by
            # phase8, so delayed actions must not depend on a later scoped
            # LaunchConfiguration lookup.
            OpaqueFunction(function=_launch_setup),
        ]
    )
