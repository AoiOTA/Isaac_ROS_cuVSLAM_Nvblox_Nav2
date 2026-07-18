"""Stage 10 hardened front-stereo navigation stack.

Stage 10 intentionally keeps the validated front Hawk stereo pair.  It layers
shorter health timeouts and opt-in fault injection over the Stage 9 stack while
reusing the frozen Nav2/nvblox parameter files.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("jackal_bringup"))
    phase9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/phase9.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "vgl_map_dir": LaunchConfiguration("vgl_map_dir"),
            "vgl_config_dir": LaunchConfiguration("vgl_config_dir"),
            "vgl_model_dir": LaunchConfiguration("vgl_model_dir"),
            "cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir"),
            "nav2_params": LaunchConfiguration("nav2_params"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "nvblox_start_delay": LaunchConfiguration("nvblox_start_delay"),
            "nav2_start_delay": LaunchConfiguration("nav2_start_delay"),
            "visual_slam_timeout": "1.50",
            "depth_timeout": "1.25",
            "map_slice_timeout": "1.25",
            "enable_fault_injection": LaunchConfiguration(
                "enable_fault_injection"
            ),
            "nvblox_dynamic_params": str(share / "config/nvblox_dynamic.yaml"),
        }.items(),
    )

    lifecycle_guard = Node(
        package="jackal_experiments",
        executable="nav2_lifecycle_guard",
        name="nav2_lifecycle_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "failure_file": LaunchConfiguration("lifecycle_failure_file"),
                "force_failure": LaunchConfiguration(
                    "lifecycle_guard_force_failure"
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("vgl_map_dir"),
            DeclareLaunchArgument("vgl_config_dir"),
            DeclareLaunchArgument("vgl_model_dir"),
            DeclareLaunchArgument("cuvslam_map_dir"),
            DeclareLaunchArgument(
                "nav2_params", default_value=str(share / "config/nav2.yaml")
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("nvblox_start_delay", default_value="15.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="25.0"),
            DeclareLaunchArgument("enable_fault_injection", default_value="false"),
            DeclareLaunchArgument("lifecycle_guard_delay", default_value="42.0"),
            DeclareLaunchArgument("lifecycle_failure_file", default_value=""),
            DeclareLaunchArgument(
                "lifecycle_guard_force_failure", default_value="false"
            ),
            DeclareLaunchArgument(
                "rviz_config", default_value=str(share / "rviz/navigation.rviz")
            ),
            phase9,
            TimerAction(
                period=LaunchConfiguration("lifecycle_guard_delay"),
                actions=[lifecycle_guard],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=lifecycle_guard,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="Nav2 lifecycle guard requested clean relaunch"
                            )
                        )
                    ],
                )
            ),
        ]
    )
