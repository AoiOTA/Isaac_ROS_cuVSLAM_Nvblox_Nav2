from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_static_trial_uses_loopback_discovery_server_for_all_processes() -> None:
    script = (ROOT / "scripts/run_static_trial.sh").read_text()
    discovery_start = script.index("setsid fastdds discovery")
    simulator_start = script.index('setsid "${ISAAC_SIM_PYTHON}"')
    navigation_start = script.index(
        'setsid "${PROJECT_ROOT}/scripts/run_navigation.sh"'
    )
    runner_start = script.index(
        "ros2 run jackal_experiments navigation_test_runner"
    )

    assert 'export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"' in script
    assert "unset ROS_LOCALHOST_ONLY" in script
    assert discovery_start < simulator_start < navigation_start < runner_start


def test_static_trial_cleans_up_discovery_server_and_keeps_a_log() -> None:
    script = (ROOT / "scripts/run_static_trial.sh").read_text()

    assert 'stop_group "${DISCOVERY_PID}"' in script
    assert '>"${RUN_DIR}/fastdds-discovery.log" 2>&1' in script
    assert "STATIC_ACCEPTANCE_DISCOVERY_PORT" in script


def test_static_trial_uses_reliable_navigation_camera_transport() -> None:
    trial = (ROOT / "scripts/run_static_trial.sh").read_text()
    navigation = (ROOT / "scripts/run_navigation.sh").read_text()
    phase7 = (
        ROOT
        / "ros2_ws/src/jackal_bringup/launch/phase7_localization.launch.py"
    ).read_text()
    phase8 = (
        ROOT / "ros2_ws/src/jackal_bringup/launch/phase8.launch.py"
    ).read_text()

    assert "--reliable-sensor-qos" in trial
    assert "image_qos:=DEFAULT" in navigation
    assert 'DeclareLaunchArgument("image_qos", default_value="DEFAULT")' in phase7
    assert '"image_qos": LaunchConfiguration("image_qos")' in phase7
    assert 'DeclareLaunchArgument("image_qos", default_value="DEFAULT")' in phase8
    assert '"image_qos": LaunchConfiguration("image_qos")' in phase8


def test_static_acceptance_separates_door_and_endpoint_clearance() -> None:
    config = yaml.safe_load((ROOT / "config/acceptance.yaml").read_text())
    route = config["route_validation"]
    north = next(goal for goal in config["goals"] if goal["name"] == "north_room")

    assert route["collision_radius_m"] == 0.28
    assert route["goal_clearance_radius_m"] == 0.34
    assert north["pose"][:2] == [-0.50, 3.50]
