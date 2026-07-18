import json
from pathlib import Path
import subprocess
import sys

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
    east = next(goal for goal in config["goals"] if goal["name"] == "east_room")

    assert route["collision_radius_m"] == 0.28
    assert route["goal_clearance_radius_m"] == 0.38
    assert north["pose"][:2] == [-0.50, 3.50]
    assert east["pose"] == [0.98, 4.93, 0.0]


def test_static_timeout_reflects_measured_full_stack_wall_time() -> None:
    config = yaml.safe_load((ROOT / "config/acceptance.yaml").read_text())
    runner = (
        ROOT
        / "ros2_ws/src/jackal_experiments/jackal_experiments/navigation_test_runner.py"
    ).read_text()

    assert config["trials"]["goal_timeout_s"] == 240.0
    assert '"checks": partial_checks' in runner
    for name in ("main_tf_chain_seen", "cuvslam_tracking", "guard_became_active"):
        assert f'"{name}"' in runner.split("partial_checks =", 1)[1]


def test_timeout_result_does_not_misclassify_healthy_localization(tmp_path: Path) -> None:
    (tmp_path / "metadata.json").write_text(
        json.dumps(
            {
                "trial_id": "timeout-diagnostic",
                "attempt_index": 1,
                "goal_index": 2,
                "goal_name": "east_room",
                "goal_pose": [0.98, 4.93, 0.0],
                "manual_intervention": False,
            }
        )
    )
    (tmp_path / "navigation.json").write_text(
        json.dumps(
            {
                "status": "failed",
                "error": "NavigateToPose timed out",
                "checks": {
                    "main_tf_chain_seen": True,
                    "cuvslam_tracking": True,
                    "guard_became_active": True,
                },
            }
        )
    )
    (tmp_path / "simulator.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "camera_profile": "navigation_6cam",
                "active_image_streams": 6,
                "runtime": {"significant_time_regressions": 0},
                "robot_contacts": {"collision_event_count": 0},
                "sensor_graphs": {"rear_render_products_created": False},
                "dynamic_obstacles": {"enabled": False},
            }
        )
    )
    (tmp_path / "command_trace.csv").write_text(
        "linear_x_mps\n0.1\n", encoding="utf-8"
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/finalize_static_trial.py"),
            str(tmp_path),
            "--runner-invoked",
            "true",
            "--runner-exit-code",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    result = json.loads((tmp_path / "result.json").read_text())

    assert completed.returncode == 10
    assert result["checks"]["localization_healthy"] is True
    assert result["failure_reasons"] == ["goal_reached"]
