from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
BRINGUP = ROOT / "ros2_ws/src/nova_carter_bringup"


def test_stage10_scope_is_front_stereo_without_appearance_randomization() -> None:
    config = yaml.safe_load((ROOT / "config/stage10.yaml").read_text())
    assert config["scope"] == {
        "front_stereo_only": True,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    launch = (BRINGUP / "launch/phase10.launch.py").read_text()
    assert "phase9.launch.py" in launch
    assert '"enable_fault_injection"' in launch
    assert "nav2_lifecycle_guard" in launch
    assert "lifecycle_failure_file" in launch
    assert "enable_surround_cameras" not in launch


def test_stage10_frozen_safety_and_mapping_parameters_match_runtime() -> None:
    stage10 = yaml.safe_load((ROOT / "config/stage10.yaml").read_text())
    frozen = stage10["frozen_parameters"]
    nav2 = yaml.safe_load((BRINGUP / "config/nav2.yaml").read_text())
    control = yaml.safe_load(
        (ROOT / "ros2_ws/src/nova_carter_control/config/control_params.yaml").read_text()
    )["command_guard"]["ros__parameters"]
    dynamic = yaml.safe_load((BRINGUP / "config/nvblox_dynamic.yaml").read_text())[
        "nvblox_node"
    ]["ros__parameters"]
    assert control["max_linear_acceleration"] == frozen["command_guard"][
        "max_linear_acceleration_mps2"
    ]
    assert control["max_linear_jerk"] == frozen["command_guard"][
        "max_linear_jerk_mps3"
    ]
    assert control["slew_response_rate"] == frozen["command_guard"][
        "slew_response_rate"
    ]
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    goal_checker = nav2["controller_server"]["ros__parameters"]["goal_checker"]
    assert goal_checker["xy_goal_tolerance"] == 0.15
    assert goal_checker["yaw_goal_tolerance"] < 0.14
    assert local["inflation_layer"]["inflation_radius"] == 0.8
    assert global_map["inflation_layer"]["inflation_radius"] == 0.8
    mapper = dynamic["dynamic_mapper"]
    assert mapper["occupied_region_decay_probability"] == 0.35
    assert mapper["free_region_decay_probability"] == 0.60


def test_stage10_automation_owns_processes_and_records_required_evidence() -> None:
    trial = (ROOT / "scripts/run_phase10_trial.sh").read_text()
    for token in (
        "setsid",
        'stop_group "${NAV_PID}"',
        'stop_simulator',
        "ros2 bag record",
        "--storage mcap",
        "trajectory.csv",
        "gpu.csv",
        "finalize_stage10_trial.py",
        "TRIAL_LOCK_FD",
        'NOVA_CARTER_NAV2_LIFECYCLE {"status": "already_active"',
    ):
        assert token in trial
    assert "killall" not in trial
    assert "pkill" not in trial
    assert "/plan" not in trial.split("for required_topic in", 1)[1].split("; do", 1)[0]
    navigation = (ROOT / "scripts/run_phase10_navigation.sh").read_text()
    assert "PHASE10_NAVIGATION_START_ATTEMPTS" in navigation
    assert "PHASE10_FORCE_LIFECYCLE_FAILURE_ONCE" in navigation
    assert "Nav2 lifecycle startup failed; relaunching clean stack" in navigation
    finalizer = (ROOT / "tools/finalize_stage10_trial.py").read_text()
    assert 'run_dir / "contacts.csv"' in finalizer
    assert '"normal_navigation_smoothness"' in finalizer
    matrix = (ROOT / "scripts/run_phase10_preacceptance.sh").read_text()
    assert "STATIC_TRIALS" in matrix and "DYNAMIC_TRIALS" in matrix
    assert "MATRIX_LOCK_FD" in matrix
    assert "summarize_stage10_trials.py" in matrix
    assert "phase10-matrix.lock" in matrix
    assert "flock -n 8" in matrix
    assert "simulator produced no report" in matrix
    assert "phase10-trial.lock" in trial
    assert "flock -n 9" in trial


def test_stage10_contact_monitor_covers_all_robot_rigid_bodies() -> None:
    monitor = (
        ROOT / "isaac_sim/nova_carter_sim/contact_monitor.py"
    ).read_text()
    assert "Usd.PrimRange(robot)" in monitor
    assert "prim.HasAPI(UsdPhysics.RigidBodyAPI)" in monitor
    assert '"reporter_count"' in monitor
    assert '"filtered_floor_event_count"' in monitor
