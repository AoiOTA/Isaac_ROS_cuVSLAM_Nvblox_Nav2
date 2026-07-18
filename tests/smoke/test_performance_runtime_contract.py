from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_performance_profiles_use_loopback_discovery_before_simulator() -> None:
    script = (ROOT / "scripts/run_performance_benchmark.sh").read_text()
    discovery_start = script.index("setsid fastdds discovery")
    simulator_start = script.index('setsid "${ISAAC_SIM_PYTHON}"')

    assert "PERFORMANCE_DISCOVERY_PORT_BASE" in script
    assert 'export ROS_DISCOVERY_SERVER="127.0.0.1:${discovery_port}"' in script
    assert "unset ROS_LOCALHOST_ONLY" in script
    assert discovery_start < simulator_start


def test_performance_discovery_is_logged_and_cleaned_per_profile() -> None:
    script = (ROOT / "scripts/run_performance_benchmark.sh").read_text()

    assert '>"${profile_dir}/fastdds-discovery.log" 2>&1' in script
    assert 'stop_group "${PROFILE_DISCOVERY_PID}"' in script
    assert 'PROFILE_DISCOVERY_PID=""' in script
    assert '--pid "${PROFILE_DISCOVERY_PID}"' in script


def test_late_mapping_recorder_uses_fastdds_super_client() -> None:
    script = (ROOT / "scripts/run_performance_benchmark.sh").read_text()

    assert 'tools/write_fastdds_super_client.py' in script
    assert '--output "${super_client_xml}"' in script
    assert 'env -u ROS_DISCOVERY_SERVER' in script
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE="${super_client_xml}" ros2 bag record' in script


def test_late_navigation_introspection_uses_super_client_without_daemon() -> None:
    script = (ROOT / "scripts/run_performance_benchmark.sh").read_text()

    assert "ros2 action list" not in script
    assert "ros2 topic list --no-daemon --spin-time 3 --include-hidden-topics" in script
    assert "/navigate_to_pose/_action/status" in script
    assert script.count('FASTRTPS_DEFAULT_PROFILES_FILE="${super_client_xml}"') >= 3


def test_navigation_driver_starts_only_after_nav2_is_active() -> None:
    script = (ROOT / "scripts/run_performance_benchmark.sh").read_text()
    lifecycle_gate = script.index(
        "lifecycle_manager_navigation.*Managed nodes are active"
    )
    driver_start = script.index(
        "setsid ros2 run jackal_experiments performance_workload_driver"
    )

    assert "Nav2 lifecycle activation timeout" in script
    assert lifecycle_gate < driver_start


def test_performance_profiles_preserve_full_preview_and_runtime_camera_qos() -> None:
    benchmark = (ROOT / "scripts/run_performance_benchmark.sh").read_text()
    simulator = (ROOT / "isaac_sim/navigation_sim.py").read_text()

    assert "--reliable-sensor-qos" in benchmark
    assert "fixed_frames=none" in benchmark
    assert "PERFORMANCE_GOAL_TIMEOUT" in benchmark
    assert 'goal_timeout_s:="${PERFORMANCE_GOAL_TIMEOUT}"' in benchmark
    assert '"preview_resolution_reduced": False' in simulator


def test_performance_summary_enforces_camera_lidar_and_preview_contract() -> None:
    summarize = (ROOT / "tools/summarize_performance.py").read_text()
    compare = (ROOT / "tools/compare_performance_profiles.py").read_text()

    for token in (
        'expected_profile = "mapping_8cam"',
        'else "navigation_6cam"',
        'rendering.get("preview_resolution_reduced") is not False',
        'rendering.get("resolution") != [1280, 720]',
        'sensor_graphs.get("lidar_enabled") is not False',
        '"simulator_contract"',
    ):
        assert token in summarize
    assert "reduced={reduced}" in compare
