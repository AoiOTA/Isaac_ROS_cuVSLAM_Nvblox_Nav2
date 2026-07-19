import csv
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))


def test_runtime_map_artifacts_are_lfs_tracked_without_raw_bags() -> None:
    attributes = (ROOT / ".gitattributes").read_text()
    for suffix in ("mdb", "db", "pb", "bin", "jpg", "png", "nvblx", "ply", "pgm"):
        pattern = f"data/maps/kujiale_jackal_8cam/**/*.{suffix}"
        assert f"{pattern} filter=lfs diff=lfs merge=lfs -text" in attributes
        checked = subprocess.run(
            [
                "git",
                "check-attr",
                "filter",
                "--",
                f"data/maps/kujiale_jackal_8cam/runtime/file.{suffix}",
            ],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        assert checked.stdout.rstrip().endswith("filter: lfs")

    for relative in (
        "nvblox/kujiale.nvblx",
        "mesh/kujiale.ply",
        "cuvslam/map.mdb",
    ):
        assert subprocess.run(
            ["git", "check-ignore", "-q", f"data/maps/kujiale_jackal_8cam/{relative}"],
            cwd=ROOT,
        ).returncode == 1
    ignores = (ROOT / ".gitignore").read_text()
    assert "*.mcap" in ignores and "*.db3" in ignores


def test_map_manifest_requires_all_runtime_groups_and_no_capture_leaks() -> None:
    writer = (ROOT / "tools/write_map_manifest.py").read_text()
    checker = (ROOT / "tools/check_map_manifest.py").read_text()
    assert 'REQUIRED_GROUPS = ("cuvslam", "cuvgl", "nvblox", "mesh", "occupancy", "config")' in writer
    assert "MAPPING_IMAGE_TOPICS" in writer
    assert '"retained_in_repository": False' in writer
    for expected in (
        "cuVSLAM database",
        "cuVGL BoW index",
        "nvblox binary map",
        "nvblox mesh",
    ):
        assert expected in writer
    assert 'FORBIDDEN_CAPTURE_NAMES' in checker
    assert '{".mcap", ".db3"}' in checker
    assert '"rear_render_products_created": False' in checker
    mapping_runner = (ROOT / "scripts/run_mapping.sh").read_text()
    assert "--generation-command" in mapping_runner
    expected_command = (
        '"./scripts/run_mapping.sh --map {map_name} '
        '--${MAPPING_MODE} ${SIM_MODE}"'
    )
    assert expected_command in mapping_runner


def test_offline_map_generation_uses_temporary_ignored_workspace() -> None:
    script = (ROOT / "scripts/create_vgl_map.sh").read_text()
    assert 'mktemp -d "${PROJECT_ROOT}/data/bags/.vgl-work.XXXXXX"' in script
    assert "mapping_topics_8cam.yaml" in script
    assert '--sample_sync_threshold_microseconds="${MAX_SYNC_US}"' in script
    assert 'tools/tum_to_pose_bag.py' in script
    assert '--pose_bag_file="${POSE_BAG}"' in script
    assert '--pose_topic_name=/visual_slam/vis/slam_odometry' in script
    assert '--reference_pose_frame=map' in script
    assert '--rectify_images=True' in script
    assert "MINIMUM_SYNCED_FRAMES=40" in script
    assert "cuVGL synchronized frame groups" in script
    assert "create_map_offline.py" not in script
    assert 'require_file "${MAP_DIR}/cuvslam/data.mdb"' in script
    assert "prepare_vgl_runtime_config.py" in script
    assert 'rm -rf -- "${WORK}"' in script
    assert (ROOT / "tools/tum_to_pose_bag.py").is_file()


def test_performance_policy_is_adaptive_observation_not_a_fixed_kpi() -> None:
    config = yaml.safe_load((ROOT / "config/acceptance.yaml").read_text())
    performance = config["performance"]
    assert performance["policy"] == "observation_only_until_local_baseline_is_recorded"
    assert performance["fixed_frame_count"] is None
    assert performance["fixed_kpi_thresholds"] is None
    adaptive = performance["adaptive_sampling"]
    assert adaptive["minimum_warmup_s"] < adaptive["maximum_warmup_s"]
    assert adaptive["minimum_sample_s"] < adaptive["maximum_sample_s"]
    assert adaptive["stable_windows_required"] >= 1
    source = (ROOT / "isaac_sim/jackal_sim/performance.py").read_text()
    for recorder in (
        "AppFrametimeRecorder",
        "PhysicsFrametimeRecorder",
        "CPUContinuousRecorder",
        "MemoryRecorder",
        "HardwareSpecRecorder",
    ):
        assert recorder in source
    assert '"fixed_frame_count": None' in source
    assert '"fixed_kpi_thresholds": None' in source


def test_whole_workload_telemetry_summary_includes_process_and_gpu(
    tmp_path: Path,
) -> None:
    from summarize_performance import telemetry_summary

    fields = [
        "timestamp_unix_s",
        "project_process_count",
        "project_rss_gib",
        "project_vms_gib",
        "project_uss_gib",
        "system_memory_used_gib",
        "system_memory_total_gib",
        "gpu_name",
        "gpu_utilization_percent",
        "gpu_memory_used_mib",
        "gpu_memory_total_mib",
        "gpu_power_w",
        "gpu_temperature_c",
    ]
    path = tmp_path / "telemetry.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for timestamp, rss, gpu in ((10.0, 8.0, 40.0), (11.0, 10.0, 60.0)):
            writer.writerow(
                {
                    "timestamp_unix_s": timestamp,
                    "project_process_count": 2,
                    "project_rss_gib": rss,
                    "project_vms_gib": 20,
                    "project_uss_gib": 7,
                    "system_memory_used_gib": 16,
                    "system_memory_total_gib": 32,
                    "gpu_name": "NVIDIA GeForce RTX 4090",
                    "gpu_utilization_percent": gpu,
                    "gpu_memory_used_mib": 4096,
                    "gpu_memory_total_mib": 24564,
                    "gpu_power_w": 200,
                    "gpu_temperature_c": 55,
                }
            )
    summary = telemetry_summary(path, 9.5, 11.5)
    assert summary["sample_count"] == 2
    assert summary["gpu_name"] == "NVIDIA GeForce RTX 4090"
    assert summary["project_rss_gib"]["mean"] == 9.0
    assert summary["gpu_utilization_percent"]["mean"] == 50.0


def test_benchmark_runs_real_mapping_and_navigation_workloads_without_600_frames() -> None:
    runner = (ROOT / "scripts/run_performance_benchmark.sh").read_text()
    for token in (
        "mapping_8cam",
        "navigation_6cam",
        "--benchmark-performance",
        "--performance-start-file",
        "phase6.launch.py",
        "run_navigation.sh",
        "ros2 bag record",
        "performance_workload_driver",
        "record_performance_metrics.py",
        "summarize_mapping_capture.py",
        "compare_performance_profiles.py",
        "fixed_frames=none",
    ):
        assert token in runner
    assert "600" not in runner
    driver = (
        ROOT
        / "ros2_ws/src/jackal_experiments/jackal_experiments/performance_workload_driver.py"
    ).read_text()
    for token in (
        'self.mode not in {"mapping", "navigation"}',
        "mapping_direction_period_s",
        "NavigateToPose",
        '"/cmd_vel_sim"',
        "nonzero_command_samples",
        "active_workload_confirmed",
        "physical_motion_confirmed",
    ):
        assert token in driver
    normalizer = (ROOT / "tools/summarize_performance.py").read_text()
    assert "performance sample lacks a confirmed active workload" in normalizer
    assert "mapping performance requires temporary MCAP evidence" in normalizer
    comparison = (ROOT / "tools/compare_performance_profiles.py").read_text()
    assert "no preset KPI gate" in comparison
