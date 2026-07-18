from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_stage11_formal_counts_and_scope() -> None:
    config = yaml.safe_load((ROOT / "config/stage11.yaml").read_text())
    assert config["acceptance"]["trial_counts"] == {
        "static": 10,
        "dynamic": 10,
        "heterogeneous": 10,
    }
    assert config["runtime"]["front_image_rate_hz"] == 15
    assert config["scope"] == {
        "front_stereo_only": True,
        "lidar_enabled": False,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    assert sum(bool(goal["long_distance"]) for goal in config["goals"]) == 3


def test_stage11_uses_actual_usd_geometry_and_footprint_se2_reference() -> None:
    extractor = (ROOT / "tools/extract_usd_collision_geometry.py").read_text()
    planner = (ROOT / "tools/build_stage11_reference_paths.py").read_text()
    assert "UsdPhysics.CollisionAPI" in extractor
    assert "stage.GetUsedLayers" in extractor
    assert "se2_shortest_path" in planner
    assert "footprint_offsets" in planner
    assert "Nav2 SmacPlanner2D global plan" not in planner


def test_stage11_automation_is_locked_resumable_and_records_latency() -> None:
    trial = (ROOT / "scripts/run_stage11_trial.sh").read_text()
    acceptance = (ROOT / "scripts/run_acceptance.sh").read_text()
    runner = (
        ROOT
        / "ros2_ws/src/nova_carter_experiments/nova_carter_experiments/navigation_test_runner.py"
    ).read_text()
    for token in (
        "stage11-trial.lock",
        "NOVA_CARTER_NAV2_LIFECYCLE",
        "ros2 bag record",
        "finalize_stage11_trial.py",
    ):
        assert token in trial
    assert "stage11-matrix.lock" in acceptance
    assert "--resume" in acceptance
    assert '[[ -s "${existing_report}" ]] || continue' in acceptance
    assert 'r.get("status") in ("passed","failed")' in acceptance
    assert 'len(n["goals"])>0' in acceptance
    assert "Resuming completed" in acceptance
    assert "HETEROGENEOUS_TRIALS" in acceptance
    assert "INFRASTRUCTURE_RETRIES" in acceptance
    assert 'len(r["goals"])>0' in acceptance
    assert "Infrastructure-only empty run" in acceptance
    assert "formal Stage 11 trial counts are fixed" in acceptance
    assert "Existing summary does not match the configured full matrix" in acceptance
    assert 'r.get("expected_trial_count")==sum(expected.values())' in acceptance
    assert "cmd_nav_raw_to_cmd_sim_freshness" in runner
    assert "data_age_metrics" in runner
    assert "enable_surround_cameras" not in trial


def test_stage11_long_distance_costmap_and_safety_contract() -> None:
    nav2 = yaml.safe_load(
        (ROOT / "ros2_ws/src/nova_carter_bringup/config/nav2.yaml").read_text()
    )
    dynamic = yaml.safe_load(
        (
            ROOT
            / "ros2_ws/src/nova_carter_bringup/config/nvblox_dynamic.yaml"
        ).read_text()
    )["nvblox_node"]["ros__parameters"]
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    source = local["obstacle_layer"]["depth_scan"]
    assert source["topic"] == "/front_depth/scan"
    assert source["data_type"] == "LaserScan"
    assert dynamic["clear_map_outside_radius_rate_hz"] == 1.0
    assert dynamic["map_clearing_radius_m"] == 8.0

    finalizer = (ROOT / "tools/finalize_stage11_trial.py").read_text()
    for required_check in (
        '"no_blind_motion"',
        '"front_stereo_only"',
        '"no_manual_intervention"',
        '"usd_theoretical_path_available"',
    ):
        assert required_check in finalizer
