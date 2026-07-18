import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))


def test_static_acceptance_metric_and_scope_are_explicit() -> None:
    config = yaml.safe_load((ROOT / "config/acceptance.yaml").read_text())
    assert config["workflow"] == "kujiale_jackal_static_avoidance"
    assert config["scope"] == {
        "environment_motion": "static",
        "dynamic_obstacles": False,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    trials = config["trials"]
    assert trials["minimum_valid_trials"] == 20
    assert trials["minimum_collision_free_passage_rate"] == 0.95
    assert trials["valid_trial_begins"] == "navigation_test_runner_invoked"
    assert trials["infrastructure_attempts_are_reported"] is True
    assert config["success_requires"] == {
        "goal_reached": True,
        "physical_collision_count": 0,
        "localization_healthy": True,
        "no_manual_intervention": True,
        "forward_only_motion": True,
        "simulator_status": "passed",
        "navigation_camera_streams": 6,
        "rear_camera_render_products_created": False,
    }


def test_trial_finalizer_counts_every_post_invocation_failure() -> None:
    trial = (ROOT / "scripts/run_static_trial.sh").read_text()
    batch = (ROOT / "scripts/run_static_acceptance.sh").read_text()
    finalizer = (ROOT / "tools/finalize_static_trial.py").read_text()
    assert 'RUNNER_INVOKED="true"' in trial
    assert "--runner-invoked" in trial
    assert "0|10)" in batch
    assert "20)" in batch
    assert "consecutive_infra" in batch
    for check in (
        '"goal_reached"',
        '"physical_collision_free"',
        '"localization_healthy"',
        '"no_manual_intervention"',
        '"forward_only_motion"',
        '"simulation_time_monotonic"',
        '"static_environment"',
        '"navigation_6cam"',
    ):
        assert check in finalizer
    assert '"valid_trial": runner_invoked' in finalizer


def _trial(path: Path, index: int, *, success: bool, valid: bool = True) -> Path:
    path.write_text(
        json.dumps(
            {
                "trial_id": f"trial-{index}",
                "attempt_index": index,
                "goal_name": "route",
                "valid_trial": valid,
                "status": "passed" if success else "failed",
                "collision_free_passage": success,
                "physical_collision_count": 0 if success else 1,
                "failure_reasons": [] if success else ["physical_collision_free"],
            }
        )
    )
    return path


def test_19_of_20_passes_while_18_of_20_fails_and_infra_is_excluded(
    tmp_path: Path,
) -> None:
    tool = ROOT / "tools/summarize_static_avoidance.py"
    config = ROOT / "config/acceptance.yaml"
    passing = [
        _trial(tmp_path / f"pass-{index}.json", index, success=index < 19)
        for index in range(20)
    ]
    passing.extend(
        _trial(
            tmp_path / f"infra-{index}.json",
            100 + index,
            success=False,
            valid=False,
        )
        for index in range(2)
    )
    output = tmp_path / "passing.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(tool),
            "--config",
            str(config),
            "--output",
            str(output),
            *map(str, passing),
        ],
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    summary = json.loads(output.read_text())
    assert summary["collision_free_passage_count"] == 19
    assert summary["valid_trial_count"] == 20
    assert summary["infrastructure_invalid_attempts"] == 2
    assert summary["collision_free_passage_rate"] == 0.95

    failing = [
        _trial(tmp_path / f"fail-{index}.json", 200 + index, success=index < 18)
        for index in range(20)
    ]
    completed = subprocess.run(
        [
            sys.executable,
            str(tool),
            "--config",
            str(config),
            "--output",
            str(tmp_path / "failing.json"),
            *map(str, failing),
        ],
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 1


def test_binary_pgm_parser_preserves_whitespace_valued_first_pixels(
    tmp_path: Path,
) -> None:
    from validate_acceptance_routes import read_pgm

    path = tmp_path / "map.pgm"
    pixels = [10, 32, 205, 254]
    path.write_bytes(b"P5\n2 2\n255\n" + bytes(pixels))
    assert read_pgm(path) == (2, 2, pixels)


def test_contact_monitor_covers_every_jackal_rigid_body() -> None:
    monitor = (ROOT / "isaac_sim/jackal_sim/contact_monitor.py").read_text()
    assert "Usd.PrimRange(robot)" in monitor
    assert "prim.HasAPI(UsdPhysics.RigidBodyAPI)" in monitor
    assert '"collision_event_count"' in monitor
    assert "CreateReportPairsRel().SetTargets" in monitor
    assert '"all_non_floor_collision_prims"' in monitor
    assert '"filtered_floor_event_count"' in monitor


def test_navigation_report_checker_uses_ros_action_status_codes(
    tmp_path: Path,
) -> None:
    report = tmp_path / "navigation.json"
    report.write_text(
        json.dumps(
            {
                "status": "passed",
                "experiment_class": "static",
                "checks": {"goal_reached": True},
                "goals": [{"status": 4, "error_code": 0}],
            }
        )
    )
    completed = subprocess.run(
        [sys.executable, str(ROOT / "tools/check_navigation_report.py"), str(report)],
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    payload = json.loads(report.read_text())
    payload["goals"][0]["status"] = "SUCCEEDED"
    report.write_text(json.dumps(payload))
    rejected = subprocess.run(
        [sys.executable, str(ROOT / "tools/check_navigation_report.py"), str(report)],
        capture_output=True,
        text=True,
    )
    assert rejected.returncode != 0
