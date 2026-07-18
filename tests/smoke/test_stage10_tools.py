import copy
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from generate_stage10_scenario import generate  # noqa: E402
from summarize_stage10_trials import build_summary  # noqa: E402


def test_scenario_generation_is_seeded_and_only_changes_dynamic_timing() -> None:
    stage10 = yaml.safe_load((ROOT / "config/stage10.yaml").read_text())
    source = yaml.safe_load((ROOT / "config/scenarios.yaml").read_text())
    first, first_metadata = generate(stage10, source, "dynamic", 1234, 1)
    second, second_metadata = generate(stage10, source, "dynamic", 1234, 1)
    different, _ = generate(stage10, source, "dynamic", 1235, 1)
    assert first == second
    assert first_metadata == second_metadata
    assert first["dynamic_profiles"]["stage10_trial"] != different[
        "dynamic_profiles"
    ]["stage10_trial"]
    original = source["dynamic_profiles"]["warehouse_crossing"]["obstacles"]
    generated = first["dynamic_profiles"]["stage10_trial"]["obstacles"]
    assert [item.get("color_rgb") for item in generated] == [
        item.get("color_rgb") for item in original
    ]
    assert first_metadata["scope"]["lighting_randomization"] is False
    assert first_metadata["scope"]["color_randomization"] is False


def trial(experiment_class: str, passed: bool = True, stretch: float = 0.1) -> dict:
    return {
        "status": "passed" if passed else "failed",
        "experiment_class": experiment_class,
        "collision_count": 0,
        "path": {"stretch": stretch},
        "realtime_factor": 0.8,
    }


def test_preacceptance_thresholds_use_19_of_20_and_18_of_20() -> None:
    trials = [trial("static", index < 19) for index in range(20)]
    trials += [trial("dynamic", index < 18) for index in range(20)]
    thresholds = {
        "static_success_rate": 0.95,
        "dynamic_success_rate": 0.90,
        "path_stretch_p95": 0.20,
    }
    summary = build_summary(trials, thresholds, {"static": 20, "dynamic": 20})
    assert summary["status"] == "passed"
    failed_trials = copy.deepcopy(trials)
    failed_trials[18]["status"] = "failed"
    assert build_summary(
        failed_trials, thresholds, {"static": 20, "dynamic": 20}
    )["status"] == "failed"
