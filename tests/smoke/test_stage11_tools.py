from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from build_stage11_reference_paths import se2_shortest_path  # noqa: E402
from generate_stage11_scenario import generate  # noqa: E402
from summarize_stage11_acceptance import build_summary  # noqa: E402


def configs() -> tuple[dict[str, object], dict[str, object]]:
    return (
        yaml.safe_load((ROOT / "config/stage11.yaml").read_text()),
        yaml.safe_load((ROOT / "config/scenarios.yaml").read_text()),
    )


def test_stage11_scenarios_are_deterministic_front_only_and_classed() -> None:
    stage11, scenarios = configs()
    first, metadata = generate(stage11, scenarios, "heterogeneous", 41001, 5)
    second, _ = generate(stage11, scenarios, "heterogeneous", 41001, 5)
    assert first == second
    assert metadata["actor_count"] == 6
    assert metadata["actor_kinds"] == ["box", "capsule", "existing_forklift"]
    assert metadata["long_distance"] is True
    assert metadata["scope"] == {
        "front_stereo_only": True,
        "lidar_enabled": False,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    static, static_metadata = generate(stage11, scenarios, "static", 21001, 0)
    assert static_metadata["actor_count"] == 0
    assert static_metadata["dynamic_profile"] == ""
    assert static["experiment"]["class"] == "static"


def test_se2_reference_path_respects_orientation_and_translation() -> None:
    valid = np.ones((8, 20, 20), dtype=bool)
    length, states, expansions = se2_shortest_path(
        valid, (5, 5, 0), (5, 10, 2), 0.1
    )
    assert abs(length - 0.5) < 1.0e-9
    assert states[0] == (5, 5, 0)
    assert states[-1] == (5, 10, 2)
    assert expansions > 0


def make_trial(
    experiment_class: str,
    seed: int,
    goal_index: int,
    goal: dict[str, object],
) -> dict[str, object]:
    return {
        "status": "passed",
        "experiment_class": experiment_class,
        "seed": seed,
        "goal_index": goal_index,
        "goal_name": goal["name"],
        "collision_count": 0,
        "realtime_factor": 0.9,
        "long_distance": bool(goal.get("long_distance", False)),
        "path": {"stretch": 0.1},
        "goal_results": [
            {
                "xy_error_m": 0.1,
                "yaw_error_deg": 4.0,
                "command_smoothness": {
                    "linear_acceleration_p95_mps2": 0.2,
                    "angular_acceleration_p95_radps2": 0.4,
                    "linear_jerk_p95_mps3": 1.0,
                    "angular_jerk_p95_radps3": 2.0,
                },
            }
        ],
        "command_latency_metrics": {
            "cmd_nav_raw_to_cmd_sim_freshness": {"p95_ms": 40.0}
        },
        "data_age_metrics": {"front_depth": {"p95_ms": 80.0}},
        "observed_rates_hz": {
            "cmd_nav_raw": 20.0,
            "visual_slam_status": 8.0,
            "depth_scan": 5.0,
            "nvblox_slice": 6.0,
        },
        "gpu": {"memory_used_peak_mib": 12000.0},
    }


def test_stage11_summary_enforces_all_three_classes_and_long_distance() -> None:
    stage11, _ = configs()
    local = copy.deepcopy(stage11)
    local["acceptance"]["thresholds"]["static_success_rate"] = 1.0
    local["acceptance"]["thresholds"]["dynamic_success_rate"] = 1.0
    local["acceptance"]["thresholds"]["heterogeneous_success_rate"] = 1.0
    trials = []
    for experiment_class, offset in (
        ("static", 0),
        ("dynamic", 10000),
        ("heterogeneous", 20000),
    ):
        trials.extend(
            make_trial(
                experiment_class,
                21000 + offset + index,
                index,
                stage11["goals"][index],
            )
            for index in range(6)
        )
    summary = build_summary(
        trials, local, {"static": 6, "dynamic": 6, "heterogeneous": 6}
    )
    assert summary["status"] == "passed"
    assert summary["trial_count"] == 18
    assert summary["aggregate"]["long_distance_success_rate"] == 1.0
    duplicated = copy.deepcopy(trials)
    duplicated[1]["seed"] = duplicated[0]["seed"]
    assert build_summary(
        duplicated, local, {"static": 6, "dynamic": 6, "heterogeneous": 6}
    )["status"] == "failed"


def test_stage11_summary_uses_collision_free_arrival_rate() -> None:
    stage11, _ = configs()
    trials = []
    expected = {"static": 20, "dynamic": 20, "heterogeneous": 20}
    for experiment_class, offset in (
        ("static", 0),
        ("dynamic", 10000),
        ("heterogeneous", 20000),
    ):
        for index in range(expected[experiment_class]):
            goal = stage11["goals"][index % len(stage11["goals"])]
            trial = make_trial(
                experiment_class,
                21000 + offset + index,
                index % len(stage11["goals"]),
                goal,
            )
            if index == 0:
                trial["status"] = "failed"
                trial["collision_count"] = 1
            trials.append(trial)
    summary = build_summary(trials, stage11, expected)
    assert summary["status"] == "passed"
    assert summary["classes"]["static"]["success_rate"] == 0.95
    assert summary["aggregate"]["collision_events"] == 3
