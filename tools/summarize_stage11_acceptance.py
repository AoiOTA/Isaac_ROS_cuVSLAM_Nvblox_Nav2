#!/usr/bin/env python3
"""Summarize the formal 10/10/10 Stage 11 acceptance matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import yaml


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    rank = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    low, high = math.floor(rank), math.ceil(rank)
    if low == high:
        return ordered[low]
    fraction = rank - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def build_summary(
    trials: list[dict[str, object]], config: dict[str, object], expected: dict[str, int]
) -> dict[str, object]:
    thresholds = config["acceptance"]["thresholds"]
    seed_base = int(config["acceptance"]["seed_base"])
    seed_offsets = {"static": 0, "dynamic": 10000, "heterogeneous": 20000}
    goals = config["goals"]
    classes: dict[str, dict[str, object]] = {}
    for name in ("static", "dynamic", "heterogeneous"):
        selected = [trial for trial in trials if trial.get("experiment_class") == name]
        passed = [trial for trial in selected if trial.get("status") == "passed"]
        expected_identities = {
            (
                name,
                seed_base + seed_offsets[name] + index,
                index % len(goals),
                str(goals[index % len(goals)]["name"]),
                bool(goals[index % len(goals)].get("long_distance", False)),
            )
            for index in range(expected[name])
        }
        observed_identities = {
            (
                str(trial.get("experiment_class")),
                int(trial.get("seed", -1)),
                int(trial.get("goal_index", -1)),
                str(trial.get("goal_name")),
                bool(trial.get("long_distance")),
            )
            for trial in selected
        }
        success_rate = len(passed) / len(selected) if selected else 0.0
        required = float(thresholds[f"{name}_success_rate"])
        collisions = sum(int(trial.get("collision_count", 0)) for trial in selected)
        classes[name] = {
            "trial_count": len(selected),
            "expected_trial_count": expected[name],
            "passed_trials": len(passed),
            "success_rate": success_rate,
            "required_success_rate": required,
            "collision_events": collisions,
            "minimum_realtime_factor": min(
                (float(trial.get("realtime_factor", 0.0)) for trial in selected),
                default=0.0,
            ),
            "path_stretch_p95": percentile(
                [float(trial["path"]["stretch"]) for trial in passed], 0.95
            ),
            "goal_names": sorted({str(trial.get("goal_name")) for trial in selected}),
            "long_distance_trials": sum(
                bool(trial.get("long_distance")) for trial in selected
            ),
            "expected_identities_match": observed_identities == expected_identities,
            "passed": len(selected) == expected[name]
            and observed_identities == expected_identities
            and success_rate >= required,
        }
    successful = [trial for trial in trials if trial.get("status") == "passed"]
    long_trials = [trial for trial in trials if bool(trial.get("long_distance"))]
    long_passed = [trial for trial in long_trials if trial.get("status") == "passed"]
    long_rate = len(long_passed) / len(long_trials) if long_trials else 0.0
    path_p95 = percentile(
        [float(trial["path"]["stretch"]) for trial in successful], 0.95
    )
    endpoint_xy = [
        float(goal["xy_error_m"])
        for trial in successful
        for goal in trial.get("goal_results", [])
    ]
    endpoint_yaw = [
        float(goal["yaw_error_deg"])
        for trial in successful
        for goal in trial.get("goal_results", [])
    ]
    command_latency = [
        float(
            trial.get("command_latency_metrics", {})
            .get("cmd_nav_raw_to_cmd_sim_freshness", {})
            .get("p95_ms", math.inf)
        )
        for trial in successful
    ]
    depth_age = [
        float(
            trial.get("data_age_metrics", {})
            .get("front_depth", {})
            .get("p95_ms", math.inf)
        )
        for trial in successful
    ]
    smoothness_metrics = (
        "linear_acceleration_p95_mps2",
        "angular_acceleration_p95_radps2",
        "linear_jerk_p95_mps3",
        "angular_jerk_p95_radps3",
    )
    smoothness = {
        metric: percentile(
            [
                float(goal.get("command_smoothness", {}).get(metric, math.inf))
                for trial in successful
                for goal in trial.get("goal_results", [])
            ],
            0.95,
        )
        for metric in smoothness_metrics
    }
    rate_topics = (
        "cmd_nav_raw",
        "visual_slam_status",
        "depth_scan",
        "nvblox_slice",
    )
    minimum_rates = {
        topic: min(
            (
                float(trial.get("observed_rates_hz", {}).get(topic, 0.0))
                for trial in successful
            ),
            default=0.0,
        )
        for topic in rate_topics
    }
    aggregate = {
        "successful_path_stretch_p95": path_p95,
        "path_stretch_threshold": float(thresholds["path_stretch_p95"]),
        "long_distance_trial_count": len(long_trials),
        "long_distance_passed_trials": len(long_passed),
        "long_distance_success_rate": long_rate,
        "long_distance_required_success_rate": float(
            thresholds["long_distance_success_rate"]
        ),
        "endpoint_xy_error_p95_m": percentile(endpoint_xy, 0.95),
        "endpoint_yaw_error_p95_deg": percentile(endpoint_yaw, 0.95),
        "command_raw_to_sim_freshness_p95_of_trials_ms": percentile(
            command_latency, 0.95
        ),
        "front_depth_age_p95_of_trials_ms": percentile(depth_age, 0.95),
        "collision_events": sum(int(trial.get("collision_count", 0)) for trial in trials),
        "minimum_realtime_factor": min(
            (float(trial.get("realtime_factor", 0.0)) for trial in trials),
            default=0.0,
        ),
        "smoothness_p95_of_successful_goals": smoothness,
        "minimum_observed_rates_hz": minimum_rates,
        "maximum_gpu_memory_mib": max(
            (
                float(trial.get("gpu", {}).get("memory_used_peak_mib", 0.0))
                for trial in successful
            ),
            default=0.0,
        ),
    }
    passed = (
        all(bool(value["passed"]) for value in classes.values())
        and path_p95 <= float(thresholds["path_stretch_p95"])
        and long_rate >= float(thresholds["long_distance_success_rate"])
    )
    rows = [
        {
            "run_dir": trial.get("run_dir"),
            "status": trial.get("status"),
            "class": trial.get("experiment_class"),
            "seed": trial.get("seed"),
            "goal_name": trial.get("goal_name"),
            "long_distance": trial.get("long_distance"),
            "collision_count": trial.get("collision_count"),
            "path_stretch": trial.get("path", {}).get("stretch"),
            "actual_path_m": trial.get("path", {}).get("actual_length_m"),
            "theoretical_path_m": trial.get("path", {}).get(
                "theoretical_optimal_length_m"
            ),
            "realtime_factor": trial.get("realtime_factor"),
        }
        for trial in trials
    ]
    return {
        "status": "passed" if passed else "failed",
        "stage": 11,
        "scope": config["scope"],
        "trial_count": len(trials),
        "expected_trial_count": sum(expected.values()),
        "passed_trials": len(successful),
        "classes": classes,
        "aggregate": aggregate,
        "trials": rows,
    }


def markdown_report(summary: dict[str, object]) -> str:
    aggregate = summary["aggregate"]
    lines = [
        "# Stage 11 Formal Acceptance Report",
        "",
        f"Overall status: **{str(summary['status']).upper()}**",
        "",
        "| Class | Passed / Trials | Success | Required | Collisions | Stretch P95 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in summary["classes"].items():
        lines.append(
            f"| {name} | {value['passed_trials']} / {value['trial_count']} | "
            f"{100.0 * value['success_rate']:.2f}% | "
            f"{100.0 * value['required_success_rate']:.2f}% | "
            f"{value['collision_events']} | {100.0 * value['path_stretch_p95']:.2f}% |"
        )
    lines.extend(
        [
            "",
            f"- Successful-path stretch P95: {100.0 * aggregate['successful_path_stretch_p95']:.2f}%.",
            f"- Long-distance success: {aggregate['long_distance_passed_trials']} / "
            f"{aggregate['long_distance_trial_count']} "
            f"({100.0 * aggregate['long_distance_success_rate']:.2f}%).",
            f"- Endpoint error P95: {aggregate['endpoint_xy_error_p95_m']:.3f} m / "
            f"{aggregate['endpoint_yaw_error_p95_deg']:.2f} deg.",
            f"- Minimum real-time factor: {aggregate['minimum_realtime_factor']:.3f}.",
            f"- Command freshness P95-of-trials: "
            f"{aggregate['command_raw_to_sim_freshness_p95_of_trials_ms']:.2f} ms.",
            f"- Front-depth age P95-of-trials: "
            f"{aggregate['front_depth_age_p95_of_trials_ms']:.2f} ms.",
            f"- Smoothness P95 (linear/angular acceleration): "
            f"{aggregate['smoothness_p95_of_successful_goals']['linear_acceleration_p95_mps2']:.3f} m/s^2 / "
            f"{aggregate['smoothness_p95_of_successful_goals']['angular_acceleration_p95_radps2']:.3f} rad/s^2.",
            f"- Smoothness P95 (linear/angular jerk): "
            f"{aggregate['smoothness_p95_of_successful_goals']['linear_jerk_p95_mps3']:.3f} m/s^3 / "
            f"{aggregate['smoothness_p95_of_successful_goals']['angular_jerk_p95_radps3']:.3f} rad/s^3.",
            f"- Minimum rates (controller/visual/depth/nvblox): "
            f"{aggregate['minimum_observed_rates_hz']['cmd_nav_raw']:.2f} / "
            f"{aggregate['minimum_observed_rates_hz']['visual_slam_status']:.2f} / "
            f"{aggregate['minimum_observed_rates_hz']['depth_scan']:.2f} / "
            f"{aggregate['minimum_observed_rates_hz']['nvblox_slice']:.2f} Hz.",
            f"- PhysX collision events: {aggregate['collision_events']}.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--expected-static", type=int)
    parser.add_argument("--expected-dynamic", type=int)
    parser.add_argument("--expected-heterogeneous", type=int)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    configured = config["acceptance"]["trial_counts"]
    expected = {
        "static": args.expected_static if args.expected_static is not None else int(configured["static"]),
        "dynamic": args.expected_dynamic if args.expected_dynamic is not None else int(configured["dynamic"]),
        "heterogeneous": args.expected_heterogeneous if args.expected_heterogeneous is not None else int(configured["heterogeneous"]),
    }
    trials: list[dict[str, object]] = []
    for path in args.reports:
        trial = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(trial, dict):
            raise ValueError(f"not a JSON object: {path}")
        trial["run_dir"] = str(path.resolve().parent)
        trials.append(trial)
    summary = build_summary(trials, config, expected)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = args.output.with_name("trials.csv")
    if not summary["trials"]:
        raise ValueError("at least one trial report is required")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary["trials"][0]))
        writer.writeheader()
        writer.writerows(summary["trials"])
    args.output.with_name("report.md").write_text(
        markdown_report(summary), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
