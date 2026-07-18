#!/usr/bin/env python3
"""Summarize Stage 10 fixed-seed pre-acceptance trials."""

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
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return ordered[low]
    fraction = position - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def build_summary(
    trials: list[dict[str, object]],
    thresholds: dict[str, object],
    expected: dict[str, int],
) -> dict[str, object]:
    classes: dict[str, dict[str, object]] = {}
    for name in ("static", "dynamic"):
        selected = [trial for trial in trials if trial.get("experiment_class") == name]
        passed_trials = [trial for trial in selected if trial.get("status") == "passed"]
        stretches = [
            float(trial.get("path", {}).get("stretch", math.inf))
            for trial in passed_trials
        ]
        rate = len(passed_trials) / len(selected) if selected else 0.0
        required_rate = float(thresholds[f"{name}_success_rate"])
        classes[name] = {
            "trial_count": len(selected),
            "expected_trial_count": expected[name],
            "passed_trials": len(passed_trials),
            "success_rate": rate,
            "required_success_rate": required_rate,
            "path_stretch_p95": percentile(stretches, 0.95),
            "collision_events": sum(int(trial.get("collision_count", 0)) for trial in selected),
            "minimum_realtime_factor": min(
                (float(trial.get("realtime_factor", 0.0)) for trial in selected),
                default=0.0,
            ),
            "passed": len(selected) == expected[name]
            and rate >= required_rate
            and sum(int(trial.get("collision_count", 0)) for trial in selected) == 0,
        }
    successful_stretches = [
        float(trial.get("path", {}).get("stretch", math.inf))
        for trial in trials
        if trial.get("status") == "passed"
    ]
    overall_p95 = percentile(successful_stretches, 0.95)
    path_passed = overall_p95 <= float(thresholds["path_stretch_p95"])
    passed = all(bool(value["passed"]) for value in classes.values()) and path_passed
    return {
        "status": "passed" if passed else "failed",
        "stage": 10,
        "scope": {
            "front_stereo_only": True,
            "lighting_randomization": False,
            "color_randomization": False,
        },
        "trial_count": len(trials),
        "passed_trials": sum(trial.get("status") == "passed" for trial in trials),
        "classes": classes,
        "successful_path_stretch_p95": overall_p95,
        "path_stretch_threshold": float(thresholds["path_stretch_p95"]),
        "path_stretch_passed": path_passed,
        "trials": [
            {
                "run_dir": trial.get("run_dir"),
                "status": trial.get("status"),
                "class": trial.get("experiment_class"),
                "seed": trial.get("seed"),
                "goal_index": trial.get("goal_index"),
                "collision_count": trial.get("collision_count"),
                "path_stretch": trial.get("path", {}).get("stretch"),
                "realtime_factor": trial.get("realtime_factor"),
            }
            for trial in trials
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--expected-static", type=int, default=None)
    parser.add_argument("--expected-dynamic", type=int, default=None)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    preacceptance = config["preacceptance"]
    expected = {
        "static": args.expected_static
        if args.expected_static is not None
        else int(preacceptance["static_trials"]),
        "dynamic": args.expected_dynamic
        if args.expected_dynamic is not None
        else int(preacceptance["dynamic_trials"]),
    }
    trials: list[dict[str, object]] = []
    for path in args.reports:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"trial report is not a JSON object: {path}")
        data["run_dir"] = str(path.resolve().parent)
        trials.append(data)
    summary = build_summary(trials, preacceptance["thresholds"], expected)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = args.output.with_name("trials.csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "run_dir",
            "status",
            "class",
            "seed",
            "goal_index",
            "collision_count",
            "path_stretch",
            "realtime_factor",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary["trials"])
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
