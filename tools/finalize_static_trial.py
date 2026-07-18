#!/usr/bin/env python3
"""Classify one static attempt without dropping valid failures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def load_object(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def command_trace_is_forward_only(path: Path, tolerance: float = 0.01) -> tuple[bool, float]:
    if not path.is_file():
        return False, math.nan
    minimum = math.inf
    samples = 0
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            value = float(row["linear_x_mps"])
            minimum = min(minimum, value)
            samples += 1
    return samples > 0 and minimum >= -tolerance, minimum


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--runner-invoked", choices=["true", "false"], required=True)
    parser.add_argument("--runner-exit-code", type=int, default=99)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    metadata = load_object(run_dir / "metadata.json")
    navigation = load_object(run_dir / "navigation.json")
    simulator = load_object(run_dir / "simulator.json")
    runner_invoked = args.runner_invoked == "true"
    nav_checks = navigation.get("checks", {})
    if not isinstance(nav_checks, dict):
        nav_checks = {}
    runtime = simulator.get("runtime", {})
    if not isinstance(runtime, dict):
        runtime = {}
    contacts = simulator.get("robot_contacts", {})
    if not isinstance(contacts, dict):
        contacts = {}
    sensor_graphs = simulator.get("sensor_graphs", {})
    if not isinstance(sensor_graphs, dict):
        sensor_graphs = {}
    dynamic = simulator.get("dynamic_obstacles", {})
    if not isinstance(dynamic, dict):
        dynamic = {}
    collision_count = int(contacts.get("collision_event_count", -1))
    forward_only, minimum_linear = command_trace_is_forward_only(
        run_dir / "command_trace.csv"
    )
    goal_reached = (
        args.runner_exit_code == 0
        and navigation.get("status") == "passed"
        and nav_checks.get("all_goals_succeeded") is True
    )
    localization_healthy = all(
        nav_checks.get(name) is True
        for name in ("main_tf_chain_seen", "cuvslam_tracking", "guard_became_active")
    )
    checks = {
        "goal_reached": goal_reached,
        "physical_collision_free": collision_count == 0,
        "localization_healthy": localization_healthy,
        "no_manual_intervention": metadata.get("manual_intervention") is False
        and not (run_dir / "manual-intervention").exists(),
        "forward_only_motion": forward_only,
        "simulator_passed": simulator.get("status") == "passed",
        "simulation_time_monotonic": runtime.get("significant_time_regressions") == 0,
        "static_environment": dynamic.get("enabled") is False,
        "navigation_6cam": simulator.get("camera_profile") == "navigation_6cam"
        and simulator.get("active_image_streams") == 6
        and sensor_graphs.get("rear_render_products_created") is False,
    }
    success = runner_invoked and all(checks.values())
    result = {
        "schema_version": 1,
        "trial_id": metadata.get("trial_id", run_dir.name),
        "attempt_index": metadata.get("attempt_index"),
        "goal_index": metadata.get("goal_index"),
        "goal_name": metadata.get("goal_name"),
        "goal_pose": metadata.get("goal_pose"),
        "valid_trial": runner_invoked,
        "status": (
            "passed" if success else "failed" if runner_invoked else "infrastructure_invalid"
        ),
        "collision_free_passage": success,
        "checks": checks,
        "failure_reasons": [name for name, passed in checks.items() if not passed]
        if runner_invoked
        else ["navigation_test_runner_not_invoked"],
        "runner_exit_code": args.runner_exit_code,
        "physical_collision_count": collision_count,
        "minimum_commanded_linear_x_mps": minimum_linear,
        "navigation_error": navigation.get("error"),
        "navigation_report": str(run_dir / "navigation.json"),
        "simulator_report": str(run_dir / "simulator.json"),
    }
    (run_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if success else 10 if runner_invoked else 20


if __name__ == "__main__":
    raise SystemExit(main())
