#!/usr/bin/env python3
"""Merge simulator, ROS, GPU and bag evidence into one Stage 10 trial result."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import yaml


def load_json(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    fraction = position - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def gpu_summary(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {"sample_count": 0}
    utilization: list[float] = []
    memory: list[float] = []
    power: list[float] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                utilization.append(float(row["utilization_percent"]))
                memory.append(float(row["memory_used_mib"]))
                power.append(float(row["power_w"]))
            except (KeyError, TypeError, ValueError):
                continue
    return {
        "sample_count": len(utilization),
        "utilization_p95_percent": percentile(utilization, 0.95),
        "memory_used_peak_mib": max(memory, default=0.0),
        "power_p95_w": percentile(power, 0.95),
    }


def directory_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--runner-exit-code", type=int, default=0)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--require-rosbag", action="store_true")
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    stage10 = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    scenario = yaml.safe_load((run_dir / "scenario.yaml").read_text(encoding="utf-8"))
    metadata = load_json(run_dir / "scenario.json")
    navigation = load_json(run_dir / "navigation.json")
    simulator = load_json(run_dir / "simulator.json")
    experiment_class = str(metadata.get("experiment_class", "unknown"))
    runtime = simulator.get("runtime", {})
    contacts = simulator.get("robot_contacts", {})
    dynamic = simulator.get("dynamic_obstacles", {})
    if not isinstance(runtime, dict):
        runtime = {}
    if not isinstance(contacts, dict):
        contacts = {}
    if not isinstance(dynamic, dict):
        dynamic = {}

    contact_rows = contacts.get("contact_pairs", [])
    if not isinstance(contact_rows, list):
        contact_rows = []
    with (run_dir / "contacts.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["actor_0", "actor_1", "event_count"])
        for row in contact_rows:
            actors = row.get("actors", []) if isinstance(row, dict) else []
            writer.writerow(
                [
                    actors[0] if len(actors) > 0 else "",
                    actors[1] if len(actors) > 1 else "",
                    row.get("count", 0) if isinstance(row, dict) else 0,
                ]
            )

    goals = navigation.get("goals", [])
    if not isinstance(goals, list):
        goals = []
    path_stretches = [
        float(goal.get("path_stretch", math.inf))
        for goal in goals
        if isinstance(goal, dict)
    ]
    actual_path = sum(
        float(goal.get("actual_path_length_m", 0.0))
        for goal in goals
        if isinstance(goal, dict)
    )
    reference_path = sum(
        float(goal.get("reference_plan_length_m", 0.0))
        for goal in goals
        if isinstance(goal, dict)
    )
    sim_delta = float(runtime.get("simulation_time_delta", 0.0))
    wall_seconds = float(runtime.get("wall_seconds", 0.0))
    realtime_factor = sim_delta / wall_seconds if wall_seconds > 0.0 else 0.0
    dynamic_obstacles = dynamic.get("obstacles", [])
    if not isinstance(dynamic_obstacles, list):
        dynamic_obstacles = []
    dynamic_valid = (
        bool(dynamic.get("enabled"))
        and len(dynamic_obstacles) == 3
        and all(float(item.get("distance_travelled_m", 0.0)) > 0.20 for item in dynamic_obstacles)
        if experiment_class == "dynamic"
        else not bool(dynamic.get("enabled"))
    )
    # Both monitors observe the same PhysX event stream. Use the larger count
    # instead of double-counting every robot/dynamic-obstacle contact.
    collision_count = max(
        int(contacts.get("collision_event_count", 0)),
        int(dynamic.get("robot_contact_count", 0)),
    )
    thresholds = stage10["preacceptance"]["thresholds"]
    smoothness_limits = {
        "linear_acceleration_p95_mps2": float(
            thresholds["normal_linear_acceleration_p95_mps2"]
        ),
        "angular_acceleration_p95_radps2": float(
            thresholds["normal_angular_acceleration_p95_radps2"]
        ),
        "linear_jerk_p95_mps3": float(
            thresholds["normal_linear_jerk_p95_mps3"]
        ),
        "angular_jerk_p95_radps3": float(
            thresholds["normal_angular_jerk_p95_radps3"]
        ),
    }
    normal_navigation_smooth = bool(goals) and all(
        isinstance(goal, dict)
        and isinstance(goal.get("command_smoothness"), dict)
        and all(
            math.isfinite(float(goal["command_smoothness"].get(metric, math.inf)))
            and float(goal["command_smoothness"].get(metric, math.inf)) <= limit
            for metric, limit in smoothness_limits.items()
        )
        for goal in goals
    )
    checks = {
        "runner_exit_success": args.runner_exit_code == 0,
        "navigation_passed": navigation.get("status") == "passed",
        "simulator_passed": simulator.get("status") == "passed",
        "official_assets_unchanged": simulator.get("official_assets_unchanged") is True,
        "simulation_time_monotonic": runtime.get("time_regressions") == 0
        and sim_delta > 0.0,
        "no_initial_or_final_overlap": runtime.get("initial_unexpected_overlaps") == []
        and runtime.get("final_unexpected_overlaps") == [],
        "no_physx_collision": collision_count == int(thresholds["collision_count"]),
        "scenario_class_valid": dynamic_valid,
        "front_stereo_only": metadata.get("scope", {}).get("front_stereo_only") is True,
        "lighting_and_color_out_of_scope": (
            metadata.get("scope", {}).get("lighting_randomization") is False
            and metadata.get("scope", {}).get("color_randomization") is False
        ),
        "realtime_factor": realtime_factor
        >= float(thresholds["minimum_realtime_factor"]),
        "trajectory_recorded": (run_dir / "trajectory.csv").is_file()
        and (run_dir / "trajectory.csv").stat().st_size > 100,
        "gpu_recorded": (run_dir / "gpu.csv").is_file(),
        "normal_navigation_smoothness": normal_navigation_smooth,
    }
    bag_bytes = directory_bytes(run_dir / "rosbag")
    if args.require_rosbag:
        checks["compact_mcap_recorded"] = bag_bytes >= 10_000
    result = {
        "status": "passed" if all(checks.values()) else "failed",
        "stage": 10,
        "experiment_class": experiment_class,
        "seed": metadata.get("seed"),
        "goal_index": metadata.get("goal_index"),
        "goal_name": metadata.get("goal_name"),
        "goal_pose": metadata.get("goal_pose"),
        "checks": checks,
        "collision_count": collision_count,
        "path": {
            "actual_length_m": actual_path,
            "reference_plan_length_m": reference_path,
            "stretch": max(path_stretches, default=math.inf),
            "reference_kind": "Nav2 SmacPlanner2D global plan (Stage 10 pre-acceptance)",
        },
        "goal_results": goals,
        "realtime_factor": realtime_factor,
        "gpu": gpu_summary(run_dir / "gpu.csv"),
        "rosbag_bytes": bag_bytes,
        "fault_results": navigation.get("fault_results", []),
        "normal_navigation_smoothness_limits": smoothness_limits,
        "navigation_report": str((run_dir / "navigation.json").resolve()),
        "simulator_report": str((run_dir / "simulator.json").resolve()),
        "scenario": scenario.get("experiment", {}),
    }
    (run_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
