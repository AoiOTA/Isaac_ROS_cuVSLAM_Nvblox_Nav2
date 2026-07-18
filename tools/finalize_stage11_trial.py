#!/usr/bin/env python3
"""Merge one Stage 11 trial into a strict machine-readable result."""

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
    rank = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    low, high = math.floor(rank), math.ceil(rank)
    if low == high:
        return ordered[low]
    fraction = rank - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def directory_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file()) if path.exists() else 0


def gpu_summary(path: Path) -> dict[str, object]:
    utilization: list[float] = []
    memory: list[float] = []
    power: list[float] = []
    if path.is_file():
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


def nested_metric(report: dict[str, object], section: str, name: str, field: str) -> float:
    try:
        return float(report[section][name][field])
    except (KeyError, TypeError, ValueError):
        return math.inf


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--runner-exit-code", type=int, default=0)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--reference-paths", type=Path, required=True)
    parser.add_argument("--require-rosbag", action="store_true")
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    thresholds = config["acceptance"]["thresholds"]
    metadata = load_json(run_dir / "scenario.json")
    navigation = load_json(run_dir / "navigation.json")
    simulator = load_json(run_dir / "simulator.json")
    reference = load_json(args.reference_paths)
    experiment_class = str(metadata.get("experiment_class", "unknown"))
    goal_name = str(metadata.get("goal_name", "unknown"))
    goal_reference = reference.get("paths", {}).get(goal_name, {})
    goals = navigation.get("goals", [])
    if not isinstance(goals, list):
        goals = []
    runtime = simulator.get("runtime", {})
    contacts = simulator.get("robot_contacts", {})
    dynamic = simulator.get("dynamic_obstacles", {})
    if not isinstance(runtime, dict):
        runtime = {}
    if not isinstance(contacts, dict):
        contacts = {}
    if not isinstance(dynamic, dict):
        dynamic = {}
    obstacles = dynamic.get("obstacles", [])
    if not isinstance(obstacles, list):
        obstacles = []
    collision_count = max(
        int(contacts.get("collision_event_count", 0)),
        int(dynamic.get("robot_contact_count", 0)),
    )
    sim_delta = float(runtime.get("simulation_time_delta", 0.0))
    wall_seconds = float(runtime.get("wall_seconds", 0.0))
    realtime_factor = sim_delta / wall_seconds if wall_seconds > 0.0 else 0.0
    actual_length = sum(
        float(goal.get("actual_path_length_m", 0.0))
        for goal in goals
        if isinstance(goal, dict)
    )
    theoretical_length = float(goal_reference.get("optimal_path_length_m", 0.0))
    path_stretch = (
        max(0.0, actual_length / theoretical_length - 1.0)
        if theoretical_length > 0.05
        else math.inf
    )
    scope = metadata.get("scope", {})
    automation = metadata.get("automation", {})
    sensor_graphs = simulator.get("sensor_graphs", {})
    if not isinstance(sensor_graphs, dict):
        sensor_graphs = {}
    graph_paths = sensor_graphs.get("graph_paths", [])
    if not isinstance(graph_paths, list):
        graph_paths = []
    dynamic_valid = False
    if experiment_class == "static":
        dynamic_valid = not bool(dynamic.get("enabled")) and len(obstacles) == 0
    elif experiment_class == "dynamic":
        dynamic_valid = (
            bool(dynamic.get("enabled"))
            and len(obstacles) == 3
            and all(float(item.get("distance_travelled_m", 0.0)) > 0.20 for item in obstacles)
        )
    elif experiment_class == "heterogeneous":
        kinds = {str(item.get("kind")) for item in obstacles}
        dynamic_valid = (
            bool(dynamic.get("enabled"))
            and len(obstacles) >= 6
            and {"existing_forklift", "box", "capsule"} <= kinds
            and all(float(item.get("distance_travelled_m", 0.0)) > 0.20 for item in obstacles)
        )
    smoothness_limits = {
        "linear_acceleration_p95_mps2": float(thresholds["normal_linear_acceleration_p95_mps2"]),
        "angular_acceleration_p95_radps2": float(thresholds["normal_angular_acceleration_p95_radps2"]),
        "linear_jerk_p95_mps3": float(thresholds["normal_linear_jerk_p95_mps3"]),
        "angular_jerk_p95_radps3": float(thresholds["normal_angular_jerk_p95_radps3"]),
    }
    smooth = bool(goals) and all(
        isinstance(goal, dict)
        and isinstance(goal.get("command_smoothness"), dict)
        and all(
            math.isfinite(float(goal["command_smoothness"].get(metric, math.inf)))
            and float(goal["command_smoothness"].get(metric, math.inf)) <= limit
            for metric, limit in smoothness_limits.items()
        )
        for goal in goals
    )
    rates = navigation.get("observed_rates_hz", {})
    if not isinstance(rates, dict):
        rates = {}
    command_freshness = nested_metric(
        navigation,
        "command_latency_metrics",
        "cmd_nav_raw_to_cmd_sim_freshness",
        "p95_ms",
    )
    depth_age = nested_metric(
        navigation, "data_age_metrics", "front_depth", "p95_ms"
    )
    bag_bytes = directory_bytes(run_dir / "rosbag")
    long_distance = bool(metadata.get("long_distance", False))
    simulator_warehouse = (
        simulator.get("assets_before", {}).get("warehouse", {}).get("path")
        if isinstance(simulator.get("assets_before"), dict)
        else None
    )
    checks = {
        "runner_exit_success": args.runner_exit_code == 0,
        "navigation_passed": navigation.get("status") == "passed",
        "simulator_passed": simulator.get("status") == "passed",
        "official_assets_unchanged": simulator.get("official_assets_unchanged") is True,
        "simulation_time_monotonic": runtime.get("time_regressions") == 0 and sim_delta > 0.0,
        "no_initial_or_final_overlap": runtime.get("initial_unexpected_overlaps") == []
        and runtime.get("final_unexpected_overlaps") == [],
        "no_physx_collision": collision_count == int(thresholds["collision_count"]),
        "scenario_class_valid": dynamic_valid,
        "front_stereo_only": isinstance(scope, dict)
        and scope.get("front_stereo_only") is True
        and scope.get("lidar_enabled") is False
        and sensor_graphs.get("enabled") is True
        and sensor_graphs.get("surround_enabled") is False
        and set(graph_paths)
        == {
            "/World/Graphs/FrontStereo",
            "/World/Graphs/FrontDepth",
            "/World/Graphs/FrontImu",
        },
        "appearance_randomization_disabled": isinstance(scope, dict)
        and scope.get("lighting_randomization") is False
        and scope.get("color_randomization") is False,
        "realtime_factor": realtime_factor >= float(thresholds["minimum_realtime_factor"]),
        "trajectory_recorded": (run_dir / "trajectory.csv").is_file()
        and (run_dir / "trajectory.csv").stat().st_size > 100,
        "gpu_recorded": (run_dir / "gpu.csv").is_file(),
        "normal_navigation_smoothness": smooth,
        "usd_theoretical_path_available": theoretical_length > 0.05
        and reference.get("usd_path") == simulator_warehouse,
        "long_distance_exercised": not long_distance
        or actual_length >= float(thresholds["minimum_long_distance_path_m"]),
        "low_latency_command_chain": command_freshness
        <= float(thresholds["command_raw_to_sim_freshness_p95_ms"]),
        "fresh_front_depth": depth_age <= float(thresholds["front_depth_age_p95_ms"]),
        "controller_rate": float(rates.get("cmd_nav_raw", 0.0))
        >= float(thresholds["minimum_cmd_nav_rate_hz"]),
        "visual_localization_rate": float(rates.get("visual_slam_status", 0.0))
        >= float(thresholds["minimum_visual_slam_status_rate_hz"]),
        "front_depth_rate": float(rates.get("depth_scan", 0.0))
        >= float(thresholds["minimum_front_depth_rate_hz"]),
        "nvblox_slice_rate": float(rates.get("nvblox_slice", 0.0))
        >= float(thresholds["minimum_nvblox_slice_rate_hz"]),
        "no_blind_motion": float(
            navigation.get("max_command_while_unready_after_grace", math.inf)
        )
        <= float(thresholds["maximum_command_while_localization_unready"]),
        "no_manual_intervention": isinstance(automation, dict)
        and automation.get("goal_dispatch") == "navigation_test_runner"
        and automation.get("manual_intervention") is False,
    }
    if args.require_rosbag:
        checks["compact_mcap_recorded"] = bag_bytes >= 10_000
    result = {
        "status": "passed" if all(checks.values()) else "failed",
        "stage": 11,
        "experiment_class": experiment_class,
        "seed": metadata.get("seed"),
        "goal_index": metadata.get("goal_index"),
        "goal_name": goal_name,
        "goal_pose": metadata.get("goal_pose"),
        "long_distance": long_distance,
        "checks": checks,
        "collision_count": collision_count,
        "path": {
            "actual_length_m": actual_length,
            "theoretical_optimal_length_m": theoretical_length,
            "stretch": path_stretch,
            "reference_kind": reference.get("method"),
            "reference_resolution_m": reference.get("resolution_m"),
        },
        "goal_results": goals,
        "realtime_factor": realtime_factor,
        "command_latency_metrics": navigation.get("command_latency_metrics", {}),
        "data_age_metrics": navigation.get("data_age_metrics", {}),
        "observed_rates_hz": rates,
        "smoothness_limits": smoothness_limits,
        "dynamic_obstacles": {
            "count": len(obstacles),
            "kinds": sorted({str(item.get("kind")) for item in obstacles}),
            "minimum_distance_travelled_m": min(
                (float(item.get("distance_travelled_m", 0.0)) for item in obstacles),
                default=0.0,
            ),
            "yield_events": sum(int(item.get("yield_event_count", 0)) for item in obstacles),
        },
        "gpu": gpu_summary(run_dir / "gpu.csv"),
        "rosbag_bytes": bag_bytes,
        "reference_paths": str(args.reference_paths.resolve()),
        "navigation_report": str((run_dir / "navigation.json").resolve()),
        "simulator_report": str((run_dir / "simulator.json").resolve()),
    }
    (run_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
