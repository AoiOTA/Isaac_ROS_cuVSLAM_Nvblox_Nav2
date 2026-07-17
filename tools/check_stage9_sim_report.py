#!/usr/bin/env python3
"""Validate the simulator half of a Stage 9 dynamic-navigation run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


EXPECTED_GRAPHS = {
    "/World/Graphs/FrontStereo",
    "/World/Graphs/FrontDepth",
    "/World/Graphs/FrontImu",
}
EXPECTED_KINDS = {"existing_forklift", "box", "capsule"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    sensors = report.get("sensor_graphs", {})
    dynamics = report.get("dynamic_obstacles", {})
    runtime = report.get("runtime", {})
    obstacles = dynamics.get("obstacles", [])
    checks = {
        "simulator_passed": report.get("status") == "passed",
        "stage_opened_once": report.get("stage_open_count") == 1,
        "official_assets_unchanged": report.get("official_assets_unchanged") is True,
        "project_owns_front_sensor_graphs": set(sensors.get("graph_paths", []))
        == EXPECTED_GRAPHS,
        "official_ros_graph_not_loaded": sensors.get("official_ros_sample_loaded") is False,
        "surround_graphs_disabled": sensors.get("surround_enabled") is False,
        "surround_gate_is_fixed": sensors.get("surround_enable_topic")
        == "/vgl/cameras_enabled",
        "surround_rate_is_10_hz": sensors.get("surround_rate_hz") == 10.0,
        "surround_resolution_is_bounded": sensors.get("surround_resolution")
        == [1280, 800],
        "dynamic_profile_enabled": dynamics.get("enabled") is True
        and dynamics.get("profile") == "warehouse_crossing",
        "all_dynamic_shapes_present": {item.get("kind") for item in obstacles}
        == EXPECTED_KINDS,
        "all_dynamic_shapes_moved": len(obstacles) == 3
        and all(float(item.get("distance_travelled_m", 0.0)) >= 0.20 for item in obstacles),
        "no_robot_dynamic_contact": dynamics.get("robot_contact_count") == 0
        and dynamics.get("robot_contact_pairs") == [],
        "simulation_time_monotonic": runtime.get("time_regressions") == 0
        and float(runtime.get("simulation_time_delta", 0.0)) > 0.0,
        "robot_has_no_final_overlap": runtime.get("final_unexpected_overlaps") == [],
    }
    output = {"status": "passed" if all(checks.values()) else "failed", "checks": checks}
    print(json.dumps(output, indent=2, sort_keys=True))
    if output["status"] != "passed":
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"Stage 9 simulator checks failed: {failed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
