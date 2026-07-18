#!/usr/bin/env python3
"""Validate the simulator half of a Stage 9 static-obstacle run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


EXPECTED_GRAPHS = {
    "/World/Graphs/FrontStereo",
    "/World/Graphs/FrontDepth",
    "/World/Graphs/FrontImu",
}
EXPECTED_KINDS = {"box", "capsule"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    sensors = report.get("sensor_graphs", {})
    statics = report.get("static_obstacles", {})
    dynamics = report.get("dynamic_obstacles", {})
    runtime = report.get("runtime", {})
    obstacles = statics.get("obstacles", [])
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
        "static_profile_enabled": statics.get("enabled") is True
        and statics.get("profile") == "warehouse_manual_static",
        "static_shapes_present": len(obstacles) == 3
        and {item.get("kind") for item in obstacles} == EXPECTED_KINDS,
        "static_shapes_have_fixed_positions": all(
            len(item.get("position_m", [])) == 3 for item in obstacles
        ),
        "dynamic_profile_disabled": dynamics.get("enabled") is False,
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
