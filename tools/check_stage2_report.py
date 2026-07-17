#!/usr/bin/env python3
"""Validate the persisted evidence from a phase-2 simulator run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("--mode", choices=("headless", "gui"))
    parser.add_argument("--minimum-wall-seconds", type=float, default=0.0)
    args = parser.parse_args()

    report = json.loads(args.report.read_text(encoding="utf-8"))
    failures: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            failures.append(message)

    require(report.get("status") == "passed", "run status is not passed")
    if args.mode is not None:
        require(report.get("mode") == args.mode, f"mode is not {args.mode}")
    require(report.get("stage_open_count") == 1, "warehouse was not opened exactly once")
    require(report.get("timeline_stopped_on_exit") is True, "timeline was not stopped on exit")
    require(report.get("official_assets_unchanged") is True, "official USD files changed")
    require(
        report.get("assets_before") == report.get("assets_after"),
        "official asset fingerprints differ",
    )
    config_path = Path(str(report.get("config", "")))
    require(config_path.is_file(), "persisted simulation config path does not exist")

    composition = report.get("composition", {})
    require(composition.get("robot_prim") == "/World/NovaCarter", "robot prim is wrong")
    require(composition.get("session_layer_anonymous") is True, "session layer is not anonymous")
    require(
        composition.get("forbidden_ros_sample_loaded") is False,
        "Nova_Carter_ROS.usd was loaded",
    )
    variants = composition.get("variants", {})
    require(variants.get("Physics") == "physx", "Physics variant is wrong")
    require(variants.get("Sensors") == "All_Sensors", "Sensors variant is wrong")
    require(variants.get("ROS") == "Disabled", "ROS variant is wrong")
    require(composition.get("omnigraph_count") == 0, "robot unexpectedly contains OmniGraph")
    require(composition.get("camera_count", 0) >= 12, "Nova Carter cameras are missing")
    require(composition.get("physics_scene") == "/PhysicsScene", "PhysicsScene is wrong")
    warehouse_path = report.get("assets_before", {}).get("warehouse", {}).get("path")
    require(composition.get("root_layer") == warehouse_path, "root layer is not the warehouse")
    require(
        composition.get("root_layer_dirty_before_session")
        == composition.get("root_layer_dirty_after_session"),
        "session composition changed the root-layer dirty state",
    )
    require(composition.get("used_layer_count", 0) > 1, "composed layer stack is incomplete")

    required_prims = set(composition.get("required_prims", []))
    for path in (
        "/World/NovaCarter/joint_wheel_left",
        "/World/NovaCarter/joint_wheel_right",
        "/World/NovaCarter/chassis_link",
    ):
        require(path in required_prims, f"required composed prim was not proven: {path}")

    spawn = composition.get("spawn", {})
    require(spawn.get("floor_count", 0) > 0, "spawn search found no collision floors")
    require(spawn.get("obstacle_count", 0) > 0, "spawn search considered no obstacles")
    require(spawn.get("candidates_evaluated", 0) > 0, "spawn search evaluated no candidates")
    require(bool(spawn.get("floor_prim")), "spawn has no supporting floor")

    runtime = report.get("runtime", {})
    require(runtime.get("frames", 0) > 0, "no simulation frames ran")
    require(runtime.get("simulation_time_delta", 0.0) > 0.0, "simulation time did not advance")
    require(runtime.get("time_regressions") == 0, "simulation time regressed")
    require(
        runtime.get("timeline_playing_during_probe") is True,
        "timeline was not playing during the runtime probe",
    )
    require(
        runtime.get("initial_unexpected_overlaps") == [],
        "robot initially overlapped an obstacle",
    )
    require(
        runtime.get("final_unexpected_overlaps") == [],
        "robot finally overlapped an obstacle",
    )
    require(
        runtime.get("wall_seconds", 0.0) + 0.05 >= args.minimum_wall_seconds,
        f"run was shorter than {args.minimum_wall_seconds} wall seconds",
    )
    start_pose = runtime.get("start_chassis_translation", [0.0, 0.0, 0.0])
    final_pose = runtime.get("final_chassis_translation", [0.0, 0.0, 0.0])
    require(
        len(start_pose) == 3
        and len(final_pose) == 3
        and abs(float(final_pose[2]) - float(start_pose[2])) <= 0.15,
        "robot chassis vertical pose is unstable",
    )

    result = {
        "ok": not failures,
        "report": str(args.report.resolve()),
        "mode": report.get("mode"),
        "frames": runtime.get("frames"),
        "wall_seconds": runtime.get("wall_seconds"),
        "simulation_time_delta": runtime.get("simulation_time_delta"),
        "spawn": {key: spawn.get(key) for key in ("x", "y", "z", "floor_prim")},
        "failures": failures,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"stage-2 report check failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
