#!/usr/bin/env python3
"""Validate persisted Kujiale/Jackal runtime-composition evidence."""

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
    require(
        report.get("stage_open_count") == 1,
        "Kujiale stage was not opened exactly once",
    )
    require(
        report.get("timeline_stopped_on_exit") is True,
        "timeline was not stopped on exit",
    )
    require(
        report.get("official_assets_unchanged") is True,
        "official USD files changed",
    )
    require(
        report.get("assets_before") == report.get("assets_after"),
        "official asset fingerprints differ",
    )
    config_path = Path(str(report.get("config", "")))
    require(config_path.is_file(), "persisted simulation config path does not exist")

    composition = report.get("composition", {})
    require(composition.get("robot_prim") == "/World/Jackal", "robot prim is wrong")
    require(
        composition.get("articulation_root") == "/World/Jackal",
        "articulation root is wrong",
    )
    require(
        composition.get("base_link_prim") == "/World/Jackal/base_link",
        "base-link prim is wrong",
    )
    require(
        composition.get("session_layer_anonymous") is True,
        "session layer is not anonymous",
    )
    require(
        composition.get("wheel_overlay") == "runtime_reference_caae0c08",
        "validated Jackal wheel overlay is missing",
    )
    require(
        composition.get("physics_scene") == "/World/PhysicsScene",
        "PhysicsScene is wrong",
    )
    environment_path = (
        report.get("assets_before", {}).get("environment", {}).get("path")
    )
    require(
        composition.get("root_layer") == environment_path,
        "root layer is not the selected Kujiale environment",
    )
    require(
        composition.get("root_layer_dirty_before_session")
        == composition.get("root_layer_dirty_after_session"),
        "session composition changed the root-layer dirty state",
    )
    require(
        composition.get("used_layer_count", 0) > 1,
        "composed layer stack is incomplete",
    )

    required_prims = set(composition.get("required_prims", []))
    for path in (
        "/World/Jackal/base_link",
        "/World/Jackal/front_left_wheel_joint",
        "/World/Jackal/front_right_wheel_joint",
        "/World/Jackal/rear_left_wheel_joint",
        "/World/Jackal/rear_right_wheel_joint",
    ):
        require(path in required_prims, f"required composed prim was not proven: {path}")

    repairs = composition.get("environment_repairs", {})
    require(
        repairs.get("double_sided_mesh_count", 0) > 0,
        "Kujiale meshes were not made double-sided in the session layer",
    )
    require(
        repairs.get("collision_prim_count", 0) > 0,
        "Kujiale collision prims were not discovered",
    )
    require(
        repairs.get("source_physics_material_preserved") is True
        and repairs.get("physics_material_binding_root") is None,
        "Kujiale source contact material was overridden",
    )

    hawk_pairs = composition.get("hawk_rig", {}).get("pairs", {})
    require(
        set(hawk_pairs) == {"front", "left", "right", "back"},
        "four cardinal Hawk pairs were not composed",
    )
    for name, pair in hawk_pairs.items():
        require(
            all(
                str(pair.get(field, "")).startswith(
                    "/World/Jackal/base_link/sensors/"
                )
                for field in ("root", "left", "right", "imu")
            ),
            f"{name} Hawk prim paths are invalid",
        )

    sensors = report.get("sensor_graphs", {})
    require(sensors.get("lidar_enabled") is False, "Jackal LiDAR was not disabled")

    spawn = composition.get("spawn", {})
    require(spawn.get("floor_count") == 1, "fixed spawn has no supporting floor")
    require(spawn.get("candidates_evaluated") == 1, "fixed spawn evidence is incomplete")
    require(spawn.get("floor_prim") == "/Root", "fixed spawn floor is wrong")
    require(
        all(
            abs(float(spawn.get(key, 0.0)) - expected) <= 1.0e-9
            for key, expected in (("x", 2.9), ("y", -0.2), ("z", 0.0635))
        ),
        "fixed Kujiale spawn is wrong",
    )

    runtime = report.get("runtime", {})
    require(runtime.get("frames", 0) > 0, "no simulation frames ran")
    require(
        runtime.get("simulation_time_delta", 0.0) > 0.0,
        "simulation time did not advance",
    )
    require(
        runtime.get("significant_time_regressions") == 0,
        "simulation time regressed significantly",
    )
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
    contacts = report.get("robot_contacts", {})
    require(
        contacts.get("collision_event_count") == 0,
        "unexpected Jackal collision was reported",
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
