#!/usr/bin/env python3
"""Open the fixed USDs and verify the minimal Nova Carter asset contract."""

from __future__ import annotations

import json
import os
from pathlib import Path

from pxr import Usd, UsdGeom


def open_stage(path: Path) -> Usd.Stage:
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"USD stage could not be opened: {path}")
    return stage


def main() -> int:
    warehouse_path = Path(os.environ["WAREHOUSE_USD"])
    robot_path = Path(os.environ["NOVA_CARTER_USD"])
    ros_sample_path = Path(os.environ["NOVA_CARTER_ROS_SAMPLE_USD"])

    for path in (warehouse_path, robot_path, ros_sample_path):
        if not path.is_file():
            raise RuntimeError(f"required asset is missing: {path}")

    warehouse = open_stage(warehouse_path)
    robot = open_stage(robot_path)

    warehouse_default = warehouse.GetDefaultPrim().GetPath().pathString
    robot_default = robot.GetDefaultPrim().GetPath().pathString
    if warehouse_default != "/World":
        raise RuntimeError(f"unexpected warehouse default prim: {warehouse_default}")
    if robot_default != "/nova_carter":
        raise RuntimeError(f"unexpected Nova Carter default prim: {robot_default}")

    required_joints = [
        "/nova_carter/joint_wheel_left",
        "/nova_carter/joint_wheel_right",
    ]
    missing_joints = [path for path in required_joints if not robot.GetPrimAtPath(path).IsValid()]
    if missing_joints:
        raise RuntimeError(f"required wheel joints are missing: {missing_joints}")

    robot_prims = list(robot.Traverse())
    camera_count = sum(1 for prim in robot_prims if prim.IsA(UsdGeom.Camera))
    graph_prims = [
        prim.GetPath().pathString
        for prim in robot_prims
        if "omnigraph" in prim.GetTypeName().lower()
    ]
    if camera_count < 2:
        raise RuntimeError(f"expected stereo cameras in Nova Carter, found {camera_count}")
    if graph_prims:
        raise RuntimeError(f"main Nova Carter asset unexpectedly contains OmniGraph prims: {graph_prims}")

    used_identifiers = [layer.identifier for layer in robot.GetUsedLayers()]
    if any("Nova_Carter_ROS.usd" in identifier for identifier in used_identifiers):
        raise RuntimeError("main Nova Carter asset unexpectedly references Nova_Carter_ROS.usd")

    report = {
        "warehouse": {
            "path": str(warehouse_path),
            "default_prim": warehouse_default,
            "prim_count": sum(1 for _ in warehouse.Traverse()),
            "used_layer_count": len(warehouse.GetUsedLayers()),
        },
        "robot": {
            "path": str(robot_path),
            "default_prim": robot_default,
            "prim_count": len(robot_prims),
            "used_layer_count": len(robot.GetUsedLayers()),
            "camera_count": camera_count,
            "omnigraph_count": len(graph_prims),
            "drive_joints": required_joints,
        },
        "ros_sample": {"path": str(ros_sample_path), "policy": "reference_only_never_load"},
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, RuntimeError) as exc:
        print(f"asset check failed: {exc}")
        raise SystemExit(1) from exc
