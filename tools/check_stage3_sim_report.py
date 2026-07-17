#!/usr/bin/env python3
"""Validate the simulator-side Phase 3 contract recorded at shutdown."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


EXPECTED_GRAPHS = {
    "/World/Graphs/Clock",
    "/World/Graphs/DifferentialDrive",
    "/World/Graphs/JointState",
    "/World/Graphs/GroundTruth",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    if report.get("status") != "passed":
        raise SystemExit(f"simulator report did not pass: {report.get('error')}")
    if report.get("stage_open_count") != 1:
        raise SystemExit("warehouse stage was not opened exactly once")
    if report.get("official_assets_unchanged") is not True:
        raise SystemExit("official asset fingerprints changed")
    control = report.get("control_graphs", {})
    if control.get("enabled") is not True:
        raise SystemExit("runtime control graphs were not enabled")
    if set(control.get("graph_paths", [])) != EXPECTED_GRAPHS:
        raise SystemExit(f"unexpected runtime graphs: {control.get('graph_paths')}")
    if control.get("commanded_joints") != ["joint_wheel_left", "joint_wheel_right"]:
        raise SystemExit("articulation command contains joints other than the two active wheels")
    if control.get("official_ros_sample_loaded") is not False:
        raise SystemExit("official ROS sample graph must remain unloaded")
    print(
        "stage3_sim_report=passed "
        f"graphs={len(EXPECTED_GRAPHS)} joints={','.join(control['commanded_joints'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
