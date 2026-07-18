#!/usr/bin/env python3
"""Validate the complete simulator-side Phase 4 graph contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from check_stage3_sim_report import EXPECTED_GRAPHS as CONTROL_GRAPHS


CAMERA_PROFILES = {
    "mapping_8cam": ("front", "left", "right", "back"),
    "navigation_6cam": ("front", "left", "right"),
}


def stereo_graph(pair: str) -> str:
    name = "FrontStereo" if pair == "front" else f"{pair.title()}Stereo"
    return f"/World/Graphs/{name}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    if report.get("status") != "passed":
        raise SystemExit(f"simulator report did not pass: {report.get('error')}")
    if report.get("stage_open_count") != 1 or report.get("official_assets_unchanged") is not True:
        raise SystemExit("stage-open or official-asset immutability contract failed")
    control = report.get("control_graphs", {})
    sensors = report.get("sensor_graphs", {})
    profile = sensors.get("camera_profile")
    if profile not in CAMERA_PROFILES:
        raise SystemExit(f"unexpected camera profile: {profile}")
    active_pairs = CAMERA_PROFILES[profile]
    expected_sensor_graphs = {
        *(stereo_graph(pair) for pair in active_pairs),
        "/World/Graphs/FrontDepth",
        "/World/Graphs/FrontImu",
    }
    if set(control.get("graph_paths", [])) != CONTROL_GRAPHS:
        raise SystemExit(f"unexpected control graphs: {control.get('graph_paths')}")
    if set(sensors.get("graph_paths", [])) != expected_sensor_graphs:
        raise SystemExit(f"unexpected sensor graphs: {sensors.get('graph_paths')}")
    if sensors.get("active_pairs") != list(active_pairs):
        raise SystemExit(f"unexpected active camera pairs: {sensors.get('active_pairs')}")
    if sensors.get("active_image_streams") != 2 * len(active_pairs):
        raise SystemExit("unexpected active RGB stream count")
    if control.get("official_ros_sample_loaded") is not False:
        raise SystemExit("official ROS sample control graph was loaded")
    if sensors.get("official_ros_sample_loaded") is not False:
        raise SystemExit("official ROS sample sensor graph was loaded")
    if sensors.get("stereo_resolution") != [1280, 800]:
        raise SystemExit("unexpected stereo resolution")
    if sensors.get("depth_resolution") != [640, 400]:
        raise SystemExit("unexpected depth resolution")
    if sensors.get("image_rate_hz") != 10.0 or sensors.get("imu_rate_hz") != 120.0:
        raise SystemExit("unexpected authored sensor rates")
    timeline = report.get("timeline", {})
    if timeline.get("looping") is not False:
        raise SystemExit("timeline must be non-looping for monotonic ROS timestamps")
    print(
        "stage4_sim_report=passed "
        f"profile={profile} streams={2 * len(active_pairs)} "
        f"control_graphs=4 sensor_graphs={len(expected_sensor_graphs)} stereo=1280x800"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
