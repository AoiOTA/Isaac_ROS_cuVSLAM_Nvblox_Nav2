#!/usr/bin/env python3
"""Create a local mapping-vs-navigation observation report with no KPI gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("reports", nargs="+", type=Path)
    args = parser.parse_args()
    profiles = [json.loads(path.read_text(encoding="utf-8")) for path in args.reports]
    if any(item.get("status") != "recorded" for item in profiles):
        raise RuntimeError("all performance profiles must be recorded successfully")
    result = {
        "schema_version": 1,
        "status": "recorded",
        "interpretation": "local observation; no fixed frame count and no preset KPI gate",
        "profiles": profiles,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Kujiale Jackal local performance observation",
        "",
        "This report uses adaptive wall-time sampling after the live workload stabilizes. It has no 600-frame baseline and no pass/fail comparison with documentation example output.",
        "",
        "| Workload | Cameras | Mean FPS | RTF | App mean ms | Physics mean ms | GPU mean % | GPU memory mean MiB |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for profile in profiles:
        official = profile["official_isaac_sim_6_0_1"]
        telemetry = profile["whole_workload_telemetry"]
        lines.append(
            "| {workload} | {cameras} | {fps} | {rtf} | {app:.3f} | {physics:.3f} | {gpu:.2f} | {memory:.1f} |".format(
                workload=profile["workload"],
                cameras=8 if profile["camera_profile"] == "mapping_8cam" else 6,
                fps=official["mean_fps"],
                rtf=official["real_time_factor"],
                app=float(official["app_update_frametime_ms"]["mean"]),
                physics=float(official["physics_frametime_ms"]["mean"]),
                gpu=float(telemetry["gpu_utilization_percent"]["mean"]),
                memory=float(telemetry["gpu_memory_used_mib"]["mean"]),
            )
        )
    args.output.with_suffix(".md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
