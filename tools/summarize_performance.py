#!/usr/bin/env python3
"""Normalize one real adaptive performance observation without inventing a KPI."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = quantile * (len(ordered) - 1)
    lower, upper = math.floor(rank), math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def summarize(values: list[float]) -> dict[str, float | int]:
    return {
        "sample_count": len(values),
        "mean": statistics.fmean(values) if values else 0.0,
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "minimum": min(values, default=0.0),
        "maximum": max(values, default=0.0),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
    }


def official_metrics(performance: dict[str, object]) -> dict[str, object]:
    sample = performance.get("sample", {})
    if not isinstance(sample, dict):
        return {}
    rows = sample.get("official_measurements", [])
    if not isinstance(rows, list):
        return {}
    return {
        str(row.get("name")): row.get("value", row.get("bvalue"))
        for row in rows
        if isinstance(row, dict) and row.get("name")
    }


def telemetry_summary(
    path: Path, start_unix_s: float, end_unix_s: float
) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [
            row
            for row in csv.DictReader(stream)
            if start_unix_s <= float(row["timestamp_unix_s"]) <= end_unix_s
        ]
    numeric = {
        "project_rss_gib": "project_rss_gib",
        "project_vms_gib": "project_vms_gib",
        "project_uss_gib": "project_uss_gib",
        "system_memory_used_gib": "system_memory_used_gib",
        "gpu_utilization_percent": "gpu_utilization_percent",
        "gpu_memory_used_mib": "gpu_memory_used_mib",
        "gpu_power_w": "gpu_power_w",
        "gpu_temperature_c": "gpu_temperature_c",
    }
    return {
        "sample_count": len(rows),
        "gpu_name": rows[0]["gpu_name"] if rows else "unavailable",
        "system_memory_total_gib": float(rows[0]["system_memory_total_gib"])
        if rows
        else 0.0,
        **{
            label: summarize([float(row[field]) for row in rows])
            for label, field in numeric.items()
        },
    }


def host_context() -> dict[str, object]:
    cpu_model = "unknown"
    for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
        if line.startswith("model name"):
            cpu_model = line.split(":", 1)[1].strip()
            break
    governors = sorted(
        {
            path.read_text(encoding="utf-8").strip()
            for path in Path("/sys/devices/system/cpu").glob(
                "cpu[0-9]*/cpufreq/scaling_governor"
            )
            if path.is_file()
        }
    )
    try:
        driver = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=driver_version",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        ).stdout.splitlines()[0].strip()
    except (OSError, subprocess.SubprocessError, IndexError):
        driver = "unavailable"
    return {
        "operating_system": platform.platform(),
        "kernel": platform.release(),
        "cpu_model": cpu_model,
        "cpu_governors": governors,
        "nvidia_driver_version": driver,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-report", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workload", choices=["mapping", "navigation"], required=True)
    parser.add_argument("--workload-report", type=Path, required=True)
    parser.add_argument("--capture-report", type=Path, default=None)
    args = parser.parse_args()
    simulator = json.loads(args.sim_report.read_text(encoding="utf-8"))
    performance = simulator.get("performance", {})
    if not isinstance(performance, dict) or performance.get("completed") is not True:
        raise RuntimeError("simulator report does not contain a completed adaptive sample")
    sample = performance["sample"]
    start = float(sample["start_unix_s"])
    end = float(sample["end_unix_s"])
    metrics = official_metrics(performance)
    workload_report = json.loads(args.workload_report.read_text(encoding="utf-8"))
    if (
        workload_report.get("status") != "passed"
        or workload_report.get("mode") != args.workload
        or workload_report.get("active_workload_confirmed") is not True
        or workload_report.get("physical_motion_confirmed") is not True
        or int(workload_report.get("commands", {}).get("nonzero_samples", 0)) <= 0
    ):
        raise RuntimeError("performance sample lacks a confirmed active workload")
    capture_report = None
    if args.workload == "mapping":
        if args.capture_report is None:
            raise RuntimeError("mapping performance requires temporary MCAP evidence")
        capture_report = json.loads(args.capture_report.read_text(encoding="utf-8"))
        if (
            capture_report.get("status") != "passed"
            or capture_report.get("active_rgb_streams") != 8
            or capture_report.get("all_mapping_streams_recorded") is not True
        ):
            raise RuntimeError("mapping performance MCAP did not record all eight streams")
    telemetry = telemetry_summary(args.telemetry, start, end)
    if telemetry["sample_count"] <= 0:
        raise RuntimeError("whole-workload telemetry has no samples in benchmark window")
    result = {
        "schema_version": 1,
        "status": "recorded",
        "workload": args.workload,
        "camera_profile": performance.get("camera_profile"),
        "measurement_policy": {
            "adaptive_wall_time": True,
            "fixed_frame_count": None,
            "fixed_kpi_thresholds": None,
            "pass_fail_comparison_to_documentation_example": False,
        },
        "sampling": {
            "parameters": performance.get("parameters"),
            "warmup": performance.get("warmup"),
            "sample_duration_s": sample.get("duration_s"),
            "sample_stop_reason": sample.get("stop_reason"),
            "sample_stability_reached": sample.get("stability_reached"),
        },
        "active_workload": workload_report,
        "temporary_mapping_capture": capture_report,
        "official_isaac_sim_6_0_1": {
            "mean_fps": metrics.get("Mean FPS"),
            "real_time_factor": metrics.get("Real Time Factor"),
            "app_update_frametime_ms": sample.get("app_update_frametime_ms"),
            "physics_frametime_ms": sample.get("physics_frametime_ms"),
            "system_memory_rss_gib": metrics.get("System Memory RSS"),
            "system_memory_vms_gib": metrics.get("System Memory VMS"),
            "system_memory_uss_gib": metrics.get("System Memory USS"),
            "gpu_memory_tracked_gib": metrics.get("GPU Memory Tracked"),
            "gpu_memory_dedicated_gib": metrics.get("GPU Memory Dedicated"),
            "mean_cpu_usage_percent": metrics.get("Mean CPU Usage"),
            "max_cpu_usage_percent": metrics.get("Max CPU Usage"),
            "num_cpus": metrics.get("num_cpus"),
            "gpu_device_name": metrics.get("gpu_device_name"),
        },
        "whole_workload_telemetry": telemetry,
        "host_context": host_context(),
        "simulator_report": str(args.sim_report.resolve()),
        "telemetry_csv": str(args.telemetry.resolve()),
        "workload_report": str(args.workload_report.resolve()),
        "capture_report": str(args.capture_report.resolve())
        if args.capture_report is not None
        else None,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
