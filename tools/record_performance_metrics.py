#!/usr/bin/env python3
"""Record whole-workload host and GPU telemetry at a fixed wall-time cadence."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import time


STOP = False


def request_stop(_signum: int, _frame: object) -> None:
    global STOP
    STOP = True


def descendants(roots: list[int]) -> set[int]:
    pending = list(roots)
    result: set[int] = set()
    while pending:
        pid = pending.pop()
        if pid in result or not Path(f"/proc/{pid}").is_dir():
            continue
        result.add(pid)
        children_path = Path(f"/proc/{pid}/task/{pid}/children")
        try:
            pending.extend(int(item) for item in children_path.read_text().split())
        except (OSError, ValueError):
            pass
    return result


def process_memory(pids: set[int]) -> tuple[int, int, int, int]:
    page_size = os.sysconf("SC_PAGE_SIZE")
    rss = vms = uss = 0
    sampled = 0
    for pid in pids:
        try:
            fields = Path(f"/proc/{pid}/statm").read_text().split()
            vms += int(fields[0]) * page_size
            rss += int(fields[1]) * page_size
            private_kib = 0
            for line in Path(f"/proc/{pid}/smaps_rollup").read_text().splitlines():
                if line.startswith(("Private_Clean:", "Private_Dirty:")):
                    private_kib += int(line.split()[1])
            uss += private_kib * 1024
            sampled += 1
        except (OSError, ValueError, IndexError):
            continue
    return sampled, rss, vms, uss


def system_memory() -> tuple[int, int]:
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        name, value = line.split(":", 1)
        values[name] = int(value.split()[0]) * 1024
    total = values["MemTotal"]
    return total, total - values["MemAvailable"]


def gpu_metrics() -> list[str]:
    completed = subprocess.run(
        [
            "nvidia-smi",
            "--query-gpu=name,utilization.gpu,memory.used,memory.total,power.draw,temperature.gpu",
            "--format=csv,noheader,nounits",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    return [item.strip() for item in completed.stdout.splitlines()[0].split(",")]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--pid", type=int, action="append", required=True)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()
    if args.period <= 0.0:
        raise ValueError("sample period must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    fields = [
        "timestamp_unix_s",
        "project_process_count",
        "project_rss_gib",
        "project_vms_gib",
        "project_uss_gib",
        "system_memory_used_gib",
        "system_memory_total_gib",
        "gpu_name",
        "gpu_utilization_percent",
        "gpu_memory_used_mib",
        "gpu_memory_total_mib",
        "gpu_power_w",
        "gpu_temperature_c",
    ]
    gib = float(1024**3)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        while not STOP and not args.stop_file.exists():
            deadline = time.monotonic() + args.period
            try:
                count, rss, vms, uss = process_memory(descendants(args.pid))
                total, used = system_memory()
                gpu = gpu_metrics()
                writer.writerow(
                    [
                        f"{time.time():.6f}",
                        count,
                        f"{rss / gib:.6f}",
                        f"{vms / gib:.6f}",
                        f"{uss / gib:.6f}",
                        f"{used / gib:.6f}",
                        f"{total / gib:.6f}",
                        *gpu,
                    ]
                )
                stream.flush()
            except (OSError, subprocess.SubprocessError, IndexError, ValueError):
                pass
            while not STOP and not args.stop_file.exists() and time.monotonic() < deadline:
                time.sleep(min(0.1, deadline - time.monotonic()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
