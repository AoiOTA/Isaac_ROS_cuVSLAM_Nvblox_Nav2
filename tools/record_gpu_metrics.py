#!/usr/bin/env python3
"""Record lightweight GPU telemetry for one project-owned experiment."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import signal
import subprocess
import time


STOP = False


def request_stop(_signum: int, _frame: object) -> None:
    global STOP
    STOP = True


def sample() -> list[str]:
    command = [
        "nvidia-smi",
        "--query-gpu=index,name,utilization.gpu,memory.used,memory.total,power.draw,temperature.gpu",
        "--format=csv,noheader,nounits",
    ]
    completed = subprocess.run(
        command, check=True, capture_output=True, text=True, timeout=5.0
    )
    # The project uses the sole RTX 4090 by default; retain the first row if a
    # workstation later has more than one GPU.
    return [item.strip() for item in completed.stdout.splitlines()[0].split(",")]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()
    if args.period <= 0.0:
        raise ValueError("sample period must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    fields = [
        "timestamp_utc",
        "gpu_index",
        "gpu_name",
        "utilization_percent",
        "memory_used_mib",
        "memory_total_mib",
        "power_w",
        "temperature_c",
    ]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        while not STOP and not args.stop_file.exists():
            try:
                values = sample()
                writer.writerow(
                    [datetime.now(timezone.utc).isoformat(timespec="milliseconds"), *values]
                )
                stream.flush()
            except (OSError, subprocess.SubprocessError, IndexError):
                # Preserve experiment progress if telemetry briefly fails; the
                # missing sample is visible in the final count.
                pass
            deadline = time.monotonic() + args.period
            while not STOP and not args.stop_file.exists() and time.monotonic() < deadline:
                time.sleep(min(0.1, deadline - time.monotonic()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
