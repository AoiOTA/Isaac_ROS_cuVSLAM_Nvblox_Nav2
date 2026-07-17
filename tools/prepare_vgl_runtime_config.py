#!/usr/bin/env python3
"""Copy a cuVGL config tree and apply measured multi-camera runtime limits."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--max-sync-us", type=int, default=40_000)
    args = parser.parse_args()
    if not args.source.is_dir():
        raise FileNotFoundError(f"cuVGL config source is missing: {args.source}")
    if not 3_000 <= args.max_sync_us <= 100_000:
        raise ValueError("max-sync-us must be between 3000 and 100000")
    if args.destination.exists():
        shutil.rmtree(args.destination)
    shutil.copytree(args.source, args.destination)
    localizer = args.destination / "localizer_config.pb.txt"
    text = localizer.read_text(encoding="utf-8")
    pattern = r"(?m)^max_synchronous_timestamps_microseconds_range:\s*\d+\s*$"
    replacement = (
        "max_synchronous_timestamps_microseconds_range: "
        f"{args.max_sync_us}"
    )
    updated, count = re.subn(pattern, replacement, text)
    if count != 1:
        raise RuntimeError(
            "expected exactly one max_synchronous_timestamps_microseconds_range"
        )
    localizer.write_text(updated, encoding="utf-8")
    print(
        f"prepared cuVGL runtime config: {args.destination} "
        f"max_sync_us={args.max_sync_us}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
