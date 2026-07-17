#!/usr/bin/env python3
"""Write the portable Stage 7 map manifest after validating every artifact group."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time


def count(path: Path) -> int:
    return sum(1 for item in path.rglob("*") if item.is_file() and item.stat().st_size > 0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    groups = ("cuvslam", "cuvgl", "nvblox", "mesh", "occupancy", "config")
    files = {name: count(args.map_dir / name) for name in groups}
    missing = [name for name, value in files.items() if value == 0]
    if missing:
        raise RuntimeError(f"map artifact groups are empty: {missing}")
    manifest = {
        "created_unix_s": time.time(),
        "run_id": args.run_id,
        "map_frame": "map",
        "bag": str(args.bag_dir.resolve()),
        "asset_paths": {
            "warehouse": os.environ["WAREHOUSE_USD"],
            "robot": os.environ["NOVA_CARTER_USD"],
        },
        "software": {"ros": "jazzy", "isaac_ros": "4.5", "isaac_sim": "6.0.1"},
        "camera": {
            "count": 2,
            "stereo_pairs": 1,
            "resolution": [1280, 800],
            "encoding": "mono8",
            "nominal_rate_hz": 30,
        },
        "files": files,
        "generation_command": f"./scripts/run_mapping.sh --map {args.map_dir.name}",
    }
    (args.map_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
