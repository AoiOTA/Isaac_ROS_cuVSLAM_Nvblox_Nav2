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
    parser.add_argument("--camera-count", type=int, default=2)
    parser.add_argument("--stereo-pairs", type=int, default=1)
    parser.add_argument("--nominal-rate-hz", type=int, default=30)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=800)
    parser.add_argument("--surround-image-width", type=int)
    parser.add_argument("--surround-image-height", type=int)
    parser.add_argument("--surround-rate-hz", type=int)
    parser.add_argument(
        "--generation-command", default="./scripts/run_mapping.sh --map {map_name}"
    )
    args = parser.parse_args()
    if args.camera_count != 2 * args.stereo_pairs:
        raise ValueError("camera count must equal twice the stereo-pair count")
    if min(args.image_width, args.image_height) <= 0:
        raise ValueError("image dimensions must be positive")
    surround_values = (
        args.surround_image_width,
        args.surround_image_height,
        args.surround_rate_hz,
    )
    if any(value is not None for value in surround_values) and not all(
        value is not None and value > 0 for value in surround_values
    ):
        raise ValueError("surround image dimensions and rate must be positive together")
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
            "count": args.camera_count,
            "stereo_pairs": args.stereo_pairs,
            "resolution": [args.image_width, args.image_height],
            "encoding": "mono8",
            "nominal_rate_hz": args.nominal_rate_hz,
        },
        "files": files,
        "generation_command": args.generation_command.format(
            map_name=args.map_dir.name
        ),
    }
    if all(value is not None for value in surround_values):
        manifest["camera"]["front"] = {
            "resolution": [args.image_width, args.image_height],
            "nominal_rate_hz": args.nominal_rate_hz,
        }
        manifest["camera"]["surround"] = {
            "resolution": [args.surround_image_width, args.surround_image_height],
            "nominal_rate_hz": args.surround_rate_hz,
            "on_demand": True,
        }
    (args.map_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
