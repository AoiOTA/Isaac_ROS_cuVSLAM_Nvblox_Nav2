#!/usr/bin/env python3
"""Fail unless a Stage 7 map directory contains all runtime artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def nonempty_files(path: Path) -> list[Path]:
    return [item for item in path.rglob("*") if item.is_file() and item.stat().st_size > 0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    args = parser.parse_args()
    required = ["cuvslam", "cuvgl", "nvblox", "mesh", "occupancy", "config"]
    counts = {name: len(nonempty_files(args.map_dir / name)) for name in required}
    missing = [name for name, count in counts.items() if count == 0]
    manifest_path = args.map_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    checks = {
        "artifact_directories_nonempty": not missing,
        "occupancy_yaml": (args.map_dir / "occupancy/map.yaml").is_file(),
        "occupancy_pgm": (args.map_dir / "occupancy/map.pgm").is_file(),
        "manifest": bool(manifest),
        "front_stereo_only": manifest.get("camera", {}).get("count") == 2,
    }
    print(json.dumps({"checks": checks, "file_counts": counts}, indent=2, sort_keys=True))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
