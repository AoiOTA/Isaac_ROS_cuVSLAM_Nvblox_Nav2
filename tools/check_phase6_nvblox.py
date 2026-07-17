#!/usr/bin/env python3
"""Check the persisted Phase 6 nvblox acceptance result."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text(encoding="utf-8"))
    failed = [name for name, passed in result.get("checks", {}).items() if not passed]
    if result.get("status") != "passed" or failed:
        raise SystemExit(f"Phase 6 nvblox suite failed: checks={failed}")
    reconstruction = result["reconstruction"]
    saved = result["saved"]["files"]
    print(
        "phase6_nvblox=passed "
        f"tsdf_blocks={reconstruction['tsdf_unique_blocks']} "
        f"mesh_blocks={reconstruction['mesh_unique_blocks']} "
        f"esdf_points={reconstruction['esdf_max_points']} "
        f"known_slice_cells={reconstruction['slice_max_known_cells']} "
        f"map_bytes={saved['map']['size_bytes']} "
        f"ply_bytes={saved['mesh']['size_bytes']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
