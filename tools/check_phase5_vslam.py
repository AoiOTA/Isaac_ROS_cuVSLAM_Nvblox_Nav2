#!/usr/bin/env python3
"""Check the persisted Phase 5 cuVSLAM acceptance result."""

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
        raise SystemExit(f"Phase 5 cuVSLAM suite failed: checks={failed}")
    tracking = result["tracking"]
    trajectories = result["trajectories"]
    maps = result["map_services"]
    print(
        "phase5_vslam=passed "
        f"tracking_s={tracking['duration_s']:.2f} "
        f"status_count={tracking['status_count']} "
        f"path_ratio={trajectories['path_length_ratio']:.3f} "
        f"optimized_poses={maps['optimized_pose_count']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
