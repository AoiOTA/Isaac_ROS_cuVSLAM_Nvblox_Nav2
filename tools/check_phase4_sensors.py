#!/usr/bin/env python3
"""Check the persisted Phase 4 end-to-end sensor result."""

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
        raise SystemExit(f"Phase 4 sensor suite failed: checks={failed}")
    rates = result["rates_hz"]
    print(
        "phase4_sensors=passed "
        f"stereo_hz={rates['left_rgb']:.2f} depth_hz={rates['depth']:.2f} "
        f"imu_hz={rates['imu']:.2f} clock_hz={rates['clock']:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
