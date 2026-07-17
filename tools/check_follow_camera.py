#!/usr/bin/env python3
"""Validate the GUI third-person camera section of a simulator report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    payload = json.loads(args.report.read_text(encoding="utf-8"))
    camera = payload.get("follow_camera", {})
    checks = {
        "simulation_passed": payload.get("status") == "passed",
        "camera_enabled": camera.get("enabled") is True,
        "correct_prim": camera.get("prim") == "/World/FollowCameraRig",
        "viewport_active": camera.get("viewport_active") is True,
    }
    print(json.dumps(checks, indent=2, sort_keys=True))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
