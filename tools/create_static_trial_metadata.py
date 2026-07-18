#!/usr/bin/env python3
"""Create immutable metadata for one static avoidance attempt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--goal-index", type=int, required=True)
    parser.add_argument("--attempt-index", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    goals = config["goals"]
    if not 0 <= args.goal_index < len(goals):
        raise ValueError(f"goal index out of range: {args.goal_index}")
    goal = goals[args.goal_index]
    metadata = {
        "schema_version": 1,
        "trial_id": args.output.resolve().parent.name,
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "attempt_index": args.attempt_index,
        "goal_index": args.goal_index,
        "goal_name": goal["name"],
        "goal_pose": [float(value) for value in goal["pose"]],
        "map_name": config["map"]["name"],
        "environment_motion": "static",
        "camera_profile": config["map"]["navigation_camera_profile"],
        "manual_intervention": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
