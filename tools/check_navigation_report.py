#!/usr/bin/env python3
"""Validate an automatic Kujiale navigation runner report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    payload = json.loads(args.report.read_text(encoding="utf-8"))
    checks = payload.get("checks", {})
    goals = payload.get("goals", [])
    if payload.get("status") != "passed":
        raise RuntimeError(f"navigation runner failed: {payload.get('error')}")
    if not isinstance(checks, dict) or not checks or not all(checks.values()):
        raise RuntimeError(f"navigation report checks failed: {checks}")
    if not isinstance(goals, list) or not goals:
        raise RuntimeError("navigation report does not contain an executed goal")
    if not all(
        isinstance(goal, dict)
        and goal.get("status") == 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
        and goal.get("error_code") == 0
        for goal in goals
    ):
        raise RuntimeError("navigation report contains a failed goal")
    print(
        f"NAVIGATION_REPORT_OK goals={len(goals)} "
        f"experiment_class={payload.get('experiment_class')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
