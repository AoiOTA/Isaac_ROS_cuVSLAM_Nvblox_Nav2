#!/usr/bin/env python3
"""Validate the persisted Stage 8 navigation acceptance report."""

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
    if payload.get("status") != "passed" or not checks or not all(checks.values()):
        raise RuntimeError(f"Stage 8 report failed: {checks}")
    if len(payload.get("goals", [])) < 3:
        raise RuntimeError("Stage 8 report does not contain the three-goal route")
    print(json.dumps({"status": payload["status"], "checks": checks}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
