#!/usr/bin/env python3
"""Combine three to five independently restarted Stage 9 reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("reports", nargs="+", type=Path)
    args = parser.parse_args()
    if not 3 <= len(args.reports) <= 5:
        raise ValueError("Stage 9 summary requires three through five reports")
    trials = []
    for path in args.reports:
        data = json.loads(path.read_text(encoding="utf-8"))
        trials.append(
            {
                "path": str(path.resolve()),
                "status": data.get("status"),
                "goals": len(data.get("goals", [])),
                "recovery_count": data.get("recovery_count", 0),
                "resume_count": data.get("resilient_resume_count", 0),
                "path_length_m": data.get("ground_truth_path_length_m", 0.0),
            }
        )
    passed = sum(item["status"] == "passed" for item in trials)
    summary = {
        "status": "passed" if passed == len(trials) else "failed",
        "trial_count": len(trials),
        "passed_trials": passed,
        "total_goals": sum(int(item["goals"]) for item in trials),
        "trials": trials,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if summary["status"] != "passed":
        raise RuntimeError(f"only {passed}/{len(trials)} Stage 9 trials passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
