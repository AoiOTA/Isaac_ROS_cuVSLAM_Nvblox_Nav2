#!/usr/bin/env python3
"""Aggregate valid static trials and enforce the >=95% passage metric."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from decimal import Decimal, ROUND_CEILING
import json
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("reports", nargs="+", type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    trial_config = config["trials"]
    minimum_trials = int(trial_config["minimum_valid_trials"])
    threshold = Decimal(str(trial_config["minimum_collision_free_passage_rate"]))
    attempts = []
    seen = set()
    for path in args.reports:
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError(f"trial result must be an object: {path}")
        trial_id = str(value.get("trial_id", ""))
        if not trial_id or trial_id in seen:
            raise ValueError(f"missing or duplicate trial_id: {trial_id!r}")
        seen.add(trial_id)
        value["result_path"] = str(path.resolve())
        attempts.append(value)
    valid = [attempt for attempt in attempts if attempt.get("valid_trial") is True]
    successes = [
        attempt for attempt in valid if attempt.get("collision_free_passage") is True
    ]
    required_successes = int(
        (threshold * Decimal(len(valid))).to_integral_value(rounding=ROUND_CEILING)
    )
    rate = len(successes) / len(valid) if valid else 0.0
    failure_reasons: Counter[str] = Counter()
    for attempt in valid:
        if attempt.get("collision_free_passage") is True:
            continue
        for reason in attempt.get("failure_reasons", []):
            failure_reasons[str(reason)] += 1
    enough_trials = len(valid) >= minimum_trials
    rate_passed = len(successes) >= required_successes and Decimal(str(rate)) >= threshold
    passed = enough_trials and rate_passed
    summary = {
        "schema_version": 1,
        "status": "passed" if passed else "failed",
        "metric": "collision_free_passages / valid_trials",
        "attempt_count": len(attempts),
        "infrastructure_invalid_attempts": len(attempts) - len(valid),
        "valid_trial_count": len(valid),
        "minimum_valid_trials": minimum_trials,
        "collision_free_passage_count": len(successes),
        "required_collision_free_passages_for_observed_denominator": required_successes,
        "collision_free_passage_rate": rate,
        "required_rate": float(threshold),
        "failure_reason_counts": dict(sorted(failure_reasons.items())),
        "trials": attempts,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "trial_id", "attempt_index", "goal_name", "valid_trial", "status",
            "collision_free_passage", "physical_collision_count", "failure_reasons",
            "result_path",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for attempt in attempts:
            row = {name: attempt.get(name) for name in fields}
            row["failure_reasons"] = ";".join(attempt.get("failure_reasons", []))
            writer.writerow(row)
    markdown_path = args.output.with_suffix(".md")
    markdown_path.write_text(
        "\n".join(
            [
                "# Kujiale Jackal static-avoidance acceptance",
                "",
                f"- Status: **{summary['status']}**",
                f"- Valid trials: {len(valid)} (minimum {minimum_trials})",
                f"- Collision-free passages: {len(successes)}/{len(valid)}",
                f"- Static avoidance rate: {rate:.2%} (required >= {float(threshold):.2%})",
                f"- Infrastructure-invalid attempts: {len(attempts) - len(valid)}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
