#!/usr/bin/env python3
"""Validate the ROS data-flow and recovery half of a Stage 9 run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    checks = report.get("checks", {})
    required = {
        "all_lifecycle_nodes_active",
        "all_goals_succeeded",
        "critical_topics_nonempty",
        "main_tf_chain_seen",
        "cuvslam_tracking",
        "forced_relocalization_accepted",
        "recovery_state_machine_completed",
        "resilient_action_paused_and_resumed",
        "vgl_camera_mode_valid",
        "combined_slice_nonempty",
        "guard_stopped_during_relocalization",
    }
    audit = {
        "runner_passed": report.get("status") == "passed",
        "all_runner_checks_passed": bool(checks) and all(checks.values()),
        "stage9_checks_present": required <= set(checks),
        "at_least_one_recovery": int(report.get("recovery_count", 0)) >= 1,
        "resilient_goal_resumed": int(report.get("resilient_resume_count", 0)) >= 1,
        "all_three_goals_recorded": len(report.get("goals", [])) == 3,
        "front_stereo_vgl": report.get("vgl_camera_mode") == "front_stereo"
        and not any(report.get("surround_counts", {}).values()),
        "no_command_during_recovery": float(
            report.get("max_command_while_unready_after_grace", 1.0)
        )
        <= 0.02,
    }
    output = {"status": "passed" if all(audit.values()) else "failed", "checks": audit}
    print(json.dumps(output, indent=2, sort_keys=True))
    if output["status"] != "passed":
        failed = [name for name, passed in audit.items() if not passed]
        raise RuntimeError(f"Stage 9 ROS checks failed: {failed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
