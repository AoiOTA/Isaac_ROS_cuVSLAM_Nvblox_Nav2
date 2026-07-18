#!/usr/bin/env python3
"""Gate a promoted mapping run on coverage, topology, and physical contacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_object(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise RuntimeError(f"JSON report is not an object: {path}")
    return document


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("simulator_report", type=Path)
    parser.add_argument("--coverage-report", type=Path)
    args = parser.parse_args()
    simulator = load_object(args.simulator_report)
    sensor_graphs = simulator.get("sensor_graphs", {})
    contacts = simulator.get("robot_contacts", {})
    composition = simulator.get("composition", {})
    lidar = composition.get("jackal_lidar", {}) if isinstance(composition, dict) else {}
    checks = {
        "simulator_passed": simulator.get("status") == "passed",
        "mapping_profile": simulator.get("camera_profile") == "mapping_8cam",
        "eight_image_streams": simulator.get("active_image_streams") == 8,
        "four_hawks_active": isinstance(sensor_graphs, dict)
        and sensor_graphs.get("active_pairs") == ["front", "left", "right", "back"],
        "rear_products_created_for_mapping": isinstance(sensor_graphs, dict)
        and sensor_graphs.get("rear_render_products_created") is True,
        "lidar_disabled": isinstance(sensor_graphs, dict)
        and sensor_graphs.get("lidar_enabled") is False
        and isinstance(lidar, dict)
        and lidar.get("enabled") is False
        and lidar.get("sensor_active") is False
        and lidar.get("publisher_graph_created") is False,
        "physical_collision_free": isinstance(contacts, dict)
        and contacts.get("collision_event_count") == 0,
    }
    coverage = None
    if args.coverage_report is not None:
        coverage = load_object(args.coverage_report)
        route = coverage.get("route", {})
        commands = coverage.get("commands", {})
        checks.update(
            {
                "coverage_passed": coverage.get("status") == "passed",
                "coverage_complete": isinstance(route, dict)
                and route.get("completed_waypoints") == route.get("waypoint_count"),
                "coverage_closed_loop": isinstance(route, dict)
                and route.get("closed_loop") is True,
                "coverage_not_partial": coverage.get("partial_route") is False,
                "coverage_forward_only": isinstance(commands, dict)
                and commands.get("negative_linear_samples") == 0,
                "coverage_planning_reference_only": isinstance(
                    coverage.get("planning_reference"), dict
                )
                and coverage["planning_reference"].get("use") == "planning_only",
            }
        )
    failed = [name for name, passed in checks.items() if not passed]
    result = {
        "schema_version": 1,
        "status": "passed" if not failed else "failed",
        "checks": checks,
        "failure_reasons": failed,
        "simulator_report": str(args.simulator_report.resolve()),
        "coverage_report": (
            str(args.coverage_report.resolve())
            if args.coverage_report is not None
            else None
        ),
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
