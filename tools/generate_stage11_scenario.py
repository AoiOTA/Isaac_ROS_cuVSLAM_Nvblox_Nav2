#!/usr/bin/env python3
"""Generate deterministic Stage 11 static, dynamic and heterogeneous trials."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import random

import yaml


VALID_CLASSES = {"static", "dynamic", "heterogeneous"}


def load_mapping(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a YAML mapping: {path}")
    return value


def _randomize_obstacles(
    obstacles: list[dict[str, object]], seed: int, heterogeneous: bool
) -> list[dict[str, object]]:
    rng = random.Random(seed)
    result = copy.deepcopy(obstacles)
    for obstacle in result:
        nominal_period = float(obstacle["period_s"])
        obstacle["period_s"] = round(nominal_period * rng.uniform(0.88, 1.12), 6)
        obstacle["phase"] = round(rng.random(), 6)
        # Geometry diversity is deterministic and bounded. Display colors and
        # all warehouse materials remain unchanged and are not randomized.
        if heterogeneous and obstacle.get("kind") == "box":
            obstacle["size_m"] = [
                round(float(value) * rng.uniform(0.92, 1.08), 6)
                for value in obstacle["size_m"]
            ]
        elif heterogeneous and obstacle.get("kind") == "capsule":
            obstacle["radius_m"] = round(
                float(obstacle["radius_m"]) * rng.uniform(0.92, 1.08), 6
            )
            obstacle["height_m"] = round(
                float(obstacle["height_m"]) * rng.uniform(0.92, 1.08), 6
            )
    return result


def generate(
    stage11: dict[str, object],
    scenarios: dict[str, object],
    experiment_class: str,
    seed: int,
    goal_index: int,
) -> tuple[dict[str, object], dict[str, object]]:
    if experiment_class not in VALID_CLASSES:
        raise ValueError(f"unsupported Stage 11 class: {experiment_class}")
    goals = stage11.get("goals")
    if not isinstance(goals, list) or not goals:
        raise ValueError("Stage 11 config has no goals")
    selected_index = goal_index % len(goals)
    selected = goals[selected_index]
    if not isinstance(selected, dict):
        raise ValueError("Stage 11 goal entry must be a mapping")
    pose = [float(value) for value in selected.get("pose", [])]
    if len(pose) != 3:
        raise ValueError("Stage 11 goals must contain x, y and yaw")

    result = copy.deepcopy(scenarios)
    dynamic_profile = ""
    actor_summary: list[dict[str, object]] = []
    if experiment_class != "static":
        profiles = scenarios.get("dynamic_profiles")
        if not isinstance(profiles, dict):
            raise ValueError("scenario config has no dynamic profiles")
        base = profiles.get("warehouse_crossing")
        if not isinstance(base, dict) or not isinstance(base.get("obstacles"), list):
            raise ValueError("warehouse_crossing profile is invalid")
        obstacles = copy.deepcopy(base["obstacles"])
        if experiment_class == "heterogeneous":
            extra = stage11.get("heterogeneous_extra_obstacles")
            if not isinstance(extra, list) or len(extra) < 3:
                raise ValueError("heterogeneous profile requires at least three extras")
            obstacles.extend(copy.deepcopy(extra))
        obstacles = _randomize_obstacles(
            obstacles, seed, experiment_class == "heterogeneous"
        )
        dynamic_profile = "stage11_trial"
        result.setdefault("dynamic_profiles", {})[dynamic_profile] = {
            "obstacles": obstacles
        }
        actor_summary = [
            {
                "name": str(obstacle["name"]),
                "kind": str(obstacle["kind"]),
                "period_s": float(obstacle["period_s"]),
                "phase": float(obstacle["phase"]),
            }
            for obstacle in obstacles
        ]

    scope = {
        "front_stereo_only": True,
        "lidar_enabled": False,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    experiment = {
        "stage": 11,
        "class": experiment_class,
        "seed": seed,
        "goal_index": selected_index,
        "goal_name": str(selected.get("name", f"goal_{selected_index}")),
        "goal_pose": pose,
        "long_distance": bool(selected.get("long_distance", False)),
        **scope,
        "dynamic_profile": dynamic_profile,
        "actors": actor_summary,
    }
    result["experiment"] = experiment
    metadata = {
        "stage": 11,
        "experiment_class": experiment_class,
        "seed": seed,
        "goal_index": selected_index,
        "goal_name": experiment["goal_name"],
        "goal_pose": pose,
        "goal_poses_ros_parameter": "["
        + ",".join(f"{value:.9f}" for value in pose)
        + "]",
        "long_distance": experiment["long_distance"],
        "dynamic_profile": dynamic_profile,
        "actor_count": len(actor_summary),
        "actor_kinds": sorted({str(item["kind"]) for item in actor_summary}),
        "scope": scope,
        "automation": {
            "goal_dispatch": "navigation_test_runner",
            "manual_intervention": False,
        },
    }
    return result, metadata


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--class", dest="experiment_class", choices=sorted(VALID_CLASSES), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--goal-index", type=int, required=True)
    parser.add_argument("--stage11-config", type=Path, required=True)
    parser.add_argument("--scenario-template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    args = parser.parse_args()
    scenario, metadata = generate(
        load_mapping(args.stage11_config),
        load_mapping(args.scenario_template),
        args.experiment_class,
        args.seed,
        args.goal_index,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8")
    args.metadata.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
