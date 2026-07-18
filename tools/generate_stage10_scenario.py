#!/usr/bin/env python3
"""Generate deterministic Stage 10 static/dynamic trial inputs.

Only obstacle timing is randomized. Lighting, color and material entries are
copied unchanged because they are explicitly outside the Stage 10 scope.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import random

import yaml


def load_mapping(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a YAML mapping: {path}")
    return value


def generate(
    stage10: dict[str, object],
    scenarios: dict[str, object],
    experiment_class: str,
    seed: int,
    goal_index: int,
) -> tuple[dict[str, object], dict[str, object]]:
    if experiment_class not in {"static", "dynamic"}:
        raise ValueError("experiment class must be static or dynamic")
    goals = stage10.get("goals")
    if not isinstance(goals, list) or not goals:
        raise ValueError("Stage 10 config has no goals")
    selected = goals[goal_index % len(goals)]
    if not isinstance(selected, dict) or not isinstance(selected.get("pose"), list):
        raise ValueError("invalid Stage 10 goal entry")
    pose = [float(value) for value in selected["pose"]]
    if len(pose) != 3:
        raise ValueError("Stage 10 goals must contain x, y and yaw")

    result = copy.deepcopy(scenarios)
    result["experiment"] = {
        "stage": 10,
        "class": experiment_class,
        "seed": seed,
        "goal_index": goal_index % len(goals),
        "goal_name": str(selected.get("name", f"goal_{goal_index}")),
        "goal_pose": pose,
        "front_stereo_only": True,
        "lighting_randomization": False,
        "color_randomization": False,
    }
    dynamic_profile = ""
    if experiment_class == "dynamic":
        profiles = scenarios.get("dynamic_profiles")
        if not isinstance(profiles, dict):
            raise ValueError("scenario config has no dynamic_profiles mapping")
        source = copy.deepcopy(profiles.get("warehouse_crossing"))
        if not isinstance(source, dict):
            raise ValueError("warehouse_crossing profile is missing")
        obstacles = source.get("obstacles")
        if not isinstance(obstacles, list) or not obstacles:
            raise ValueError("warehouse_crossing has no obstacles")
        rng = random.Random(seed)
        timing = []
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                raise ValueError("dynamic obstacle must be a mapping")
            nominal_period = float(obstacle["period_s"])
            obstacle["period_s"] = round(nominal_period * rng.uniform(0.90, 1.10), 6)
            obstacle["phase"] = round(rng.random(), 6)
            timing.append(
                {
                    "name": str(obstacle["name"]),
                    "period_s": obstacle["period_s"],
                    "phase": obstacle["phase"],
                }
            )
        result.setdefault("dynamic_profiles", {})["stage10_trial"] = source
        result["experiment"]["dynamic_timing"] = timing
        dynamic_profile = "stage10_trial"

    metadata = {
        "experiment_class": experiment_class,
        "seed": seed,
        "goal_index": goal_index % len(goals),
        "goal_name": result["experiment"]["goal_name"],
        "goal_pose": pose,
        "goal_poses_ros_parameter": "[" + ",".join(f"{value:.9f}" for value in pose) + "]",
        "dynamic_profile": dynamic_profile,
        "scope": {
            "front_stereo_only": True,
            "lighting_randomization": False,
            "color_randomization": False,
        },
    }
    return result, metadata


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--class", dest="experiment_class", choices=("static", "dynamic"), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--goal-index", type=int, required=True)
    parser.add_argument("--stage10-config", type=Path, required=True)
    parser.add_argument("--scenario-template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    args = parser.parse_args()
    scenario, metadata = generate(
        load_mapping(args.stage10_config),
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
