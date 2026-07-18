#!/usr/bin/env python3
"""Build footprint-aware SE(2) shortest paths from extracted USD colliders."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def rasterize_colliders(
    colliders: list[dict[str, object]],
    origin: tuple[float, float],
    resolution: float,
    shape: tuple[int, int],
) -> np.ndarray:
    occupied = np.zeros(shape, dtype=np.uint8)
    height, width = shape
    for collider in colliders:
        lower = collider["map_aabb_min_m"]
        upper = collider["map_aabb_max_m"]
        c0 = max(0, math.floor((float(lower[0]) - origin[0]) / resolution))
        c1 = min(width - 1, math.ceil((float(upper[0]) - origin[0]) / resolution))
        r0 = max(0, math.floor((float(lower[1]) - origin[1]) / resolution))
        r1 = min(height - 1, math.ceil((float(upper[1]) - origin[1]) / resolution))
        if c0 <= c1 and r0 <= r1:
            occupied[r0 : r1 + 1, c0 : c1 + 1] = 1
    return occupied


def footprint_offsets(
    footprint: list[list[float]],
    padding: float,
    resolution: float,
    orientation_bins: int,
) -> list[np.ndarray]:
    points = np.asarray(footprint, dtype=np.float64)
    if points.shape != (4, 2):
        raise ValueError("Stage 11 reference footprint must contain four x/y vertices")
    minimum = points.min(axis=0) - padding
    maximum = points.max(axis=0) + padding
    rectangle = np.asarray(
        [
            [maximum[0], maximum[1]],
            [maximum[0], minimum[1]],
            [minimum[0], minimum[1]],
            [minimum[0], maximum[1]],
        ]
    )
    all_offsets: list[np.ndarray] = []
    for index in range(orientation_bins):
        angle = 2.0 * math.pi * index / orientation_bins
        rotation = np.asarray(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )
        rotated = rectangle @ rotation.T
        radius = int(math.ceil(np.max(np.abs(rotated)) / resolution)) + 2
        center = np.asarray([radius, radius], dtype=np.float64)
        pixels = np.rint(rotated[:, [0, 1]] / resolution + center).astype(np.int32)
        kernel = np.zeros((2 * radius + 1, 2 * radius + 1), dtype=np.uint8)
        cv2.fillConvexPoly(kernel, pixels, 1)
        rows, columns = np.nonzero(kernel)
        all_offsets.append(np.column_stack((rows - radius, columns - radius)))
    return all_offsets


def configuration_space(
    occupied: np.ndarray, offsets_by_orientation: list[np.ndarray]
) -> np.ndarray:
    height, width = occupied.shape
    valid = np.empty((len(offsets_by_orientation), height, width), dtype=bool)
    source = occupied.astype(np.float32)
    for orientation, offsets in enumerate(offsets_by_orientation):
        min_row, min_col = offsets.min(axis=0)
        max_row, max_col = offsets.max(axis=0)
        radius = int(max(abs(min_row), abs(max_row), abs(min_col), abs(max_col)))
        kernel = np.zeros((2 * radius + 1, 2 * radius + 1), dtype=np.float32)
        kernel[offsets[:, 0] + radius, offsets[:, 1] + radius] = 1.0
        hits = cv2.filter2D(
            source,
            -1,
            kernel,
            anchor=(radius, radius),
            borderType=cv2.BORDER_CONSTANT,
        )
        orientation_valid = hits < 0.5
        if min_row < 0:
            orientation_valid[: -min_row, :] = False
        if max_row > 0:
            orientation_valid[height - max_row :, :] = False
        if min_col < 0:
            orientation_valid[:, : -min_col] = False
        if max_col > 0:
            orientation_valid[:, width - max_col :] = False
        valid[orientation] = orientation_valid
    return valid


def se2_shortest_path(
    valid: np.ndarray,
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
    resolution: float,
) -> tuple[float, list[tuple[int, int, int]], int]:
    bins, height, width = valid.shape
    for label, state in (("start", start), ("goal", goal)):
        row, column, orientation = state
        if not (0 <= row < height and 0 <= column < width):
            raise ValueError(f"{label} lies outside the reference grid: {state}")
        if not valid[orientation % bins, row, column]:
            raise ValueError(f"{label} footprint overlaps USD collision geometry: {state}")
    directions = [
        (
            int(round(math.sin(2.0 * math.pi * index / bins))),
            int(round(math.cos(2.0 * math.pi * index / bins))),
        )
        for index in range(bins)
    ]
    start = (start[0], start[1], start[2] % bins)
    goal = (goal[0], goal[1], goal[2] % bins)
    queue: list[tuple[float, float, tuple[int, int, int]]] = []
    heapq.heappush(queue, (0.0, 0.0, start))
    distance = {start: 0.0}
    parent: dict[tuple[int, int, int], tuple[int, int, int]] = {}
    expansions = 0
    while queue:
        _, cost, state = heapq.heappop(queue)
        if cost > distance.get(state, math.inf) + 1.0e-12:
            continue
        expansions += 1
        if state == goal:
            path = [state]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            translational = 0.0
            for first, second in zip(path, path[1:]):
                translational += resolution * math.hypot(
                    second[0] - first[0], second[1] - first[1]
                )
            return translational, path, expansions
        row, column, orientation = state
        candidates: list[tuple[tuple[int, int, int], float]] = []
        for turn in (-1, 1):
            turned = (orientation + turn) % bins
            if valid[turned, row, column]:
                candidates.append(((row, column, turned), 1.0e-6))
        delta_row, delta_column = directions[orientation]
        step_cost = resolution * math.hypot(delta_row, delta_column)
        for direction in (-1, 1):
            next_row = row + direction * delta_row
            next_column = column + direction * delta_column
            if (
                0 <= next_row < height
                and 0 <= next_column < width
                and valid[orientation, next_row, next_column]
            ):
                candidates.append(((next_row, next_column, orientation), step_cost))
        for candidate, edge_cost in candidates:
            new_cost = cost + edge_cost
            if new_cost + 1.0e-12 >= distance.get(candidate, math.inf):
                continue
            distance[candidate] = new_cost
            parent[candidate] = state
            heuristic = resolution * math.hypot(
                goal[0] - candidate[0], goal[1] - candidate[1]
            )
            heapq.heappush(queue, (new_cost + heuristic, new_cost, candidate))
    raise RuntimeError(f"no footprint-valid SE(2) path from {start} to {goal}")


def world_to_cell(
    x: float, y: float, origin: tuple[float, float], resolution: float
) -> tuple[int, int]:
    return (
        int(round((y - origin[1]) / resolution)),
        int(round((x - origin[0]) / resolution)),
    )


def orientation_index(yaw: float, bins: int) -> int:
    return int(round((yaw % (2.0 * math.pi)) * bins / (2.0 * math.pi))) % bins


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--stage11-config", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--grid-pgm", type=Path)
    args = parser.parse_args()
    geometry = json.loads(args.geometry.read_text(encoding="utf-8"))
    config = yaml.safe_load(args.stage11_config.read_text(encoding="utf-8"))
    map_config = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    map_image = cv2.imread(
        str((args.map_yaml.parent / map_config["image"]).resolve()),
        cv2.IMREAD_GRAYSCALE,
    )
    if map_image is None:
        raise FileNotFoundError("failed to read occupancy map image for grid bounds")
    reference = config["reference_path"]
    resolution = float(reference["resolution_m"])
    origin = (float(map_config["origin"][0]), float(map_config["origin"][1]))
    occupied = rasterize_colliders(
        geometry["colliders"], origin, resolution, map_image.shape
    )
    offsets = footprint_offsets(
        reference["footprint"],
        float(reference["footprint_padding_m"]),
        resolution,
        int(reference["orientation_bins"]),
    )
    valid = configuration_space(occupied, offsets)
    bins = valid.shape[0]
    start_row, start_column = world_to_cell(0.0, 0.0, origin, resolution)
    paths: dict[str, object] = {}
    for goal in config["goals"]:
        x, y, yaw = (float(value) for value in goal["pose"])
        goal_row, goal_column = world_to_cell(x, y, origin, resolution)
        length, states, expansions = se2_shortest_path(
            valid,
            (start_row, start_column, orientation_index(0.0, bins)),
            (goal_row, goal_column, orientation_index(yaw, bins)),
            resolution,
        )
        centerline: list[list[float]] = []
        previous = None
        for row, column, _ in states:
            point = [origin[0] + column * resolution, origin[1] + row * resolution]
            if point != previous:
                centerline.append(point)
                previous = point
        paths[str(goal["name"])] = {
            "goal_pose": [x, y, yaw],
            "long_distance": bool(goal.get("long_distance", False)),
            "optimal_path_length_m": length,
            "state_count": len(states),
            "expansions": expansions,
            "centerline": centerline,
        }
    result = {
        "schema_version": 1,
        "method": "8-heading SE(2) A* over actual USD CollisionAPI AABBs with the padded Jackal footprint",
        "geometry_file": str(args.geometry.resolve()),
        "usd_path": geometry["usd_path"],
        "collider_count": geometry["collider_count"],
        "resolution_m": resolution,
        "origin_m": list(origin),
        "shape": list(map_image.shape),
        "orientation_bins": bins,
        "footprint": reference["footprint"],
        "footprint_padding_m": float(reference["footprint_padding_m"]),
        "occupied_cells": int(occupied.sum()),
        "paths": paths,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.grid_pgm is not None:
        args.grid_pgm.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.grid_pgm), np.where(occupied, 0, 254).astype(np.uint8))
    print(json.dumps({
        "usd_path": result["usd_path"],
        "collider_count": result["collider_count"],
        "occupied_cells": result["occupied_cells"],
        "paths": {name: value["optimal_path_length_m"] for name, value in paths.items()},
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
