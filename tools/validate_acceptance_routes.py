#!/usr/bin/env python3
"""Validate static acceptance goals against the generated occupancy map."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path

import yaml


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    data = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(data):
            if data[index:index + 1] == b"#":
                end = data.find(b"\n", index)
                index = len(data) if end < 0 else end + 1
            elif data[index:index + 1].isspace():
                index += 1
            else:
                break
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        if start == index:
            raise ValueError(f"truncated PGM header: {path}")
        return data[start:index]

    magic = token()
    width, height, maximum = int(token()), int(token()), int(token())
    if magic not in {b"P2", b"P5"} or min(width, height) <= 0 or maximum != 255:
        raise ValueError(f"unsupported PGM encoding: {path}")
    if magic == b"P5":
        if index >= len(data) or not data[index:index + 1].isspace():
            raise ValueError(f"missing PGM header separator: {path}")
        # Consume only the required header separator.  Consuming every
        # whitespace byte would corrupt a valid binary image whose first
        # pixel happens to be 0x09, 0x0a, 0x0d, or 0x20.
        index += 2 if data[index:index + 2] == b"\r\n" else 1
        pixels = list(data[index:index + width * height])
    else:
        pixels = [int(item) for item in data[index:].split()]
    if len(pixels) != width * height:
        raise ValueError(f"PGM pixel count mismatch: {path}")
    return width, height, pixels


def map_cell(
    x: float,
    y: float,
    origin: tuple[float, float],
    resolution: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    column = math.floor((x - origin[0]) / resolution)
    row_from_bottom = math.floor((y - origin[1]) / resolution)
    return height - 1 - row_from_bottom, column


def line_cells(start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    row0, column0 = start
    row1, column1 = goal
    steps = max(abs(row1 - row0), abs(column1 - column0), 1)
    return list(
        dict.fromkeys(
            (
                round(row0 + (row1 - row0) * index / steps),
                round(column0 + (column1 - column0) * index / steps),
            )
            for index in range(steps + 1)
        )
    )


def reachable(
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: list[bool],
    width: int,
    height: int,
) -> bool:
    queue = deque([start])
    seen = {start}
    while queue:
        row, column = queue.popleft()
        if (row, column) == goal:
            return True
        for delta_row, delta_column in (
            (-1, -1), (-1, 0), (-1, 1), (0, -1),
            (0, 1), (1, -1), (1, 0), (1, 1),
        ):
            candidate = row + delta_row, column + delta_column
            next_row, next_column = candidate
            if not (0 <= next_row < height and 0 <= next_column < width):
                continue
            if candidate in seen or blocked[next_row * width + next_column]:
                continue
            seen.add(candidate)
            queue.append(candidate)
    return False


def inflate_mask(
    occupied: list[bool],
    width: int,
    height: int,
    radius_cells: int,
) -> list[bool]:
    inflated = occupied.copy()
    offsets = [
        (row, column)
        for row in range(-radius_cells, radius_cells + 1)
        for column in range(-radius_cells, radius_cells + 1)
        if row * row + column * column <= radius_cells * radius_cells
    ]
    for index, value in enumerate(occupied):
        if not value:
            continue
        row, column = divmod(index, width)
        for delta_row, delta_column in offsets:
            target_row, target_column = row + delta_row, column + delta_column
            if 0 <= target_row < height and 0 <= target_column < width:
                inflated[target_row * width + target_column] = True
    return inflated


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/acceptance.yaml"
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    map_dir = args.map_dir.resolve()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    map_yaml = yaml.safe_load(
        (map_dir / "occupancy/map.yaml").read_text(encoding="utf-8")
    )
    width, height, pixels = read_pgm(map_dir / "occupancy/map.pgm")
    resolution = float(map_yaml["resolution"])
    origin_values = map_yaml["origin"]
    origin = float(origin_values[0]), float(origin_values[1])
    negate = bool(map_yaml.get("negate", 0))
    occupied_threshold = float(map_yaml.get("occupied_thresh", 0.65))
    free_threshold = float(map_yaml.get("free_thresh", 0.196))
    route_config = config["route_validation"]
    collision_radius = float(route_config["collision_radius_m"])
    goal_clearance_radius = float(
        route_config.get("goal_clearance_radius_m", collision_radius)
    )
    if goal_clearance_radius < collision_radius:
        raise RuntimeError(
            "goal_clearance_radius_m must be greater than or equal to "
            "collision_radius_m"
        )
    radius_cells = math.ceil(collision_radius / resolution)
    goal_radius_cells = math.ceil(goal_clearance_radius / resolution)

    occupied = [False] * (width * height)
    known_free = [False] * (width * height)
    for index, pixel in enumerate(pixels):
        occupancy = (pixel / 255.0) if negate else ((255 - pixel) / 255.0)
        occupied[index] = occupancy > occupied_threshold
        known_free[index] = occupancy < free_threshold
    inflated = inflate_mask(occupied, width, height, radius_cells)
    goal_inflated = inflate_mask(occupied, width, height, goal_radius_cells)
    if bool(route_config.get("require_known_free", True)):
        inflated = [
            blocked or not known
            for blocked, known in zip(inflated, known_free)
        ]
        goal_inflated = [
            blocked or not known
            for blocked, known in zip(goal_inflated, known_free)
        ]

    start_xy = tuple(float(value) for value in route_config["start_pose"])
    start = map_cell(*start_xy, origin, resolution, width, height)
    if not (0 <= start[0] < height and 0 <= start[1] < width):
        raise RuntimeError("acceptance start pose is outside the occupancy map")
    if inflated[start[0] * width + start[1]]:
        raise RuntimeError("acceptance start pose is not footprint-clear known free space")

    routes = []
    for goal in config["goals"]:
        pose = [float(value) for value in goal["pose"]]
        cell = map_cell(pose[0], pose[1], origin, resolution, width, height)
        inside = 0 <= cell[0] < height and 0 <= cell[1] < width
        clear = inside and not goal_inflated[cell[0] * width + cell[1]]
        connected = clear and reachable(start, cell, inflated, width, height)
        straight = line_cells(start, cell) if inside else []
        direct_obstructed = bool(straight) and any(
            inflated[row * width + column]
            for row, column in straight
            if 0 <= row < height and 0 <= column < width
        )
        require_detour = bool(route_config.get("require_direct_line_obstructed", True))
        passed = clear and connected and (direct_obstructed or not require_detour)
        routes.append(
            {
                "name": str(goal["name"]),
                "pose": pose,
                "cell": list(cell),
                "inside_map": inside,
                "footprint_clear_known_free": clear,
                "connected_to_spawn": connected,
                "direct_line_obstructed": direct_obstructed,
                "passed": passed,
            }
        )
    report = {
        "status": "passed" if routes and all(route["passed"] for route in routes) else "failed",
        "map_name": map_dir.name,
        "map_resolution_m": resolution,
        "collision_radius_m": collision_radius,
        "goal_clearance_radius_m": goal_clearance_radius,
        "start_pose": list(start_xy),
        "start_cell": list(start),
        "routes": routes,
    }
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
