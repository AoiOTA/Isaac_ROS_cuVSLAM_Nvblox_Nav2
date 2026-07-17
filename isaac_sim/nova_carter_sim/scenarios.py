"""Collision-bound based spawn selection for the fixed warehouse scene."""

from __future__ import annotations

from dataclasses import dataclass
import math

from pxr import Usd, UsdGeom, UsdPhysics


@dataclass(frozen=True)
class CollisionBounds:
    prim_path: str
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    z: float
    yaw_radians: float
    floor_prim: str
    candidates_evaluated: int
    floor_count: int
    obstacle_count: int
    footprint_aabb: tuple[float, float, float, float]


def _finite_bounds(prim: Usd.Prim, cache: UsdGeom.BBoxCache) -> CollisionBounds | None:
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    minimum = tuple(float(value) for value in box.GetMin())
    maximum = tuple(float(value) for value in box.GetMax())
    values = minimum + maximum
    if not all(math.isfinite(value) and abs(value) < 1.0e6 for value in values):
        return None
    if maximum[0] <= minimum[0] or maximum[1] <= minimum[1]:
        return None
    return CollisionBounds(prim.GetPath().pathString, minimum, maximum)


def collect_collision_bounds(stage: Usd.Stage) -> list[CollisionBounds]:
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True)
    bounds: list[CollisionBounds] = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        item = _finite_bounds(prim, cache)
        if item is not None:
            bounds.append(item)
    return bounds


def _rotated_footprint_aabb(
    footprint: tuple[tuple[float, float], ...], yaw_radians: float, clearance: float
) -> tuple[float, float, float, float]:
    cosine = math.cos(yaw_radians)
    sine = math.sin(yaw_radians)
    points = [
        (cosine * x - sine * y, sine * x + cosine * y) for x, y in footprint
    ]
    return (
        min(point[0] for point in points) - clearance,
        max(point[0] for point in points) + clearance,
        min(point[1] for point in points) - clearance,
        max(point[1] for point in points) + clearance,
    )


def _overlaps_2d(
    robot_box: tuple[float, float, float, float], obstacle: CollisionBounds
) -> bool:
    return (
        robot_box[0] < obstacle.maximum[0]
        and robot_box[1] > obstacle.minimum[0]
        and robot_box[2] < obstacle.maximum[1]
        and robot_box[3] > obstacle.minimum[1]
    )


def find_collision_free_spawn(
    stage: Usd.Stage,
    *,
    footprint: tuple[tuple[float, float], ...],
    yaw_radians: float = 0.0,
    clearance: float = 0.15,
    grid_resolution: float = 0.5,
    spawn_height: float = 0.03,
    obstacle_height: float = 1.60,
    preferred_xy: tuple[float, float] = (0.0, 0.0),
) -> SpawnPose:
    """Find a floor-supported pose whose padded footprint misses every obstacle AABB."""

    if (
        clearance < 0.0
        or grid_resolution <= 0.0
        or spawn_height < 0.0
        or obstacle_height <= 0.0
    ):
        raise ValueError("spawn search distances must be non-negative and grid must be positive")

    collision_bounds = collect_collision_bounds(stage)
    floors = [
        item
        for item in collision_bounds
        if item.maximum[2] <= 0.25
        and item.maximum[2] - item.minimum[2] <= 0.05
        and item.maximum[0] - item.minimum[0] >= 1.0
        and item.maximum[1] - item.minimum[1] >= 1.0
    ]
    if not floors:
        raise RuntimeError("no finite horizontal collision floor was found in the warehouse")

    highest_floor = max(item.maximum[2] for item in floors)
    obstacles = [
        item
        for item in collision_bounds
        if item not in floors
        and item.maximum[2] > highest_floor + 0.08
        and item.minimum[2] < highest_floor + obstacle_height
    ]

    offsets = _rotated_footprint_aabb(footprint, yaw_radians, clearance)
    raw_candidates: dict[tuple[float, float], tuple[float, str]] = {}
    for floor in floors:
        x_lower = floor.minimum[0] - offsets[0]
        x_upper = floor.maximum[0] - offsets[1]
        y_lower = floor.minimum[1] - offsets[2]
        y_upper = floor.maximum[1] - offsets[3]
        x_index_min = math.ceil((x_lower - 1.0e-9) / grid_resolution)
        x_index_max = math.floor((x_upper + 1.0e-9) / grid_resolution)
        y_index_min = math.ceil((y_lower - 1.0e-9) / grid_resolution)
        y_index_max = math.floor((y_upper + 1.0e-9) / grid_resolution)
        for x_index in range(x_index_min, x_index_max + 1):
            for y_index in range(y_index_min, y_index_max + 1):
                xy = (x_index * grid_resolution, y_index * grid_resolution)
                current = raw_candidates.get(xy)
                if current is None or floor.maximum[2] > current[0]:
                    raw_candidates[xy] = (floor.maximum[2], floor.prim_path)

    preferred_x, preferred_y = preferred_xy
    candidates = sorted(
        raw_candidates.items(),
        key=lambda item: (
            (item[0][0] - preferred_x) ** 2 + (item[0][1] - preferred_y) ** 2,
            abs(item[0][1] - preferred_y),
            abs(item[0][0] - preferred_x),
            item[0][0],
            item[0][1],
        ),
    )

    for evaluated, ((x, y), (floor_top, floor_path)) in enumerate(candidates, start=1):
        robot_box = (x + offsets[0], x + offsets[1], y + offsets[2], y + offsets[3])
        blocking = [
            obstacle
            for obstacle in obstacles
            if obstacle.maximum[2] > floor_top + 0.02
            and obstacle.minimum[2] < floor_top + obstacle_height
            and _overlaps_2d(robot_box, obstacle)
        ]
        if not blocking:
            return SpawnPose(
                x=x,
                y=y,
                z=floor_top + spawn_height,
                yaw_radians=yaw_radians,
                floor_prim=floor_path,
                candidates_evaluated=evaluated,
                floor_count=len(floors),
                obstacle_count=len(obstacles),
                footprint_aabb=robot_box,
            )

    raise RuntimeError(
        f"no collision-free spawn remained after evaluating {len(candidates)} floor candidates"
    )
