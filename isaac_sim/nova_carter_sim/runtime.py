"""Runtime health probes for the phase-2 standalone simulation."""

from __future__ import annotations

import math

import carb
import omni.usd
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, Usd, UsdGeom


def world_translation(stage: Usd.Stage, prim_path: str) -> tuple[float, float, float]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"cannot sample missing prim pose: {prim_path}")
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translation = matrix.ExtractTranslation()
    values = tuple(float(value) for value in translation)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"non-finite pose for {prim_path}: {values}")
    return values


def unexpected_robot_overlaps(
    *,
    robot_prim_path: str,
    footprint_aabb: tuple[float, float, float, float],
    floor_z: float,
) -> list[dict[str, str]]:
    """Ask PhysX whether the robot volume intersects anything except itself or the floor."""

    x_min, x_max, y_min, y_max = footprint_aabb
    center = carb.Float3((x_min + x_max) * 0.5, (y_min + y_max) * 0.5, floor_z + 0.35)
    half_extent = carb.Float3((x_max - x_min) * 0.5, (y_max - y_min) * 0.5, 0.30)
    rotation = carb.Float4(0.0, 0.0, 0.0, 1.0)
    hits: list[dict[str, str]] = []

    def report(hit: object) -> bool:
        rigid_body = str(getattr(hit, "rigid_body", ""))
        collision = str(getattr(hit, "collision", ""))
        hits.append({"rigid_body": rigid_body, "collision": collision})
        return True

    get_physx_scene_query_interface().overlap_box(
        half_extent, center, rotation, report, False
    )
    unexpected = []
    for hit in hits:
        paths = (hit["rigid_body"], hit["collision"])
        if any(path.startswith(robot_prim_path) for path in paths):
            continue
        if any("floor" in path.lower() or "groundplane" in path.lower() for path in paths):
            continue
        unexpected.append(hit)
    return unexpected


def stage_identity() -> int:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("USD context lost the active stage")
    return id(stage)
