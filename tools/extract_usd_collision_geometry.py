#!/usr/bin/env python3
"""Extract navigation-height collision AABBs from the official Warehouse USD.

Run this tool with the Isaac Sim 6.0.1 Python runtime. It reads the actual USD
and referenced layers; it does not infer geometry from the Nav2 map.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics


def extract(
    usd_path: Path,
    spawn_world: tuple[float, float, float],
    vertical_band: tuple[float, float],
) -> dict[str, object]:
    if not usd_path.is_file():
        raise FileNotFoundError(usd_path)
    stage = Usd.Stage.Open(str(usd_path), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")
    default_prim = stage.GetDefaultPrim()
    if not default_prim.IsValid():
        raise RuntimeError("Warehouse USD has no default prim")
    z_min, z_max = vertical_band
    if not 0.0 <= z_min < z_max:
        raise ValueError("vertical band must be positive and ordered")
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), purposes, useExtentsHint=True
    )
    colliders: list[dict[str, object]] = []
    prim_count = 0
    for prim in stage.Traverse():
        prim_count += 1
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if enabled is False:
            continue
        world_range = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lower = tuple(float(value) for value in world_range.GetMin())
        upper = tuple(float(value) for value in world_range.GetMax())
        if not all(math.isfinite(value) for value in (*lower, *upper)):
            continue
        if upper[2] <= z_min or lower[2] >= z_max:
            continue
        if upper[0] <= lower[0] or upper[1] <= lower[1]:
            continue
        map_lower = [lower[index] - spawn_world[index] for index in range(3)]
        map_upper = [upper[index] - spawn_world[index] for index in range(3)]
        colliders.append(
            {
                "prim_path": str(prim.GetPath()),
                "world_aabb_min_m": list(lower),
                "world_aabb_max_m": list(upper),
                "map_aabb_min_m": map_lower,
                "map_aabb_max_m": map_upper,
            }
        )
    if not colliders:
        raise RuntimeError("no collision prim intersects the robot vertical band")
    return {
        "schema_version": 1,
        "source": "actual UsdPhysics.CollisionAPI prims from the official Warehouse USD",
        "usd_path": str(usd_path.resolve()),
        "default_prim": str(default_prim.GetPath()),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "stage_prim_count": prim_count,
        "used_layer_count": len(stage.GetUsedLayers()),
        "spawn_world_m": list(spawn_world),
        "robot_vertical_band_m": list(vertical_band),
        "collider_count": len(colliders),
        "colliders": colliders,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spawn-world", nargs=3, type=float, default=(-0.5, -0.5, 0.03))
    parser.add_argument("--vertical-band", nargs=2, type=float, default=(0.05, 1.20))
    args = parser.parse_args()
    result = extract(
        args.usd,
        tuple(args.spawn_world),
        tuple(args.vertical_band),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: result[key] for key in (
        "usd_path", "default_prim", "stage_prim_count", "used_layer_count", "collider_count"
    )}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
