"""Session-layer static obstacles for interactive Phase 9 navigation."""

from __future__ import annotations

import math

from pxr import Gf, Usd, UsdGeom, UsdPhysics


OBSTACLE_ROOT = "/World/Phase9StaticObstacles"


def _vec3(values: object, label: str) -> tuple[float, float, float]:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains non-finite values")
    return result


def _box(stage: Usd.Stage, path: str, spec: dict[str, object]) -> Usd.Prim:
    size = _vec3(spec["size_m"], f"{path}.size_m")
    if min(size) <= 0.0:
        raise ValueError(f"{path}.size_m must be positive")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*_vec3(spec["color_rgb"], "color_rgb"))])
    # Keep translation before scale: reversing the order scales the position.
    cube.AddTranslateOp()
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _capsule(stage: Usd.Stage, path: str, spec: dict[str, object]) -> Usd.Prim:
    radius = float(spec["radius_m"])
    height = float(spec["height_m"])
    if min(radius, height) <= 0.0:
        raise ValueError(f"{path} capsule dimensions must be positive")
    capsule = UsdGeom.Capsule.Define(stage, path)
    capsule.CreateRadiusAttr(radius)
    capsule.CreateHeightAttr(height)
    capsule.CreateAxisAttr(UsdGeom.Tokens.z)
    capsule.CreateDisplayColorAttr().Set(
        [Gf.Vec3f(*_vec3(spec["color_rgb"], "color_rgb"))]
    )
    capsule.AddTranslateOp()
    prim = capsule.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


class StaticObstacleManager:
    """Create collidable, non-moving obstacles in the anonymous session layer."""

    def __init__(
        self,
        stage: Usd.Stage,
        profile_name: str,
        profile: dict[str, object],
        spawn_xyz: tuple[float, float, float],
    ) -> None:
        specs = profile.get("obstacles")
        if not isinstance(specs, list) or not specs:
            raise ValueError(f"static profile {profile_name!r} has no obstacles")
        self.profile_name = profile_name
        self.obstacles: list[dict[str, object]] = []
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform.Define(stage, OBSTACLE_ROOT)
            for spec_value in specs:
                if not isinstance(spec_value, dict):
                    raise ValueError("static obstacle entries must be mappings")
                self.obstacles.append(self._create(stage, spec_value, spawn_xyz))

    @staticmethod
    def _position(
        spec: dict[str, object], spawn_xyz: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        if "position_relative_spawn_m" in spec:
            relative = _vec3(
                spec["position_relative_spawn_m"], "position_relative_spawn_m"
            )
            return tuple(spawn_xyz[index] + relative[index] for index in range(3))
        return _vec3(spec["position_m"], "position_m")

    def _create(
        self,
        stage: Usd.Stage,
        spec: dict[str, object],
        spawn_xyz: tuple[float, float, float],
    ) -> dict[str, object]:
        name = str(spec["name"])
        kind = str(spec["kind"])
        path = f"{OBSTACLE_ROOT}/{name}"
        if kind == "box":
            prim = _box(stage, path, spec)
        elif kind == "capsule":
            prim = _capsule(stage, path, spec)
        else:
            raise ValueError(f"unsupported static obstacle kind: {kind}")
        position = self._position(spec, spawn_xyz)
        xformable = UsdGeom.Xformable(prim)
        operations = {item.GetOpName(): item for item in xformable.GetOrderedXformOps()}
        translate = operations.get("xformOp:translate")
        if translate is None:
            raise RuntimeError(f"static obstacle has no translation op: {path}")
        translate.Set(Gf.Vec3d(*position))
        return {
            "name": name,
            "kind": kind,
            "prim_path": path,
            "position_m": list(position),
        }

    def summary(self) -> dict[str, object]:
        return {
            "enabled": True,
            "profile": self.profile_name,
            "root_prim": OBSTACLE_ROOT,
            "obstacles": self.obstacles,
        }
