"""Session-layer kinematic obstacles and PhysX robot-contact accounting."""

from __future__ import annotations

from dataclasses import dataclass
import math

from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


OBSTACLE_ROOT = "/World/Stage9DynamicObstacles"


@dataclass
class RuntimeObstacle:
    name: str
    kind: str
    prim_path: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    period_s: float
    phase: float
    translate_op: UsdGeom.XformOp
    rotate_op: UsdGeom.XformOp | None
    distance_travelled_m: float = 0.0
    last_position: tuple[float, float, float] | None = None


def _vec3(values: object, label: str) -> tuple[float, float, float]:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains non-finite values")
    return result


def _ordered_ops(prim: Usd.Prim) -> dict[str, UsdGeom.XformOp]:
    return {
        operation.GetOpName(): operation
        for operation in UsdGeom.Xformable(prim).GetOrderedXformOps()
    }


def _apply_kinematic(prim: Usd.Prim) -> None:
    body = UsdPhysics.RigidBodyAPI.Apply(prim)
    body.CreateKinematicEnabledAttr().Set(True)
    body.CreateRigidBodyEnabledAttr().Set(True)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(True)
    reporter = PhysxSchema.PhysxContactReportAPI.Apply(prim)
    reporter.CreateThresholdAttr().Set(0.0)


def _create_box(stage: Usd.Stage, path: str, spec: dict[str, object]) -> Usd.Prim:
    size = _vec3(spec["size_m"], f"{path}.size_m")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*_vec3(spec["color_rgb"], "color_rgb"))])
    # Author SRT in the conventional translate-before-scale op order. If the
    # translate op is appended after scale, USD scales the waypoint itself and
    # silently moves the obstacle much closer to the robot than configured.
    cube.AddTranslateOp()
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    _apply_kinematic(prim)
    return prim


def _create_capsule(stage: Usd.Stage, path: str, spec: dict[str, object]) -> Usd.Prim:
    radius = float(spec["radius_m"])
    height = float(spec["height_m"])
    if min(radius, height) <= 0.0:
        raise ValueError("capsule dimensions must be positive")
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
    _apply_kinematic(prim)
    return prim


class DynamicObstacleManager:
    """Move configured kinematic bodies without saving the composed stage."""

    def __init__(
        self,
        stage: Usd.Stage,
        profile_name: str,
        profile: dict[str, object],
        spawn_xyz: tuple[float, float, float],
    ) -> None:
        specs = profile.get("obstacles")
        if not isinstance(specs, list) or not specs:
            raise ValueError(f"dynamic profile {profile_name!r} has no obstacles")
        self.stage = stage
        self.profile_name = profile_name
        self.obstacles: list[RuntimeObstacle] = []
        self.robot_contact_count = 0
        self.robot_contact_pairs: set[tuple[str, str]] = set()
        session = stage.GetSessionLayer()
        with Usd.EditContext(stage, session):
            UsdGeom.Xform.Define(stage, OBSTACLE_ROOT)
            for spec_value in specs:
                if not isinstance(spec_value, dict):
                    raise ValueError("dynamic obstacle entries must be mappings")
                self.obstacles.append(self._create(spec_value, spawn_xyz))
            robot = stage.GetPrimAtPath("/World/NovaCarter/chassis_link")
            if not robot.IsValid():
                raise RuntimeError("Nova Carter chassis is missing for contact reporting")
            PhysxSchema.PhysxContactReportAPI.Apply(robot).CreateThresholdAttr().Set(0.0)
        self.managed_paths = {item.prim_path for item in self.obstacles}
        self.contact_subscription = (
            get_physx_simulation_interface().subscribe_contact_report_events(
                self._on_contact_report
            )
        )
        self.update(0.0)

    def _create(
        self, spec: dict[str, object], spawn_xyz: tuple[float, float, float]
    ) -> RuntimeObstacle:
        name = str(spec["name"])
        kind = str(spec["kind"])
        period = float(spec["period_s"])
        phase = float(spec.get("phase", 0.0))
        if period <= 0.0 or not 0.0 <= phase < 1.0:
            raise ValueError(f"invalid trajectory timing for obstacle {name}")
        if "waypoints_relative_spawn_m" in spec:
            relative = spec["waypoints_relative_spawn_m"]
            if not isinstance(relative, list) or len(relative) != 2:
                raise ValueError(f"{name} must have exactly two relative waypoints")
            values = [_vec3(item, f"{name}.waypoint") for item in relative]
            start, end = (
                tuple(spawn_xyz[index] + value[index] for index in range(3))
                for value in values
            )
        else:
            values = spec.get("waypoints_m")
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f"{name} must have exactly two absolute waypoints")
            start, end = (_vec3(item, f"{name}.waypoint") for item in values)

        if kind == "existing_forklift":
            path = str(spec["prim_path"])
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"official forklift prim is missing: {path}")
            _apply_kinematic(prim)
        else:
            path = f"{OBSTACLE_ROOT}/{name}"
            if kind == "box":
                prim = _create_box(self.stage, path, spec)
            elif kind == "capsule":
                prim = _create_capsule(self.stage, path, spec)
            else:
                raise ValueError(f"unsupported dynamic obstacle kind: {kind}")

        xformable = UsdGeom.Xformable(prim)
        operations = _ordered_ops(prim)
        translate = operations.get("xformOp:translate") or xformable.AddTranslateOp()
        rotate = operations.get("xformOp:rotateZYX")
        if kind == "existing_forklift":
            yaw = float(spec.get("yaw_deg", 0.0))
            if rotate is None:
                rotate = xformable.AddRotateZYXOp()
            rotate.Set(Gf.Vec3f(0.0, 0.0, yaw))
        return RuntimeObstacle(
            name=name,
            kind=kind,
            prim_path=path,
            start=start,
            end=end,
            period_s=period,
            phase=phase,
            translate_op=translate,
            rotate_op=rotate,
        )

    @staticmethod
    def _trajectory(item: RuntimeObstacle, simulation_time: float) -> tuple[float, ...]:
        unit = (simulation_time / item.period_s + item.phase) % 1.0
        alpha = 2.0 * unit if unit <= 0.5 else 2.0 * (1.0 - unit)
        return tuple(
            item.start[index] + alpha * (item.end[index] - item.start[index])
            for index in range(3)
        )

    def update(self, simulation_time: float) -> None:
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for item in self.obstacles:
                position = self._trajectory(item, simulation_time)
                item.translate_op.Set(Gf.Vec3d(*position))
                if item.last_position is not None:
                    item.distance_travelled_m += math.dist(position, item.last_position)
                item.last_position = position

    def _on_contact_report(self, headers: object, _data: object) -> None:
        for header in headers:
            actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            pair = (actor0, actor1)
            has_robot = any(path.startswith("/World/NovaCarter") for path in pair)
            has_obstacle = any(
                path.startswith(OBSTACLE_ROOT)
                or any(path.startswith(managed) for managed in self.managed_paths)
                for path in pair
            )
            if has_robot and has_obstacle:
                self.robot_contact_count += 1
                self.robot_contact_pairs.add(tuple(sorted(pair)))

    def summary(self) -> dict[str, object]:
        def world_position(item: RuntimeObstacle) -> list[float]:
            prim = self.stage.GetPrimAtPath(item.prim_path)
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            return [float(value) for value in matrix.ExtractTranslation()]

        return {
            "enabled": True,
            "profile": self.profile_name,
            "root_prim": OBSTACLE_ROOT,
            "robot_contact_count": self.robot_contact_count,
            "robot_contact_pairs": [list(pair) for pair in sorted(self.robot_contact_pairs)],
            "obstacles": [
                {
                    "name": item.name,
                    "kind": item.kind,
                    "prim_path": item.prim_path,
                    "start_m": list(item.start),
                    "end_m": list(item.end),
                    "period_s": item.period_s,
                    "distance_travelled_m": item.distance_travelled_m,
                    "final_position_m": list(item.last_position or item.start),
                    "final_world_position_m": world_position(item),
                }
                for item in self.obstacles
            ],
        }

    def close(self) -> None:
        self.contact_subscription = None
