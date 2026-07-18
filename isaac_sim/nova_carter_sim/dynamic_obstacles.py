"""Session-layer kinematic obstacles and PhysX robot-contact accounting."""

from __future__ import annotations

from dataclasses import dataclass
import math

from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from .dynamic_motion import (
    clearance_preserving_step,
    farthest_candidate_index,
    planar_distance,
    should_yield_to_robot,
)


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
    yield_distance_m: float
    yield_release_distance_m: float
    yield_refuge: tuple[float, float, float] | None
    yield_speed_mps: float
    distance_travelled_m: float = 0.0
    last_position: tuple[float, float, float] | None = None
    trajectory_time_s: float = 0.0
    yield_event_count: int = 0
    yielded_frames: int = 0
    yielded_simulation_s: float = 0.0
    is_yielding: bool = False
    yield_latched: bool = False


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
        self.last_update_simulation_time: float | None = None
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
        self.robot_prim = robot
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
        yield_distance = float(spec.get("yield_distance_m", 1.0))
        yield_release_distance = yield_distance + float(
            spec.get("yield_release_margin_m", 0.05)
        )
        yield_speed = float(spec.get("yield_speed_mps", 0.75))
        if (
            period <= 0.0
            or not 0.0 <= phase < 1.0
            or yield_distance <= 0.0
            or not math.isfinite(yield_distance)
            or yield_release_distance <= yield_distance
            or not math.isfinite(yield_release_distance)
            or yield_speed <= 0.0
            or not math.isfinite(yield_speed)
        ):
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

        yield_refuge = None
        if "yield_refuge_relative_spawn_m" in spec:
            relative_refuge = _vec3(
                spec["yield_refuge_relative_spawn_m"], f"{name}.yield_refuge"
            )
            yield_refuge = tuple(
                spawn_xyz[index] + relative_refuge[index] for index in range(3)
            )
        elif "yield_refuge_m" in spec:
            yield_refuge = _vec3(spec["yield_refuge_m"], f"{name}.yield_refuge")

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
            yield_distance_m=yield_distance,
            yield_release_distance_m=yield_release_distance,
            yield_refuge=yield_refuge,
            yield_speed_mps=yield_speed,
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
        if self.last_update_simulation_time is None:
            dt = 0.0
        else:
            dt = max(0.0, simulation_time - self.last_update_simulation_time)
        self.last_update_simulation_time = simulation_time
        robot_matrix = UsdGeom.Xformable(self.robot_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        robot_translation = robot_matrix.ExtractTranslation()
        robot_position = tuple(float(value) for value in robot_translation)
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for item in self.obstacles:
                candidate_time = item.trajectory_time_s + dt
                candidate = self._trajectory(item, candidate_time)
                entering_yield = should_yield_to_robot(
                    robot_position, candidate, item.yield_distance_m
                )
                if entering_yield and item.yield_refuge is not None:
                    item.yield_latched = True
                yielding = item.is_yielding or entering_yield or item.yield_latched
                if yielding and item.last_position is not None:
                    # A stationary kinematic body can deadlock a correctly
                    # stopped navigation controller. Retreat along the same
                    # configured route, choosing the direction that increases
                    # robot clearance, while retaining the physical collider.
                    if item.yield_latched and item.yield_refuge is not None:
                        # The visual map need not be axis-aligned with the USD
                        # world. Consider both route endpoints as well as the
                        # configured refuge and take only a bounded step that
                        # does not reduce current robot clearance. This keeps
                        # the collider while preventing its retreat trajectory
                        # from sweeping through the robot.
                        position = clearance_preserving_step(
                            robot_position,
                            item.last_position,
                            [item.start, item.end, item.yield_refuge],
                            item.yield_speed_mps * dt,
                        )
                    else:
                        retreat_times = [
                            item.trajectory_time_s,
                            item.trajectory_time_s + dt,
                            item.trajectory_time_s - dt,
                        ]
                        retreat_positions = [
                            item.last_position,
                            self._trajectory(item, retreat_times[1]),
                            self._trajectory(item, retreat_times[2]),
                        ]
                        selected = farthest_candidate_index(
                            robot_position, retreat_positions
                        )
                        item.trajectory_time_s = retreat_times[selected]
                        position = retreat_positions[selected]
                    item.yielded_frames += 1
                    item.yielded_simulation_s += dt
                    if not item.is_yielding:
                        item.yield_event_count += 1
                    yielding = item.yield_latched or (
                        planar_distance(robot_position, position)
                        < item.yield_release_distance_m
                    )
                else:
                    item.trajectory_time_s = candidate_time
                    position = candidate
                item.is_yielding = yielding
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
                    "yield_distance_m": item.yield_distance_m,
                    "yield_release_distance_m": item.yield_release_distance_m,
                    "yield_refuge_m": (
                        list(item.yield_refuge) if item.yield_refuge is not None else None
                    ),
                    "yield_speed_mps": item.yield_speed_mps,
                    "yield_latched": item.yield_latched,
                    "yield_event_count": item.yield_event_count,
                    "yielded_frames": item.yielded_frames,
                    "yielded_simulation_s": item.yielded_simulation_s,
                    "is_yielding_on_exit": item.is_yielding,
                    "distance_travelled_m": item.distance_travelled_m,
                    "final_position_m": list(item.last_position or item.start),
                    "final_world_position_m": world_position(item),
                }
                for item in self.obstacles
            ],
        }

    def close(self) -> None:
        self.contact_subscription = None
