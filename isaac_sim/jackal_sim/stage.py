"""Compose the Kujiale room, Jackal and four Hawk stereo pairs at runtime."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import time

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from .scenarios import SpawnPose


ROBOT_PRIM_PATH = "/World/Jackal"
ARTICULATION_ROOT = ROBOT_PRIM_PATH
BASE_LINK_PRIM_PATH = f"{ROBOT_PRIM_PATH}/base_link"
PHYSICS_SCENE_PATH = "/World/PhysicsScene"
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class AssetFingerprint:
    path: str
    size: int
    mtime_ns: int
    sha256: str


def fingerprint(path: Path) -> AssetFingerprint:
    stat = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return AssetFingerprint(
        str(path.resolve()), stat.st_size, stat.st_mtime_ns, digest.hexdigest()
    )


def open_environment_once(simulation_app: object, environment_path: Path) -> Usd.Stage:
    """Open the selected source stage exactly once without making it writable."""

    context = omni.usd.get_context()
    context.disable_save_to_recent_files()
    try:
        opened = context.open_stage(str(environment_path.resolve()))
    finally:
        context.enable_save_to_recent_files()
    if not opened:
        raise RuntimeError(f"Isaac Sim could not open Kujiale stage: {environment_path}")

    deadline = time.monotonic() + 180.0
    while context.get_stage_loading_status()[2] > 0:
        simulation_app.update()
        if time.monotonic() > deadline:
            raise TimeoutError("Kujiale dependencies did not finish loading within 180 seconds")
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("USD context returned no stage after opening the Kujiale scene")
    root_identifier = stage.GetRootLayer().realPath or stage.GetRootLayer().identifier
    if root_identifier != str(environment_path.resolve()):
        raise RuntimeError(f"unexpected root layer: {root_identifier}")
    if stage.GetDefaultPrim().GetPath() != Sdf.Path("/Root"):
        raise RuntimeError(f"unexpected Kujiale default prim: {stage.GetDefaultPrim().GetPath()}")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise RuntimeError("Kujiale stage must be Z-up")
    if abs(float(UsdGeom.GetStageMetersPerUnit(stage)) - 1.0) > 1.0e-9:
        raise RuntimeError("Kujiale stage must use metersPerUnit=1")
    return stage


def _set_pose(prim: Usd.Prim, xyz: tuple[float, float, float], yaw: float) -> None:
    xformable = UsdGeom.Xformable(prim)
    operations = {operation.GetOpName(): operation for operation in xformable.GetOrderedXformOps()}
    translate = operations.get("xformOp:translate") or xformable.AddTranslateOp()
    orient = operations.get("xformOp:orient") or xformable.AddOrientOp()
    translate.Set(Gf.Vec3d(*xyz))
    orient.Set(Gf.Quatd(math.cos(yaw * 0.5), Gf.Vec3d(0.0, 0.0, math.sin(yaw * 0.5))))


def _get_or_add_xform_op(
    xformable: UsdGeom.Xformable,
    name: str,
    factory,
) -> UsdGeom.XformOp:
    """Return an inherited/local transform op without authoring a duplicate."""

    operations = {
        operation.GetOpName(): operation
        for operation in xformable.GetOrderedXformOps()
    }
    operation = operations.get(name)
    if operation is not None:
        return operation
    return factory()


def _configure_physics(stage: Usd.Stage, physics_hz: int) -> str:
    scenes = [prim for prim in stage.TraverseAll() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scene_prim = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH).GetPrim()
    elif len(scenes) == 1:
        scene_prim = scenes[0]
        if str(scene_prim.GetPath()) != PHYSICS_SCENE_PATH:
            raise RuntimeError(
                f"unexpected PhysicsScene {scene_prim.GetPath()}; expected {PHYSICS_SCENE_PATH}"
            )
    else:
        raise RuntimeError(
            f"expected zero or one PhysicsScene, found {[str(p.GetPath()) for p in scenes]}"
        )
    scene = UsdPhysics.Scene(scene_prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    scene_prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(
        physics_hz
    )
    scene_prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
    scene_prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    scene_prim.CreateAttribute("physxScene:enableStabilization", Sdf.ValueTypeNames.Bool).Set(
        True
    )
    stage.SetTimeCodesPerSecond(float(physics_hz))
    return str(scene_prim.GetPath())


def _repair_environment(
    stage: Usd.Stage,
    source_directory: Path,
    control: dict[str, object],
) -> dict[str, object]:
    """Author Kujiale rendering and collision fixes in the session layer."""

    repaired_assets: list[str] = []
    double_sided: list[str] = []
    for prim in stage.TraverseAll():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            if not bool(mesh.GetDoubleSidedAttr().Get()):
                mesh.CreateDoubleSidedAttr().Set(True)
                double_sided.append(str(prim.GetPath()))
        for attribute in prim.GetAttributes():
            value = attribute.Get()
            if not isinstance(value, Sdf.AssetPath) or not value.path.startswith(".../"):
                continue
            target = (source_directory / value.path[4:]).resolve()
            if target.is_file():
                attribute.Set(Sdf.AssetPath(str(target)))
                repaired_assets.append(f"{prim.GetPath()}.{attribute.GetName()}")

    # Preserve the source environment's contact properties.  The proven
    # Kujiale reference branch repairs malformed asset paths and makes local
    # meshes double-sided, but deliberately does not impose a strong inherited
    # physics material on /Root.  Such a binding changes skid-steer contact
    # friction for every floor and obstacle in the room.
    collision_prim_count = sum(
        prim.HasAPI(UsdPhysics.CollisionAPI) for prim in stage.TraverseAll()
    )
    return {
        "double_sided_mesh_count": len(double_sided),
        "repaired_asset_path_count": len(repaired_assets),
        "repaired_asset_paths": repaired_assets,
        "source_physics_material_preserved": True,
        "physics_material_binding_root": None,
        "collision_prim_count": collision_prim_count,
    }


def _configure_wheel_overlay(stage: Usd.Stage, control: dict[str, object]) -> None:
    """Apply the validated Jackal wheel/drive repair without touching its source USD."""

    physics = control["physics"]
    robot = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    robot.CreateAttribute(
        "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int
    ).Set(int(physics["solver_position_iterations"]))
    robot.CreateAttribute(
        "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int
    ).Set(int(physics["solver_velocity_iterations"]))

    material_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/PhysicsMaterials/wheels")
    if not material_prim.IsValid():
        raise RuntimeError("Jackal wheel physics material is missing")
    material = UsdPhysics.MaterialAPI.Apply(material_prim)
    material.CreateStaticFrictionAttr().Set(float(physics["wheel_static_friction"]))
    material.CreateDynamicFrictionAttr().Set(float(physics["wheel_dynamic_friction"]))
    material.CreateRestitutionAttr().Set(0.0)

    for name in WHEEL_NAMES:
        link_path = f"{ROBOT_PRIM_PATH}/{name}_wheel_link"
        old_collision = stage.GetPrimAtPath(f"{link_path}/collisions")
        if old_collision.IsValid():
            old_collision.SetActive(False)
        collider = UsdGeom.Cylinder.Define(stage, f"{link_path}/collisions_v2")
        collider.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        collider.CreateRadiusAttr().Set(0.098)
        collider.CreateHeightAttr().Set(0.04)
        collider.CreateExtentAttr().Set(
            [Gf.Vec3f(-0.098, -0.098, -0.02), Gf.Vec3f(0.098, 0.098, 0.02)]
        )
        collider.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr().Set(True)
        UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(UsdShade.Material(material_prim))
        xform = UsdGeom.Xformable(collider.GetPrim())
        _get_or_add_xform_op(
            xform,
            "xformOp:translate",
            xform.AddTranslateOp,
        ).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        orient_operation = _get_or_add_xform_op(
            xform,
            "xformOp:orient",
            xform.AddOrientOp,
        )
        if orient_operation.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
            orient_value = Gf.Quatd(
                0.7071067811865476,
                Gf.Vec3d(0.7071067811865476, 0.0, 0.0),
            )
        else:
            orient_value = Gf.Quatf(
                0.70710677,
                Gf.Vec3f(0.70710677, 0.0, 0.0),
            )
        orient_operation.Set(orient_value)

        joint = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/{name}_wheel_joint")
        if not joint.IsValid():
            raise RuntimeError(f"missing Jackal wheel joint: {joint.GetPath()}")
        joint.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(0.0)
        joint.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(
            float(physics["wheel_drive_damping"])
        )
        joint.CreateAttribute("drive:angular:physics:maxForce", Sdf.ValueTypeNames.Float).Set(
            float(physics["wheel_drive_max_force"])
        )
        joint.CreateAttribute("drive:angular:physics:type", Sdf.ValueTypeNames.Token).Set("force")
        joint.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float).Set(0.0)
        joint.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(
            float(physics["wheel_drive_max_velocity_deg_s"])
        )


def _compose_hawk_rig(stage: Usd.Stage, hawk_path: Path) -> dict[str, object]:
    pair_yaws = {
        "front": 0.0,
        "left": math.pi * 0.5,
        "right": -math.pi * 0.5,
        "back": math.pi,
    }
    prim_paths: dict[str, dict[str, str]] = {}
    sensors_root = UsdGeom.Xform.Define(stage, f"{BASE_LINK_PRIM_PATH}/sensors")
    if not sensors_root.GetPrim().IsValid():
        raise RuntimeError("failed to create Jackal sensor mount")
    for name, yaw in pair_yaws.items():
        root_path = f"{BASE_LINK_PRIM_PATH}/sensors/{name}_hawk"
        root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
        if not root.GetReferences().AddReference(str(hawk_path.resolve()), "/hawk"):
            raise RuntimeError(f"failed to reference Hawk asset for {name}")
        # The Hawk default prim is located at the left optical center. Offset
        # it by half the baseline so every configured pose describes the pair midpoint.
        root_xyz = (
            -math.sin(yaw) * 0.075,
            math.cos(yaw) * 0.075,
            0.42,
        )
        _set_pose(root, root_xyz, yaw)
        prim_paths[name] = {
            "root": root_path,
            "left": f"{root_path}/left/camera_left",
            "right": f"{root_path}/right/camera_right",
            "imu": f"{root_path}/Imu/Imu_Sensor",
        }
    missing = [
        path
        for pair in prim_paths.values()
        for path in (pair["left"], pair["right"], pair["imu"])
        if not stage.GetPrimAtPath(path).IsValid()
    ]
    if missing:
        raise RuntimeError(f"composed Hawk rig is missing prims: {missing}")
    return {"pair_midpoint_xyz_m": [0.0, 0.0, 0.42], "pairs": prim_paths}


def _spawn_pose(x: float, y: float, z: float, yaw: float) -> SpawnPose:
    footprint = ((0.255, 0.210), (0.255, -0.210), (-0.230, -0.210), (-0.230, 0.210))
    transformed = [
        (x + math.cos(yaw) * px - math.sin(yaw) * py,
         y + math.sin(yaw) * px + math.cos(yaw) * py)
        for px, py in footprint
    ]
    return SpawnPose(
        x=x,
        y=y,
        z=z,
        yaw_radians=yaw,
        floor_prim="/Root",
        candidates_evaluated=1,
        floor_count=1,
        obstacle_count=0,
        footprint_aabb=(
            min(point[0] for point in transformed),
            max(point[0] for point in transformed),
            min(point[1] for point in transformed),
            max(point[1] for point in transformed),
        ),
    )


def compose_robot(
    stage: Usd.Stage,
    robot_path: Path,
    hawk_path: Path,
    control: dict[str, object],
    *,
    physics_hz: int,
    spawn_height: float,
    spawn_preferred_xy: tuple[float, float],
    spawn_yaw: float,
    **_unused: object,
) -> tuple[Usd.Prim, SpawnPose, dict[str, object]]:
    """Compose the complete runtime stage in its anonymous session layer."""

    session_layer = stage.GetSessionLayer()
    root_layer = stage.GetRootLayer()
    if session_layer is None or not session_layer.anonymous:
        raise RuntimeError("Kujiale stage does not have an anonymous session layer")
    root_dirty_before = root_layer.dirty
    spawn = _spawn_pose(
        float(spawn_preferred_xy[0]),
        float(spawn_preferred_xy[1]),
        float(spawn_height),
        float(spawn_yaw),
    )

    with Usd.EditContext(stage, session_layer):
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/Graphs")
        environment_repairs = _repair_environment(
            stage,
            Path(root_layer.realPath or root_layer.identifier).resolve().parent,
            control,
        )
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM_PATH).GetPrim()
        if not robot.GetReferences().AddReference(str(robot_path.resolve()), "/jackal"):
            raise RuntimeError(f"failed to reference Jackal asset: {robot_path}")
        _set_pose(robot, (spawn.x, spawn.y, spawn.z), spawn.yaw_radians)
        _configure_wheel_overlay(stage, control)
        hawk_rig = _compose_hawk_rig(stage, hawk_path)
        physics_scene = _configure_physics(stage, physics_hz)

    required = [
        BASE_LINK_PRIM_PATH,
        *[f"{ROBOT_PRIM_PATH}/{name}_wheel_joint" for name in WHEEL_NAMES],
    ]
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"composed Jackal is missing required prims: {missing}")
    scenes = [prim for prim in stage.TraverseAll() if prim.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise RuntimeError(f"composed stage must contain one PhysicsScene, found {len(scenes)}")
    if root_layer.dirty != root_dirty_before:
        raise RuntimeError("runtime composition changed the Kujiale root layer")

    used_layers = [layer.identifier for layer in stage.GetUsedLayers()]
    details = {
        "robot_prim": ROBOT_PRIM_PATH,
        "articulation_root": ARTICULATION_ROOT,
        "base_link_prim": BASE_LINK_PRIM_PATH,
        "session_layer": session_layer.identifier,
        "session_layer_anonymous": True,
        "root_layer": root_layer.realPath or root_layer.identifier,
        "root_layer_dirty_before_session": root_dirty_before,
        "root_layer_dirty_after_session": root_layer.dirty,
        "physics_scene": physics_scene,
        "required_prims": required,
        "used_layer_count": len(used_layers),
        "spawn": asdict(spawn),
        "environment_repairs": environment_repairs,
        "hawk_rig": hawk_rig,
        "wheel_overlay": "runtime_reference_caae0c08",
    }
    return stage.GetPrimAtPath(ROBOT_PRIM_PATH), spawn, details
