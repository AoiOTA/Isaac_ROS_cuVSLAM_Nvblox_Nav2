"""Open the official warehouse once and compose Nova Carter in the session layer."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import time

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .scenarios import SpawnPose, find_collision_free_spawn


ROBOT_PRIM_PATH = "/World/NovaCarter"
FORBIDDEN_ROS_SAMPLE_NAME = "Nova_Carter_ROS.usd"


@dataclass(frozen=True)
class AssetFingerprint:
    path: str
    size: int
    mtime_ns: int


def fingerprint(path: Path) -> AssetFingerprint:
    stat = path.stat()
    return AssetFingerprint(str(path.resolve()), stat.st_size, stat.st_mtime_ns)


def open_warehouse_once(simulation_app: object, warehouse_path: Path) -> Usd.Stage:
    context = omni.usd.get_context()
    context.disable_save_to_recent_files()
    try:
        opened = context.open_stage(str(warehouse_path.resolve()))
    finally:
        context.enable_save_to_recent_files()
    if not opened:
        raise RuntimeError(f"Isaac Sim could not open warehouse stage: {warehouse_path}")

    deadline = time.monotonic() + 120.0
    while context.get_stage_loading_status()[2] > 0:
        simulation_app.update()
        if time.monotonic() > deadline:
            raise TimeoutError("warehouse dependencies did not finish loading within 120 seconds")
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("USD context returned no stage after opening the warehouse")
    if stage.GetRootLayer().realPath != str(warehouse_path.resolve()):
        root_identifier = stage.GetRootLayer().realPath or stage.GetRootLayer().identifier
        raise RuntimeError(
            f"unexpected root layer: {root_identifier}"
        )
    if stage.GetDefaultPrim().GetPath() != Sdf.Path("/World"):
        raise RuntimeError(f"unexpected warehouse default prim: {stage.GetDefaultPrim().GetPath()}")
    return stage


def _select_variant(prim: Usd.Prim, set_name: str, selection: str) -> None:
    variant_set = prim.GetVariantSets().GetVariantSet(set_name)
    choices = variant_set.GetVariantNames()
    if selection not in choices:
        raise RuntimeError(
            f"Nova Carter variant {set_name}={selection} is unavailable; choices={choices}"
        )
    if not variant_set.SetVariantSelection(selection):
        raise RuntimeError(f"failed to select Nova Carter variant {set_name}={selection}")


def _set_pose(prim: Usd.Prim, spawn: SpawnPose) -> None:
    xformable = UsdGeom.Xformable(prim)
    operations = {operation.GetOpName(): operation for operation in xformable.GetOrderedXformOps()}
    translate = operations.get("xformOp:translate") or xformable.AddTranslateOp()
    orient = operations.get("xformOp:orient") or xformable.AddOrientOp()
    translate.Set(Gf.Vec3d(spawn.x, spawn.y, spawn.z))
    half_yaw = spawn.yaw_radians * 0.5
    orient.Set(
        Gf.Quatd(math.cos(half_yaw), Gf.Vec3d(0.0, 0.0, math.sin(half_yaw)))
    )


def _configure_physics(stage: Usd.Stage, physics_hz: int) -> str:
    physics_scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if len(physics_scenes) != 1:
        raise RuntimeError(f"expected one PhysicsScene, found {len(physics_scenes)}")
    scene_prim = physics_scenes[0]
    scene = UsdPhysics.Scene(scene_prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    scene_prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(
        physics_hz
    )
    stage.SetTimeCodesPerSecond(float(physics_hz))
    return scene_prim.GetPath().pathString


def compose_robot(
    stage: Usd.Stage,
    robot_path: Path,
    forbidden_ros_sample_path: Path,
    *,
    physics_hz: int,
    spawn_clearance: float,
    spawn_grid_resolution: float,
    spawn_height: float,
    spawn_obstacle_height: float,
    spawn_preferred_xy: tuple[float, float],
    spawn_yaw: float = 0.0,
) -> tuple[Usd.Prim, SpawnPose, dict[str, object]]:
    if robot_path.resolve() == forbidden_ros_sample_path.resolve():
        raise RuntimeError("the project is forbidden from loading Nova_Carter_ROS.usd")

    footprint = ((0.14, 0.25), (0.14, -0.25), (-0.607, -0.25), (-0.607, 0.25))
    spawn = find_collision_free_spawn(
        stage,
        footprint=footprint,
        clearance=spawn_clearance,
        grid_resolution=spawn_grid_resolution,
        spawn_height=spawn_height,
        obstacle_height=spawn_obstacle_height,
        preferred_xy=spawn_preferred_xy,
        yaw_radians=spawn_yaw,
    )

    session_layer = stage.GetSessionLayer()
    root_layer = stage.GetRootLayer()
    if session_layer is None or not session_layer.anonymous:
        raise RuntimeError("warehouse stage does not have an anonymous session layer")
    root_layer_dirty_before_session = root_layer.dirty

    with Usd.EditContext(stage, session_layer):
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM_PATH).GetPrim()
        reference = Sdf.Reference(str(robot_path.resolve()), "/nova_carter")
        if not robot.GetReferences().AddReference(reference):
            raise RuntimeError(f"failed to reference Nova Carter asset: {robot_path}")
        _select_variant(robot, "Physics", "physx")
        _select_variant(robot, "Sensors", "All_Sensors")
        _select_variant(robot, "ROS", "Disabled")
        _set_pose(robot, spawn)
        physics_scene_path = _configure_physics(stage, physics_hz)

    robot = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot.IsValid():
        raise RuntimeError(f"composed robot prim is invalid: {ROBOT_PRIM_PATH}")
    expected_paths = [
        f"{ROBOT_PRIM_PATH}/joint_wheel_left",
        f"{ROBOT_PRIM_PATH}/joint_wheel_right",
        f"{ROBOT_PRIM_PATH}/chassis_link",
    ]
    missing = [path for path in expected_paths if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"composed Nova Carter is missing required prims: {missing}")

    used_layers = [layer.identifier for layer in stage.GetUsedLayers()]
    forbidden = [
        identifier for identifier in used_layers if FORBIDDEN_ROS_SAMPLE_NAME in identifier
    ]
    if forbidden:
        raise RuntimeError(f"forbidden official ROS sample was loaded: {forbidden}")
    if str(robot_path.resolve()) not in used_layers:
        raise RuntimeError("Nova Carter main USD is not present in the composed layer stack")
    root_layer_dirty_after_session = root_layer.dirty
    if root_layer_dirty_after_session != root_layer_dirty_before_session:
        raise RuntimeError("session composition changed the warehouse root-layer dirty state")

    robot_prims = list(Usd.PrimRange(robot))
    graph_count = sum("omnigraph" in prim.GetTypeName().lower() for prim in robot_prims)
    camera_count = sum(prim.IsA(UsdGeom.Camera) for prim in robot_prims)
    variants = {
        name: robot.GetVariantSets().GetVariantSet(name).GetVariantSelection()
        for name in ("Physics", "Sensors", "ROS")
    }
    details = {
        "robot_prim": ROBOT_PRIM_PATH,
        "session_layer": session_layer.identifier,
        "session_layer_anonymous": session_layer.anonymous,
        "root_layer": root_layer.realPath or root_layer.identifier,
        "root_layer_dirty_before_session": root_layer_dirty_before_session,
        "root_layer_dirty_after_session": root_layer_dirty_after_session,
        "physics_scene": physics_scene_path,
        "variants": variants,
        "camera_count": camera_count,
        "omnigraph_count": graph_count,
        "required_prims": expected_paths,
        "forbidden_ros_sample_loaded": False,
        "used_layer_count": len(used_layers),
        "spawn": asdict(spawn),
    }
    return robot, spawn, details
