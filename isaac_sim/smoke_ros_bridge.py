#!/usr/bin/env python3
"""Isaac Sim 6.0 Standalone smoke publisher for Clock, Image, and CameraInfo."""

from __future__ import annotations

import argparse
import os
import signal
import time

# SimulationApp must be created before importing any other Isaac Sim modules.
from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=45.0, help="Wall-clock publish duration")
    return parser.parse_args()


ARGS = parse_args()
APP = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": 320,
        "height": 240,
    }
)

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402


STOP_REQUESTED = False


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def create_scene() -> None:
    create_new_stage()
    APP.update()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim did not create a USD stage")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    cube = UsdGeom.Cube.Define(stage, "/World/Target")
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    color = cube.CreateDisplayColorAttr()
    color.Set([Gf.Vec3f(0.1, 0.7, 0.2)])

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 3.0))

    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 30.0, 20.0))


def create_graphs() -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/Graphs/Clock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("SimulationTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )

    camera_prim = [usdrt.Sdf.Path("/World/Camera")]
    og.Controller.edit(
        {"graph_path": "/World/Graphs/Camera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim),
                ("CreateRenderProduct.inputs:height", 240),
                ("CreateRenderProduct.inputs:width", 320),
                ("PublishImage.inputs:topicName", "stage1/camera/image_raw"),
                ("PublishImage.inputs:frameId", "stage1_camera_optical"),
                ("PublishImage.inputs:type", "rgb"),
                ("PublishImage.inputs:resetSimulationTimeOnStop", False),
                ("PublishInfo.inputs:topicName", "stage1/camera/camera_info"),
                ("PublishInfo.inputs:frameId", "stage1_camera_optical"),
                ("PublishInfo.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishImage.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishInfo.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishImage.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishInfo.inputs:renderProductPath",
                ),
            ],
        },
    )


def main() -> int:
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError("ROS_DISTRO=jazzy must be sourced before starting Isaac Sim")
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    timeline = None
    try:
        enable_extension("isaacsim.ros2.bridge")
        APP.update()
        create_scene()
        create_graphs()
        APP.update()

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_time_codes_per_second(60.0)
        timeline.play()
        print("STAGE1_BRIDGE_READY", flush=True)
        deadline = time.monotonic() + ARGS.duration
        while APP.is_running() and not STOP_REQUESTED and time.monotonic() < deadline:
            APP.update()
            time.sleep(1.0 / 120.0)
    finally:
        if timeline is not None:
            timeline.stop()
        APP.update()
        APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
