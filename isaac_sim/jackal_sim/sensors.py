"""Runtime Hawk RGB, native front-depth and front-IMU ROS graphs."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, Usd, UsdGeom

from .graphs import GRAPH_ROOT


CAMERA_PROFILES = {
    "mapping_8cam": ("front", "left", "right", "back"),
    "navigation_6cam": ("front", "left", "right"),
}


@dataclass(frozen=True)
class SensorGraphSummary:
    graph_paths: tuple[str, ...]
    camera_profile: str
    active_pairs: tuple[str, ...]
    active_image_streams: int
    rear_render_products_created: bool
    lidar_enabled: bool
    camera_prims: dict[str, dict[str, str]]
    stereo_resolution: tuple[int, int]
    depth_resolution: tuple[int, int]
    depth_min_range_m: float
    image_rate_hz: float
    imu_rate_hz: float
    topics: dict[str, str]
    frames: dict[str, str]

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["graph_paths"] = list(self.graph_paths)
        result["active_pairs"] = list(self.active_pairs)
        result["stereo_resolution"] = list(self.stereo_resolution)
        result["depth_resolution"] = list(self.depth_resolution)
        return result


def _pair_config(sensor: dict[str, object], pair_name: str) -> dict[str, object]:
    if pair_name == "front":
        front = sensor["front_stereo"]
        return {
            "left_camera_prim": front["left_camera_prim"],
            "right_camera_prim": front["right_camera_prim"],
            "image_width": front["image_width"],
            "image_height": front["image_height"],
            "image_rate_hz": front["image_rate_hz"],
            "navigation_projection": front["navigation_projection"],
        }
    surround = sensor["surround_stereo"]
    return {
        **surround["cameras"][pair_name],
        "image_width": surround["image_width"],
        "image_height": surround["image_height"],
        "image_rate_hz": surround["image_rate_hz"],
        "navigation_projection": surround["navigation_projection"],
    }


def _topic(sensor: dict[str, object], pair_name: str, side: str, kind: str) -> str:
    topics = sensor["topics"]
    key = f"{side}_{kind}" if pair_name == "front" else f"{pair_name}_{side}_{kind}"
    return str(topics[key])


def _frame(sensor: dict[str, object], pair_name: str, side: str) -> str:
    frames = sensor["frames"]
    key = f"{side}_optical" if pair_name == "front" else f"{pair_name}_{side}_optical"
    return str(frames[key])


def _configure_camera(prim: object, rate_hz: float, projection_name: str) -> None:
    tick_rate = prim.GetAttribute("omni:sensor:tickRate")
    projection = prim.GetAttribute("cameraProjectionType")
    if not tick_rate.IsValid() or not projection.IsValid():
        raise RuntimeError(f"Hawk camera is missing sensor attributes: {prim.GetPath()}")
    tick_rate.Set(rate_hz)
    projection.Set(projection_name)
    distortion = prim.GetAttribute("physicalDistortionCoefficients")
    if distortion.IsValid() and distortion.Get() is not None:
        distortion.Set([0.0] * len(distortion.Get()))


def _configure_front_depth_near_clip(stage: object, sensor: dict[str, object]) -> float:
    front = sensor["front_stereo"]
    minimum = float(front["depth_min_range_m"])
    if minimum <= 0.0:
        raise RuntimeError("front Hawk depth_min_range_m must be positive")
    prim = stage.GetPrimAtPath(str(front["left_camera_prim"]))
    if not prim.IsValid() or not prim.IsA(UsdGeom.Camera):
        raise RuntimeError(f"front Hawk depth camera is invalid: {prim.GetPath()}")
    clipping = UsdGeom.Camera(prim).GetClippingRangeAttr()
    current = clipping.Get()
    if current is None or float(current[1]) <= minimum:
        raise RuntimeError(
            f"front Hawk far clipping plane must exceed {minimum:.3f} m"
        )
    clipping.Set(Gf.Vec2f(minimum, float(current[1])))
    return minimum


def _configure_sensor_rates(
    stage: object, sensor: dict[str, object], active_pairs: tuple[str, ...]
) -> None:
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for name in active_pairs:
            pair = _pair_config(sensor, name)
            for side in ("left", "right"):
                prim = stage.GetPrimAtPath(str(pair[f"{side}_camera_prim"]))
                if not prim.IsValid():
                    raise RuntimeError(f"missing {name} {side} Hawk camera")
                _configure_camera(
                    prim,
                    float(pair["image_rate_hz"]),
                    str(pair["navigation_projection"]),
                )
        _configure_front_depth_near_clip(stage, sensor)
        imu_path = str(sensor["front_stereo"]["imu_prim"])
        imu = stage.GetPrimAtPath(imu_path)
        period = imu.GetAttribute("sensorPeriod")
        if not imu.IsValid() or not period.IsValid():
            raise RuntimeError(f"front Hawk IMU is missing or invalid: {imu_path}")
        period.Set(1.0 / float(sensor["front_stereo"]["imu_rate_hz"]))


def _qos_nodes() -> list[tuple[str, str]]:
    return [
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("SensorQos", "isaacsim.ros2.bridge.ROS2QoSProfile"),
    ]


def _qos_values(sensor: dict[str, object]) -> list[tuple[str, object]]:
    return [
        ("SensorQos.inputs:createProfile", "Default for publishers/subscribers"),
        ("SensorQos.inputs:reliability", str(sensor.get("qos_reliability", "bestEffort"))),
        ("SensorQos.inputs:depth", 20),
    ]


def _create_stereo_graph(sensor: dict[str, object], pair_name: str) -> str:
    graph_name = "FrontStereo" if pair_name == "front" else f"{pair_name.title()}Stereo"
    path = f"{GRAPH_ROOT}/{graph_name}"
    pair = _pair_config(sensor, pair_name)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *_qos_nodes(),
                ("CreateLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishLeftRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishRightRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishStereoInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                *_qos_values(sensor),
                ("CreateLeft.inputs:cameraPrim", [usdrt.Sdf.Path(str(pair["left_camera_prim"]))]),
                ("CreateRight.inputs:cameraPrim", [usdrt.Sdf.Path(str(pair["right_camera_prim"]))]),
                ("CreateLeft.inputs:width", int(pair["image_width"])),
                ("CreateLeft.inputs:height", int(pair["image_height"])),
                ("CreateRight.inputs:width", int(pair["image_width"])),
                ("CreateRight.inputs:height", int(pair["image_height"])),
                ("PublishLeftRgb.inputs:topicName", _topic(sensor, pair_name, "left", "rgb_raw")),
                ("PublishLeftRgb.inputs:frameId", _frame(sensor, pair_name, "left")),
                ("PublishLeftRgb.inputs:type", "rgb"),
                ("PublishLeftRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishRightRgb.inputs:topicName", _topic(sensor, pair_name, "right", "rgb_raw")),
                ("PublishRightRgb.inputs:frameId", _frame(sensor, pair_name, "right")),
                ("PublishRightRgb.inputs:type", "rgb"),
                ("PublishRightRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishStereoInfo.inputs:topicName", _topic(sensor, pair_name, "left", "camera_info")),
                ("PublishStereoInfo.inputs:topicNameRight", _topic(sensor, pair_name, "right", "camera_info")),
                ("PublishStereoInfo.inputs:frameId", _frame(sensor, pair_name, "left")),
                ("PublishStereoInfo.inputs:frameIdRight", _frame(sensor, pair_name, "right")),
                ("PublishStereoInfo.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateLeft.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateRight.inputs:execIn"),
                ("CreateLeft.outputs:execOut", "PublishLeftRgb.inputs:execIn"),
                ("CreateRight.outputs:execOut", "PublishRightRgb.inputs:execIn"),
                ("CreateLeft.outputs:execOut", "PublishStereoInfo.inputs:execIn"),
                ("CreateLeft.outputs:renderProductPath", "PublishLeftRgb.inputs:renderProductPath"),
                ("CreateRight.outputs:renderProductPath", "PublishRightRgb.inputs:renderProductPath"),
                ("CreateLeft.outputs:renderProductPath", "PublishStereoInfo.inputs:renderProductPath"),
                ("CreateRight.outputs:renderProductPath", "PublishStereoInfo.inputs:renderProductPathRight"),
                ("Context.outputs:context", "PublishLeftRgb.inputs:context"),
                ("Context.outputs:context", "PublishRightRgb.inputs:context"),
                ("Context.outputs:context", "PublishStereoInfo.inputs:context"),
                ("SensorQos.outputs:qosProfile", "PublishLeftRgb.inputs:qosProfile"),
                ("SensorQos.outputs:qosProfile", "PublishRightRgb.inputs:qosProfile"),
                ("SensorQos.outputs:qosProfile", "PublishStereoInfo.inputs:qosProfile"),
            ],
        },
    )
    return path


def _create_depth_graph(sensor: dict[str, object]) -> str:
    path = f"{GRAPH_ROOT}/FrontDepth"
    front = sensor["front_stereo"]
    topics = sensor["topics"]
    frames = sensor["frames"]
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *_qos_nodes(),
                ("CreateDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                *_qos_values(sensor),
                ("CreateDepth.inputs:cameraPrim", [usdrt.Sdf.Path(str(front["left_camera_prim"]))]),
                ("CreateDepth.inputs:width", int(front["depth_width"])),
                ("CreateDepth.inputs:height", int(front["depth_height"])),
                ("PublishDepth.inputs:topicName", str(topics["depth"])),
                ("PublishDepth.inputs:frameId", str(frames["left_optical"])),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:resetSimulationTimeOnStop", False),
                ("PublishDepthInfo.inputs:topicName", str(topics["depth_camera_info"])),
                ("PublishDepthInfo.inputs:frameId", str(frames["left_optical"])),
                ("PublishDepthInfo.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateDepth.inputs:execIn"),
                ("CreateDepth.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("CreateDepth.outputs:execOut", "PublishDepthInfo.inputs:execIn"),
                ("CreateDepth.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("CreateDepth.outputs:renderProductPath", "PublishDepthInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishDepth.inputs:context"),
                ("Context.outputs:context", "PublishDepthInfo.inputs:context"),
                ("SensorQos.outputs:qosProfile", "PublishDepth.inputs:qosProfile"),
                ("SensorQos.outputs:qosProfile", "PublishDepthInfo.inputs:qosProfile"),
            ],
        },
    )
    return path


def _create_imu_graph(sensor: dict[str, object]) -> str:
    path = f"{GRAPH_ROOT}/FrontImu"
    front = sensor["front_stereo"]
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                *_qos_nodes(),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                *_qos_values(sensor),
                ("SimulationTime.inputs:resetOnStop", False),
                ("ReadImu.inputs:imuPrim", [usdrt.Sdf.Path(str(front["imu_prim"]))]),
                ("ReadImu.inputs:readGravity", True),
                ("ReadImu.inputs:useLatestData", False),
                ("PublishImu.inputs:topicName", str(sensor["topics"]["imu"])),
                ("PublishImu.inputs:frameId", str(sensor["frames"]["imu"])),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "ReadImu.inputs:execIn"),
                ("ReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
                ("SimulationTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
                ("Context.outputs:context", "PublishImu.inputs:context"),
                ("SensorQos.outputs:qosProfile", "PublishImu.inputs:qosProfile"),
            ],
        },
    )
    return path


def create_sensor_graphs(
    stage: object,
    sensor: dict[str, object],
    *,
    camera_profile: str,
) -> SensorGraphSummary:
    if camera_profile not in CAMERA_PROFILES:
        raise ValueError(f"unknown camera profile: {camera_profile}")
    if sensor.get("lidar_enabled") is not False:
        raise RuntimeError("Jackal LiDAR must remain disabled; only Hawk sensors are supported")
    active_pairs = CAMERA_PROFILES[camera_profile]
    _configure_sensor_rates(stage, sensor, active_pairs)
    graph_paths = [_create_stereo_graph(sensor, name) for name in active_pairs]
    graph_paths.extend((_create_depth_graph(sensor), _create_imu_graph(sensor)))
    camera_prims = {
        name: {
            "left": str(_pair_config(sensor, name)["left_camera_prim"]),
            "right": str(_pair_config(sensor, name)["right_camera_prim"]),
        }
        for name in active_pairs
    }
    front = sensor["front_stereo"]
    return SensorGraphSummary(
        graph_paths=tuple(graph_paths),
        camera_profile=camera_profile,
        active_pairs=active_pairs,
        active_image_streams=2 * len(active_pairs),
        rear_render_products_created="back" in active_pairs,
        lidar_enabled=False,
        camera_prims=camera_prims,
        stereo_resolution=(int(front["image_width"]), int(front["image_height"])),
        depth_resolution=(int(front["depth_width"]), int(front["depth_height"])),
        depth_min_range_m=float(front["depth_min_range_m"]),
        image_rate_hz=float(front["image_rate_hz"]),
        imu_rate_hz=float(front["imu_rate_hz"]),
        topics={key: str(value) for key, value in sensor["topics"].items()},
        frames={key: str(value) for key, value in sensor["frames"].items()},
    )
