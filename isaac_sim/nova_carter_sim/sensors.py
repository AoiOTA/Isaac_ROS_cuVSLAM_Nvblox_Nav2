"""Runtime-only front Hawk camera, depth, and IMU OmniGraphs."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import omni.graph.core as og
import usdrt.Sdf
from pxr import Usd

from .graphs import GRAPH_ROOT


@dataclass(frozen=True)
class SensorGraphSummary:
    graph_paths: tuple[str, ...]
    left_camera_prim: str
    right_camera_prim: str
    imu_prim: str
    stereo_resolution: tuple[int, int]
    depth_resolution: tuple[int, int]
    image_rate_hz: float
    imu_rate_hz: float
    topics: dict[str, str]
    frames: dict[str, str]

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["graph_paths"] = list(self.graph_paths)
        result["stereo_resolution"] = list(self.stereo_resolution)
        result["depth_resolution"] = list(self.depth_resolution)
        return result


def _require_sensor_prims(stage: object, sensor: dict[str, object]) -> None:
    front = sensor["front_stereo"]
    required = (
        str(front["left_camera_prim"]),
        str(front["right_camera_prim"]),
        str(front["imu_prim"]),
    )
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"front Hawk sensor prims are missing: {missing}")


def _configure_sensor_rates(stage: object, sensor: dict[str, object]) -> None:
    front = sensor["front_stereo"]
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for key in ("left_camera_prim", "right_camera_prim"):
            prim = stage.GetPrimAtPath(str(front[key]))
            attribute = prim.GetAttribute("omni:sensor:tickRate")
            if not attribute.IsValid():
                raise RuntimeError(f"camera lacks omni:sensor:tickRate: {prim.GetPath()}")
            attribute.Set(float(front["image_rate_hz"]))
        imu = stage.GetPrimAtPath(str(front["imu_prim"]))
        period = imu.GetAttribute("sensorPeriod")
        if not period.IsValid():
            raise RuntimeError(f"IMU lacks sensorPeriod: {imu.GetPath()}")
        period.Set(1.0 / float(front["imu_rate_hz"]))


def _common_qos_nodes() -> list[tuple[str, str]]:
    return [
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("SensorQos", "isaacsim.ros2.bridge.ROS2QoSProfile"),
    ]


def _common_qos_values() -> list[tuple[str, object]]:
    return [
        ("SensorQos.inputs:createProfile", "Default for publishers/subscribers"),
        ("SensorQos.inputs:reliability", "bestEffort"),
        ("SensorQos.inputs:depth", 5),
    ]


def _create_stereo_graph(sensor: dict[str, object]) -> str:
    path = f"{GRAPH_ROOT}/FrontStereo"
    keys = og.Controller.Keys
    front = sensor["front_stereo"]
    topics = sensor["topics"]
    frames = sensor["frames"]
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *_common_qos_nodes(),
                ("CreateLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishLeftRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishRightRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishStereoInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                *_common_qos_values(),
                ("CreateLeft.inputs:cameraPrim", [usdrt.Sdf.Path(str(front["left_camera_prim"]))]),
                ("CreateLeft.inputs:width", int(front["image_width"])),
                ("CreateLeft.inputs:height", int(front["image_height"])),
                (
                    "CreateRight.inputs:cameraPrim",
                    [usdrt.Sdf.Path(str(front["right_camera_prim"]))],
                ),
                ("CreateRight.inputs:width", int(front["image_width"])),
                ("CreateRight.inputs:height", int(front["image_height"])),
                ("PublishLeftRgb.inputs:topicName", str(topics["left_rgb_raw"])),
                ("PublishLeftRgb.inputs:frameId", str(frames["left_optical"])),
                ("PublishLeftRgb.inputs:type", "rgb"),
                ("PublishLeftRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishLeftRgb.inputs:frameSkipCount", 0),
                ("PublishRightRgb.inputs:topicName", str(topics["right_rgb_raw"])),
                ("PublishRightRgb.inputs:frameId", str(frames["right_optical"])),
                ("PublishRightRgb.inputs:type", "rgb"),
                ("PublishRightRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishRightRgb.inputs:frameSkipCount", 0),
                ("PublishStereoInfo.inputs:topicName", str(topics["left_camera_info"])),
                (
                    "PublishStereoInfo.inputs:topicNameRight",
                    str(topics["right_camera_info"]),
                ),
                ("PublishStereoInfo.inputs:frameId", str(frames["left_optical"])),
                ("PublishStereoInfo.inputs:frameIdRight", str(frames["right_optical"])),
                ("PublishStereoInfo.inputs:resetSimulationTimeOnStop", False),
                ("PublishStereoInfo.inputs:frameSkipCount", 0),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateLeft.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateRight.inputs:execIn"),
                ("CreateLeft.outputs:execOut", "PublishLeftRgb.inputs:execIn"),
                ("CreateRight.outputs:execOut", "PublishRightRgb.inputs:execIn"),
                ("CreateLeft.outputs:execOut", "PublishStereoInfo.inputs:execIn"),
                ("CreateLeft.outputs:renderProductPath", "PublishLeftRgb.inputs:renderProductPath"),
                (
                    "CreateRight.outputs:renderProductPath",
                    "PublishRightRgb.inputs:renderProductPath",
                ),
                (
                    "CreateLeft.outputs:renderProductPath",
                    "PublishStereoInfo.inputs:renderProductPath",
                ),
                (
                    "CreateRight.outputs:renderProductPath",
                    "PublishStereoInfo.inputs:renderProductPathRight",
                ),
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
    keys = og.Controller.Keys
    front = sensor["front_stereo"]
    topics = sensor["topics"]
    frames = sensor["frames"]
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *_common_qos_nodes(),
                ("CreateDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                *_common_qos_values(),
                (
                    "CreateDepth.inputs:cameraPrim",
                    [usdrt.Sdf.Path(str(front["left_camera_prim"]))],
                ),
                ("CreateDepth.inputs:width", int(front["depth_width"])),
                ("CreateDepth.inputs:height", int(front["depth_height"])),
                ("PublishDepth.inputs:topicName", str(topics["depth"])),
                ("PublishDepth.inputs:frameId", str(frames["left_optical"])),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:resetSimulationTimeOnStop", False),
                ("PublishDepth.inputs:frameSkipCount", 0),
                ("PublishDepthInfo.inputs:topicName", str(topics["depth_camera_info"])),
                ("PublishDepthInfo.inputs:frameId", str(frames["left_optical"])),
                ("PublishDepthInfo.inputs:resetSimulationTimeOnStop", False),
                ("PublishDepthInfo.inputs:frameSkipCount", 0),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateDepth.inputs:execIn"),
                ("CreateDepth.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("CreateDepth.outputs:execOut", "PublishDepthInfo.inputs:execIn"),
                (
                    "CreateDepth.outputs:renderProductPath",
                    "PublishDepth.inputs:renderProductPath",
                ),
                (
                    "CreateDepth.outputs:renderProductPath",
                    "PublishDepthInfo.inputs:renderProductPath",
                ),
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
    keys = og.Controller.Keys
    front = sensor["front_stereo"]
    topics = sensor["topics"]
    frames = sensor["frames"]
    og.Controller.edit(
        {
            "graph_path": path,
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                *_common_qos_nodes(),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                *_common_qos_values(),
                ("SimulationTime.inputs:resetOnStop", False),
                ("ReadImu.inputs:imuPrim", [usdrt.Sdf.Path(str(front["imu_prim"]))]),
                ("ReadImu.inputs:readGravity", True),
                ("ReadImu.inputs:useLatestData", False),
                ("PublishImu.inputs:topicName", str(topics["imu"])),
                ("PublishImu.inputs:frameId", str(frames["imu"])),
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


def create_sensor_graphs(stage: object, sensor: dict[str, object]) -> SensorGraphSummary:
    """Author the front sensor rate overrides and project graphs in the session layer."""
    _require_sensor_prims(stage, sensor)
    _configure_sensor_rates(stage, sensor)
    front = sensor["front_stereo"]
    graph_paths = (
        _create_stereo_graph(sensor),
        _create_depth_graph(sensor),
        _create_imu_graph(sensor),
    )
    return SensorGraphSummary(
        graph_paths=graph_paths,
        left_camera_prim=str(front["left_camera_prim"]),
        right_camera_prim=str(front["right_camera_prim"]),
        imu_prim=str(front["imu_prim"]),
        stereo_resolution=(int(front["image_width"]), int(front["image_height"])),
        depth_resolution=(int(front["depth_width"]), int(front["depth_height"])),
        image_rate_hz=float(front["image_rate_hz"]),
        imu_rate_hz=float(front["imu_rate_hz"]),
        topics={key: str(value) for key, value in sensor["topics"].items()},
        frames={key: str(value) for key, value in sensor["frames"].items()},
    )
