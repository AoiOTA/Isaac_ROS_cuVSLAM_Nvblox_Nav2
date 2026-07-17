"""Runtime-only Hawk camera, depth, and IMU OmniGraphs.

The front stereo pair remains active for cuVSLAM and nvblox.  Stage 9 adds
three 10 Hz stereo pairs whose render writers are enabled only while a
transient-local ``std_msgs/Bool`` is true.  The graphs are authored here at
runtime; no OmniGraph from the official ROS sample USD is loaded.
"""

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
    surround_enabled: bool
    surround_resolution: tuple[int, int]
    surround_rate_hz: float
    surround_enable_topic: str
    surround_camera_prims: dict[str, dict[str, str]]
    topics: dict[str, str]
    frames: dict[str, str]

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["graph_paths"] = list(self.graph_paths)
        result["stereo_resolution"] = list(self.stereo_resolution)
        result["depth_resolution"] = list(self.depth_resolution)
        result["surround_resolution"] = list(self.surround_resolution)
        return result


def _require_sensor_prims(
    stage: object, sensor: dict[str, object], include_surround: bool
) -> None:
    front = sensor["front_stereo"]
    required = (
        str(front["left_camera_prim"]),
        str(front["right_camera_prim"]),
        str(front["imu_prim"]),
    )
    if include_surround:
        surround = sensor["surround_stereo"]
        for pair in surround["cameras"].values():
            required += (
                str(pair["left_camera_prim"]),
                str(pair["right_camera_prim"]),
            )
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Nova Carter Hawk sensor prims are missing: {missing}")


def _configure_camera(prim: object, rate_hz: float, projection_name: str) -> None:
    attribute = prim.GetAttribute("omni:sensor:tickRate")
    if not attribute.IsValid():
        raise RuntimeError(f"camera lacks omni:sensor:tickRate: {prim.GetPath()}")
    attribute.Set(rate_hz)
    projection = prim.GetAttribute("cameraProjectionType")
    if not projection.IsValid():
        raise RuntimeError(f"camera lacks projection type: {prim.GetPath()}")
    projection.Set(projection_name)
    distortion = prim.GetAttribute("physicalDistortionCoefficients")
    if distortion.IsValid():
        coefficients = distortion.Get()
        distortion.Set([0.0] * len(coefficients))


def _configure_sensor_rates(
    stage: object, sensor: dict[str, object], include_surround: bool
) -> None:
    front = sensor["front_stereo"]
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for key in ("left_camera_prim", "right_camera_prim"):
            prim = stage.GetPrimAtPath(str(front[key]))
            _configure_camera(
                prim,
                float(front["image_rate_hz"]),
                str(front["navigation_projection"]),
            )
        if include_surround:
            surround = sensor["surround_stereo"]
            for pair in surround["cameras"].values():
                for key in ("left_camera_prim", "right_camera_prim"):
                    _configure_camera(
                        stage.GetPrimAtPath(str(pair[key])),
                        float(surround["image_rate_hz"]),
                        str(surround["navigation_projection"]),
                    )
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


def _common_qos_values(sensor: dict[str, object]) -> list[tuple[str, object]]:
    return [
        ("SensorQos.inputs:createProfile", "Default for publishers/subscribers"),
        (
            "SensorQos.inputs:reliability",
            str(sensor.get("qos_reliability", "bestEffort")),
        ),
        ("SensorQos.inputs:depth", 20),
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
                *_common_qos_values(sensor),
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
                *_common_qos_values(sensor),
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
                *_common_qos_values(sensor),
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


def _create_on_demand_stereo_graph(
    sensor: dict[str, object], pair_name: str, update_app: object
) -> str:
    """Create one gated 10 Hz stereo publisher and wire its dynamic Bool output."""
    path = f"{GRAPH_ROOT}/{pair_name.title()}Stereo"
    keys = og.Controller.Keys
    surround = sensor["surround_stereo"]
    pair = surround["cameras"][pair_name]
    topics = sensor["topics"]
    frames = sensor["frames"]
    topic_prefix = pair_name
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *_common_qos_nodes(),
                ("GateQos", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                ("EnableSubscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("CreateLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishLeftRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishRightRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishStereoInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                *_common_qos_values(sensor),
                ("GateQos.inputs:createProfile", "Default for publishers/subscribers"),
                ("GateQos.inputs:reliability", "reliable"),
                ("GateQos.inputs:durability", "transientLocal"),
                ("GateQos.inputs:depth", 1),
                ("CreateLeft.inputs:enabled", False),
                ("CreateRight.inputs:enabled", False),
                (
                    "CreateLeft.inputs:cameraPrim",
                    [usdrt.Sdf.Path(str(pair["left_camera_prim"]))],
                ),
                ("CreateLeft.inputs:width", int(surround["image_width"])),
                ("CreateLeft.inputs:height", int(surround["image_height"])),
                (
                    "CreateRight.inputs:cameraPrim",
                    [usdrt.Sdf.Path(str(pair["right_camera_prim"]))],
                ),
                ("CreateRight.inputs:width", int(surround["image_width"])),
                ("CreateRight.inputs:height", int(surround["image_height"])),
                ("PublishLeftRgb.inputs:enabled", False),
                ("PublishLeftRgb.inputs:topicName", str(topics[f"{topic_prefix}_left_rgb_raw"])),
                ("PublishLeftRgb.inputs:frameId", str(frames[f"{topic_prefix}_left_optical"])),
                ("PublishLeftRgb.inputs:type", "rgb"),
                ("PublishLeftRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishRightRgb.inputs:enabled", False),
                ("PublishRightRgb.inputs:topicName", str(topics[f"{topic_prefix}_right_rgb_raw"])),
                ("PublishRightRgb.inputs:frameId", str(frames[f"{topic_prefix}_right_optical"])),
                ("PublishRightRgb.inputs:type", "rgb"),
                ("PublishRightRgb.inputs:resetSimulationTimeOnStop", False),
                ("PublishStereoInfo.inputs:enabled", False),
                (
                    "PublishStereoInfo.inputs:topicName",
                    str(topics[f"{topic_prefix}_left_camera_info"]),
                ),
                (
                    "PublishStereoInfo.inputs:topicNameRight",
                    str(topics[f"{topic_prefix}_right_camera_info"]),
                ),
                ("PublishStereoInfo.inputs:frameId", str(frames[f"{topic_prefix}_left_optical"])),
                (
                    "PublishStereoInfo.inputs:frameIdRight",
                    str(frames[f"{topic_prefix}_right_optical"]),
                ),
                ("PublishStereoInfo.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "EnableSubscriber.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateLeft.inputs:execIn"),
                ("Context.outputs:context", "EnableSubscriber.inputs:context"),
                ("GateQos.outputs:qosProfile", "EnableSubscriber.inputs:qosProfile"),
                ("CreateLeft.outputs:execOut", "PublishLeftRgb.inputs:execIn"),
                ("CreateLeft.outputs:execOut", "CreateRight.inputs:execIn"),
                ("CreateRight.outputs:execOut", "PublishRightRgb.inputs:execIn"),
                ("CreateRight.outputs:execOut", "PublishStereoInfo.inputs:execIn"),
                ("CreateLeft.outputs:renderProductPath", "PublishLeftRgb.inputs:renderProductPath"),
                ("CreateRight.outputs:renderProductPath", "PublishRightRgb.inputs:renderProductPath"),
                ("CreateLeft.outputs:renderProductPath", "PublishStereoInfo.inputs:renderProductPath"),
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

    # ROS2Subscriber creates message-field outputs dynamically when its type is
    # configured.  Configure it in order, update Kit once, then wire Bool.data
    # into both render-product and writer enable inputs.
    subscriber = og.Controller.node(f"{path}/EnableSubscriber")
    for attribute, value in (
        ("inputs:topicName", str(surround["enable_topic"])),
        ("inputs:messagePackage", "std_msgs"),
        ("inputs:messageSubfolder", "msg"),
        ("inputs:messageName", "Bool"),
    ):
        og.Controller.attribute(attribute, subscriber).set(value)
    update_app()
    enabled = og.Controller.attribute("outputs:data", subscriber)
    if not enabled.is_valid() or enabled.get_type_name() != "bool":
        raise RuntimeError(f"{path} failed to create std_msgs/Bool data output")
    enabled.set(False)
    for destination in (
        "CreateLeft.inputs:enabled",
        "CreateRight.inputs:enabled",
        "PublishLeftRgb.inputs:enabled",
        "PublishRightRgb.inputs:enabled",
        "PublishStereoInfo.inputs:enabled",
    ):
        og.Controller.connect(enabled, og.Controller.attribute(f"{path}/{destination}"))
    return path


def create_sensor_graphs(
    stage: object,
    sensor: dict[str, object],
    *,
    include_surround: bool = False,
    update_app: object | None = None,
) -> SensorGraphSummary:
    """Author sensor rate overrides and project graphs in the session layer."""
    if include_surround and update_app is None:
        raise ValueError("update_app is required for dynamic surround-camera gates")
    _require_sensor_prims(stage, sensor, include_surround)
    _configure_sensor_rates(stage, sensor, include_surround)
    front = sensor["front_stereo"]
    graph_paths = [
        _create_stereo_graph(sensor),
        _create_depth_graph(sensor),
        _create_imu_graph(sensor),
    ]
    if include_surround:
        graph_paths.extend(
            _create_on_demand_stereo_graph(sensor, name, update_app)
            for name in ("left", "right", "back")
        )
    surround = sensor["surround_stereo"]
    return SensorGraphSummary(
        graph_paths=tuple(graph_paths),
        left_camera_prim=str(front["left_camera_prim"]),
        right_camera_prim=str(front["right_camera_prim"]),
        imu_prim=str(front["imu_prim"]),
        stereo_resolution=(int(front["image_width"]), int(front["image_height"])),
        depth_resolution=(int(front["depth_width"]), int(front["depth_height"])),
        image_rate_hz=float(front["image_rate_hz"]),
        imu_rate_hz=float(front["imu_rate_hz"]),
        surround_enabled=include_surround,
        surround_resolution=(
            int(surround["image_width"]),
            int(surround["image_height"]),
        ),
        surround_rate_hz=float(surround["image_rate_hz"]),
        surround_enable_topic=str(surround["enable_topic"]),
        surround_camera_prims={
            name: {key: str(value) for key, value in pair.items()}
            for name, pair in surround["cameras"].items()
        },
        topics={key: str(value) for key, value in sensor["topics"].items()},
        frames={key: str(value) for key, value in sensor["frames"].items()},
    )
