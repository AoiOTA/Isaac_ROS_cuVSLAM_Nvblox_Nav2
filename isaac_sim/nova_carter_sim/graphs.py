"""Runtime-only ROS 2 control and state OmniGraphs for Nova Carter."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import omni.graph.core as og
import usdrt.Sdf


GRAPH_ROOT = "/World/Graphs"
ARTICULATION_ROOT = "/World/NovaCarter/chassis_link"
LEFT_WHEEL_JOINT_PATH = "/World/NovaCarter/joint_wheel_left"
RIGHT_WHEEL_JOINT_PATH = "/World/NovaCarter/joint_wheel_right"


@dataclass(frozen=True)
class ControlGraphSummary:
    graph_root: str
    graph_paths: tuple[str, ...]
    command_topic: str
    joint_state_topic: str
    ground_truth_topic: str
    articulation_root: str
    commanded_joints: tuple[str, str]
    wheel_radius_m: float
    wheel_separation_m: float

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["graph_paths"] = list(self.graph_paths)
        result["commanded_joints"] = list(self.commanded_joints)
        return result


def _require_robot_prims(stage: object) -> None:
    required = (ARTICULATION_ROOT, LEFT_WHEEL_JOINT_PATH, RIGHT_WHEEL_JOINT_PATH)
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Nova Carter control prims are missing: {missing}")


def _create_clock_graph(topic: str) -> str:
    path = f"{GRAPH_ROOT}/Clock"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("SimulationTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", topic),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("SimulationTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    return path


def _create_differential_drive_graph(control: dict[str, object]) -> str:
    path = f"{GRAPH_ROOT}/DifferentialDrive"
    keys = og.Controller.Keys
    kinematics = control["kinematics"]
    limits = control["limits"]
    topics = control["topics"]
    left_joint = str(kinematics["left_wheel_joint"])
    right_joint = str(kinematics["right_wheel_joint"])
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", str(topics["command_to_sim"])),
                (
                    "DifferentialController.inputs:wheelRadius",
                    float(kinematics["wheel_radius_m"]),
                ),
                (
                    "DifferentialController.inputs:wheelDistance",
                    float(kinematics["wheel_separation_m"]),
                ),
                (
                    "DifferentialController.inputs:maxLinearSpeed",
                    float(limits["max_linear_speed_mps"]),
                ),
                (
                    "DifferentialController.inputs:maxAngularSpeed",
                    float(limits["max_angular_speed_radps"]),
                ),
                (
                    "DifferentialController.inputs:maxAcceleration",
                    float(limits["max_linear_acceleration_mps2"]),
                ),
                (
                    "DifferentialController.inputs:maxDeceleration",
                    float(limits["max_linear_deceleration_mps2"]),
                ),
                (
                    "DifferentialController.inputs:maxAngularAcceleration",
                    float(limits["max_angular_acceleration_radps2"]),
                ),
                ("ArticulationController.inputs:jointNames", [left_joint, right_joint]),
                (
                    "ArticulationController.inputs:targetPrim",
                    [usdrt.Sdf.Path(ARTICULATION_ROOT)],
                ),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:deltaSeconds", "DifferentialController.inputs:dt"),
                ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
            ],
        },
    )
    return path


def _create_joint_state_graph(topic: str) -> str:
    path = f"{GRAPH_ROOT}/JointState"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("SimulationTime.inputs:resetOnStop", False),
                ("PublishJointState.inputs:topicName", topic),
                (
                    "ReadJointState.inputs:prim",
                    [usdrt.Sdf.Path(ARTICULATION_ROOT)],
                ),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                (
                    "ReadJointState.outputs:jointPositions",
                    "PublishJointState.inputs:jointPositions",
                ),
                (
                    "ReadJointState.outputs:jointVelocities",
                    "PublishJointState.inputs:jointVelocities",
                ),
                (
                    "ReadJointState.outputs:jointEfforts",
                    "PublishJointState.inputs:jointEfforts",
                ),
                (
                    "ReadJointState.outputs:jointDofTypes",
                    "PublishJointState.inputs:jointDofTypes",
                ),
                (
                    "ReadJointState.outputs:stageMetersPerUnit",
                    "PublishJointState.inputs:stageMetersPerUnit",
                ),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                (
                    "SimulationTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
            ],
        },
    )
    return path


def _create_ground_truth_graph(topic: str, world_frame: str, base_frame: str) -> str:
    path = f"{GRAPH_ROOT}/GroundTruth"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.SET_VALUES: [
                ("SimulationTime.inputs:resetOnStop", False),
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [usdrt.Sdf.Path(ARTICULATION_ROOT)],
                ),
                ("PublishOdometry.inputs:topicName", topic),
                ("PublishOdometry.inputs:odomFrameId", world_frame),
                ("PublishOdometry.inputs:chassisFrameId", base_frame),
                ("PublishOdometry.inputs:publishRawVelocities", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                (
                    "SimulationTime.outputs:simulationTime",
                    "PublishOdometry.inputs:timeStamp",
                ),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
            ],
        },
    )
    return path


def create_control_graphs(stage: object, control: dict[str, object]) -> ControlGraphSummary:
    """Create project graphs in the session layer; never modify or reuse official graphs."""
    _require_robot_prims(stage)
    topics = control["topics"]
    frames = control["frames"]
    kinematics = control["kinematics"]
    graph_paths = (
        _create_clock_graph(str(topics["clock"])),
        _create_differential_drive_graph(control),
        _create_joint_state_graph(str(topics["joint_states"])),
        _create_ground_truth_graph(
            str(topics["ground_truth_odometry"]),
            str(frames["ground_truth_world"]),
            str(frames["base"]),
        ),
    )
    return ControlGraphSummary(
        graph_root=GRAPH_ROOT,
        graph_paths=graph_paths,
        command_topic=str(topics["command_to_sim"]),
        joint_state_topic=str(topics["joint_states"]),
        ground_truth_topic=str(topics["ground_truth_odometry"]),
        articulation_root=ARTICULATION_ROOT,
        commanded_joints=(
            str(kinematics["left_wheel_joint"]),
            str(kinematics["right_wheel_joint"]),
        ),
        wheel_radius_m=float(kinematics["wheel_radius_m"]),
        wheel_separation_m=float(kinematics["wheel_separation_m"]),
    )
