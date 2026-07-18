"""Runtime-only ROS 2 control and state OmniGraphs for Jackal."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import omni.graph.core as og
import usdrt.Sdf


GRAPH_ROOT = "/World/Graphs"
ARTICULATION_ROOT = "/World/Jackal"
BASE_LINK_PRIM = "/World/Jackal/base_link"
WHEEL_JOINT_PATHS = (
    "/World/Jackal/front_left_wheel_joint",
    "/World/Jackal/front_right_wheel_joint",
    "/World/Jackal/rear_left_wheel_joint",
    "/World/Jackal/rear_right_wheel_joint",
)


@dataclass(frozen=True)
class ControlGraphSummary:
    graph_root: str
    graph_paths: tuple[str, ...]
    command_topic: str
    joint_state_topic: str
    ground_truth_topic: str
    articulation_root: str
    commanded_joints: tuple[str, str, str, str]
    wheel_radius_m: float
    wheel_separation_m: float

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["graph_paths"] = list(self.graph_paths)
        result["commanded_joints"] = list(self.commanded_joints)
        return result


def _require_robot_prims(stage: object) -> None:
    required = (ARTICULATION_ROOT, BASE_LINK_PRIM, *WHEEL_JOINT_PATHS)
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Jackal control prims are missing: {missing}")


def _create_clock_graph(topic: str) -> str:
    path = f"{GRAPH_ROOT}/Clock"
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": path,
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("SimulationTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", topic),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
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
    left_joints = [str(value) for value in kinematics["left_wheel_joints"]]
    right_joints = [str(value) for value in kinematics["right_wheel_joints"]]
    if len(left_joints) != 2 or len(right_joints) != 2:
        raise RuntimeError("Jackal control requires two left and two right wheel joints")
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                ("LeftWheelCommand", "omni.graph.nodes.ArrayIndex"),
                ("RightWheelCommand", "omni.graph.nodes.ArrayIndex"),
                ("FourWheelCommand", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CREATE_ATTRIBUTES: [
                ("FourWheelCommand.inputs:input1", "double"),
                ("FourWheelCommand.inputs:input2", "double"),
                ("FourWheelCommand.inputs:input3", "double"),
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
                    "DifferentialController.inputs:maxWheelSpeed",
                    float(limits["max_wheel_speed_radps"]),
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
                ("LeftWheelCommand.inputs:index", 0),
                ("RightWheelCommand.inputs:index", 1),
                ("FourWheelCommand.inputs:arrayType", "double[]"),
                ("FourWheelCommand.inputs:arraySize", 4),
                (
                    "ArticulationController.inputs:jointNames",
                    [left_joints[0], right_joints[0], left_joints[1], right_joints[1]],
                ),
                (
                    "ArticulationController.inputs:targetPrim",
                    [usdrt.Sdf.Path(ARTICULATION_ROOT)],
                ),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("OnPhysicsStep.outputs:step", "DifferentialController.inputs:execIn"),
                (
                    "OnPhysicsStep.outputs:deltaSimulationTime",
                    "DifferentialController.inputs:dt",
                ),
                ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "LeftWheelCommand.inputs:array",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "RightWheelCommand.inputs:array",
                ),
                ("LeftWheelCommand.outputs:value", "FourWheelCommand.inputs:input0"),
                ("RightWheelCommand.outputs:value", "FourWheelCommand.inputs:input1"),
                ("LeftWheelCommand.outputs:value", "FourWheelCommand.inputs:input2"),
                ("RightWheelCommand.outputs:value", "FourWheelCommand.inputs:input3"),
                ("OnPhysicsStep.outputs:step", "ArticulationController.inputs:execIn"),
                (
                    "FourWheelCommand.outputs:array",
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
                    [usdrt.Sdf.Path(BASE_LINK_PRIM)],
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
            str(kinematics["left_wheel_joints"][0]),
            str(kinematics["right_wheel_joints"][0]),
            str(kinematics["left_wheel_joints"][1]),
            str(kinematics["right_wheel_joints"][1]),
        ),
        wheel_radius_m=float(kinematics["wheel_radius_m"]),
        wheel_separation_m=float(kinematics["wheel_separation_m"]),
    )
