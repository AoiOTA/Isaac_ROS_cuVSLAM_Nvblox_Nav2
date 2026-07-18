from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "isaac_sim"))

from jackal_sim.articulation_runtime import (  # noqa: E402
    ArticulationRuntimeError,
    articulation_physics_config_from_mapping,
)
from jackal_sim.idle_brake import IdleBrake, IdleBrakeError, IdleBrakeState  # noqa: E402
from jackal_sim.skid_steer_motion_assist import (  # noqa: E402
    SkidSteerMotionAssistError,
    SkidSteerMotionAssistState,
    _yaw_command_scale,
)


def test_runtime_config_preserves_calibrated_skid_steer_contract() -> None:
    control = yaml.safe_load((ROOT / "config/control.yaml").read_text())
    settings = articulation_physics_config_from_mapping(control)
    assert control["kinematics"]["wheel_separation_m"] == 0.800
    assert control["limits"] == {
        "max_linear_speed_mps": 1.0,
        "max_angular_speed_radps": 1.5,
        "max_wheel_speed_radps": 15.0,
        "max_linear_acceleration_mps2": 2.0,
        "max_linear_deceleration_mps2": 3.0,
        "max_angular_acceleration_radps2": 6.0,
        "command_timeout_s": 0.25,
    }
    assert settings.solver_position_iterations == 32
    assert settings.solver_velocity_iterations == 4
    assert settings.motion_assist_enabled is True
    assert settings.motion_assist_command_timeout_sec == 0.25
    assert settings.idle_brake_command_timeout_sec == 0.25


def test_runtime_defaults_match_reference_and_headless_skips_editor_viewport() -> None:
    simulation = yaml.safe_load((ROOT / "config/simulation.yaml").read_text())
    assert simulation["runtime"]["physics_hz"] == 60
    assert simulation["runtime"]["update_hz"] == 60

    entrypoint = (ROOT / "isaac_sim/navigation_sim.py").read_text()
    assert '"disable_viewport_updates": args.headless' in entrypoint
    assert '"headless_viewport_updates_disabled": bool(args.headless)' in entrypoint


def test_runtime_overlay_and_graph_preserve_reference_physics_fixes() -> None:
    stage = (ROOT / "isaac_sim/jackal_sim/stage.py").read_text()
    graph = (ROOT / "isaac_sim/jackal_sim/graphs.py").read_text()
    for token in (
        "old_collision.SetActive(False)",
        "UsdGeom.Cylinder.Define",
        "collider.CreateExtentAttr()",
        'physics["wheel_static_friction"]',
        'physics["wheel_drive_max_force"]',
        'physics["wheel_drive_max_velocity_deg_s"]',
    ):
        assert token in stage
    for token in (
        "OnPhysicsStep.outputs:deltaSimulationTime",
        "DifferentialController.inputs:dt",
        "DifferentialController.inputs:maxWheelSpeed",
        "FourWheelCommand.outputs:array",
        "GRAPH_PIPELINE_STAGE_ONDEMAND",
    ):
        assert token in graph
    assert graph.count('("ArticulationController",') == 1
    differential_drive = graph.split(
        "def _create_differential_drive_graph", 1
    )[1].split("def _create_joint_state_graph", 1)[0]
    assert '"evaluator_name": "execution"' not in differential_drive


def test_runtime_config_rejects_invalid_joint_friction() -> None:
    control = yaml.safe_load((ROOT / "config/control.yaml").read_text())
    control["physics"]["wheel_dynamic_friction_effort"] = 0.1
    with pytest.raises(ArticulationRuntimeError, match="static friction"):
        articulation_physics_config_from_mapping(control)


def test_motion_assist_tracks_reverse_and_turning_with_bounded_acceleration() -> None:
    state = SkidSteerMotionAssistState(
        command_timeout_sec=0.25,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    state.observe(-0.4, 1.2, 10.0)
    assert state.next_command(0.0, 0.0, 10.0, 0.1) == pytest.approx((-0.2, 0.6))
    assert state.next_command(-0.2, 0.6, 10.1, 0.1) == pytest.approx((-0.4, 1.2))


def test_motion_assist_rejects_stale_regressed_and_invalid_commands() -> None:
    state = SkidSteerMotionAssistState(
        command_timeout_sec=0.25,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    assert state.next_command(0.2, 0.2, 1.0, 0.1) is None
    state.observe(0.4, 0.8, 2.0)
    assert state.next_command(0.2, 0.2, 1.9, 0.1) is None
    assert state.next_command(0.2, 0.2, 2.3, 0.1) is None
    with pytest.raises(SkidSteerMotionAssistError):
        state.observe(float("nan"), 0.0, 1.0)
    with pytest.raises(SkidSteerMotionAssistError):
        state.next_command(0.0, 0.0, 1.0, 0.0)


def test_in_place_yaw_scale_blends_smoothly_into_arc_tracking() -> None:
    assert _yaw_command_scale(0.0) == pytest.approx(0.925)
    assert _yaw_command_scale(0.10) == pytest.approx(0.9625)
    assert _yaw_command_scale(0.20) == pytest.approx(1.0)
    assert _yaw_command_scale(-0.40) == pytest.approx(1.0)


def test_idle_brake_stops_without_a_command_and_after_timeout() -> None:
    state = IdleBrakeState(timeout_sec=0.25, command_deadband=0.001)
    assert state.should_brake(1.0)
    state.observe(0.2, 0.0, 1.0)
    assert not state.should_brake(1.20)
    assert state.should_brake(1.26)
    state.observe(0.0, 0.0, 2.0)
    assert state.should_brake(2.0)
    with pytest.raises(IdleBrakeError, match="finite"):
        state.observe(float("nan"), 0.0, 2.0)


def test_idle_brake_reset_synchronizes_sleep_and_next_command_wakes() -> None:
    calls: list[str] = []

    class Robot:
        def zero_all_velocities(self) -> None:
            calls.append("zero")

        def put_to_sleep(self) -> None:
            calls.append("sleep")

        def wake_up(self) -> None:
            calls.append("wake")

    now = {"value": 10.0}
    brake = IdleBrake.__new__(IdleBrake)
    brake.robot = Robot()
    brake.clock = lambda: now["value"]
    brake.state = IdleBrakeState(timeout_sec=0.25, command_deadband=0.001)
    brake._braking = False

    brake.reset()
    assert calls == ["zero", "sleep"]
    brake.state.observe(0.2, 0.0, now["value"])
    assert brake.update() is False
    assert calls[-1] == "wake"
