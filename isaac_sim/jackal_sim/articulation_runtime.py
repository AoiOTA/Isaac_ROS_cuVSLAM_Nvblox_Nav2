"""Thin Jackal runtime adapter over Isaac Sim 6.0 experimental Articulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


class ArticulationRuntimeError(RuntimeError):
    """Raised when the Jackal articulation or its physics config is invalid."""


@dataclass(frozen=True)
class ArticulationPhysicsConfig:
    solver_position_iterations: int
    solver_velocity_iterations: int
    sleep_threshold: float
    stabilization_threshold: float
    wheel_static_friction_effort: float
    wheel_dynamic_friction_effort: float
    wheel_viscous_friction_coefficient: float
    idle_brake_command_timeout_sec: float
    idle_brake_command_deadband: float
    motion_assist_enabled: bool
    motion_assist_command_timeout_sec: float
    motion_assist_max_linear_acceleration: float
    motion_assist_max_angular_acceleration: float


def articulation_physics_config_from_mapping(
    control: Mapping[str, object],
) -> ArticulationPhysicsConfig:
    """Validate runtime-only articulation settings from ``control.yaml``."""

    physics = control.get("physics")
    if not isinstance(physics, Mapping):
        raise ArticulationRuntimeError("control.physics must be a mapping")

    def positive_integer(name: str) -> int:
        value = physics.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ArticulationRuntimeError(
                f"control.physics.{name} must be an integer in [1, 255]"
            )
        return value

    def finite_number(name: str, *, positive: bool = False) -> float:
        value = physics.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ArticulationRuntimeError(
                f"control.physics.{name} must be a finite number"
            )
        result = float(value)
        if not math.isfinite(result) or (positive and result <= 0.0):
            qualifier = "finite and positive" if positive else "finite"
            raise ArticulationRuntimeError(
                f"control.physics.{name} must be {qualifier}"
            )
        return result

    enabled = physics.get("motion_assist_enabled")
    if not isinstance(enabled, bool):
        raise ArticulationRuntimeError(
            "control.physics.motion_assist_enabled must be boolean"
        )

    static_friction = finite_number("wheel_static_friction_effort")
    dynamic_friction = finite_number("wheel_dynamic_friction_effort")
    viscous_friction = finite_number("wheel_viscous_friction_coefficient")
    if min(static_friction, dynamic_friction, viscous_friction) < 0.0:
        raise ArticulationRuntimeError(
            "control.physics wheel joint friction values must be non-negative"
        )
    if static_friction < dynamic_friction:
        raise ArticulationRuntimeError(
            "wheel static friction effort must be at least dynamic friction effort"
        )

    return ArticulationPhysicsConfig(
        solver_position_iterations=positive_integer("solver_position_iterations"),
        solver_velocity_iterations=positive_integer("solver_velocity_iterations"),
        sleep_threshold=finite_number("sleep_threshold", positive=True),
        stabilization_threshold=finite_number(
            "stabilization_threshold", positive=True
        ),
        wheel_static_friction_effort=static_friction,
        wheel_dynamic_friction_effort=dynamic_friction,
        wheel_viscous_friction_coefficient=viscous_friction,
        idle_brake_command_timeout_sec=finite_number(
            "idle_brake_command_timeout_sec", positive=True
        ),
        idle_brake_command_deadband=finite_number(
            "idle_brake_command_deadband", positive=True
        ),
        motion_assist_enabled=enabled,
        motion_assist_command_timeout_sec=finite_number(
            "motion_assist_command_timeout_sec", positive=True
        ),
        motion_assist_max_linear_acceleration=finite_number(
            "motion_assist_max_linear_acceleration_mps2", positive=True
        ),
        motion_assist_max_angular_acceleration=finite_number(
            "motion_assist_max_angular_acceleration_radps2", positive=True
        ),
    )


class ArticulationRuntime:
    def __init__(self, prim_path: str, base_link_prim_path: str, app):
        self.prim_path = prim_path
        self.base_link_prim_path = base_link_prim_path
        self.app = app
        self._articulation = None

    def initialize(self) -> None:
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager

        self._articulation = Articulation(self.prim_path)
        self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            SimulationManager.initialize_physics()
            self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            self._articulation = Articulation(self.prim_path)
            self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            raise ArticulationRuntimeError(
                f"physics articulation view is invalid for {self.prim_path}"
            )

    @property
    def articulation(self):
        if self._articulation is None:
            raise ArticulationRuntimeError("articulation is not initialized")
        return self._articulation

    @property
    def num_dof(self) -> int:
        return int(self.articulation.num_dofs)

    def get_dof_names(self) -> tuple[str, ...]:
        return tuple(self.articulation.dof_names)

    def configure_stability(self, settings: ArticulationPhysicsConfig) -> None:
        self.articulation.set_solver_iteration_counts(
            [settings.solver_position_iterations],
            [settings.solver_velocity_iterations],
        )
        self.articulation.set_sleep_thresholds([settings.sleep_threshold])
        self.articulation.set_stabilization_thresholds(
            [settings.stabilization_threshold]
        )
        self.articulation.set_dof_friction_properties(
            static_frictions=[settings.wheel_static_friction_effort],
            dynamic_frictions=[settings.wheel_dynamic_friction_effort],
            viscous_frictions=[settings.wheel_viscous_friction_coefficient],
        )

    def get_world_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        positions, orientations = self.articulation.get_world_poses()
        position = tuple(float(value) for value in positions.numpy()[0])
        orientation = tuple(float(value) for value in orientations.numpy()[0])
        return position, orientation  # type: ignore[return-value]

    def set_base_velocities(
        self, linear: Sequence[float], angular: Sequence[float]
    ) -> None:
        self.articulation.set_velocities(
            linear_velocities=[list(linear)], angular_velocities=[list(angular)]
        )

    def get_base_velocities(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        linear, angular = self.articulation.get_velocities()
        return (
            tuple(float(value) for value in linear.numpy()[0]),
            tuple(float(value) for value in angular.numpy()[0]),
        )

    def set_joint_velocities(self, values: Sequence[float]) -> None:
        if len(values) != self.num_dof:
            raise ArticulationRuntimeError(
                f"expected {self.num_dof} joint velocities, got {len(values)}"
            )
        self.articulation.set_dof_velocities([list(values)])

    def set_joint_velocity_targets(self, values: Sequence[float]) -> None:
        if len(values) != self.num_dof:
            raise ArticulationRuntimeError(
                f"expected {self.num_dof} joint targets, got {len(values)}"
            )
        self.articulation.set_dof_velocity_targets([list(values)])

    def zero_all_velocities(self) -> None:
        zeros = [0.0] * self.num_dof
        self.set_base_velocities([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.set_joint_velocities(zeros)
        self.set_joint_velocity_targets(zeros)

    def _physx_body_handle(self):
        import omni.usd
        from omni.physx.bindings._physx import acquire_physx_simulation_interface
        from pxr.PhysicsSchemaTools import sdfPathToInt

        stage_id = omni.usd.get_context().get_stage_id()
        return (
            acquire_physx_simulation_interface(),
            stage_id,
            sdfPathToInt(self.base_link_prim_path),
        )

    def put_to_sleep(self) -> None:
        interface, stage_id, body_path = self._physx_body_handle()
        interface.put_to_sleep(stage_id, body_path)

    def wake_up(self) -> None:
        interface, stage_id, body_path = self._physx_body_handle()
        interface.wake_up(stage_id, body_path)
