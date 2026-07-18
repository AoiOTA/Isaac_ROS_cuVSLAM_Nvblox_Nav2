import math
from pathlib import Path

import pytest

from jackal_experiments.mapping_coverage_driver import (
    ControllerConfig,
    Pose2D,
    RoutePoint,
    load_config,
    local_pose,
    point_to_segment_distance,
    steering_command,
    wrap_angle,
)


ROOT = Path(__file__).resolve().parents[4]


def controller() -> ControllerConfig:
    return ControllerConfig(
        publish_rate_hz=20.0,
        maximum_linear_speed_mps=0.28,
        minimum_linear_speed_mps=0.08,
        maximum_angular_speed_radps=0.60,
        minimum_angular_speed_radps=0.12,
        heading_gain=1.8,
        rotate_in_place_threshold_rad=0.45,
        waypoint_tolerance_m=0.10,
        maximum_cross_track_error_m=0.30,
        progress_epsilon_m=0.02,
        progress_timeout_sim_s=25.0,
        visual_tracking_timeout_sim_s=5.0,
        settle_sim_s=0.5,
        maximum_sim_duration_s=600.0,
        maximum_wall_duration_s=1200.0,
    )


def test_local_pose_uses_the_initial_robot_heading_as_map_x() -> None:
    origin = Pose2D(2.9, -0.2, math.pi)
    absolute = Pose2D(2.4, -1.2, -math.pi / 2.0)
    relative = local_pose(origin, absolute)
    assert relative.x == pytest.approx(0.5)
    assert relative.y == pytest.approx(1.0)
    assert relative.yaw == pytest.approx(math.pi / 2.0)


def test_steering_never_commands_reverse_and_rotates_for_large_error() -> None:
    target = RoutePoint(0.0, 1.0)
    linear, angular, distance, heading = steering_command(
        Pose2D(0.0, 0.0, 0.0), target, controller()
    )
    assert linear == 0.0
    assert angular > 0.0
    assert distance == pytest.approx(1.0)
    assert heading == pytest.approx(math.pi / 2.0)


def test_segment_distance_detects_lateral_route_departure() -> None:
    assert point_to_segment_distance((0.5, 0.2), (0.0, 0.0), (1.0, 0.0)) == pytest.approx(0.2)
    assert point_to_segment_distance((-0.2, 0.0), (0.0, 0.0), (1.0, 0.0)) == pytest.approx(0.2)


def test_committed_coverage_route_is_closed_bounded_and_planning_only() -> None:
    config, route, reference = load_config(ROOT / "config/mapping_coverage.yaml")
    assert len(route) == 23
    assert math.dist((route[0].x, route[0].y), (route[-1].x, route[-1].y)) <= 0.15
    assert config.maximum_linear_speed_mps == 0.28
    assert reference["use"] == "planning_only"
    assert reference["environment_sha256"] == (
        "f76f1957e8f4cbfccb13f9670d6ce187793007c56d9b3aad8a8ee72507d658d2"
    )


def test_wrap_angle_is_continuous_across_pi() -> None:
    assert wrap_angle(-math.pi + 0.1 - (math.pi - 0.1)) == pytest.approx(0.2)
