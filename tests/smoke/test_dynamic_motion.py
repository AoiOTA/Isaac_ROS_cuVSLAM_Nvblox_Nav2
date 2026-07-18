import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "isaac_sim"))

from nova_carter_sim.dynamic_motion import (  # noqa: E402
    farthest_candidate_index,
    move_towards,
    planar_distance,
    should_yield_to_robot,
)


def test_kinematic_obstacle_yields_before_robot_envelope() -> None:
    robot = (0.0, 0.0, 0.0)
    assert should_yield_to_robot(robot, (0.99, 0.0, 0.0), 1.0) is True
    assert should_yield_to_robot(robot, (1.01, 0.0, 0.0), 1.0) is False


def test_yield_distance_must_be_positive() -> None:
    with pytest.raises(ValueError):
        should_yield_to_robot((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.0)


def test_retreat_selects_route_candidate_with_greatest_clearance() -> None:
    robot = (0.5, 0.0, 0.0)
    candidates = [(0.9, 0.60, 0.0), (0.9, 0.65, 0.0), (0.9, 0.55, 0.0)]
    assert farthest_candidate_index(robot, candidates) == 1
    assert planar_distance(robot, candidates[1]) > planar_distance(
        robot, candidates[0]
    )


def test_retreat_requires_at_least_one_candidate() -> None:
    with pytest.raises(ValueError):
        farthest_candidate_index((0.0, 0.0, 0.0), [])


def test_move_towards_is_speed_bounded_and_does_not_overshoot() -> None:
    assert move_towards((0.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.5) == (
        0.0,
        0.5,
        0.0,
    )
    assert move_towards((0.0, 1.9, 0.0), (0.0, 2.0, 0.0), 0.5) == (
        0.0,
        2.0,
        0.0,
    )
    with pytest.raises(ValueError):
        move_towards((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -0.1)
