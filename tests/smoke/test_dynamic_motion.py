import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "isaac_sim"))

from nova_carter_sim.dynamic_motion import (  # noqa: E402
    clearance_preserving_step,
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


def test_clearance_step_rejects_a_refuge_segment_through_the_robot() -> None:
    robot = (2.0, 0.0, 0.0)
    current = (1.2, 0.0, 0.0)
    # The nominal refuge at x=3 is beyond the robot and therefore unsafe from
    # this side. The route start at x=-2 gives a clearance-increasing step.
    selected = clearance_preserving_step(
        robot, current, [(3.0, 0.0, 0.0), (-2.0, 0.0, 0.0)], 0.1
    )
    assert selected == pytest.approx((1.1, 0.0, 0.0))
    assert planar_distance(robot, selected) >= planar_distance(robot, current)


def test_clearance_step_can_hold_when_every_target_is_worse() -> None:
    robot = (0.0, 0.0, 0.0)
    current = (2.0, 0.0, 0.0)
    assert clearance_preserving_step(
        robot, current, [(1.0, 0.0, 0.0)], 0.25
    ) == current
    with pytest.raises(ValueError):
        clearance_preserving_step(robot, current, [], 0.1)
