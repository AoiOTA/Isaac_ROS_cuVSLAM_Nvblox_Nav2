"""Pure helpers for collision-aware kinematic obstacle motion."""

from __future__ import annotations

import math


def should_yield_to_robot(
    robot_position: tuple[float, float, float],
    candidate_position: tuple[float, float, float],
    yield_distance_m: float,
) -> bool:
    """Return true before a kinematic actor enters the robot safety envelope."""

    if yield_distance_m <= 0.0 or not math.isfinite(yield_distance_m):
        raise ValueError("yield distance must be finite and positive")
    return math.hypot(
        candidate_position[0] - robot_position[0],
        candidate_position[1] - robot_position[1],
    ) < yield_distance_m


def planar_distance(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> float:
    """Return the navigation-plane distance between two stage positions."""

    return math.hypot(first[0] - second[0], first[1] - second[1])


def farthest_candidate_index(
    robot_position: tuple[float, float, float],
    candidates: list[tuple[float, float, float]],
) -> int:
    """Select the route candidate that most quickly clears the robot.

    Keeping this decision independent of USD/PhysX makes the dynamic-actor
    safety behavior deterministic and directly unit-testable.
    """

    if not candidates:
        raise ValueError("at least one retreat candidate is required")
    return max(
        range(len(candidates)),
        key=lambda index: planar_distance(robot_position, candidates[index]),
    )


def move_towards(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    maximum_distance: float,
) -> tuple[float, float, float]:
    """Move at most ``maximum_distance`` towards a 3-D target."""

    if maximum_distance < 0.0 or not math.isfinite(maximum_distance):
        raise ValueError("maximum distance must be finite and non-negative")
    distance = math.dist(current, target)
    if distance <= maximum_distance or distance <= 1.0e-12:
        return target
    scale = maximum_distance / distance
    return tuple(
        current[index] + scale * (target[index] - current[index])
        for index in range(3)
    )


def clearance_preserving_step(
    robot_position: tuple[float, float, float],
    current_position: tuple[float, float, float],
    targets: list[tuple[float, float, float]],
    maximum_distance: float,
) -> tuple[float, float, float]:
    """Take a bounded retreat step that never reduces robot clearance.

    A fixed refuge can lie on the opposite side of the robot after visual-map
    and simulator-world frames are aligned. Moving blindly toward that refuge
    would make a kinematic actor ram a correctly stopped robot. Keeping the
    current position as a candidate makes the returned center distance
    monotonically non-decreasing, while route endpoints provide two escape
    directions if the direct refuge segment is temporarily unsafe.
    """

    if not targets:
        raise ValueError("at least one clearance target is required")
    candidates = [current_position]
    candidates.extend(
        move_towards(current_position, target, maximum_distance)
        for target in targets
    )
    return candidates[farthest_candidate_index(robot_position, candidates)]
