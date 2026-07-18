"""Pure contact classification helpers shared by runtime and smoke tests."""

from __future__ import annotations

import math


def is_nonimpact_proximity_contact(
    records: list[object],
    *,
    maximum_impulse_norm: float = 1.0e-9,
) -> bool:
    """Identify PhysX contact-offset reports with no touch or applied impulse.

    PhysX can emit a contact report while two shapes are still separated by a
    positive contact offset.  Such a speculative/proximity record is not a
    physical collision.  A negative separation or any measurable impulse keeps
    the record in the collision/support classifier.
    """

    if not records:
        return False
    for record in records:
        if float(record.separation) < 0.0:
            return False
        impulse_norm = math.sqrt(
            sum(float(component) ** 2 for component in record.impulse)
        )
        if impulse_norm > maximum_impulse_norm:
            return False
    return True


def is_wheel_support_contact(
    robot_path: str,
    records: list[object],
    support_surface_z: float,
    *,
    maximum_height_above_support_m: float = 0.03,
    minimum_absolute_normal_z: float = 0.80,
) -> bool:
    """Identify floor-coplanar wheel support without hiding wall impacts."""

    lowered = robot_path.lower()
    if not any(token in lowered for token in ("wheel", "caster")) or not records:
        return False
    maximum_z = support_surface_z + maximum_height_above_support_m
    for record in records:
        position = record.position
        normal = record.normal
        if float(position[2]) > maximum_z:
            return False
        if abs(float(normal[2])) < minimum_absolute_normal_z:
            return False
    return True
