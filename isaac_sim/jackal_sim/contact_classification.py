"""Pure contact classification helpers shared by runtime and smoke tests."""

from __future__ import annotations


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
