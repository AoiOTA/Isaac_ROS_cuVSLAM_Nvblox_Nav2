"""PhysX contact accounting for acceptance experiments."""

from __future__ import annotations

from collections import Counter
import math
import time

from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

from .contact_classification import (
    is_nonimpact_proximity_contact,
    is_wheel_support_contact,
)


class RobotContactMonitor:
    """Count chassis contacts with non-robot, non-floor actors.

    Wheel/caster contact with the floor is necessary locomotion and does not
    represent a navigation collision. Reporters cover every robot rigid body,
    while explicit floor/ground actor names are excluded.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        robot_root: str,
        *,
        support_surface_z: float,
    ) -> None:
        self.robot_root = robot_root
        self.support_surface_z = float(support_surface_z)
        robot = stage.GetPrimAtPath(robot_root)
        if not robot.IsValid():
            raise RuntimeError("Jackal root is missing for contact monitoring")
        chassis = stage.GetPrimAtPath(f"{robot_root}/base_link")
        if not chassis.IsValid():
            raise RuntimeError("Jackal base_link is missing for contact monitoring")
        # PhysX can restrict contact reports to explicit counterpart prims.
        # Without this relationship, every wheel/floor contact is decoded and
        # the imported Kujiale triangle mesh emits three invalid material-face
        # warnings per report.  Targeting every non-floor collider preserves
        # obstacle collision evidence without changing contact/friction physics
        # or hiding unrelated PhysX warnings.
        report_pair_paths = sorted(
            prim.GetPath()
            for prim in stage.TraverseAll()
            if prim.HasAPI(UsdPhysics.CollisionAPI)
            and not str(prim.GetPath()).startswith(robot_root)
            and not self._is_floor(str(prim.GetPath()))
        )
        if not report_pair_paths:
            raise RuntimeError("no non-floor colliders are available for contact monitoring")
        self.report_pair_paths = [str(path) for path in report_pair_paths]
        self.reporter_paths: list[str] = []
        for prim in Usd.PrimRange(robot):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            reporter = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            reporter.CreateThresholdAttr().Set(0.0)
            reporter.CreateReportPairsRel().SetTargets(report_pair_paths)
            self.reporter_paths.append(str(prim.GetPath()))
        if str(chassis.GetPath()) not in self.reporter_paths:
            reporter = PhysxSchema.PhysxContactReportAPI.Apply(chassis)
            reporter.CreateThresholdAttr().Set(0.0)
            reporter.CreateReportPairsRel().SetTargets(report_pair_paths)
            self.reporter_paths.append(str(chassis.GetPath()))
        self.started_wall = time.monotonic()
        self.events = 0
        self.filtered_floor_events = 0
        self.filtered_proximity_events = 0
        self.filtered_support_events = 0
        self.empty_contact_events = 0
        self.pairs: Counter[tuple[str, str]] = Counter()
        self.proximity_pairs: Counter[tuple[str, str]] = Counter()
        self.support_pairs: Counter[tuple[str, str]] = Counter()
        self.pair_audits: dict[tuple[str, str], dict[str, float | int]] = {}
        self.collision_samples: list[dict[str, object]] = []
        self.subscription = (
            get_physx_simulation_interface().subscribe_contact_report_events(
                self._on_contact_report
            )
        )

    @staticmethod
    def _is_floor(path: str) -> bool:
        lowered = path.lower()
        return any(token in lowered for token in ("floor", "groundplane", "ground_plane"))

    def _on_contact_report(self, headers: object, data: object) -> None:
        for header in headers:
            actor_paths = (
                str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
            )
            pair = tuple(sorted(actor_paths))
            robot_members = [path.startswith(self.robot_root) for path in pair]
            if not any(robot_members) or all(robot_members):
                continue
            other = pair[0] if not robot_members[0] else pair[1]
            robot_actor = pair[0] if robot_members[0] else pair[1]
            if self._is_floor(other):
                self.filtered_floor_events += 1
                continue
            offset = int(header.contact_data_offset)
            count = int(header.num_contact_data)
            if count <= 0:
                # CONTACT_LOST carries no collision sample. Counting it would
                # double-count a completed contact and cannot classify normal.
                self.empty_contact_events += 1
                continue
            records = [data[index] for index in range(offset, offset + count)]
            self._audit_records(pair, records)
            if is_nonimpact_proximity_contact(records):
                self.filtered_proximity_events += 1
                self.proximity_pairs[pair] += 1
                continue
            if is_wheel_support_contact(
                robot_actor,
                records,
                self.support_surface_z,
            ):
                self.filtered_support_events += 1
                self.support_pairs[pair] += 1
                continue
            self.events += 1
            self.pairs[pair] += 1
            if len(self.collision_samples) < 32:
                self.collision_samples.append(
                    {
                        "actors": list(pair),
                        "contacts": [
                            {
                                "position": [float(value) for value in record.position],
                                "normal": [float(value) for value in record.normal],
                                "impulse": [float(value) for value in record.impulse],
                                "separation": float(record.separation),
                            }
                            for record in records
                        ],
                    }
                )

    def _audit_records(
        self,
        pair: tuple[str, str],
        records: list[object],
    ) -> None:
        audit = self.pair_audits.setdefault(
            pair,
            {
                "event_count": 0,
                "record_count": 0,
                "minimum_separation_m": math.inf,
                "maximum_impulse_norm": 0.0,
                "maximum_contact_z_m": -math.inf,
                "minimum_absolute_normal_z": math.inf,
            },
        )
        audit["event_count"] = int(audit["event_count"]) + 1
        audit["record_count"] = int(audit["record_count"]) + len(records)
        for record in records:
            audit["minimum_separation_m"] = min(
                float(audit["minimum_separation_m"]),
                float(record.separation),
            )
            audit["maximum_impulse_norm"] = max(
                float(audit["maximum_impulse_norm"]),
                math.sqrt(sum(float(value) ** 2 for value in record.impulse)),
            )
            audit["maximum_contact_z_m"] = max(
                float(audit["maximum_contact_z_m"]),
                float(record.position[2]),
            )
            audit["minimum_absolute_normal_z"] = min(
                float(audit["minimum_absolute_normal_z"]),
                abs(float(record.normal[2])),
            )

    def summary(self) -> dict[str, object]:
        return {
            "collision_event_count": self.events,
            "filtered_floor_event_count": self.filtered_floor_events,
            "filtered_nonimpact_proximity_event_count": self.filtered_proximity_events,
            "filtered_support_event_count": self.filtered_support_events,
            "empty_contact_event_count": self.empty_contact_events,
            "reporter_count": len(self.reporter_paths),
            "reporter_paths": sorted(self.reporter_paths),
            "report_pair_filter": "all_non_floor_collision_prims",
            "report_pair_count": len(self.report_pair_paths),
            "contact_pairs": [
                {"actors": list(pair), "count": count}
                for pair, count in sorted(self.pairs.items())
            ],
            "nonimpact_proximity_filter": {
                "minimum_separation_m": 0.0,
                "maximum_impulse_norm": 1.0e-9,
            },
            "nonimpact_proximity_pairs": [
                {"actors": list(pair), "count": count}
                for pair, count in sorted(self.proximity_pairs.items())
            ],
            "support_contact_filter": {
                "wheel_or_caster_only": True,
                "support_surface_z_m": self.support_surface_z,
                "maximum_height_above_support_m": 0.03,
                "minimum_absolute_normal_z": 0.80,
            },
            "support_contact_pairs": [
                {"actors": list(pair), "count": count}
                for pair, count in sorted(self.support_pairs.items())
            ],
            "contact_pair_audits": [
                {
                    "actors": list(pair),
                    **{
                        key: value
                        for key, value in audit.items()
                        if not isinstance(value, float) or math.isfinite(value)
                    },
                }
                for pair, audit in sorted(self.pair_audits.items())
            ],
            "collision_samples": self.collision_samples,
            "monitor_wall_seconds": time.monotonic() - self.started_wall,
        }

    def close(self) -> None:
        self.subscription = None
