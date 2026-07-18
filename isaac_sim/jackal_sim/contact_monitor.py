"""PhysX contact accounting for acceptance experiments."""

from __future__ import annotations

from collections import Counter
import time

from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics


class RobotContactMonitor:
    """Count chassis contacts with non-robot, non-floor actors.

    Wheel/caster contact with the floor is necessary locomotion and does not
    represent a navigation collision. Reporters cover every robot rigid body,
    while explicit floor/ground actor names are excluded.
    """

    def __init__(self, stage: Usd.Stage, robot_root: str) -> None:
        self.robot_root = robot_root
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
        self.pairs: Counter[tuple[str, str]] = Counter()
        self.subscription = (
            get_physx_simulation_interface().subscribe_contact_report_events(
                self._on_contact_report
            )
        )

    @staticmethod
    def _is_floor(path: str) -> bool:
        lowered = path.lower()
        return any(token in lowered for token in ("floor", "groundplane", "ground_plane"))

    def _on_contact_report(self, headers: object, _data: object) -> None:
        for header in headers:
            pair = tuple(
                sorted(
                    (
                        str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                        str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
                    )
                )
            )
            robot_members = [path.startswith(self.robot_root) for path in pair]
            if not any(robot_members) or all(robot_members):
                continue
            other = pair[0] if not robot_members[0] else pair[1]
            if self._is_floor(other):
                self.filtered_floor_events += 1
                continue
            self.events += 1
            self.pairs[pair] += 1

    def summary(self) -> dict[str, object]:
        return {
            "collision_event_count": self.events,
            "filtered_floor_event_count": self.filtered_floor_events,
            "reporter_count": len(self.reporter_paths),
            "reporter_paths": sorted(self.reporter_paths),
            "report_pair_filter": "all_non_floor_collision_prims",
            "report_pair_count": len(self.report_pair_paths),
            "contact_pairs": [
                {"actors": list(pair), "count": count}
                for pair, count in sorted(self.pairs.items())
            ],
            "monitor_wall_seconds": time.monotonic() - self.started_wall,
        }

    def close(self) -> None:
        self.subscription = None
