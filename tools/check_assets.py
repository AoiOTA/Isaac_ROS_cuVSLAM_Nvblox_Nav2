#!/usr/bin/env python3
"""Verify the pinned Kujiale, Jackal and Hawk USD contracts."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

from pxr import Usd, UsdGeom


EXPECTED_HASHES = {
    "environment": "f76f1957e8f4cbfccb13f9670d6ce187793007c56d9b3aad8a8ee72507d658d2",
    "robot": "be499d8ed3c83deff8a1ce43dc2976ba2b1a8cc66eae80360a19f6d8bb8720ec",
    "hawk": "93c5708ff14d931373c2b5c512ddb277117fbe2c2fc86868f53c2156bf8fa03f",
}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def open_stage(path: Path) -> Usd.Stage:
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"USD stage could not be opened: {path}")
    return stage


def main() -> int:
    paths = {
        "environment": Path(os.environ["KUJIALE_USD"]),
        "robot": Path(os.environ["JACKAL_USD"]),
        "hawk": Path(os.environ["HAWK_USD"]),
    }
    for name, path in paths.items():
        if not path.is_file():
            raise RuntimeError(f"required {name} asset is missing: {path}")
        actual = digest(path)
        if actual != EXPECTED_HASHES[name]:
            raise RuntimeError(
                f"{name} checksum mismatch: expected {EXPECTED_HASHES[name]}, got {actual}"
            )

    environment = open_stage(paths["environment"])
    robot = open_stage(paths["robot"])
    hawk = open_stage(paths["hawk"])
    defaults = {
        "environment": str(environment.GetDefaultPrim().GetPath()),
        "robot": str(robot.GetDefaultPrim().GetPath()),
        "hawk": str(hawk.GetDefaultPrim().GetPath()),
    }
    if defaults != {"environment": "/Root", "robot": "/jackal", "hawk": "/hawk"}:
        raise RuntimeError(f"unexpected default prims: {defaults}")

    required_joints = [
        f"/jackal/{position}_{side}_wheel_joint"
        for position in ("front", "rear")
        for side in ("left", "right")
    ]
    missing = [path for path in required_joints if not robot.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Jackal wheel joints are missing: {missing}")
    hawk_cameras = [prim for prim in hawk.Traverse() if prim.IsA(UsdGeom.Camera)]
    if len(hawk_cameras) != 2:
        raise RuntimeError(f"Hawk must contain two cameras, found {len(hawk_cameras)}")

    report = {
        "assets": {
            name: {
                "path": str(path.resolve()),
                "sha256": EXPECTED_HASHES[name],
                "default_prim": defaults[name],
            }
            for name, path in paths.items()
        },
        "jackal_wheel_joints": required_joints,
        "hawk_camera_count": len(hawk_cameras),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, RuntimeError) as exc:
        print(f"asset check failed: {exc}")
        raise SystemExit(1) from exc
