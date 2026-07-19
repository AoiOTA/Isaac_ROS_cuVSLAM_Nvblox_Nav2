#!/usr/bin/env python3
"""Validate and describe shared frames carrying optimized cuVSLAM poses."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


POSE_ROLE = "shared_cuvslam_optimized_map_frames"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalized_camera_id(frame: dict[str, Any]) -> str:
    # Protobuf JSON omits scalar fields that retain their default value. Camera
    # id zero is therefore represented by either an absent field or "0".
    return str(frame.get("camera_params_id") or "0")


def analyze_frames_metadata(
    metadata: dict[str, Any], expected_camera_streams: int = 8
) -> dict[str, Any]:
    parameters = metadata.get("camera_params_id_to_camera_params")
    frames = metadata.get("keyframes_metadata")
    if not isinstance(parameters, dict) or not parameters:
        raise RuntimeError("optimized frame camera parameters are missing")
    if not isinstance(frames, list) or not frames:
        raise RuntimeError("optimized frame metadata contains no frames")

    camera_ids = {str(identifier) for identifier in parameters}
    if len(camera_ids) != expected_camera_streams:
        raise RuntimeError(
            "optimized frame metadata must contain "
            f"{expected_camera_streams} camera streams, got {sorted(camera_ids)}"
        )
    sensor_names = {
        str(entry.get("sensor_meta_data", {}).get("sensor_name", ""))
        for entry in parameters.values()
        if isinstance(entry, dict)
    }
    if "" in sensor_names or len(sensor_names) != expected_camera_streams:
        raise RuntimeError("optimized frame sensor names are missing or duplicated")

    groups: dict[int, set[str]] = {}
    keys: set[tuple[str, int]] = set()
    for index, raw_frame in enumerate(frames):
        if not isinstance(raw_frame, dict):
            raise RuntimeError(f"optimized frame row {index} is not an object")
        camera_id = normalized_camera_id(raw_frame)
        if camera_id not in camera_ids:
            raise RuntimeError(f"optimized frame row {index} has unknown camera {camera_id}")
        try:
            timestamp_us = int(raw_frame["timestamp_microseconds"])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                f"optimized frame row {index} has an invalid timestamp"
            ) from error
        if timestamp_us <= 0 or not raw_frame.get("image_name"):
            raise RuntimeError(f"optimized frame row {index} is incomplete")
        if not isinstance(raw_frame.get("camera_to_world"), dict):
            raise RuntimeError(f"optimized frame row {index} has no camera pose")
        key = (camera_id, timestamp_us)
        if key in keys:
            raise RuntimeError(f"duplicate optimized frame camera/timestamp: {key}")
        keys.add(key)
        groups.setdefault(timestamp_us, set()).add(camera_id)

    incomplete = {
        stamp: sorted(camera_ids - observed)
        for stamp, observed in groups.items()
        if observed != camera_ids
    }
    if incomplete:
        first_stamp = min(incomplete)
        raise RuntimeError(
            "optimized frame group is incomplete at "
            f"{first_stamp} us; missing cameras {incomplete[first_stamp]}"
        )
    return {
        "camera_streams": len(camera_ids),
        "camera_ids": sorted(camera_ids, key=int),
        "sensor_names": sorted(sensor_names),
        "synchronized_groups": len(groups),
        "frame_rows": len(frames),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("frames_meta", type=Path)
    parser.add_argument("output_report", type=Path)
    parser.add_argument("--tum-pose-file", type=Path, required=True)
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument("--selection-report", type=Path, required=True)
    parser.add_argument("--expected-camera-streams", type=int, default=8)
    args = parser.parse_args()

    for path, label in (
        (args.frames_meta, "optimized frame metadata"),
        (args.tum_pose_file, "optimized cuVSLAM trajectory"),
        (args.source_bag / "metadata.yaml", "source MCAP metadata"),
        (args.selection_report, "optimized frame selection report"),
    ):
        if not path.is_file() or path.stat().st_size == 0:
            raise FileNotFoundError(f"{label} is missing or empty: {path}")
    if args.expected_camera_streams <= 0:
        parser.error("--expected-camera-streams must be positive")

    metadata = json.loads(args.frames_meta.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise RuntimeError("optimized frame metadata root must be an object")
    summary = analyze_frames_metadata(metadata, args.expected_camera_streams)
    selection = json.loads(args.selection_report.read_text(encoding="utf-8"))
    if selection.get("status") != "passed":
        raise RuntimeError("optimized frame selection report did not pass")
    selected_groups = int(selection.get("selected_synchronized_groups", 0))
    if selected_groups != summary["synchronized_groups"]:
        raise RuntimeError(
            "converter output does not preserve every selected synchronized group: "
            f"selected={selected_groups} output={summary['synchronized_groups']}"
        )

    report = {
        "schema_version": 1,
        "status": "passed",
        "pose_role": POSE_ROLE,
        "pose_source": "isaac_ros_visual_slam_get_all_poses_global_optimization",
        "source_optimized_poses": str(args.tum_pose_file.resolve()),
        "source_optimized_poses_sha256": sha256_file(args.tum_pose_file),
        "source_bag": str(args.source_bag.resolve()),
        "source_bag_metadata_sha256": sha256_file(args.source_bag / "metadata.yaml"),
        "selection_report_sha256": sha256_file(args.selection_report),
        "frames_meta_sha256": sha256_file(args.frames_meta),
        **summary,
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        "SHARED_OPTIMIZED_FRAMES_OK "
        f"streams={summary['camera_streams']} "
        f"groups={summary['synchronized_groups']} rows={summary['frame_rows']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
