#!/usr/bin/env python3
"""Validate the reusable cuVSLAM/cuVGL stage before occupancy fusion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def require_nonempty(path: Path, label: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"{label} is missing or empty: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("--minimum-synchronized-groups", type=int, default=40)
    parser.add_argument("--source-bag", type=Path, default=None)
    args = parser.parse_args()
    if args.minimum_synchronized_groups <= 0:
        parser.error("--minimum-synchronized-groups must be positive")
    root = args.map_dir.resolve()

    paths = (
        (root / "cuvslam/data.mdb", "cuVSLAM database"),
        (root / "cuvslam/optimized_poses.tum", "optimized cuVSLAM trajectory"),
        (root / "cuvslam/save_report.json", "cuVSLAM quality report"),
        (root / "cuvgl/keyframes/frames_meta.json", "cuVGL keyframe metadata"),
        (
            root / "config/vgl_sensor_selection_report.json",
            "cuVGL sensor selection report",
        ),
    )
    for path, label in paths:
        require_nonempty(path, label)

    cuvslam_report = json.loads(paths[2][0].read_text(encoding="utf-8"))
    if cuvslam_report.get("status") != "passed":
        raise RuntimeError("cuVSLAM quality report did not pass")
    selection = json.loads(paths[4][0].read_text(encoding="utf-8"))
    if selection.get("status") != "passed":
        raise RuntimeError("cuVGL sensor selection did not pass")
    if args.source_bag is not None:
        metadata_path = args.source_bag.resolve() / "metadata.yaml"
        require_nonempty(metadata_path, "source MCAP metadata")
        bag_metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        information = bag_metadata.get("rosbag2_bagfile_information", {})
        topic_counts = {
            str(item.get("topic_metadata", {}).get("name")): int(
                item.get("message_count", 0)
            )
            for item in information.get("topics_with_message_count", [])
        }
        selected_counts = selection.get("input_image_counts")
        if not isinstance(selected_counts, dict) or not selected_counts or any(
            topic_counts.get(str(topic), 0) != int(count)
            for topic, count in selected_counts.items()
        ):
            raise RuntimeError("cuVGL stage does not match the requested source MCAP")

    metadata = json.loads(paths[3][0].read_text(encoding="utf-8"))
    frames = metadata.get("keyframes_metadata")
    if not isinstance(frames, list) or not frames:
        raise RuntimeError("cuVGL keyframe metadata has no frames")
    timestamps = {
        int(frame["timestamp_microseconds"])
        for frame in frames
        if isinstance(frame, dict) and "timestamp_microseconds" in frame
    }
    camera_ids = {
        str(frame.get("camera_params_id") or "0")
        for frame in frames
        if isinstance(frame, dict)
    }
    if len(timestamps) < args.minimum_synchronized_groups:
        raise RuntimeError(
            f"cuVGL has only {len(timestamps)} synchronized groups; "
            f"require {args.minimum_synchronized_groups}"
        )
    if len(camera_ids) != 8:
        raise RuntimeError(f"cuVGL must retain all eight mapping streams, got {camera_ids}")

    keyframe_files = [
        path
        for path in (root / "cuvgl/keyframes").rglob("*.pb")
        if path.is_file() and path.stat().st_size > 0
    ]
    if len(keyframe_files) < len(frames):
        raise RuntimeError(
            f"cuVGL has {len(keyframe_files)} keyframe payloads for "
            f"{len(frames)} metadata rows"
        )
    vocabulary = [
        path
        for path in (root / "cuvgl/vocabulary").glob("bow_vocabulary*.pb")
        if path.is_file() and path.stat().st_size > 0
    ]
    bow_indexes = [
        path
        for path in (root / "cuvgl").glob("bow_index*")
        if path.is_file() and path.stat().st_size > 0
    ]
    if not vocabulary or not bow_indexes:
        raise RuntimeError("cuVGL vocabulary or BoW index is missing")

    print(
        f"VISUAL_MAP_STAGE_OK map={root.name} streams={len(camera_ids)} "
        f"synchronized_groups={len(timestamps)} keyframes={len(frames)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
