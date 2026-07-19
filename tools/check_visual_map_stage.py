#!/usr/bin/env python3
"""Validate shared cuVSLAM optimized frames and their cuVGL child map."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml

from write_optimized_frames_report import (
    POSE_ROLE,
    analyze_frames_metadata,
    normalized_camera_id,
    sha256_file,
)


def require_nonempty(path: Path, label: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"{label} is missing or empty: {path}")


def camera_pose(frame: dict[str, object]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    pose = frame.get("camera_to_world")
    if not isinstance(pose, dict):
        raise RuntimeError("frame camera_to_world pose is missing")
    translation = pose.get("translation")
    axis_angle = pose.get("axis_angle")
    if not isinstance(translation, dict) or not isinstance(axis_angle, dict):
        raise RuntimeError("frame camera_to_world pose is incomplete")
    xyz = tuple(float(translation[name]) for name in ("x", "y", "z"))
    axis = tuple(float(axis_angle[name]) for name in ("x", "y", "z"))
    angle = math.radians(float(axis_angle["angle_degrees"]))
    norm = math.sqrt(sum(value * value for value in axis))
    if abs(angle) <= 1.0e-15:
        quaternion = (0.0, 0.0, 0.0, 1.0)
    elif norm <= 1.0e-15:
        raise RuntimeError("nonzero camera rotation has a zero-length axis")
    else:
        scale = math.sin(angle / 2.0) / norm
        quaternion = tuple(value * scale for value in axis) + (math.cos(angle / 2.0),)
    return xyz, quaternion


def require_same_pose(
    shared_frame: dict[str, object], cuvgl_frame: dict[str, object]
) -> None:
    shared_xyz, shared_q = camera_pose(shared_frame)
    cuvgl_xyz, cuvgl_q = camera_pose(cuvgl_frame)
    if math.dist(shared_xyz, cuvgl_xyz) > 1.0e-8:
        raise RuntimeError("cuVGL keyframe translation diverges from shared cuVSLAM pose")
    shared_norm = math.sqrt(sum(value * value for value in shared_q))
    cuvgl_norm = math.sqrt(sum(value * value for value in cuvgl_q))
    if shared_norm <= 1.0e-15 or cuvgl_norm <= 1.0e-15:
        raise RuntimeError("camera rotation produced an invalid zero quaternion")
    dot = abs(
        sum(first * second for first, second in zip(shared_q, cuvgl_q))
        / (shared_norm * cuvgl_norm)
    )
    rotation_error = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    if rotation_error > 1.0e-8:
        raise RuntimeError("cuVGL keyframe rotation diverges from shared cuVSLAM pose")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("--minimum-synchronized-groups", type=int, default=40)
    parser.add_argument("--source-bag", type=Path, default=None)
    args = parser.parse_args()
    if args.minimum_synchronized_groups <= 0:
        parser.error("--minimum-synchronized-groups must be positive")
    root = args.map_dir.resolve()

    cuvslam_database = root / "cuvslam/data.mdb"
    optimized_poses = root / "cuvslam/optimized_poses.tum"
    cuvslam_report_path = root / "cuvslam/save_report.json"
    shared_meta_path = root / "optimized_frames/frames_meta.json"
    shared_report_path = root / "optimized_frames/report.json"
    selection_path = root / "optimized_frames/selection_report.json"
    cuvgl_meta_path = root / "cuvgl/keyframes/frames_meta.json"
    paths = (
        (cuvslam_database, "cuVSLAM database"),
        (optimized_poses, "optimized cuVSLAM trajectory"),
        (cuvslam_report_path, "cuVSLAM quality report"),
        (shared_meta_path, "shared optimized frame metadata"),
        (shared_report_path, "shared optimized frame provenance"),
        (selection_path, "shared optimized frame selection report"),
        (cuvgl_meta_path, "cuVGL keyframe metadata"),
    )
    for path, label in paths:
        require_nonempty(path, label)

    cuvslam_report = json.loads(cuvslam_report_path.read_text(encoding="utf-8"))
    if cuvslam_report.get("status") != "passed":
        raise RuntimeError("cuVSLAM quality report did not pass")
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    if selection.get("status") != "passed":
        raise RuntimeError("shared optimized frame selection did not pass")
    shared_report = json.loads(shared_report_path.read_text(encoding="utf-8"))
    if (
        shared_report.get("status") != "passed"
        or shared_report.get("pose_role") != POSE_ROLE
    ):
        raise RuntimeError("shared optimized frame provenance did not pass")
    if shared_report.get("source_optimized_poses_sha256") != sha256_file(
        optimized_poses
    ):
        raise RuntimeError("shared frames do not match the optimized cuVSLAM trajectory")
    if shared_report.get("frames_meta_sha256") != sha256_file(shared_meta_path):
        raise RuntimeError("shared optimized frame metadata hash does not match")
    if shared_report.get("selection_report_sha256") != sha256_file(selection_path):
        raise RuntimeError("shared optimized frame selection hash does not match")
    if args.source_bag is not None:
        metadata_path = args.source_bag.resolve() / "metadata.yaml"
        require_nonempty(metadata_path, "source MCAP metadata")
        if shared_report.get("source_bag_metadata_sha256") != hashlib.sha256(
            metadata_path.read_bytes()
        ).hexdigest():
            raise RuntimeError("shared optimized frames do not match the source MCAP")
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
            raise RuntimeError("optimized frame stage does not match the source MCAP")

    shared_metadata = json.loads(shared_meta_path.read_text(encoding="utf-8"))
    shared_summary = analyze_frames_metadata(shared_metadata, 8)
    if any(shared_report.get(key) != value for key, value in shared_summary.items()):
        raise RuntimeError("shared optimized frame summary does not match metadata")
    if int(selection.get("selected_synchronized_groups", 0)) != int(
        shared_summary["synchronized_groups"]
    ):
        raise RuntimeError("shared optimized frame count does not match selection")

    metadata = json.loads(cuvgl_meta_path.read_text(encoding="utf-8"))
    cuvgl_summary = analyze_frames_metadata(metadata, 8)
    frames = metadata["keyframes_metadata"]
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

    shared_frames = {
        (normalized_camera_id(frame), int(frame["timestamp_microseconds"])): frame
        for frame in shared_metadata["keyframes_metadata"]
    }
    for frame in frames:
        key = (normalized_camera_id(frame), int(frame["timestamp_microseconds"]))
        shared_frame = shared_frames.get(key)
        if shared_frame is None:
            raise RuntimeError(f"cuVGL keyframe is absent from shared optimized frames: {key}")
        require_same_pose(shared_frame, frame)

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
        f"shared_groups={shared_summary['synchronized_groups']} "
        f"cuvgl_groups={cuvgl_summary['synchronized_groups']} keyframes={len(frames)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
