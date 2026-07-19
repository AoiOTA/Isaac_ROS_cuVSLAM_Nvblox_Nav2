#!/usr/bin/env python3
"""Build a compact, gap-bounded eight-camera bag for cuVGL conversion.

The Isaac Mapping ``rosbag_to_mapping_data`` executable applies its motion
threshold before checking temporal continuity.  A normal stationary interval
can therefore leave its last accepted frame several seconds behind; once that
happens, the converter rejects every later image as a timestamp discontinuity.

This tool performs the keyframe selection explicitly.  Only complete
eight-camera groups are retained, pose-based motion still selects dense frames,
and a periodic keepalive bounds stationary gaps.  The downstream converter is
then invoked with zero motion thresholds and cannot enter the unrecoverable
gap state.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
from typing import Iterable

import yaml


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True)
class SynchronizedGroup:
    stamp_ns: int
    image_stamps: dict[str, int]


def parse_tum(path: Path) -> list[PoseSample]:
    poses: list[PoseSample] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM fields")
        timestamp, x, y, z, qx, qy, qz, qw = map(float, fields)
        poses.append(
            PoseSample(
                stamp_ns=round(timestamp * NANOSECONDS_PER_SECOND),
                translation=(x, y, z),
                rotation=normalize_quaternion((qx, qy, qz, qw)),
            )
        )
    if not poses:
        raise ValueError(f"TUM trajectory is empty: {path}")
    if any(
        current.stamp_ns <= previous.stamp_ns
        for previous, current in zip(poses, poses[1:])
    ):
        raise ValueError(f"TUM timestamps are not strictly increasing: {path}")
    return poses


def normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_angle_degrees(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def interpolate_pose(poses: list[PoseSample], stamp_ns: int) -> PoseSample:
    timestamps = [pose.stamp_ns for pose in poses]
    index = bisect_left(timestamps, stamp_ns)
    if index == 0:
        return poses[0]
    if index >= len(poses):
        return poses[-1]
    before = poses[index - 1]
    after = poses[index]
    span = after.stamp_ns - before.stamp_ns
    fraction = (stamp_ns - before.stamp_ns) / span
    translation = tuple(
        start + fraction * (end - start)
        for start, end in zip(before.translation, after.translation)
    )
    second_rotation = after.rotation
    dot = sum(a * b for a, b in zip(before.rotation, second_rotation))
    if dot < 0.0:
        second_rotation = tuple(-value for value in second_rotation)
    rotation = normalize_quaternion(
        tuple(
            start + fraction * (end - start)
            for start, end in zip(before.rotation, second_rotation)
        )
    )
    return PoseSample(stamp_ns, translation, rotation)


def translation_distance(first: PoseSample, second: PoseSample) -> float:
    return math.dist(first.translation, second.translation)


def nearest_index(
    values: list[int], target: int, minimum_index: int
) -> int | None:
    index = bisect_left(values, target, lo=minimum_index)
    candidates = []
    if index < len(values):
        candidates.append(index)
    if index - 1 >= minimum_index:
        candidates.append(index - 1)
    if not candidates:
        return None
    return min(candidates, key=lambda candidate: abs(values[candidate] - target))


def synchronize_stamps(
    stamps_by_topic: dict[str, list[int]], threshold_ns: int
) -> list[SynchronizedGroup]:
    if not stamps_by_topic or any(not stamps for stamps in stamps_by_topic.values()):
        raise ValueError("every image topic must contain at least one timestamp")
    anchor_topic = min(stamps_by_topic, key=lambda topic: len(stamps_by_topic[topic]))
    cursors = {topic: 0 for topic in stamps_by_topic}
    groups: list[SynchronizedGroup] = []
    for anchor_stamp in stamps_by_topic[anchor_topic]:
        matches = {anchor_topic: anchor_stamp}
        indices: dict[str, int] = {}
        for topic, stamps in stamps_by_topic.items():
            if topic == anchor_topic:
                continue
            index = nearest_index(stamps, anchor_stamp, cursors[topic])
            if index is None or abs(stamps[index] - anchor_stamp) > threshold_ns:
                break
            matches[topic] = stamps[index]
            indices[topic] = index
        if len(matches) != len(stamps_by_topic):
            continue
        for topic, index in indices.items():
            cursors[topic] = index + 1
        canonical_stamp = sorted(matches.values())[len(matches) // 2]
        groups.append(SynchronizedGroup(canonical_stamp, matches))
    if any(
        current.stamp_ns <= previous.stamp_ns
        for previous, current in zip(groups, groups[1:])
    ):
        raise RuntimeError("synchronized groups are not strictly increasing")
    return groups


def select_keyframe_groups(
    groups: list[SynchronizedGroup],
    poses: list[PoseSample],
    minimum_translation_m: float,
    minimum_rotation_degrees: float,
    maximum_interval_s: float,
) -> tuple[list[SynchronizedGroup], int]:
    if not groups:
        raise ValueError("no synchronized camera groups")
    initial_pose = interpolate_pose(poses, groups[0].stamp_ns)
    start_index = next(
        (
            index
            for index, group in enumerate(groups)
            if translation_distance(
                initial_pose, interpolate_pose(poses, group.stamp_ns)
            )
            >= minimum_translation_m
            or quaternion_angle_degrees(
                initial_pose.rotation,
                interpolate_pose(poses, group.stamp_ns).rotation,
            )
            >= minimum_rotation_degrees
        ),
        None,
    )
    if start_index is None:
        raise RuntimeError(
            "trajectory never leaves its initialization pose; no cuVGL map can be built"
        )
    selected = [groups[start_index]]
    last_pose = interpolate_pose(poses, selected[0].stamp_ns)
    maximum_interval_ns = round(maximum_interval_s * NANOSECONDS_PER_SECOND)
    for group in groups[start_index + 1 :]:
        pose = interpolate_pose(poses, group.stamp_ns)
        elapsed_ns = group.stamp_ns - selected[-1].stamp_ns
        if (
            translation_distance(last_pose, pose) >= minimum_translation_m
            or quaternion_angle_degrees(last_pose.rotation, pose.rotation)
            >= minimum_rotation_degrees
            or elapsed_ns >= maximum_interval_ns
        ):
            selected.append(group)
            last_pose = pose
    if groups[-1].stamp_ns > selected[-1].stamp_ns:
        selected.append(groups[-1])
    return selected, start_index


def header_stamp_ns(serialized_message: bytes) -> int:
    if len(serialized_message) < 12:
        raise ValueError("serialized Image message is too short")
    representation = bytes(serialized_message[:2])
    if representation in {b"\x00\x01", b"\x00\x03"}:
        byte_order = "<"
    elif representation in {b"\x00\x00", b"\x00\x02"}:
        byte_order = ">"
    else:
        raise ValueError(f"unsupported CDR representation: {representation.hex()}")
    seconds, nanoseconds = struct.unpack_from(f"{byte_order}iI", serialized_message, 4)
    if nanoseconds >= NANOSECONDS_PER_SECOND:
        raise ValueError(f"invalid header nanoseconds: {nanoseconds}")
    return seconds * NANOSECONDS_PER_SECOND + nanoseconds


def camera_topics(config_path: Path) -> tuple[tuple[str, ...], tuple[str, ...]]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    cameras = config.get("stereo_cameras", []) if isinstance(config, dict) else []
    image_topics: list[str] = []
    info_topics: list[str] = []
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        for side in ("left", "right"):
            image_topics.append(str(camera[side]))
            info_topics.append(str(camera[f"{side}_camera_info"]))
    if len(image_topics) != 8 or len(set(image_topics)) != 8:
        raise ValueError("topic config must define four stereo pairs/eight images")
    return tuple(image_topics), tuple(info_topics)


def maximum_gap_seconds(groups: Iterable[SynchronizedGroup]) -> float:
    timestamps = [group.stamp_ns for group in groups]
    return max(
        ((current - previous) / NANOSECONDS_PER_SECOND
         for previous, current in zip(timestamps, timestamps[1:])),
        default=0.0,
    )


def prepare_bag(args: argparse.Namespace) -> dict[str, object]:
    import rosbag2_py

    image_topics, info_topics = camera_topics(args.topic_config)
    input_bag = args.input_bag.resolve()
    output_bag = args.output_bag.resolve()
    if not (input_bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"input rosbag metadata is missing: {input_bag}")
    if output_bag.exists():
        raise FileExistsError(f"refusing to overwrite output rosbag: {output_bag}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    metadata = {item.name: item for item in reader.get_all_topics_and_types()}
    missing_topics = [topic for topic in image_topics if topic not in metadata]
    if missing_topics:
        raise RuntimeError(f"input bag is missing image topics: {missing_topics}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(image_topics)))
    stamps_by_topic = {topic: [] for topic in image_topics}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        stamps_by_topic[topic].append(header_stamp_ns(serialized))

    threshold_ns = args.max_sync_us * 1_000
    groups = synchronize_stamps(stamps_by_topic, threshold_ns)
    poses = parse_tum(args.tum_pose_file)
    selected, initial_groups_removed = select_keyframe_groups(
        groups,
        poses,
        args.minimum_translation_m,
        args.minimum_rotation_degrees,
        args.maximum_interval_s,
    )
    if len(selected) < args.minimum_groups:
        raise RuntimeError(
            f"only {len(selected)} cuVGL keyframe groups selected; "
            f"require at least {args.minimum_groups}"
        )
    maximum_gap = maximum_gap_seconds(selected)
    if maximum_gap > args.maximum_allowed_gap_s:
        raise RuntimeError(
            f"selected cuVGL keyframe gap {maximum_gap:.3f}s exceeds "
            f"{args.maximum_allowed_gap_s:.3f}s"
        )

    selected_stamps = {
        topic: {group.image_stamps[topic] for group in selected}
        for topic in image_topics
    }
    retained_topics = set(image_topics) | set(info_topics) | {"/tf", "/tf_static"}
    missing_auxiliary = [
        topic for topic in (*info_topics, "/tf_static") if topic not in metadata
    ]
    if missing_auxiliary:
        raise RuntimeError(f"input bag is missing calibration topics: {missing_auxiliary}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(retained_topics)))
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(output_bag),
            storage_id="mcap",
            storage_preset_profile="zstd_fast",
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in sorted(retained_topics):
        if topic in metadata:
            writer.create_topic(metadata[topic])
    output_counts = {topic: 0 for topic in retained_topics if topic in metadata}
    while reader.has_next():
        topic, serialized, receive_timestamp = reader.read_next()
        if topic in selected_stamps:
            if header_stamp_ns(serialized) not in selected_stamps[topic]:
                continue
        writer.write(topic, serialized, receive_timestamp)
        output_counts[topic] += 1
    del writer

    report: dict[str, object] = {
        "schema_version": 1,
        "status": "passed",
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "input_image_counts": {
            topic: len(stamps_by_topic[topic]) for topic in image_topics
        },
        "complete_synchronized_groups": len(groups),
        "selected_synchronized_groups": len(selected),
        "initial_groups_removed": initial_groups_removed,
        "first_selected_stamp_s": selected[0].stamp_ns / NANOSECONDS_PER_SECOND,
        "last_selected_stamp_s": selected[-1].stamp_ns / NANOSECONDS_PER_SECOND,
        "maximum_selected_gap_s": maximum_gap,
        "maximum_group_skew_us": max(
            (
                max(group.image_stamps.values())
                - min(group.image_stamps.values())
            )
            / 1_000
            for group in selected
        ),
        "selection": {
            "minimum_translation_m": args.minimum_translation_m,
            "minimum_rotation_degrees": args.minimum_rotation_degrees,
            "maximum_interval_s": args.maximum_interval_s,
            "maximum_allowed_gap_s": args.maximum_allowed_gap_s,
            "maximum_sync_us": args.max_sync_us,
        },
        "output_message_counts": dict(sorted(output_counts.items())),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--tum-pose-file", type=Path, required=True)
    parser.add_argument("--topic-config", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--max-sync-us", type=int, default=40_000)
    parser.add_argument("--minimum-translation-m", type=float, default=0.1)
    parser.add_argument("--minimum-rotation-degrees", type=float, default=2.0)
    parser.add_argument("--maximum-interval-s", type=float, default=0.5)
    parser.add_argument("--maximum-allowed-gap-s", type=float, default=1.0)
    parser.add_argument("--minimum-groups", type=int, default=40)
    args = parser.parse_args()
    if args.max_sync_us <= 0:
        parser.error("--max-sync-us must be positive")
    if args.minimum_translation_m <= 0.0:
        parser.error("--minimum-translation-m must be positive")
    if args.minimum_rotation_degrees <= 0.0:
        parser.error("--minimum-rotation-degrees must be positive")
    if args.maximum_interval_s <= 0.0:
        parser.error("--maximum-interval-s must be positive")
    if args.maximum_allowed_gap_s < args.maximum_interval_s:
        parser.error("--maximum-allowed-gap-s must cover --maximum-interval-s")
    report = prepare_bag(args)
    print(
        "Prepared cuVGL sensor bag: "
        f"{report['selected_synchronized_groups']} synchronized groups, "
        f"maximum gap {report['maximum_selected_gap_s']:.3f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
