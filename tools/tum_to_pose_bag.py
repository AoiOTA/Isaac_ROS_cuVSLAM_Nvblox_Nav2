#!/usr/bin/env python3
"""Convert a cuVSLAM TUM trajectory into an indexed ROS 2 odometry bag."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def parse_tum(path: Path) -> list[tuple[int, tuple[float, ...]]]:
    poses: list[tuple[int, tuple[float, ...]]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM fields")
        values = tuple(float(field) for field in fields)
        timestamp_ns = round(values[0] * 1_000_000_000)
        if timestamp_ns <= 0:
            raise ValueError(f"{path}:{line_number}: timestamp must be positive")
        poses.append((timestamp_ns, values[1:]))
    if not poses:
        raise ValueError(f"TUM trajectory is empty: {path}")
    if any(second[0] <= first[0] for first, second in zip(poses, poses[1:])):
        raise ValueError(f"TUM trajectory timestamps are not strictly increasing: {path}")
    return poses


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tum_file", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--topic", default="/visual_slam/vis/slam_odometry")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--child-frame", default="base_link")
    args = parser.parse_args()
    if not args.tum_file.is_file():
        parser.error(f"TUM trajectory does not exist: {args.tum_file}")
    if args.output_bag.exists():
        parser.error(f"refusing to overwrite existing output: {args.output_bag}")
    poses = parse_tum(args.tum_file)

    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    import rosbag2_py

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output_bag),
            storage_id="mcap",
            storage_preset_profile="zstd_fast",
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0,
            name=args.topic,
            type="nav_msgs/msg/Odometry",
            serialization_format="cdr",
            offered_qos_profiles=[],
        )
    )
    for timestamp_ns, (x, y, z, qx, qy, qz, qw) in poses:
        message = Odometry()
        message.header.stamp = Time(
            sec=timestamp_ns // 1_000_000_000,
            nanosec=timestamp_ns % 1_000_000_000,
        )
        message.header.frame_id = args.frame
        message.child_frame_id = args.child_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = z
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        writer.write(args.topic, serialize_message(message), timestamp_ns)
    del writer
    print(f"Wrote {len(poses)} map-frame odometry poses to {args.output_bag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
