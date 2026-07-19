#!/usr/bin/env python3
"""Rewrite an MCAP bag with ROS header stamps as its indexed record time.

``ros2 bag record --storage-preset-profile fastwrite`` disables MCAP chunks and
summary indexes.  That is useful for short, high-rate captures, but visual-map
conversion then has to scan a long multi-camera bag in physical file order.
This utility writes the eight mapping-camera payloads plus their transforms
(or all topics when requested) into an indexed MCAP whose record times are the
message header stamps, so consumers can seek and merge the camera streams
deterministically.
"""

from __future__ import annotations

import argparse
from collections import Counter
import heapq
from pathlib import Path
import sys


MAPPING_CAMERA_TOPICS = frozenset(
    f"/{pair}_stereo_camera/{side}/{product}"
    for pair in ("front", "left", "right", "back")
    for side in ("left", "right")
    for product in ("image_raw", "camera_info")
)
REPAIR_TOPICS = MAPPING_CAMERA_TOPICS | frozenset(("/tf", "/tf_static"))


def stamp_to_nanoseconds(stamp: object) -> int | None:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    if sec < 0 or nanosec < 0:
        return None
    return sec * 1_000_000_000 + nanosec


def message_timestamp(topic: str, type_name: str, serialized: bytes) -> int | None:
    """Return a ROS header/clock timestamp, without coupling to project topics."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    message = deserialize_message(serialized, get_message(type_name))
    if type_name == "rosgraph_msgs/msg/Clock":
        return stamp_to_nanoseconds(message.clock)
    if type_name == "tf2_msgs/msg/TFMessage":
        transforms = message.transforms
        if transforms:
            timestamp = stamp_to_nanoseconds(transforms[0].header.stamp)
            # Static transforms conventionally use stamp=0.  Place them ahead
            # of simulated camera frames rather than using wall-clock record
            # time, which would make the physical stream non-monotonic.
            return 1 if topic == "/tf_static" and timestamp == 0 else timestamp
        return None
    header = getattr(message, "header", None)
    return stamp_to_nanoseconds(getattr(header, "stamp", None))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="rewrite a ROS 2 MCAP with indexed header timestamps"
    )
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--storage-profile", default="zstd_fast")
    parser.add_argument(
        "--reorder-window-seconds",
        type=float,
        default=2.0,
        help="header-time disorder to buffer before writing (default: 2.0)",
    )
    parser.add_argument(
        "--include-all-topics",
        action="store_true",
        help="copy every topic instead of only the eight mapping cameras",
    )
    args = parser.parse_args()
    if not args.input_bag.is_dir():
        parser.error(f"input bag directory does not exist: {args.input_bag}")
    if args.output_bag.exists():
        parser.error(f"refusing to overwrite existing output: {args.output_bag}")
    if args.reorder_window_seconds <= 0:
        parser.error("--reorder-window-seconds must be positive")

    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.input_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    selected_topics = [
        topic
        for topic in topics
        if args.include_all_topics or topic.name in REPAIR_TOPICS
    ]
    if not selected_topics:
        raise RuntimeError("the input bag has no selected camera topics")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output_bag),
            storage_id="mcap",
            storage_preset_profile=args.storage_profile,
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in selected_topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=topic.name,
                type=topic.type,
                serialization_format=topic.serialization_format,
                offered_qos_profiles=[],
            )
        )

    counts: Counter[str] = Counter()
    overridden: Counter[str] = Counter()
    fallback: Counter[str] = Counter()
    selected_names = {topic.name for topic in selected_topics}
    reorder_window_ns = int(args.reorder_window_seconds * 1_000_000_000)
    pending: list[tuple[int, int, str, bytes]] = []
    sequence = 0
    latest_timestamp = 0

    def flush_before(limit_ns: int) -> None:
        while pending and pending[0][0] <= limit_ns:
            timestamp_ns, _, topic, serialized = heapq.heappop(pending)
            writer.write(topic, serialized, timestamp_ns)

    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic not in selected_names:
            continue
        counts[topic] += 1
        try:
            stamped_ns = message_timestamp(topic, topic_types[topic], serialized)
        except Exception:  # A nonstandard message must remain lossless.
            stamped_ns = None
        timestamp_ns = stamped_ns if stamped_ns and stamped_ns > 0 else recorded_ns
        if timestamp_ns == recorded_ns:
            fallback[topic] += 1
        else:
            overridden[topic] += 1
        latest_timestamp = max(latest_timestamp, timestamp_ns)
        heapq.heappush(pending, (timestamp_ns, sequence, topic, serialized))
        sequence += 1
        flush_before(latest_timestamp - reorder_window_ns)
    flush_before(sys.maxsize)

    # Close explicitly: it writes the MCAP summary/index required by cuVGL.
    del writer
    print(f"Indexed MCAP written: {args.output_bag}")
    print(f"Messages copied: {sum(counts.values())}")
    print(f"Physical ordering: header-time sorted with {args.reorder_window_seconds:g}s buffer")
    print("Header-timestamped topics:")
    for topic in sorted(counts):
        print(f"  {topic}: {overridden[topic]}/{counts[topic]} overridden, " f"{fallback[topic]} fallback")
    return 0


if __name__ == "__main__":
    sys.exit(main())
