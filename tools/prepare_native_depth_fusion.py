#!/usr/bin/env python3
"""Prepare optimized native-depth keyframes for the NVIDIA nvblox fuser.

The live nvblox map is integrated with the pose estimate that was available at
each camera callback.  A later cuVSLAM loop-closure correction does not
retroactively move voxels that have already been fused.  This tool instead
matches the recorded native depth to the final, globally optimized keyframes
used by cuVGL and writes the directory layout consumed by ``fuse_cusfm``.

Native Isaac Sim depth is 32FC1 metres.  The offline fuser expects 16-bit PNG
depth in millimetres, so the conversion is explicit and audited in report.json.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from statistics import median
from typing import Any

import cv2
import numpy as np


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_ns(message: Any) -> int:
    return (
        int(message.header.stamp.sec) * NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


def safe_relative_image_path(image_name: str, suffix: str) -> Path:
    path = Path(image_name).with_suffix(suffix)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe metadata image path: {image_name}")
    return path


def nearest_target_index(timestamps: list[int], value: int) -> int:
    index = bisect_left(timestamps, value)
    candidates = []
    if index < len(timestamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        raise ValueError("cannot match against an empty timestamp sequence")
    return min(candidates, key=lambda candidate: abs(timestamps[candidate] - value))


def image_array(message: Any) -> np.ndarray:
    """Decode a ROS Image while respecting row stride and byte order."""

    encoding = str(message.encoding).lower()
    encodings: dict[str, tuple[np.dtype[Any], int]] = {
        "mono8": (np.dtype("u1"), 1),
        "8uc1": (np.dtype("u1"), 1),
        "rgb8": (np.dtype("u1"), 3),
        "bgr8": (np.dtype("u1"), 3),
        "rgba8": (np.dtype("u1"), 4),
        "bgra8": (np.dtype("u1"), 4),
        "16uc1": (np.dtype("u2"), 1),
        "32fc1": (np.dtype("f4"), 1),
    }
    if encoding not in encodings:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    dtype, channels = encodings[encoding]
    if dtype.itemsize > 1:
        dtype = dtype.newbyteorder(">" if message.is_bigendian else "<")
    row_values = int(message.step) // dtype.itemsize
    required_values = int(message.height) * row_values
    values = np.frombuffer(memoryview(message.data), dtype=dtype, count=required_values)
    values = values.reshape(int(message.height), row_values)
    width_values = int(message.width) * channels
    values = values[:, :width_values]
    if channels > 1:
        values = values.reshape(int(message.height), int(message.width), channels)
    return np.asarray(values)


def depth_to_millimetres(
    message: Any, minimum_depth_m: float, maximum_depth_m: float
) -> tuple[np.ndarray, dict[str, float]]:
    values = image_array(message)
    encoding = str(message.encoding).lower()
    if encoding == "32fc1":
        depth_m = values.astype(np.float32, copy=False)
    elif encoding == "16uc1":
        depth_m = values.astype(np.float32) / 1000.0
    else:
        raise ValueError(f"depth topic must be 32FC1 or 16UC1, got {message.encoding}")
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= minimum_depth_m)
        & (depth_m <= maximum_depth_m)
    )
    output = np.zeros(depth_m.shape, dtype=np.uint16)
    output[valid] = np.rint(depth_m[valid] * 1000.0).astype(np.uint16)
    valid_values = depth_m[valid]
    metrics = {
        "valid_fraction": float(valid.mean()),
        "minimum_valid_depth_m": float(valid_values.min()) if valid_values.size else 0.0,
        "maximum_valid_depth_m": float(valid_values.max()) if valid_values.size else 0.0,
    }
    return output, metrics


def color_for_depth(message: Any, width: int, height: int) -> np.ndarray:
    values = image_array(message)
    encoding = str(message.encoding).lower()
    if encoding in {"mono8", "8uc1"}:
        values = cv2.cvtColor(values, cv2.COLOR_GRAY2BGR)
    elif encoding in {"rgb8", "rgba8"}:
        conversion = cv2.COLOR_RGB2BGR if encoding == "rgb8" else cv2.COLOR_RGBA2BGR
        values = cv2.cvtColor(values, conversion)
    elif encoding == "bgra8":
        values = cv2.cvtColor(values, cv2.COLOR_BGRA2BGR)
    if values.shape[:2] != (height, width):
        values = cv2.resize(values, (width, height), interpolation=cv2.INTER_AREA)
    return values


def matrix(values: Any, rows: int, columns: int) -> dict[str, Any]:
    return {
        "data": [float(value) for value in values],
        "row_count": rows,
        "column_count": columns,
    }


def update_calibration(camera_parameters: dict[str, Any], info: Any) -> None:
    calibration = camera_parameters["calibration_parameters"]
    calibration["image_width"] = int(info.width)
    calibration["image_height"] = int(info.height)
    calibration["camera_matrix"] = matrix(info.k, 3, 3)
    calibration["distortion_coefficients"] = matrix(info.d, 1, len(info.d))
    calibration["rectification_matrix"] = matrix(info.r, 3, 3)
    calibration["projection_matrix"] = matrix(info.p, 3, 4)


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=np.float64), percent))


def load_selected_frames(
    metadata_path: Path, sensor_name: str
) -> tuple[dict[str, Any], str, list[dict[str, Any]]]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    parameters = metadata.get("camera_params_id_to_camera_params", {})
    matching_ids = [
        str(identifier)
        for identifier, entry in parameters.items()
        if entry.get("sensor_meta_data", {}).get("sensor_name") == sensor_name
    ]
    if len(matching_ids) != 1:
        raise ValueError(
            f"expected one camera id for {sensor_name}, found {matching_ids}"
        )
    camera_id = matching_ids[0]
    frames = [
        deepcopy(frame)
        for frame in metadata.get("keyframes_metadata", [])
        if str(frame.get("camera_params_id") or "0") == camera_id
    ]
    frames.sort(key=lambda frame: int(frame["timestamp_microseconds"]))
    if not frames:
        raise ValueError(f"no optimized keyframes found for {sensor_name}")
    timestamps = [int(frame["timestamp_microseconds"]) for frame in frames]
    if len(set(timestamps)) != len(timestamps) or any(
        current <= previous for previous, current in zip(timestamps, timestamps[1:])
    ):
        raise ValueError("optimized keyframe timestamps are not strictly increasing")
    return metadata, camera_id, frames


def prepare(args: argparse.Namespace) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not (args.bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"rosbag metadata is missing: {args.bag}")
    if not args.frames_meta.is_file():
        raise FileNotFoundError(f"optimized frames metadata is missing: {args.frames_meta}")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    color_root = args.output_dir / "color"
    depth_root = args.output_dir / "depth"
    color_root.mkdir()
    depth_root.mkdir()

    metadata, camera_id, frames = load_selected_frames(args.frames_meta, args.sensor_name)
    timestamps_ns = [int(frame["timestamp_microseconds"]) * 1000 for frame in frames]
    maximum_sync_ns = round(args.maximum_sync_ms * 1_000_000)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    required_topics = [args.depth_topic, args.depth_info_topic, args.color_topic]
    missing = [topic for topic in required_topics if topic not in topic_types]
    if missing:
        raise RuntimeError(f"input bag is missing required topics: {missing}")
    message_types = {
        topic: get_message(topic_types[topic]) for topic in required_topics
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=required_topics))

    depth_info = None
    depth_matches: dict[int, tuple[int, dict[str, float]]] = {}
    color_matches: dict[int, int] = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        if topic == args.depth_info_topic:
            if depth_info is None:
                depth_info = message
            continue
        if depth_info is None:
            continue
        message_stamp = stamp_ns(message)
        target_index = nearest_target_index(timestamps_ns, message_stamp)
        delta_ns = abs(timestamps_ns[target_index] - message_stamp)
        if delta_ns > maximum_sync_ns:
            continue
        frame = frames[target_index]
        image_name = str(frame["image_name"])
        if topic == args.depth_topic:
            previous = depth_matches.get(target_index)
            if previous is not None and previous[0] <= delta_ns:
                continue
            depth_mm, metrics = depth_to_millimetres(
                message, args.minimum_depth_m, args.maximum_depth_m
            )
            output = depth_root / safe_relative_image_path(image_name, ".png")
            output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(output), depth_mm):
                raise RuntimeError(f"failed to write depth image: {output}")
            depth_matches[target_index] = (delta_ns, metrics)
        elif topic == args.color_topic:
            previous_delta = color_matches.get(target_index)
            if previous_delta is not None and previous_delta <= delta_ns:
                continue
            image = color_for_depth(message, int(depth_info.width), int(depth_info.height))
            output = color_root / safe_relative_image_path(image_name, ".jpg")
            output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(
                str(output), image, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]
            ):
                raise RuntimeError(f"failed to write color image: {output}")
            color_matches[target_index] = delta_ns

    if depth_info is None:
        raise RuntimeError(f"no CameraInfo received on {args.depth_info_topic}")
    missing_depth = [index for index in range(len(frames)) if index not in depth_matches]
    missing_color = [index for index in range(len(frames)) if index not in color_matches]
    output_metadata = deepcopy(metadata)
    output_metadata["keyframes_metadata"] = frames
    update_calibration(
        output_metadata["camera_params_id_to_camera_params"][camera_id], depth_info
    )
    (args.output_dir / "frames_meta.json").write_text(
        json.dumps(output_metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    depth_deltas_ms = [value[0] / 1_000_000 for value in depth_matches.values()]
    color_deltas_ms = [value / 1_000_000 for value in color_matches.values()]
    valid_fractions = [value[1]["valid_fraction"] for value in depth_matches.values()]
    minimum_depths = [
        value[1]["minimum_valid_depth_m"]
        for value in depth_matches.values()
        if value[1]["minimum_valid_depth_m"] > 0.0
    ]
    maximum_depths = [
        value[1]["maximum_valid_depth_m"]
        for value in depth_matches.values()
        if value[1]["maximum_valid_depth_m"] > 0.0
    ]
    report: dict[str, Any] = {
        "schema_version": 2,
        "status": "passed" if not missing_depth and not missing_color else "failed",
        "input_bag": str(args.bag.resolve()),
        "input_bag_metadata_sha256": hashlib.sha256(
            (args.bag / "metadata.yaml").read_bytes()
        ).hexdigest(),
        "source_frames_meta": str(args.frames_meta.resolve()),
        "source_frames_meta_sha256": hashlib.sha256(
            args.frames_meta.read_bytes()
        ).hexdigest(),
        "output_dir": str(args.output_dir.resolve()),
        "sensor_name": args.sensor_name,
        "camera_params_id": camera_id,
        "topics": {
            "depth": args.depth_topic,
            "depth_camera_info": args.depth_info_topic,
            "color": args.color_topic,
        },
        "conversion": {
            "input_depth_encoding": "32FC1_m_or_16UC1_mm",
            "output_depth_encoding": "16UC1_png_mm",
            "minimum_depth_m": args.minimum_depth_m,
            "maximum_depth_m": args.maximum_depth_m,
            "color_size_matches_native_depth": True,
        },
        "camera": {
            "frame_id": str(depth_info.header.frame_id) if depth_info else "",
            "width": int(depth_info.width) if depth_info else 0,
            "height": int(depth_info.height) if depth_info else 0,
            "fx": float(depth_info.k[0]) if depth_info else 0.0,
            "fy": float(depth_info.k[4]) if depth_info else 0.0,
            "cx": float(depth_info.k[2]) if depth_info else 0.0,
            "cy": float(depth_info.k[5]) if depth_info else 0.0,
        },
        "selected_keyframes": len(frames),
        "matched_depth_frames": len(depth_matches),
        "matched_color_frames": len(color_matches),
        "missing_depth_frame_indices": missing_depth,
        "missing_color_frame_indices": missing_color,
        "synchronization": {
            "maximum_allowed_ms": args.maximum_sync_ms,
            "depth_maximum_ms": max(depth_deltas_ms, default=0.0),
            "depth_p99_ms": percentile(depth_deltas_ms, 99.0),
            "color_maximum_ms": max(color_deltas_ms, default=0.0),
            "color_p99_ms": percentile(color_deltas_ms, 99.0),
        },
        "depth_quality": {
            "valid_fraction_minimum": min(valid_fractions, default=0.0),
            "valid_fraction_p05": percentile(valid_fractions, 5.0),
            "valid_fraction_median": median(valid_fractions) if valid_fractions else 0.0,
            "valid_fraction_mean": (
                sum(valid_fractions) / len(valid_fractions) if valid_fractions else 0.0
            ),
            "minimum_observed_depth_m": min(minimum_depths, default=0.0),
            "maximum_observed_depth_m": max(maximum_depths, default=0.0),
        },
    }
    (args.output_dir / "report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if report["status"] != "passed":
        raise RuntimeError(
            "native-depth extraction incomplete: "
            f"missing depth={len(missing_depth)}, color={len(missing_color)}; "
            f"see {args.output_dir / 'report.json'}"
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("frames_meta", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--sensor-name", default="front_stereo_camera_left")
    parser.add_argument(
        "--depth-topic", default="/front_stereo_camera/depth/image_raw"
    )
    parser.add_argument(
        "--depth-info-topic", default="/front_stereo_camera/depth/camera_info"
    )
    parser.add_argument(
        "--color-topic", default="/front_stereo_camera/left/image_raw"
    )
    parser.add_argument("--maximum-sync-ms", type=float, default=40.0)
    parser.add_argument("--minimum-depth-m", type=float, default=0.4)
    parser.add_argument("--maximum-depth-m", type=float, default=8.0)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    args = parser.parse_args()
    if args.maximum_sync_ms <= 0.0:
        parser.error("--maximum-sync-ms must be positive")
    if not 0.0 < args.minimum_depth_m < args.maximum_depth_m <= 65.535:
        parser.error("depth limits must satisfy 0 < minimum < maximum <= 65.535")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")
    report = prepare(args)
    print(
        "Prepared optimized native-depth fusion input: "
        f"{report['selected_keyframes']} keyframes, "
        f"depth p99 sync={report['synchronization']['depth_p99_ms']:.3f} ms"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
