#!/usr/bin/env python3
"""Validate and describe the portable Kujiale/Jackal runtime map set."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_GROUPS = ("cuvslam", "cuvgl", "nvblox", "mesh", "occupancy", "config")
MAPPING_IMAGE_TOPICS = tuple(
    f"/{pair}_stereo_camera/{side}/image_raw"
    for pair in ("front", "left", "right", "back")
    for side in ("left", "right")
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def summarize_group(root: Path) -> dict[str, object]:
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise RuntimeError(f"map artifact group is empty: {root}")
    tree = hashlib.sha256()
    total_bytes = 0
    for path in files:
        relative = path.relative_to(root).as_posix()
        file_hash = sha256_file(path)
        total_bytes += path.stat().st_size
        tree.update(relative.encode("utf-8"))
        tree.update(b"\0")
        tree.update(file_hash.encode("ascii"))
        tree.update(b"\n")
    return {
        "file_count": len(files),
        "bytes": total_bytes,
        "sha256_tree": tree.hexdigest(),
    }


def bag_summary(path: Path) -> dict[str, object]:
    metadata_path = path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata is missing: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = metadata.get("rosbag2_bagfile_information", {})
    topics = {
        str(item.get("topic_metadata", {}).get("name")): int(
            item.get("message_count", 0)
        )
        for item in information.get("topics_with_message_count", [])
    }
    missing = [topic for topic in MAPPING_IMAGE_TOPICS if topics.get(topic, 0) <= 0]
    if missing:
        raise RuntimeError(f"four-Hawk/eight-stream mapping bag has empty image topics: {missing}")
    return {
        "storage_identifier": information.get("storage_identifier"),
        "message_count": int(information.get("message_count", 0)),
        "mapping_image_message_counts": {
            topic: topics[topic] for topic in MAPPING_IMAGE_TOPICS
        },
        "retained_in_repository": False,
    }


def require_runtime_files(map_dir: Path) -> None:
    exact = (
        map_dir / "occupancy/map.yaml",
        map_dir / "occupancy/map.pgm",
    )
    for path in exact:
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"required runtime map file is missing or empty: {path}")
    patterns = {
        "cuVSLAM database": (map_dir / "cuvslam", ("*.mdb", "*.db")),
        "cuVGL BoW index": (map_dir / "cuvgl", ("bow_index*",)),
        "nvblox binary map": (map_dir / "nvblox", ("*.nvblx",)),
        "nvblox mesh": (map_dir / "mesh", ("*.ply",)),
    }
    for label, (root, globs) in patterns.items():
        found = any(
            item.is_file() and item.stat().st_size > 0
            for pattern in globs
            for item in root.glob(pattern)
        )
        if not found:
            raise RuntimeError(f"{label} is missing from {root}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--assets", type=Path, default=PROJECT_ROOT / "config/assets.yaml"
    )
    parser.add_argument(
        "--generation-command",
        default="./scripts/run_mapping.sh --map {map_name} --interactive",
    )
    args = parser.parse_args()
    map_dir = args.map_dir.resolve()
    if not map_dir.is_dir():
        raise FileNotFoundError(f"map directory is missing: {map_dir}")
    assets = yaml.safe_load(args.assets.read_text(encoding="utf-8"))
    require_runtime_files(map_dir)
    groups = {
        name: summarize_group(map_dir / name) for name in REQUIRED_GROUPS
    }
    manifest = {
        "schema_version": 1,
        "map_name": map_dir.name,
        "created_unix_s": time.time(),
        "run_id": args.run_id,
        "map_frame": "map",
        "source_assets": {
            name: {
                "path": entry.get("resolved_path"),
                "default_prim": entry.get("expected_default_prim"),
                "sha256": entry.get("expected_sha256"),
            }
            for name, entry in (
                ("environment", assets["environment"]),
                ("robot", assets["robot"]),
                ("hawk", assets["stereo_sensor"]),
            )
        },
        "robot": {
            "name": "Clearpath Jackal",
            "spawn_usd_xyz_m": [2.9, -0.2, 0.0635],
            "spawn_yaw_deg": 180.0,
        },
        "mapping_profile": {
            "name": "mapping_8cam",
            "active_stereo_pairs": ["front", "left", "right", "back"],
            "active_rgb_streams": 8,
            "resolution": [1280, 800],
            "nominal_rate_hz": 10.0,
            "native_depth_source": "front_hawk_left",
        },
        "navigation_profile": {
            "name": "navigation_6cam",
            "active_stereo_pairs": ["front", "left", "right"],
            "disabled_stereo_pairs": ["back"],
            "active_rgb_streams": 6,
            "rear_render_products_created": False,
        },
        "software": {
            "ros_distro": "jazzy",
            "isaac_ros_release": "4.5",
            "isaac_sim_version": "6.0.1",
        },
        "capture": bag_summary(args.bag_dir.resolve()),
        "artifact_groups": groups,
        "generation_command": args.generation_command.format(map_name=map_dir.name),
        "git_lfs_required": True,
        "validation": {
            "mapping_artifacts_complete": True,
            "navigation_6cam_smoke_passed": False,
            "static_avoidance_acceptance_passed": False,
        },
    }
    (map_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
