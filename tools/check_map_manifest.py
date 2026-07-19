#!/usr/bin/env python3
"""Reject incomplete, stale, or capture-contaminated runtime map sets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from write_map_manifest import (
    PROJECT_ROOT,
    REQUIRED_GROUPS,
    require_runtime_files,
    summarize_group,
)


EXPECTED_MAPPING_PROFILE = {
    "name": "mapping_8cam",
    "active_stereo_pairs": ["front", "left", "right", "back"],
    "active_rgb_streams": 8,
    "native_depth_min_range_m": 0.4,
    "static_reconstruction_frame": "map",
}
EXPECTED_NAVIGATION_PROFILE = {
    "name": "navigation_6cam",
    "active_stereo_pairs": ["front", "left", "right"],
    "disabled_stereo_pairs": ["back"],
    "active_rgb_streams": 6,
    "rear_render_products_created": False,
}
FORBIDDEN_CAPTURE_NAMES = {
    "capture",
    "offline",
    "online_cuvslam",
    "rosbag",
    "rosbag2",
}


def require_fields(actual: object, expected: dict[str, object], label: str) -> None:
    if not isinstance(actual, dict):
        raise RuntimeError(f"{label} must be a JSON object")
    mismatches = {
        key: {"expected": value, "actual": actual.get(key)}
        for key, value in expected.items()
        if actual.get(key) != value
    }
    if mismatches:
        raise RuntimeError(f"{label} does not match the runtime contract: {mismatches}")


def validate_occupancy_yaml(map_dir: Path) -> None:
    path = map_dir / "occupancy/map.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise RuntimeError(f"occupancy YAML is not a mapping: {path}")
    image = Path(str(document.get("image", "")))
    image_path = image if image.is_absolute() else path.parent / image
    if image_path.resolve() != (map_dir / "occupancy/map.pgm").resolve():
        raise RuntimeError(
            "occupancy YAML must reference the portable occupancy/map.pgm artifact"
        )
    report_path = map_dir / "occupancy/save_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("status") != "passed" or report.get("frame_id") != "map":
        raise RuntimeError("occupancy artifact must be a passed map-frame nvblox slice")
    if float(report.get("obstacle_distance_m", float("nan"))) != 0.0:
        raise RuntimeError("occupancy artifact must not pre-inflate the nvblox ESDF")
    policy = report.get("conversion_policy")
    if policy == "nvblox_distance_le_zero_is_occupied":
        return
    if policy != "nvblox_offline_static_occupancy_optimized_keyframes":
        raise RuntimeError("occupancy artifact has an unknown conversion policy")
    if report.get("schema_version") != 2:
        raise RuntimeError("offline occupancy artifact has an unsupported schema")
    checks = report.get("checks")
    if not isinstance(checks, dict) or not checks or not all(checks.values()):
        raise RuntimeError("offline occupancy artifact did not pass every quality gate")
    if report.get("failure_reasons"):
        raise RuntimeError("offline occupancy artifact retains failure reasons")
    fusion_report = map_dir / "nvblox/fusion_report.json"
    if not fusion_report.is_file():
        raise RuntimeError("offline occupancy provenance report is missing")
    digest = hashlib.sha256(fusion_report.read_bytes()).hexdigest()
    if report.get("fusion_report_sha256") != digest:
        raise RuntimeError("offline occupancy provenance hash does not match")
    fusion = json.loads(fusion_report.read_text(encoding="utf-8"))
    config_snapshot = map_dir / "config/offline_mapping.yaml"
    if not config_snapshot.is_file():
        raise RuntimeError("offline occupancy parameter snapshot is missing")
    config_digest = hashlib.sha256(config_snapshot.read_bytes()).hexdigest()
    if fusion.get("config_sha256") != config_digest:
        raise RuntimeError("offline occupancy parameter snapshot hash does not match")
    route_config = map_dir / "config/offline_route_acceptance.yaml"
    if not route_config.is_file():
        raise RuntimeError("offline occupancy route contract snapshot is missing")
    route_report_path = map_dir / "occupancy/route_validation.json"
    if not route_report_path.is_file():
        raise RuntimeError("offline occupancy route validation is missing")
    route_report = json.loads(route_report_path.read_text(encoding="utf-8"))
    routes = route_report.get("routes")
    if (
        route_report.get("status") != "passed"
        or not isinstance(routes, list)
        or not routes
        or any(
            not isinstance(route, dict) or not route.get("passed")
            for route in routes
        )
    ):
        raise RuntimeError("offline occupancy did not pass every topology route gate")


def validate_assets(manifest: dict[str, object]) -> None:
    assets_path = PROJECT_ROOT / "config/assets.yaml"
    assets = yaml.safe_load(assets_path.read_text(encoding="utf-8"))
    source_assets = manifest.get("source_assets")
    if not isinstance(source_assets, dict):
        raise RuntimeError("manifest source_assets is missing")
    for manifest_name, config_name in (
        ("environment", "environment"),
        ("robot", "robot"),
        ("hawk", "stereo_sensor"),
    ):
        expected = str(assets[config_name]["expected_sha256"])
        actual = source_assets.get(manifest_name, {})
        actual_hash = actual.get("sha256") if isinstance(actual, dict) else None
        if actual_hash != expected:
            raise RuntimeError(
                f"map {manifest_name} asset hash is stale: "
                f"expected {expected}, got {actual_hash}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=Path)
    parser.add_argument(
        "--require-navigation-smoke",
        action="store_true",
        help="also require a completed live navigation smoke recorded in the manifest",
    )
    args = parser.parse_args()
    map_dir = args.map_dir.resolve()
    manifest_path = map_dir / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"map manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise RuntimeError("unsupported map manifest schema")
    if manifest.get("map_name") != map_dir.name:
        raise RuntimeError("manifest map_name does not match its directory")
    require_fields(
        manifest.get("mapping_profile"), EXPECTED_MAPPING_PROFILE, "mapping_profile"
    )
    require_fields(
        manifest.get("navigation_profile"),
        EXPECTED_NAVIGATION_PROFILE,
        "navigation_profile",
    )
    validate_assets(manifest)
    require_runtime_files(map_dir)
    validate_occupancy_yaml(map_dir)

    recorded_groups = manifest.get("artifact_groups")
    if not isinstance(recorded_groups, dict):
        raise RuntimeError("manifest artifact_groups is missing")
    for name in REQUIRED_GROUPS:
        actual = summarize_group(map_dir / name)
        if recorded_groups.get(name) != actual:
            raise RuntimeError(f"map artifact group changed after generation: {name}")

    forbidden = sorted(
        str(path.relative_to(map_dir))
        for path in map_dir.rglob("*")
        if path.name.lower() in FORBIDDEN_CAPTURE_NAMES
        or (path.is_file() and path.suffix.lower() in {".mcap", ".db3"})
    )
    if forbidden:
        raise RuntimeError(f"raw/intermediate capture data leaked into map: {forbidden}")
    capture = manifest.get("capture", {})
    if not isinstance(capture, dict) or capture.get("retained_in_repository") is not False:
        raise RuntimeError("manifest must state that raw capture data is not retained")
    validation = manifest.get("validation", {})
    if not isinstance(validation, dict) or validation.get("mapping_artifacts_complete") is not True:
        raise RuntimeError("manifest does not mark mapping artifacts complete")
    if args.require_navigation_smoke and validation.get("navigation_6cam_smoke_passed") is not True:
        raise RuntimeError("navigation_6cam smoke has not been recorded for this map")
    print(
        f"MAP_MANIFEST_OK map={map_dir.name} mapping_streams=8 "
        "navigation_streams=6 rear_navigation=disabled"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
