#!/usr/bin/env python3
"""Quality-gate an offline nvblox image and emit portable Nav2 PGM/YAML."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


def component_statistics(occupied: np.ndarray) -> dict[str, Any]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied.astype(np.uint8), connectivity=8
    )
    sizes = sorted((int(value) for value in stats[1:, cv2.CC_STAT_AREA]), reverse=True)
    return {
        "count": count - 1,
        "largest_cells": sizes[:10],
        "single_cell_count": sum(size == 1 for size in sizes),
        "at_most_two_cells_count": sum(size <= 2 for size in sizes),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate_png", type=Path)
    parser.add_argument("candidate_yaml", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--fusion-report", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(f"refusing to overwrite occupancy output: {args.output_dir}")
    image = cv2.imread(str(args.candidate_png), cv2.IMREAD_GRAYSCALE)
    if image is None or image.size == 0:
        raise RuntimeError(f"cannot read candidate occupancy image: {args.candidate_png}")
    metadata = yaml.safe_load(args.candidate_yaml.read_text(encoding="utf-8"))
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    fusion = json.loads(args.fusion_report.read_text(encoding="utf-8"))
    quality = config["quality"]

    occupied = image <= 50
    free = image >= 240
    unknown = ~(occupied | free)
    total = int(image.size)
    counts = {
        "occupied": int(occupied.sum()),
        "free": int(free.sum()),
        "unknown": int(unknown.sum()),
    }
    fractions = {name: value / total for name, value in counts.items()}
    fractions["known"] = fractions["occupied"] + fractions["free"]
    checks = {
        "fusion_passed": fusion.get("status") == "passed",
        "contains_all_cell_classes": all(value > 0 for value in counts.values()),
        "minimum_free_fraction": fractions["free"]
        >= float(quality["minimum_free_fraction"]),
        "minimum_known_fraction": fractions["known"]
        >= float(quality["minimum_known_fraction"]),
        "maximum_unknown_fraction": fractions["unknown"]
        <= float(quality["maximum_unknown_fraction"]),
        "minimum_occupied_fraction": fractions["occupied"]
        >= float(quality["minimum_occupied_fraction"]),
        "maximum_occupied_fraction": fractions["occupied"]
        <= float(quality["maximum_occupied_fraction"]),
    }
    failures = [name for name, passed in checks.items() if not passed]
    report = {
        "schema_version": 2,
        "status": "passed" if not failures else "failed",
        "frame_id": "map",
        "conversion_policy": "nvblox_offline_static_occupancy_optimized_keyframes",
        "obstacle_distance_m": 0.0,
        "source": "nvblox_fuse_cusfm_static_occupancy",
        "fusion_report_sha256": hashlib.sha256(
            args.fusion_report.read_bytes()
        ).hexdigest(),
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "resolution": float(metadata["resolution"]),
        "origin": [float(value) for value in metadata["origin"]],
        "counts": counts,
        "fractions": fractions,
        "occupied_components": component_statistics(occupied),
        "checks": checks,
        "failure_reasons": failures,
        "thresholds": quality,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "save_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if failures:
        raise RuntimeError(
            f"offline occupancy quality failed: {failures}; "
            f"see {args.output_dir / 'save_report.json'}"
        )

    portable = np.full(image.shape, 205, dtype=np.uint8)
    portable[free] = 254
    portable[occupied] = 0
    pgm = args.output_dir / "map.pgm"
    pgm.write_bytes(
        f"P5\n# optimized offline nvblox static occupancy\n"
        f"{portable.shape[1]} {portable.shape[0]}\n255\n".encode("ascii")
        + portable.tobytes()
    )
    origin = report["origin"]
    (args.output_dir / "map.yaml").write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {report['resolution']:.9f}\n"
        f"origin: [{origin[0]:.9f}, {origin[1]:.9f}, {origin[2]:.9f}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )
    print(
        "Offline occupancy promoted: "
        f"free={fractions['free']:.3f}, occupied={fractions['occupied']:.3f}, "
        f"unknown={fractions['unknown']:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
