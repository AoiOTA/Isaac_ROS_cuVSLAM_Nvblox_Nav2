#!/usr/bin/env python3
"""Verify the temporary MCAP used during an active mapping benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from write_map_manifest import bag_summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    capture = bag_summary(args.bag_dir.resolve())
    if capture["storage_identifier"] != "mcap":
        raise RuntimeError(
            f"mapping performance capture must be MCAP, got {capture['storage_identifier']}"
        )
    counts = capture["mapping_image_message_counts"]
    result = {
        "schema_version": 1,
        "status": "passed",
        "active_rgb_streams": len(counts),
        "all_mapping_streams_recorded": len(counts) == 8
        and min(int(value) for value in counts.values()) > 0,
        "capture": capture,
    }
    if not result["all_mapping_streams_recorded"]:
        result["status"] = "failed"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
