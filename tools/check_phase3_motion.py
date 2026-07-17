#!/usr/bin/env python3
"""Check the persisted Phase 3 motion result."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text(encoding="utf-8"))
    failed = [name for name, passed in result.get("checks", {}).items() if not passed]
    if result.get("status") != "passed" or failed:
        raise SystemExit(f"Phase 3 motion suite failed: error={result.get('error')} checks={failed}")
    print(
        "phase3_motion=passed "
        f"segments={len(result['segments'])} samples={result['sample_counts']['ground_truth']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
