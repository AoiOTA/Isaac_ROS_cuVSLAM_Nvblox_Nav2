#!/usr/bin/env python3
"""Run reproducible optimized-pose TSDF and static-occupancy fusion passes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
from typing import Any

import yaml


def gflags(parameters: dict[str, Any]) -> list[str]:
    arguments: list[str] = []
    for name, value in parameters.items():
        if value is None:
            continue
        if isinstance(value, bool):
            arguments.append(f"--{name}" if value else f"--{name}=false")
        else:
            arguments.append(f"--{name}={value}")
    return arguments


def require_nonempty(path: Path) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"expected non-empty fusion artifact: {path}")


def run_pass(
    label: str,
    base_command: list[str],
    parameters: dict[str, Any],
    log_path: Path,
    timeout_s: float,
) -> dict[str, Any]:
    command = base_command + gflags(parameters)
    started = time.monotonic()
    with log_path.open("wb") as log:
        result = subprocess.run(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            check=False,
        )
    elapsed = time.monotonic() - started
    report = {
        "label": label,
        "command": command,
        "log": str(log_path),
        "return_code": result.returncode,
        "runtime_s": elapsed,
    }
    if result.returncode != 0:
        raise RuntimeError(
            f"{label} nvblox fusion failed with code {result.returncode}; see {log_path}"
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=1800.0)
    args = parser.parse_args()
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    for relative in ("color", "depth", "frames_meta.json", "report.json"):
        path = args.input_dir / relative
        if not path.exists():
            raise FileNotFoundError(f"offline fusion input is missing: {path}")
    if args.output_dir.exists():
        raise FileExistsError(f"refusing to overwrite fusion output: {args.output_dir}")
    config_bytes = args.config.read_bytes()
    config = yaml.safe_load(config_bytes)
    if not isinstance(config, dict):
        raise ValueError(f"offline mapping config is not a mapping: {args.config}")
    sections = {}
    for name in ("nvblox_common", "tsdf_fuser", "occupancy_fuser"):
        section = config.get(name, {})
        if not isinstance(section, dict):
            raise ValueError(f"offline mapping config section must be a mapping: {name}")
        sections[name] = section
    common = sections["nvblox_common"]
    tsdf_parameters = {**common, **sections["tsdf_fuser"]}
    occupancy_parameters = {**common, **sections["occupancy_fuser"]}

    output = args.output_dir
    logs = output / "logs"
    mesh = output / "mesh" / "kujiale.ply"
    tsdf_map = output / "nvblox" / "kujiale_tsdf.nvblx"
    occupancy_map = output / "nvblox" / "kujiale.nvblx"
    tsdf_preview_stem = output / "tsdf_preview" / "map"
    candidate_stem = output / "occupancy_candidate" / "map"
    for directory in (
        logs,
        mesh.parent,
        tsdf_map.parent,
        tsdf_preview_stem.parent,
        candidate_stem.parent,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    base = [
        "ros2",
        "run",
        "nvblox_ros",
        "fuse_cusfm",
        f"--color_image_dir={args.input_dir / 'color'}",
        f"--depth_image_dir={args.input_dir / 'depth'}",
        f"--frames_meta_file={args.input_dir / 'frames_meta.json'}",
    ]
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "running",
        "input_dir": str(args.input_dir.resolve()),
        "input_report": json.loads(
            (args.input_dir / "report.json").read_text(encoding="utf-8")
        ),
        "config": str(args.config.resolve()),
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "parameters": {
            "tsdf_mesh": tsdf_parameters,
            "static_occupancy": occupancy_parameters,
        },
        "passes": [],
    }
    try:
        report["passes"].append(
            run_pass(
                "tsdf_mesh",
                base
                + [
                    f"--save_2d_occupancy_map_path={tsdf_preview_stem}",
                    f"--mesh_output_path={mesh}",
                    f"--map_output_path={tsdf_map}",
                    f"--timing_output_path={output / 'nvblox/tsdf_timings.txt'}",
                ],
                tsdf_parameters,
                logs / "fuse-tsdf.log",
                args.timeout_s,
            )
        )
        report["passes"].append(
            run_pass(
                "static_occupancy",
                base
                + [
                    f"--save_2d_occupancy_map_path={candidate_stem}",
                    f"--map_output_path={occupancy_map}",
                    f"--timing_output_path={output / 'nvblox/occupancy_timings.txt'}",
                ],
                occupancy_parameters,
                logs / "fuse-occupancy.log",
                args.timeout_s,
            )
        )
        required = (
            mesh,
            tsdf_map,
            occupancy_map,
            candidate_stem.with_suffix(".png"),
            candidate_stem.with_suffix(".yaml"),
        )
        for path in required:
            require_nonempty(path)
        report["artifacts"] = {
            str(path.relative_to(output)): path.stat().st_size for path in required
        }
        report["status"] = "passed"
    except Exception as error:
        report["status"] = "failed"
        report["failure"] = str(error)
        raise
    finally:
        (output / "fusion_report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        "Offline nvblox fusion passed: "
        f"TSDF {report['passes'][0]['runtime_s']:.2f}s, "
        f"occupancy {report['passes'][1]['runtime_s']:.2f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
