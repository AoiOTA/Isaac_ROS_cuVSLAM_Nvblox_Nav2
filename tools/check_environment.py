#!/usr/bin/env python3
"""Lightweight checks for the fixed host and currently installed project phase."""

from __future__ import annotations

import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys


def run(command: list[str]) -> str:
    completed = subprocess.run(
        command,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return completed.stdout.strip()


def require_path(name: str, *, executable: bool = False) -> str:
    raw_value = os.environ.get(name)
    if not raw_value:
        raise RuntimeError(f"environment variable is not set: {name}")
    path = Path(raw_value)
    if not path.is_file():
        raise RuntimeError(f"required file does not exist: {path}")
    if executable and not os.access(path, os.X_OK):
        raise RuntimeError(f"required file is not executable: {path}")
    return str(path)


def main() -> int:
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError(f"expected ROS_DISTRO=jazzy, got {os.environ.get('ROS_DISTRO')!r}")

    ros_setup = require_path("ROS_SETUP")
    isaac_python = require_path("ISAAC_SIM_PYTHON", executable=True)
    environment_usd = require_path("KUJIALE_USD")
    robot_usd = require_path("JACKAL_USD")
    hawk_usd = require_path("HAWK_USD")

    gpu_fields = run(
        [
            "nvidia-smi",
            "--query-gpu=name,driver_version,memory.total",
            "--format=csv,noheader,nounits",
        ]
    ).splitlines()[0]
    gpu_name, driver_version, memory_mib = [part.strip() for part in gpu_fields.split(",")]
    if "RTX 4090" not in gpu_name:
        raise RuntimeError(f"expected an RTX 4090, got {gpu_name}")

    ros_prefix = run(["ros2", "pkg", "prefix", "rclcpp"])
    nav2_prefix = run(["ros2", "pkg", "prefix", "nav2_bringup"])
    isaac_version = run(
        [
            isaac_python,
            "-c",
            "import importlib.metadata as m; print(m.version('isaacsim'))",
        ]
    )
    if not isaac_version.startswith("6.0.1"):
        raise RuntimeError(f"expected Isaac Sim 6.0.1, got {isaac_version}")

    isaac_ros_packages = [
        "isaac_ros_visual_slam",
        "isaac_ros_nvblox",
        "isaac_ros_visual_global_localization",
    ]
    installed_isaac_ros: dict[str, bool] = {}
    for package in isaac_ros_packages:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        installed_isaac_ros[package] = result.returncode == 0

    report = {
        "python": sys.version.split()[0],
        "ros": {"distro": "jazzy", "setup": ros_setup, "rclcpp_prefix": ros_prefix},
        "nav2_prefix": nav2_prefix,
        "isaac_sim": {"python": isaac_python, "version": isaac_version},
        "gpu": {
            "name": gpu_name,
            "driver": driver_version,
            "memory_mib": int(memory_mib),
        },
        "assets": {
            "environment": environment_usd,
            "robot": robot_usd,
            "hawk": hawk_usd,
        },
        "isaac_ros_installed": installed_isaac_ros,
        "phase1_environment_ready": all(installed_isaac_ros.values()),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError, importlib.metadata.PackageNotFoundError) as exc:
        print(f"environment check failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
