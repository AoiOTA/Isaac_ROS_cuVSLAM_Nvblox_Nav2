#!/usr/bin/env python3
"""Verify the Phase 1 Isaac ROS bare-metal runtime with executable smoke tests."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_ROS_PACKAGES = (
    "isaac_ros_visual_slam",
    "isaac_ros_nvblox",
    "isaac_ros_visual_global_localization",
    "isaac_ros_visual_mapping",
    "isaac_mapping_ros",
)


def run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=check, text=True, capture_output=True)


def package_prefix(name: str) -> Path | None:
    result = run(["ros2", "pkg", "prefix", name], check=False)
    return Path(result.stdout.strip()) if result.returncode == 0 else None


def package_version(prefix: Path, name: str) -> str:
    package_xml = prefix / "share" / name / "package.xml"
    root = ET.parse(package_xml).getroot()
    return root.findtext("version", default="unknown")


def check_tensorrt_engine() -> dict[str, object]:
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.ERROR)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    tensor = network.add_input("input", trt.float32, (1, 1))
    layer = network.add_identity(tensor)
    network.mark_output(layer.get_output(0))
    config = builder.create_builder_config()
    engine = builder.build_serialized_network(network, config)
    if engine is None or engine.nbytes == 0:
        raise RuntimeError("TensorRT failed to serialize a trivial engine")
    return {"version": trt.__version__, "serialized_engine_bytes": engine.nbytes}


def debian_version(package: str) -> str | None:
    result = run(["dpkg-query", "-W", "-f=${db:Status-Abbrev}\t${Version}", package], check=False)
    if result.returncode != 0:
        return None
    status, version = result.stdout.strip().split("\t", maxsplit=1)
    return version if status == "ii " else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--quick", action="store_true", help="Skip CUDA compilation and TensorRT engine build"
    )
    args = parser.parse_args()

    failures: list[str] = []
    report: dict[str, object] = {}

    driver = run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
        check=False,
    )
    report["gpu"] = driver.stdout.strip()
    if driver.returncode != 0:
        failures.append("nvidia-smi failed")

    nvcc = shutil.which("nvcc")
    if nvcc is None:
        failures.append("nvcc is unavailable")
    else:
        nvcc_result = run([nvcc, "--version"])
        report["nvcc"] = nvcc_result.stdout.strip().splitlines()[-1]
        if "release 13.0" not in nvcc_result.stdout:
            failures.append("CUDA Toolkit is not release 13.0")

    ros_packages: dict[str, object] = {}
    for package in EXPECTED_ROS_PACKAGES:
        prefix = package_prefix(package)
        if prefix is None:
            failures.append(f"ROS package missing: {package}")
            continue
        version = package_version(prefix, package)
        ros_packages[package] = {"prefix": str(prefix), "version": version}
        if not version.startswith("4.5.0"):
            failures.append(f"{package} is {version}, expected 4.5.0")
    report["ros_packages"] = ros_packages

    debian_packages = {
        package: debian_version(package)
        for package in ("cuda-toolkit-13-0", "tensorrt", "isaac-ros-cli")
    }
    report["debian_packages"] = debian_packages
    cuda_debian_version = debian_packages["cuda-toolkit-13-0"]
    if cuda_debian_version is None or not cuda_debian_version.startswith("13.0."):
        failures.append(f"cuda-toolkit-13-0 package has unexpected version: {cuda_debian_version}")
    tensorrt_debian_version = debian_packages["tensorrt"]
    if tensorrt_debian_version is None or not tensorrt_debian_version.startswith(
        "10.13.3.9-"
    ):
        failures.append(f"tensorrt package has unexpected version: {tensorrt_debian_version}")

    ldconfig = run(["ldconfig", "-p"], check=False).stdout
    for library in ("libcudart.so", "libnvinfer.so"):
        if library not in ldconfig:
            failures.append(f"shared library missing from linker cache: {library}")

    if not args.quick and nvcc:
        with tempfile.TemporaryDirectory(prefix="stage1-cuda-") as temp_dir:
            binary = Path(temp_dir) / "cuda_smoke"
            compile_result = run(
                [nvcc, str(ROOT / "tests/smoke/cuda_smoke.cu"), "-o", str(binary)], check=False
            )
            if compile_result.returncode != 0:
                failures.append(f"CUDA smoke compilation failed: {compile_result.stderr.strip()}")
            else:
                cuda_result = run([str(binary)], check=False)
                report["cuda_smoke"] = cuda_result.stdout.strip()
                if cuda_result.returncode != 0:
                    failures.append(f"CUDA smoke execution failed with {cuda_result.returncode}")

        try:
            trt_report = check_tensorrt_engine()
            report["tensorrt"] = trt_report
            if str(trt_report["version"]) != "10.13.3.9":
                failures.append(f"TensorRT is {trt_report['version']}, expected 10.13.3.9")
        except Exception as exc:  # noqa: BLE001 - diagnostics must collect all failures
            failures.append(f"TensorRT smoke failed: {exc}")

    report["ok"] = not failures
    report["failures"] = failures
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
