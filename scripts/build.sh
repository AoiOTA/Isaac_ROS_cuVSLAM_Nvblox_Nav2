#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

load_ros_environment

echo "Checking shell scripts"
while IFS= read -r script; do
  bash -n "${script}"
done < <(find "${PROJECT_ROOT}/scripts" -type f -name '*.sh' -print | sort)

echo "Checking Python syntax"
python3 -m compileall -q "${PROJECT_ROOT}/tools" "${PROJECT_ROOT}/isaac_sim"

echo "Checking project configuration syntax"
python3 - "${PROJECT_ROOT}" <<'PY'
from pathlib import Path
import sys
import tomllib
import yaml

root = Path(sys.argv[1])
with (root / "pyproject.toml").open("rb") as stream:
    tomllib.load(stream)
for path in sorted((root / "config").glob("*.yaml")):
    with path.open("r", encoding="utf-8") as stream:
        yaml.safe_load(stream)
print("project_config=ok")
PY

echo "Checking Fast DDS XML"
python3 - "${FASTRTPS_DEFAULT_PROFILES_FILE}" <<'PY'
import sys
import xml.etree.ElementTree as ET

ET.parse(sys.argv[1])
print(f"fastdds_xml=ok path={sys.argv[1]}")
PY

echo "Checking host, ROS, GPU, and Isaac Sim environment"
python3 "${PROJECT_ROOT}/tools/check_environment.py"

echo "Opening fixed USD assets with the Isaac Sim Python runtime"
"${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/tools/check_assets.py"

if find "${PROJECT_ROOT}/ros2_ws/src" -name package.xml -print -quit | grep -q .; then
  echo "Building ROS workspace"
  colcon build \
    --base-paths "${PROJECT_ROOT}/ros2_ws/src" \
    --build-base "${PROJECT_ROOT}/ros2_ws/build" \
    --install-base "${PROJECT_ROOT}/ros2_ws/install" \
    --log-base "${PROJECT_ROOT}/ros2_ws/log" \
    --symlink-install
else
  echo "ROS workspace has no packages through phase 2; colcon build skipped"
fi

if ros2 pkg prefix isaac_ros_visual_slam >/dev/null 2>&1; then
  echo "Checking installed Phase 1 runtime"
  python3 "${PROJECT_ROOT}/tools/check_stage1.py" --quick
else
  echo "Isaac ROS is not installed; Phase 1 runtime check skipped"
fi

echo "Project build and available environment checks passed"
