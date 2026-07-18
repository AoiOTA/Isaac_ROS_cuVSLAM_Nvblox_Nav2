#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v1}"
info "Building and running Stage 8 contract tests"
"${PROJECT_ROOT}/scripts/build.sh"
python3 -m pytest -q \
  "${PROJECT_ROOT}/ros2_ws/src/jackal_control/test" \
  "${PROJECT_ROOT}/ros2_ws/src/jackal_bringup/test" \
  "${PROJECT_ROOT}/ros2_ws/src/jackal_experiments/test"

info "Running live RViz + cuVGL + cuVSLAM + nvblox + Nav2 three-goal acceptance"
"${PROJECT_ROOT}/scripts/run_all.sh" --map "${MAP_NAME}" --headless --rviz

info "Running real Isaac Sim GUI third-person follow-camera smoke test"
"${PROJECT_ROOT}/scripts/run_sim.sh" --gui --duration 8
python3 "${PROJECT_ROOT}/tools/check_follow_camera.py" \
  "${PROJECT_ROOT}/data/logs/stage4/latest.json"

info "Stage 8 tests passed"
