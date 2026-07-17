#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

LOG_DIR="${PROJECT_ROOT}/data/logs/stage1"
mkdir -p "${LOG_DIR}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="${LOG_DIR}/ros-bridge-${RUN_ID}.log"
SIM_PID=""

cleanup() {
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill -INT -- "-${SIM_PID}" 2>/dev/null || true
    for _ in {1..40}; do
      kill -0 "${SIM_PID}" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-${SIM_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

info "Starting the isolated Isaac Sim ROS Bridge smoke publisher"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/smoke_ros_bridge.py" \
  --duration 90 >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

info "Waiting for actual Clock, Image, and CameraInfo messages"
if ! python3 "${PROJECT_ROOT}/tools/check_ros_bridge.py" --timeout 85; then
  tail -n 120 "${SIM_LOG}" >&2 || true
  die "ROS Bridge smoke test failed; full log: ${SIM_LOG}"
fi

info "ROS Bridge smoke test passed; log: ${SIM_LOG}"
