#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

export ROS_DOMAIN_ID="${PHASE5_TEST_ROS_DOMAIN_ID:-45}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage5/${RUN_ID}"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase5"
MAP_DIR="${PROJECT_ROOT}/data/maps/phase5_validation/cuvslam"
TRACKING_DURATION="${PHASE5_TRACKING_DURATION_SIM_SECONDS:-122.0}"
SIM_REPORT="${LOG_DIR}/simulator.json"
RESULT="${REPORT_DIR}/run-${RUN_ID}.json"
SIM_LOG="${LOG_DIR}/simulator.log"
BRINGUP_LOG="${LOG_DIR}/bringup.log"
TEST_LOG="${LOG_DIR}/visual-slam-test.log"
SIM_STOP_FILE="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "${REPORT_DIR}" "$(dirname "${MAP_DIR}")"

SIM_PID=""
BRINGUP_PID=""

process_alive() {
  local pid="$1" state
  [[ -n "${pid}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != Z* ]]
}

stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if process_alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..80}; do process_alive "${pid}" || break; sleep 0.25; done
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_group "${BRINGUP_PID}"
  if process_alive "${SIM_PID}"; then
    touch "${SIM_STOP_FILE}"
    for _ in {1..240}; do process_alive "${SIM_PID}" || break; sleep 0.25; done
  fi
  stop_group "${SIM_PID}"
}
trap cleanup EXIT INT TERM

info "Phase 5 isolated test domain: ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  --headless --duration 480 --stop-file "${SIM_STOP_FILE}" --report "${SIM_REPORT}" \
  >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

info "Waiting for Standalone runtime sensor graphs"
ready=false
for _ in {1..180}; do
  if ! process_alive "${SIM_PID}"; then
    tail -n 180 "${SIM_LOG}" >&2 || true
    die "simulator exited during Phase 5 startup"
  fi
  if grep -Fq "NOVA_CARTER_SENSORS_READY" "${SIM_LOG}" 2>/dev/null; then
    ready=true
    break
  fi
  sleep 0.5
done
[[ "${ready}" == true ]] || die "timed out waiting for sensor graphs: ${SIM_LOG}"

info "Starting Phase 3 control, sensor TF, normalization, and cuVSLAM"
setsid ros2 launch nova_carter_bringup phase5.launch.py >"${BRINGUP_LOG}" 2>&1 &
BRINGUP_PID=$!

info "Waiting for cuVSLAM services"
ready=false
for _ in {1..180}; do
  if ! process_alive "${BRINGUP_PID}"; then
    tail -n 180 "${BRINGUP_LOG}" >&2 || true
    die "Phase 5 bringup exited during startup"
  fi
  if ros2 service type /visual_slam/save_map 2>/dev/null \
      | grep -Fq "isaac_ros_visual_slam_interfaces/srv/FilePath"; then
    ready=true
    break
  fi
  sleep 0.5
done
[[ "${ready}" == true ]] || die "timed out waiting for cuVSLAM: ${BRINGUP_LOG}"

info "Running the ${TRACKING_DURATION}-second bidirectional S-course and map service acceptance"
set +e
ros2 run nova_carter_experiments visual_slam_test_runner --ros-args \
  -p use_sim_time:=true -p result_path:="${RESULT}" -p map_path:="${MAP_DIR}" \
  -p tracking_duration_sim_seconds:="${TRACKING_DURATION}" \
  2>&1 | tee "${TEST_LOG}"
test_status=${PIPESTATUS[0]}
set -e

stop_group "${BRINGUP_PID}"
BRINGUP_PID=""
touch "${SIM_STOP_FILE}"
for _ in {1..240}; do process_alive "${SIM_PID}" || break; sleep 0.25; done
wait "${SIM_PID}" 2>/dev/null || true
SIM_PID=""

[[ -f "${SIM_REPORT}" ]] || die "simulator report is missing: ${SIM_REPORT}"
python3 "${PROJECT_ROOT}/tools/check_stage4_sim_report.py" "${SIM_REPORT}"
install -m 0644 "${SIM_REPORT}" "${REPORT_DIR}/simulator-latest.json"
if [[ ${test_status} -ne 0 ]]; then
  tail -n 200 "${BRINGUP_LOG}" >&2 || true
  die "Phase 5 visual SLAM runner failed; result: ${RESULT}"
fi
python3 "${PROJECT_ROOT}/tools/check_phase5_vslam.py" "${RESULT}"
install -m 0644 "${RESULT}" "${REPORT_DIR}/latest.json"
info "Phase 5 passed; result: ${RESULT}"
