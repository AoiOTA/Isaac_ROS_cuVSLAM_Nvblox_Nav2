#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

export ROS_DOMAIN_ID="${PHASE4_TEST_ROS_DOMAIN_ID:-44}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage4/${RUN_ID}"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase4"
SIM_REPORT="${LOG_DIR}/simulator.json"
RESULT="${REPORT_DIR}/run-${RUN_ID}.json"
SIM_LOG="${LOG_DIR}/simulator.log"
BRINGUP_LOG="${LOG_DIR}/bringup.log"
TEST_LOG="${LOG_DIR}/sensor-test.log"
PAYLOAD_LOG="${LOG_DIR}/payload-probe.log"
PAYLOAD_RESULT="${LOG_DIR}/payload.json"
SIM_STOP_FILE="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "${REPORT_DIR}"

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
    for _ in {1..60}; do process_alive "${pid}" || break; sleep 0.25; done
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

info "Phase 4 isolated test domain: ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  --headless --duration 240 --stop-file "${SIM_STOP_FILE}" --report "${SIM_REPORT}" \
  >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

info "Waiting for runtime camera, depth, IMU, and control graphs"
ready=false
for _ in {1..180}; do
  if ! process_alive "${SIM_PID}"; then
    tail -n 180 "${SIM_LOG}" >&2 || true
    die "simulator exited during Phase 4 startup"
  fi
  if grep -Fq "NOVA_CARTER_SENSORS_READY" "${SIM_LOG}" 2>/dev/null; then ready=true; break; fi
  sleep 0.5
done
[[ "${ready}" == true ]] || die "timed out waiting for Phase 4 graphs: ${SIM_LOG}"

info "Starting robot_state_publisher, image normalization, and control nodes"
setsid ros2 launch nova_carter_bringup phase4.launch.py >"${BRINGUP_LOG}" 2>&1 &
BRINGUP_PID=$!

info "Exercising stationary, twin arcs, and spin while auditing the full data flow"
set +e
ros2 run nova_carter_experiments sensor_payload_probe --ros-args \
  -p result_path:="${PAYLOAD_RESULT}" 2>&1 | tee "${PAYLOAD_LOG}"
payload_status=${PIPESTATUS[0]}
if [[ ${payload_status} -eq 0 ]]; then
ros2 run nova_carter_experiments sensor_test_runner --ros-args \
  -p use_sim_time:=true -p result_path:="${RESULT}" \
  -p payload_probe_path:="${PAYLOAD_RESULT}" 2>&1 | tee "${TEST_LOG}"
  test_status=${PIPESTATUS[0]}
else
  test_status=${payload_status}
fi
set -e

stop_group "${BRINGUP_PID}"
BRINGUP_PID=""
touch "${SIM_STOP_FILE}"
for _ in {1..240}; do process_alive "${SIM_PID}" || break; sleep 0.25; done
wait "${SIM_PID}" 2>/dev/null || true
SIM_PID=""

[[ -f "${SIM_REPORT}" ]] || die "simulator report is missing: ${SIM_REPORT}"
python3 "${PROJECT_ROOT}/tools/check_stage4_sim_report.py" "${SIM_REPORT}"
install -m 0644 "${SIM_REPORT}" "${PROJECT_ROOT}/data/logs/stage4/latest.json"
install -m 0644 "${SIM_REPORT}" "${REPORT_DIR}/simulator-latest.json"
if [[ ${test_status} -ne 0 ]]; then
  tail -n 160 "${BRINGUP_LOG}" >&2 || true
  die "Phase 4 data-flow runner failed; result: ${RESULT}"
fi
python3 "${PROJECT_ROOT}/tools/check_phase4_sensors.py" "${RESULT}"
install -m 0644 "${RESULT}" "${REPORT_DIR}/latest.json"
info "Phase 4 passed; result: ${RESULT}"
