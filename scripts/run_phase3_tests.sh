#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

# Keep automated experiments separate from the unrelated simulator already running
# on the machine. Public project commands remain on the configured domain 42.
export ROS_DOMAIN_ID="${PHASE3_TEST_ROS_DOMAIN_ID:-43}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage3/${RUN_ID}"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase3"
SIM_REPORT="${LOG_DIR}/simulator.json"
RESULT="${REPORT_DIR}/run-${RUN_ID}.json"
SIM_LOG="${LOG_DIR}/simulator.log"
CONTROL_LOG="${LOG_DIR}/control.log"
TEST_LOG="${LOG_DIR}/motion-test.log"
SIM_STOP_FILE="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "${REPORT_DIR}"

SIM_PID=""
CONTROL_PID=""

process_alive() {
  local pid="$1"
  local state
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
    for _ in {1..60}; do
      process_alive "${pid}" || break
      sleep 0.25
    done
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_group "${CONTROL_PID}"
  if process_alive "${SIM_PID}"; then
    touch "${SIM_STOP_FILE}"
    for _ in {1..240}; do
      process_alive "${SIM_PID}" || break
      sleep 0.25
    done
  fi
  stop_group "${SIM_PID}"
}
trap cleanup EXIT INT TERM

info "Phase 3 test domain: ${ROS_DOMAIN_ID}"
info "Starting isolated Standalone simulator"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  --headless \
  --duration 150 \
  --stop-file "${SIM_STOP_FILE}" \
  --report "${SIM_REPORT}" >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

info "Waiting for the simulator runtime graphs"
sim_ready=false
for _ in {1..90}; do
  if ! process_alive "${SIM_PID}"; then
    tail -n 160 "${SIM_LOG}" >&2 || true
    die "simulator exited during startup"
  fi
  if grep -Fq "NOVA_CARTER_CONTROL_READY" "${SIM_LOG}" 2>/dev/null; then
    sim_ready=true
    break
  fi
  sleep 0.5
done
[[ "${sim_ready}" == true ]] || die "timed out waiting for runtime graphs; log: ${SIM_LOG}"

info "Starting Command Guard and wheel odometry"
setsid ros2 launch nova_carter_control control.launch.py >"${CONTROL_LOG}" 2>&1 &
CONTROL_PID=$!

info "Running straight, reverse, spin, arc, S-curve, sharp-turn, and safety tests"
set +e
ros2 run nova_carter_experiments motion_test_runner \
  --ros-args \
  -p use_sim_time:=true \
  -p result_path:="${RESULT}" 2>&1 | tee "${TEST_LOG}"
test_status=${PIPESTATUS[0]}
set -e

stop_group "${CONTROL_PID}"
CONTROL_PID=""
touch "${SIM_STOP_FILE}"
for _ in {1..240}; do
  process_alive "${SIM_PID}" || break
  sleep 0.25
done
wait "${SIM_PID}" 2>/dev/null || true
SIM_PID=""

[[ -f "${SIM_REPORT}" ]] || die "simulator did not write its report: ${SIM_REPORT}"
python3 "${PROJECT_ROOT}/tools/check_stage3_sim_report.py" "${SIM_REPORT}"
install -m 0644 "${SIM_REPORT}" "${PROJECT_ROOT}/data/logs/stage3/latest.json"
install -m 0644 "${SIM_REPORT}" "${REPORT_DIR}/simulator-latest.json"

if [[ ${test_status} -ne 0 ]]; then
  tail -n 120 "${CONTROL_LOG}" >&2 || true
  die "Phase 3 motion runner failed; result: ${RESULT}"
fi
python3 "${PROJECT_ROOT}/tools/check_phase3_motion.py" "${RESULT}"
install -m 0644 "${RESULT}" "${REPORT_DIR}/latest.json"
info "Phase 3 passed; result: ${RESULT}"
