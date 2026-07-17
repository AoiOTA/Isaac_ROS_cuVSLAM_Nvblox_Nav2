#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="warehouse_v1"
SIM_MODE="--headless"
RVIZ="false"
REPORT=""
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --report) REPORT="${2:?missing report path}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_all.sh [--map NAME] [--headless|--gui] [--rviz|--no-rviz] [--report FILE]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

export ROS_DOMAIN_ID="${PHASE8_ROS_DOMAIN_ID:-49}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage8/${RUN_ID}"
REPORT="${REPORT:-${PROJECT_ROOT}/data/reports/phase8/navigation-${RUN_ID}.json}"
SIM_STOP="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "$(dirname "${REPORT}")"
SIM_PID=""; NAV_PID=""

alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
group_alive() { [[ -n "$1" ]] && kill -0 -- "-$1" 2>/dev/null; }
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if group_alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..40}; do group_alive "${pid}" || break; sleep 0.25; done
    if group_alive "${pid}"; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
      for _ in {1..20}; do group_alive "${pid}" || break; sleep 0.25; done
    fi
    if group_alive "${pid}"; then
      info "forcing remaining process group ${pid} to exit"
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}
stop_simulator() {
  if alive "${SIM_PID}"; then
    touch "${SIM_STOP}"
    for _ in {1..120}; do alive "${SIM_PID}" || break; sleep 0.25; done
  fi
  stop_group "${SIM_PID}"
}
cleanup() {
  stop_group "${NAV_PID}"
  stop_simulator
}
trap cleanup EXIT INT TERM

info "Starting Stage 8 simulator (${SIM_MODE}) in ROS domain ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${SIM_MODE}" --duration "${PHASE8_SIM_DURATION_S:-900}" \
  --update-hz "${PHASE8_SIM_UPDATE_HZ:-120}" \
  --stop-file "${SIM_STOP}" \
  --report "${LOG_DIR}/simulator.json" >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..240}; do
  grep -Fq NOVA_CARTER_SENSORS_READY "${LOG_DIR}/simulator.log" 2>/dev/null && break
  alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq NOVA_CARTER_SENSORS_READY "${LOG_DIR}/simulator.log" || die "sensor startup timeout"

nav_args=(--map "${MAP_NAME}")
[[ "${RVIZ}" == "true" ]] && nav_args+=(--rviz) || nav_args+=(--no-rviz)
setsid "${PROJECT_ROOT}/scripts/run_navigation.sh" "${nav_args[@]}" \
  >"${LOG_DIR}/navigation.log" 2>&1 & NAV_PID=$!
sleep 2
alive "${NAV_PID}" || { tail -n 200 "${LOG_DIR}/navigation.log" >&2; die "navigation bringup exited"; }

printf -v GOAL_TIMEOUT_S '%.3f' "${PHASE8_GOAL_TIMEOUT_S:-180.0}"
test_args=(--ros-args -p use_sim_time:=true -p result_path:="${REPORT}"
  -p require_rviz:="${RVIZ}" -p goal_timeout_s:="${GOAL_TIMEOUT_S}")
if [[ -n "${PHASE8_GOAL_POSES:-}" ]]; then
  test_args+=(-p "goal_poses:=${PHASE8_GOAL_POSES}")
fi
set +e
ros2 run nova_carter_experiments navigation_test_runner "${test_args[@]}" \
  >"${LOG_DIR}/test-runner.log" 2>&1
status=$?
set -e
if (( status != 0 )); then
  tail -n 240 "${LOG_DIR}/navigation.log" >&2
  cat "${LOG_DIR}/test-runner.log" >&2
  die "Stage 8 navigation test failed"
fi
stop_group "${NAV_PID}"; NAV_PID=""
stop_simulator; SIM_PID=""
python3 "${PROJECT_ROOT}/tools/check_stage4_sim_report.py" "${LOG_DIR}/simulator.json"
python3 "${PROJECT_ROOT}/tools/check_phase8_report.py" "${REPORT}"
cp "${REPORT}" "${PROJECT_ROOT}/data/reports/phase8/latest.json"
info "Stage 8 full pipeline passed: ${REPORT}"
