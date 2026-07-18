#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="warehouse_v2_front"
SIM_MODE="--headless"
RVIZ="false"
REPORT=""
SCENARIO="warehouse_manual_static"
RUN_MODE="manual"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --manual) RUN_MODE="manual"; shift ;;
    --auto) RUN_MODE="auto"; shift ;;
    --report) REPORT="${2:?missing report path}"; shift 2 ;;
    --scenario) SCENARIO="${2:?missing static scenario profile}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_phase9.sh [--map NAME] [--headless|--gui] [--rviz|--no-rviz] [--manual|--auto] [--scenario NAME] [--report FILE]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

if [[ "${RUN_MODE}" == "manual" ]]; then
  [[ "${RVIZ}" == "true" ]] || \
    die "manual mode requires --rviz; use --auto for the scripted acceptance run"
  [[ -z "${REPORT}" ]] || die "--report is only valid with --auto"
  [[ -z "${PHASE9_GOAL_POSES:-}" ]] || \
    die "PHASE9_GOAL_POSES is only valid with --auto"
fi

export ROS_DOMAIN_ID="${PHASE9_ROS_DOMAIN_ID:-59}"
RUN_ID="$(date -u +%Y%m%dT%H%M%S)-$$"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage9/${RUN_ID}"
if [[ "${RUN_MODE}" == "auto" ]]; then
  REPORT="${REPORT:-${PROJECT_ROOT}/data/reports/phase9/navigation-${RUN_ID}.json}"
fi
SIM_REPORT="${LOG_DIR}/simulator.json"
SIM_STOP="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "$(dirname "${REPORT}")"
DISCOVERY_PID=""; SIM_PID=""; NAV_PID=""

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
    group_alive "${pid}" && kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
stop_simulator() {
  if alive "${SIM_PID}"; then
    touch "${SIM_STOP}"
    for _ in {1..160}; do alive "${SIM_PID}" || break; sleep 0.25; done
  fi
  stop_group "${SIM_PID}"
}
cleanup() {
  stop_group "${NAV_PID}"
  stop_simulator
  stop_group "${DISCOVERY_PID}"
}
interrupted() {
  info "Stopping the Stage 9 navigation stack"
  exit 0
}
trap cleanup EXIT
trap interrupted INT TERM

DISCOVERY_PORT="${PHASE9_DISCOVERY_PORT:-11859}"
export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
# ROS_LOCALHOST_ONLY forces Fast DDS back to SIMPLE discovery on this Jazzy
# build and therefore overrides ROS_DISCOVERY_SERVER.  The server itself is
# bound exclusively to loopback, preserving the project's local-only policy.
unset ROS_LOCALHOST_ONLY
info "Starting project-local Fast DDS discovery server on ${ROS_DISCOVERY_SERVER}"
setsid fastdds discovery -i 0 -l 127.0.0.1 -p "${DISCOVERY_PORT}" \
  >"${LOG_DIR}/fastdds-discovery.log" 2>&1 & DISCOVERY_PID=$!
for _ in {1..30}; do
  if ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q .; then break; fi
  alive "${DISCOVERY_PID}" || \
    die "Fast DDS discovery server exited; see ${LOG_DIR}/fastdds-discovery.log"
  sleep 0.1
done
ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . || \
  die "Fast DDS discovery server did not bind UDP port ${DISCOVERY_PORT}"

if [[ "${RUN_MODE}" == "manual" ]]; then
  SIM_DURATION_S="${PHASE9_SIM_DURATION_S:-0}"
else
  SIM_DURATION_S="${PHASE9_SIM_DURATION_S:-900}"
fi
info "Starting Stage 9 simulator (${SIM_MODE}), mode=${RUN_MODE}, profile=${SCENARIO}, ROS domain=${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${SIM_MODE}" --duration "${SIM_DURATION_S}" \
  --update-hz "${PHASE9_SIM_UPDATE_HZ:-120}" \
  --front-image-width 1280 --front-image-height 800 \
  --front-image-rate-hz "${PHASE9_FRONT_IMAGE_RATE_HZ:-10}" \
  --reliable-sensor-qos \
  --static-profile "${SCENARIO}" \
  --stop-file "${SIM_STOP}" --report "${SIM_REPORT}" \
  >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..240}; do
  grep -Fq JACKAL_SENSORS_READY "${LOG_DIR}/simulator.log" 2>/dev/null && break
  alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq JACKAL_SENSORS_READY "${LOG_DIR}/simulator.log" || die "sensor startup timeout"

nav_args=(--map "${MAP_NAME}")
[[ "${RVIZ}" == "true" ]] && nav_args+=(--rviz) || nav_args+=(--no-rviz)
setsid "${PROJECT_ROOT}/scripts/run_phase9_navigation.sh" "${nav_args[@]}" \
  >"${LOG_DIR}/navigation.log" 2>&1 & NAV_PID=$!
sleep 2
alive "${NAV_PID}" || {
  tail -n 240 "${LOG_DIR}/navigation.log" >&2
  die "Stage 9 navigation bringup exited"
}

if [[ "${RUN_MODE}" == "manual" ]]; then
  info "Manual navigation is running; wait for the map and robot to appear in RViz"
  info "Use the RViz '2D Goal Pose' tool to click a position and drag the heading arrow"
  info "Each new pose replaces the active manual goal; press Ctrl-C here to stop"
  info "Logs: ${LOG_DIR}"
  while alive "${NAV_PID}" && alive "${SIM_PID}"; do
    sleep 0.5
  done
  if ! alive "${SIM_PID}"; then
    set +e
    wait "${SIM_PID}" 2>/dev/null
    status=$?
    set -e
    SIM_PID=""
    if (( status != 0 )); then
      tail -n 120 "${LOG_DIR}/simulator.log" >&2
      die "Stage 9 simulator exited unexpectedly; logs: ${LOG_DIR}"
    fi
    info "Simulator closed; stopping the manual navigation stack"
    exit 0
  fi
  set +e
  wait "${NAV_PID}" 2>/dev/null
  status=$?
  set -e
  NAV_PID=""
  tail -n 240 "${LOG_DIR}/navigation.log" >&2
  die "Stage 9 navigation bringup exited with status ${status}; logs: ${LOG_DIR}"
fi

printf -v GOAL_TIMEOUT_S '%.3f' "${PHASE9_GOAL_TIMEOUT_S:-220.0}"
test_args=(--ros-args -p use_sim_time:=true -p result_path:="${REPORT}"
  -p require_rviz:="${RVIZ}" -p goal_timeout_s:="${GOAL_TIMEOUT_S}"
  -p action_topic:=/navigate_to_pose_resilient
  -p nvblox_slice_topic:=/nvblox_node/combined_map_slice
  -p phase9_mode:=true -p force_relocalization:=true
  -p force_relocalization_delay_s:="${PHASE9_RECOVERY_FALLBACK_DELAY_S:-45.0}"
  -p force_relocalization_distance_m:="${PHASE9_RECOVERY_DISTANCE_M:-0.15}"
  -p require_surround_cameras:=false)
if [[ -n "${PHASE9_GOAL_POSES:-}" ]]; then
  test_args+=(-p "goal_poses:=${PHASE9_GOAL_POSES}")
fi
set +e
ros2 run jackal_experiments navigation_test_runner "${test_args[@]}" \
  >"${LOG_DIR}/test-runner.log" 2>&1
status=$?
set -e
if (( status != 0 )); then
  tail -n 300 "${LOG_DIR}/navigation.log" >&2
  cat "${LOG_DIR}/test-runner.log" >&2
  die "Stage 9 navigation test failed; logs: ${LOG_DIR}"
fi

stop_group "${NAV_PID}"; NAV_PID=""
stop_simulator; SIM_PID=""
python3 "${PROJECT_ROOT}/tools/check_stage9_sim_report.py" "${SIM_REPORT}"
python3 "${PROJECT_ROOT}/tools/check_phase9_report.py" "${REPORT}"
cp "${REPORT}" "${PROJECT_ROOT}/data/reports/phase9/latest.json"
cp "${SIM_REPORT}" "${PROJECT_ROOT}/data/reports/phase9/simulator-latest.json"
info "Stage 9 static-obstacle navigation passed: ${REPORT}"
info "Logs: ${LOG_DIR}"
