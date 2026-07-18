#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

ACCEPTANCE_CONFIG="${PROJECT_ROOT}/config/acceptance.yaml"
MAP_NAME="kujiale_jackal_8cam"
GOAL_INDEX="0"
ATTEMPT_INDEX="1"
RUN_DIR=""
SIM_MODE="--headless"
while (($#)); do
  case "$1" in
    --config) ACCEPTANCE_CONFIG="${2:?missing config}"; shift 2 ;;
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --goal-index) GOAL_INDEX="${2:?missing goal index}"; shift 2 ;;
    --attempt-index) ATTEMPT_INDEX="${2:?missing attempt index}"; shift 2 ;;
    --run-dir) RUN_DIR="${2:?missing run directory}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_static_trial.sh [--goal-index N] [--attempt-index N] [--run-dir DIR] [--headless|--gui]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
require_file "${ACCEPTANCE_CONFIG}"
[[ "${GOAL_INDEX}" =~ ^[0-9]+$ ]] || die "--goal-index must be non-negative"
[[ "${ATTEMPT_INDEX}" =~ ^[1-9][0-9]*$ ]] || die "--attempt-index must be positive"

CONFIG_MAP_NAME="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["map"]["name"])' "${ACCEPTANCE_CONFIG}")"
[[ "${MAP_NAME}" == "${CONFIG_MAP_NAME}" ]] || \
  die "acceptance config is locked to map ${CONFIG_MAP_NAME}, not ${MAP_NAME}"
GOAL_COUNT="$(python3 -c 'import sys,yaml; print(len(yaml.safe_load(open(sys.argv[1]))["goals"]))' "${ACCEPTANCE_CONFIG}")"
(( GOAL_INDEX < GOAL_COUNT )) || die "goal index ${GOAL_INDEX} is outside 0..$((GOAL_COUNT - 1))"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"
python3 "${PROJECT_ROOT}/tools/validate_acceptance_routes.py" \
  "${MAP_DIR}" --config "${ACCEPTANCE_CONFIG}" >/dev/null
ros2 pkg prefix jackal_bringup >/dev/null
ros2 pkg prefix jackal_experiments >/dev/null

RUN_ID="$(date -u +%Y%m%dT%H%M%S)-static-a${ATTEMPT_INDEX}-g${GOAL_INDEX}-$$"
RUN_DIR="${RUN_DIR:-${PROJECT_ROOT}/data/runs/static-acceptance/${RUN_ID}}"
[[ ! -e "${RUN_DIR}" ]] || die "run directory already exists: ${RUN_DIR}"
mkdir -p "${RUN_DIR}" "${PROJECT_ROOT}/data/locks"
python3 "${PROJECT_ROOT}/tools/create_static_trial_metadata.py" \
  --config "${ACCEPTANCE_CONFIG}" --goal-index "${GOAL_INDEX}" \
  --attempt-index "${ATTEMPT_INDEX}" --output "${RUN_DIR}/metadata.json"
GOAL_POSE="$(python3 -c 'import json,sys; print("["+",".join(str(v) for v in json.load(open(sys.argv[1]))["goal_pose"])+"]")' "${RUN_DIR}/metadata.json")"
GOAL_TIMEOUT="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["goal_timeout_s"]))' "${ACCEPTANCE_CONFIG}")"
STARTUP_TIMEOUT="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["startup_timeout_s"]))' "${ACCEPTANCE_CONFIG}")"
POSITION_TOLERANCE="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["goal_position_tolerance_m"]))' "${ACCEPTANCE_CONFIG}")"
YAW_TOLERANCE="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["goal_yaw_tolerance_deg"]))' "${ACCEPTANCE_CONFIG}")"
MINIMUM_MOTION="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["minimum_ground_truth_motion_m"]))' "${ACCEPTANCE_CONFIG}")"

exec 9>"${PROJECT_ROOT}/data/locks/static-acceptance-trial.lock"
flock -n 9 || die "another static acceptance trial is already running"
export ROS_DOMAIN_ID="$((88 + (ATTEMPT_INDEX - 1) % 100))"
SIM_STOP="${RUN_DIR}/stop-simulator"
SIM_PID=""
NAV_PID=""
DISCOVERY_PID=""
RUNNER_INVOKED="false"
RUNNER_STATUS=99

process_alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
group_alive() { [[ -n "$1" ]] && kill -0 -- "-$1" 2>/dev/null; }
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if group_alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..60}; do group_alive "${pid}" || break; sleep 0.25; done
    group_alive "${pid}" && kill -TERM -- "-${pid}" 2>/dev/null || true
    for _ in {1..20}; do group_alive "${pid}" || break; sleep 0.25; done
    group_alive "${pid}" && kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
stop_simulator() {
  if process_alive "${SIM_PID}"; then
    touch "${SIM_STOP}"
    for _ in {1..240}; do process_alive "${SIM_PID}" || break; sleep 0.25; done
  fi
  stop_group "${SIM_PID}"
}
cleanup() {
  stop_group "${NAV_PID}"
  stop_simulator
  stop_group "${DISCOVERY_PID}"
}
on_exit() {
  local original_status=$?
  trap - EXIT INT TERM
  cleanup
  set +e
  python3 "${PROJECT_ROOT}/tools/finalize_static_trial.py" "${RUN_DIR}" \
    --runner-invoked "${RUNNER_INVOKED}" --runner-exit-code "${RUNNER_STATUS}" \
    >"${RUN_DIR}/finalize.log" 2>&1
  local final_status=$?
  set -e
  if (( final_status == 0 )); then
    info "Static trial passed: ${RUN_DIR}/result.json"
  elif (( final_status == 10 )); then
    info "Static trial is a valid failed sample: ${RUN_DIR}/result.json"
  else
    echo "error: infrastructure-invalid attempt (workflow status ${original_status}): ${RUN_DIR}" >&2
  fi
  exit "${final_status}"
}
trap on_exit EXIT INT TERM

DEFAULT_DISCOVERY_PORT="$((13100 + (ATTEMPT_INDEX - 1) % 100))"
DISCOVERY_PORT="${STATIC_ACCEPTANCE_DISCOVERY_PORT:-${DEFAULT_DISCOVERY_PORT}}"
[[ "${DISCOVERY_PORT}" =~ ^[0-9]+$ ]] || \
  die "STATIC_ACCEPTANCE_DISCOVERY_PORT must be an integer"
DISCOVERY_PORT="$((10#${DISCOVERY_PORT}))"
(( DISCOVERY_PORT >= 1024 && DISCOVERY_PORT <= 65535 )) || \
  die "STATIC_ACCEPTANCE_DISCOVERY_PORT must be in 1024..65535"
command -v fastdds >/dev/null || die "fastdds discovery executable not found"
if ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q .; then
  die "Fast DDS discovery port ${DISCOVERY_PORT} is already in use"
fi
export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
# ROS_LOCALHOST_ONLY takes precedence over discovery-server mode in the ROS 2
# Jazzy/Fast DDS runtime used here. The server itself remains loopback-only.
unset ROS_LOCALHOST_ONLY
info "Starting trial-local Fast DDS discovery server on ${ROS_DISCOVERY_SERVER}"
setsid fastdds discovery -i 0 -l 127.0.0.1 -p "${DISCOVERY_PORT}" \
  >"${RUN_DIR}/fastdds-discovery.log" 2>&1 & DISCOVERY_PID=$!
for _ in {1..30}; do
  ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . && break
  process_alive "${DISCOVERY_PID}" || \
    die "Fast DDS discovery server exited; see ${RUN_DIR}/fastdds-discovery.log"
  sleep 0.1
done
ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . || \
  die "Fast DDS discovery server did not bind UDP port ${DISCOVERY_PORT}"

info "Starting static Kujiale attempt ${ATTEMPT_INDEX}, goal ${GOAL_INDEX}, ROS domain ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${SIM_MODE}" --duration 0 --camera-profile navigation_6cam \
  --reliable-sensor-qos \
  --stop-file "${SIM_STOP}" --report "${RUN_DIR}/simulator.json" \
  >"${RUN_DIR}/simulator.log" 2>&1 & SIM_PID=$!
STARTUP_POLLS="$(python3 -c 'import math,sys; print(math.ceil(float(sys.argv[1])*2))' "${STARTUP_TIMEOUT}")"
for ((poll=0; poll<STARTUP_POLLS; poll++)); do
  grep -Fq "camera_profile=navigation_6cam streams=6" \
    "${RUN_DIR}/simulator.log" 2>/dev/null && break
  process_alive "${SIM_PID}" || die "simulator exited during startup"
  sleep 0.5
done
grep -Fq "camera_profile=navigation_6cam streams=6" \
  "${RUN_DIR}/simulator.log" || die "6-camera simulator startup timeout"

setsid "${PROJECT_ROOT}/scripts/run_navigation.sh" --map "${MAP_NAME}" --no-rviz \
  >"${RUN_DIR}/navigation-bringup.log" 2>&1 & NAV_PID=$!
sleep 3
process_alive "${NAV_PID}" || die "navigation bringup exited before the trial"

RUNNER_INVOKED="true"
runner_args=(--ros-args -p use_sim_time:=true
  -p result_path:="${RUN_DIR}/navigation.json"
  -p "goal_poses:=${GOAL_POSE}" -p goal_timeout_s:="${GOAL_TIMEOUT}"
  -p require_rviz:=false -p experiment_class:=kujiale_static_acceptance
  -p trajectory_path:="${RUN_DIR}/trajectory.csv"
  -p command_trace_path:="${RUN_DIR}/command_trace.csv"
  -p goal_xy_tolerance_m:="${POSITION_TOLERANCE}"
  -p goal_yaw_tolerance_deg:="${YAW_TOLERANCE}"
  -p minimum_ground_truth_motion_m:="${MINIMUM_MOTION}")
set +e
ros2 run jackal_experiments navigation_test_runner "${runner_args[@]}" \
  >"${RUN_DIR}/navigation-runner.log" 2>&1
RUNNER_STATUS=$?
set -e
exit 0
