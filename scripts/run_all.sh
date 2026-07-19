#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="kujiale_latest_20260719_160004"
SIM_MODE="--headless"
RVIZ="false"
REPORT=""
RUN_MODE="auto"
ACCEPTANCE_CONFIG="${PROJECT_ROOT}/config/acceptance.yaml"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --auto) RUN_MODE="auto"; shift ;;
    --manual) RUN_MODE="manual"; shift ;;
    --acceptance-config) ACCEPTANCE_CONFIG="${2:?missing config path}"; shift 2 ;;
    --report) REPORT="${2:?missing report path}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_all.sh [--map NAME] [--headless|--gui] [--rviz|--no-rviz] [--auto|--manual] [--report FILE]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

if [[ "${RUN_MODE}" == "manual" ]]; then
  [[ "${SIM_MODE}" == "--gui" ]] || \
    die "manual navigation requires --gui so robot behavior remains visible"
  [[ "${RVIZ}" == "true" ]] || \
    die "manual navigation requires --rviz for 2D Goal Pose"
fi

require_file "${ACCEPTANCE_CONFIG}"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"
if [[ "${RUN_MODE}" == "auto" ]]; then
  CONFIG_MAP_NAME="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["map"]["name"])' "${ACCEPTANCE_CONFIG}")"
  [[ "${MAP_NAME}" == "${CONFIG_MAP_NAME}" ]] || \
    die "automatic goals are locked to map ${CONFIG_MAP_NAME}, not ${MAP_NAME}"
  python3 "${PROJECT_ROOT}/tools/validate_acceptance_routes.py" "${MAP_DIR}" \
    --config "${ACCEPTANCE_CONFIG}" >/dev/null
fi

export ROS_DOMAIN_ID="${NAVIGATION_ROS_DOMAIN_ID:-49}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/navigation/${RUN_ID}"
REPORT="${REPORT:-${PROJECT_ROOT}/data/reports/navigation/navigation-${RUN_ID}.json}"
SIM_STOP="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "$(dirname "${REPORT}")"
SIM_PID=""; NAV_PID=""; DISCOVERY_PID=""

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
  stop_group "${DISCOVERY_PID}"
}
trap cleanup EXIT INT TERM

DISCOVERY_PORT="${NAVIGATION_DISCOVERY_PORT:-11849}"
[[ "${DISCOVERY_PORT}" =~ ^[0-9]+$ ]] || \
  die "NAVIGATION_DISCOVERY_PORT must be an integer"
DISCOVERY_PORT="$((10#${DISCOVERY_PORT}))"
(( DISCOVERY_PORT >= 1024 && DISCOVERY_PORT <= 65535 )) || \
  die "NAVIGATION_DISCOVERY_PORT must be in 1024..65535"
command -v fastdds >/dev/null || die "fastdds discovery executable not found"
if ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q .; then
  die "Fast DDS discovery port ${DISCOVERY_PORT} is already in use"
fi
export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
# This Jazzy build gives ROS_LOCALHOST_ONLY precedence over discovery-server
# mode. The server itself is bound to loopback, so communication stays local.
unset ROS_LOCALHOST_ONLY
info "Starting project-local Fast DDS discovery server on ${ROS_DISCOVERY_SERVER}"
setsid fastdds discovery -i 0 -l 127.0.0.1 -p "${DISCOVERY_PORT}" \
  >"${LOG_DIR}/fastdds-discovery.log" 2>&1 & DISCOVERY_PID=$!
for _ in {1..30}; do
  ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . && break
  alive "${DISCOVERY_PID}" || \
    die "Fast DDS discovery server exited; see ${LOG_DIR}/fastdds-discovery.log"
  sleep 0.1
done
ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . || \
  die "Fast DDS discovery server did not bind UDP port ${DISCOVERY_PORT}"

info "Starting Kujiale/Jackal navigation simulator (${SIM_MODE}) in ROS domain ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${SIM_MODE}" --duration 0 \
  --update-hz "${NAVIGATION_SIM_UPDATE_HZ:-120}" \
  --camera-profile navigation_6cam \
  --reliable-sensor-qos \
  --stop-file "${SIM_STOP}" \
  --report "${LOG_DIR}/simulator.json" >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..240}; do
  grep -Fq JACKAL_SENSORS_READY "${LOG_DIR}/simulator.log" 2>/dev/null && break
  alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq "camera_profile=navigation_6cam streams=6" \
  "${LOG_DIR}/simulator.log" || die "6-camera sensor startup timeout"

nav_args=(--map "${MAP_NAME}")
[[ "${RVIZ}" == "true" ]] && nav_args+=(--rviz) || nav_args+=(--no-rviz)
setsid "${PROJECT_ROOT}/scripts/run_navigation.sh" "${nav_args[@]}" \
  >"${LOG_DIR}/navigation.log" 2>&1 & NAV_PID=$!
sleep 2
alive "${NAV_PID}" || { tail -n 200 "${LOG_DIR}/navigation.log" >&2; die "navigation bringup exited"; }

if [[ "${RUN_MODE}" == "manual" ]]; then
  info "Waiting for cuVGL global localization, map->odom, occupancy map, and Nav2"
  set +e
  ros2 run jackal_experiments manual_navigation_ready --ros-args \
    -p use_sim_time:=true -p action_topic:=/navigate_to_pose -p timeout_s:=180.0
  ready_status=$?
  set -e
  if (( ready_status != 0 )); then
    tail -n 240 "${LOG_DIR}/navigation.log" >&2
    die "manual navigation did not become ready"
  fi
  info "Navigation ready: use RViz 2D Goal Pose; no 2D Pose Estimate is required"
  info "A newer clicked goal replaces the active goal; press Ctrl-C here to stop"
  wait "${NAV_PID}"
  exit $?
fi

GOAL_POSES="$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1])); print("["+",".join(str(v) for g in c["goals"] for v in g["pose"])+"]")' "${ACCEPTANCE_CONFIG}")"
GOAL_TIMEOUT="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["goal_timeout_s"]))' "${ACCEPTANCE_CONFIG}")"
printf -v GOAL_TIMEOUT_S '%.3f' "${GOAL_TIMEOUT}"
test_args=(--ros-args -p use_sim_time:=true -p result_path:="${REPORT}"
  -p require_rviz:="${RVIZ}" -p goal_timeout_s:="${GOAL_TIMEOUT_S}"
  -p "goal_poses:=${GOAL_POSES}" -p experiment_class:=kujiale_static_navigation)
set +e
ros2 run jackal_experiments navigation_test_runner "${test_args[@]}" \
  >"${LOG_DIR}/test-runner.log" 2>&1
status=$?
set -e
if (( status != 0 )); then
  tail -n 240 "${LOG_DIR}/navigation.log" >&2
  cat "${LOG_DIR}/test-runner.log" >&2
  die "Kujiale static navigation test failed"
fi
stop_group "${NAV_PID}"; NAV_PID=""
stop_simulator; SIM_PID=""
python3 "${PROJECT_ROOT}/tools/check_stage4_sim_report.py" "${LOG_DIR}/simulator.json"
python3 "${PROJECT_ROOT}/tools/check_navigation_report.py" "${REPORT}"
cp "${REPORT}" "${PROJECT_ROOT}/data/reports/navigation/latest.json"
info "Kujiale/Jackal navigation route passed: ${REPORT}"
