#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

EXPERIMENT_CLASS="dynamic"
SEED="1000"
GOAL_INDEX="0"
GOAL_MODE="selected"
MAP_NAME="warehouse_v2_front"
SIM_MODE="--headless"
RVIZ="false"
FAULT_SEQUENCE=""
RECORD_BAG="true"
RUN_ID=""
RUN_DIR=""
while (($#)); do
  case "$1" in
    --class) EXPERIMENT_CLASS="${2:?missing class}"; shift 2 ;;
    --seed) SEED="${2:?missing seed}"; shift 2 ;;
    --goal-index) GOAL_INDEX="${2:?missing goal index}"; shift 2 ;;
    --all-goals) GOAL_MODE="all"; shift ;;
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --fault-sequence) FAULT_SEQUENCE="${2:?missing fault sequence}"; shift 2 ;;
    --record-bag) RECORD_BAG="true"; shift ;;
    --no-bag) RECORD_BAG="false"; shift ;;
    --run-id) RUN_ID="${2:?missing run id}"; shift 2 ;;
    --run-dir) RUN_DIR="${2:?missing run directory}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_phase10_trial.sh [--class static|dynamic] [--seed N] [--goal-index N|--all-goals] [--fault-sequence CSV] [--record-bag|--no-bag] [--rviz|--no-rviz]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ "${EXPERIMENT_CLASS}" == "static" || "${EXPERIMENT_CLASS}" == "dynamic" ]] || \
  die "--class must be static or dynamic"
[[ "${SEED}" =~ ^[0-9]+$ ]] || die "--seed must be a non-negative integer"
[[ "${GOAL_INDEX}" =~ ^[0-9]+$ ]] || die "--goal-index must be a non-negative integer"

# Only one heavyweight trial may own Isaac Sim, Isaac ROS and the GPU at a
# time. This also protects fixed run-id directories from concurrent writers;
# navigation_sim.py retains its own lower-level process lock.
mkdir -p "${PROJECT_ROOT}/data/locks"
exec 9>"${PROJECT_ROOT}/data/locks/phase10-trial.lock"
flock -n 9 || die "another Stage 10 trial is already running"

export ROS_DOMAIN_ID="${PHASE10_ROS_DOMAIN_ID:-70}"
RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%S)-${EXPERIMENT_CLASS}-${SEED}-$$}"
RUN_DIR="${RUN_DIR:-${PROJECT_ROOT}/data/runs/${RUN_ID}}"
mkdir -p "${RUN_DIR}"
exec {TRIAL_LOCK_FD}>"${RUN_DIR}/.trial.lock"
flock -n "${TRIAL_LOCK_FD}" || die "trial run directory is already active: ${RUN_DIR}"
for existing_output in scenario.yaml scenario.json result.json rosbag; do
  [[ ! -e "${RUN_DIR}/${existing_output}" ]] || \
    die "trial run directory already contains ${existing_output}: ${RUN_DIR}"
done
SCENARIO_YAML="${RUN_DIR}/scenario.yaml"
SCENARIO_JSON="${RUN_DIR}/scenario.json"
SIM_REPORT="${RUN_DIR}/simulator.json"
NAV_REPORT="${RUN_DIR}/navigation.json"
SIM_STOP="${RUN_DIR}/stop-simulator"
GPU_STOP="${RUN_DIR}/stop-gpu"
rm -f "${SIM_STOP}" "${GPU_STOP}"

python3 "${PROJECT_ROOT}/tools/generate_stage10_scenario.py" \
  --class "${EXPERIMENT_CLASS}" --seed "${SEED}" --goal-index "${GOAL_INDEX}" \
  --stage10-config "${PROJECT_ROOT}/config/stage10.yaml" \
  --scenario-template "${PROJECT_ROOT}/config/scenarios.yaml" \
  --output "${SCENARIO_YAML}" --metadata "${SCENARIO_JSON}" \
  >"${RUN_DIR}/scenario-generation.log"
if [[ "${GOAL_MODE}" == "all" ]]; then
  GOAL_POSES="$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1])); print("["+",".join(str(v) for g in c["goals"] for v in g["pose"])+"]")' "${PROJECT_ROOT}/config/stage10.yaml")"
else
  GOAL_POSES="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["goal_poses_ros_parameter"])' "${SCENARIO_JSON}")"
fi

DISCOVERY_PID=""; SIM_PID=""; NAV_PID=""; BAG_PID=""; GPU_PID=""
group_alive() { [[ -n "$1" ]] && kill -0 -- "-$1" 2>/dev/null; }
process_alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
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
  if process_alive "${SIM_PID}"; then
    touch "${SIM_STOP}"
    for _ in {1..200}; do process_alive "${SIM_PID}" || break; sleep 0.25; done
  fi
  stop_group "${SIM_PID}"
}
cleanup() {
  stop_group "${BAG_PID}"
  stop_group "${NAV_PID}"
  stop_simulator
  touch "${GPU_STOP}" 2>/dev/null || true
  stop_group "${GPU_PID}"
  stop_group "${DISCOVERY_PID}"
}
on_exit() {
  local status=$?
  trap - EXIT INT TERM
  cleanup
  if [[ -f "${SCENARIO_YAML}" && ! -f "${RUN_DIR}/result.json" ]]; then
    fallback_bag_arg=()
    [[ "${RECORD_BAG}" == "true" ]] && fallback_bag_arg+=(--require-rosbag)
    python3 "${PROJECT_ROOT}/tools/finalize_stage10_trial.py" "${RUN_DIR}" \
      --runner-exit-code "${RUNNER_STATUS:-99}" \
      --config "${PROJECT_ROOT}/config/stage10.yaml" \
      "${fallback_bag_arg[@]}" \
      >"${RUN_DIR}/finalize.log" 2>&1 || true
  fi
  exit "${status}"
}
trap on_exit EXIT INT TERM

DISCOVERY_PORT="${PHASE10_DISCOVERY_PORT:-11900}"
export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
unset ROS_LOCALHOST_ONLY
SUPER_CLIENT_XML="${RUN_DIR}/fastdds-super-client.xml"
python3 "${PROJECT_ROOT}/tools/write_fastdds_super_client.py" \
  --port "${DISCOVERY_PORT}" --output "${SUPER_CLIENT_XML}"
setsid fastdds discovery -i 0 -l 127.0.0.1 -p "${DISCOVERY_PORT}" \
  >"${RUN_DIR}/fastdds-discovery.log" 2>&1 & DISCOVERY_PID=$!
for _ in {1..30}; do
  ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . && break
  process_alive "${DISCOVERY_PID}" || die "Fast DDS discovery server exited"
  sleep 0.1
done
ss -H -lun "sport = :${DISCOVERY_PORT}" 2>/dev/null | grep -q . || \
  die "Fast DDS discovery server did not bind UDP port ${DISCOVERY_PORT}"

setsid python3 "${PROJECT_ROOT}/tools/record_gpu_metrics.py" \
  --output "${RUN_DIR}/gpu.csv" --stop-file "${GPU_STOP}" \
  --period "${PHASE10_GPU_PERIOD_S:-1.0}" \
  >"${RUN_DIR}/gpu.log" 2>&1 & GPU_PID=$!

sim_args=("${SIM_MODE}" --duration "${PHASE10_SIM_DURATION_S:-600}"
  --update-hz "${PHASE10_SIM_UPDATE_HZ:-120}"
  --front-image-width 1280 --front-image-height 800
  --front-image-rate-hz "${PHASE10_FRONT_IMAGE_RATE_HZ:-10}"
  --reliable-sensor-qos --scenario-config "${SCENARIO_YAML}"
  --stop-file "${SIM_STOP}" --report "${SIM_REPORT}")
if [[ "${EXPERIMENT_CLASS}" == "dynamic" ]]; then
  sim_args+=(--dynamic-profile stage10_trial)
fi
info "Starting Stage 10 ${EXPERIMENT_CLASS} trial seed=${SEED}, goal=${GOAL_MODE}:${GOAL_INDEX}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${sim_args[@]}" >"${RUN_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..300}; do
  grep -Fq NOVA_CARTER_SENSORS_READY "${RUN_DIR}/simulator.log" 2>/dev/null && break
  process_alive "${SIM_PID}" || die "simulator exited; see ${RUN_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq NOVA_CARTER_SENSORS_READY "${RUN_DIR}/simulator.log" || die "sensor startup timeout"

nav_args=(--map "${MAP_NAME}" --no-rviz)
[[ "${RVIZ}" == "true" ]] && nav_args=(--map "${MAP_NAME}" --rviz)
[[ -n "${FAULT_SEQUENCE}" ]] && nav_args+=(--enable-fault-injection)
setsid "${PROJECT_ROOT}/scripts/run_phase10_navigation.sh" "${nav_args[@]}" \
  >"${RUN_DIR}/ros.log" 2>&1 & NAV_PID=$!
sleep 2
process_alive "${NAV_PID}" || die "Stage 10 navigation bringup exited; see ${RUN_DIR}/ros.log"

# The navigation wrapper may replace a partially activated first launch. Keep
# the experiment runner outside that restart window: the lifecycle guard only
# emits this marker after all eight managed Nav2 nodes are active in the final
# clean process instance.
info "Waiting for the final Nav2 lifecycle guard before starting the trial"
for _ in {1..300}; do
  grep -Fq 'NOVA_CARTER_NAV2_LIFECYCLE {"status": "already_active"' \
    "${RUN_DIR}/ros.log" 2>/dev/null && break
  process_alive "${NAV_PID}" || \
    die "navigation exited while waiting for final lifecycle validation"
  sleep 0.5
done
grep -Fq 'NOVA_CARTER_NAV2_LIFECYCLE {"status": "already_active"' \
  "${RUN_DIR}/ros.log" || die "Nav2 lifecycle validation timeout"

if [[ "${RECORD_BAG}" == "true" ]]; then
  info "Waiting for ROS publishers before starting compact MCAP recording"
  for _ in {1..60}; do
    topic_list="$(env -u ROS_DISCOVERY_SERVER \
      FASTRTPS_DEFAULT_PROFILES_FILE="${SUPER_CLIENT_XML}" \
      ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true)"
    if grep -Fxq /clock <<<"${topic_list}" \
      && grep -Fxq /visual_slam/status <<<"${topic_list}" \
      && grep -Fxq /localization/ready <<<"${topic_list}" \
      && grep -Fxq /cmd_vel_sim <<<"${topic_list}"; then
      break
    fi
    process_alive "${NAV_PID}" || die "navigation exited while waiting for bag topics"
    sleep 0.5
  done
  topic_list="$(env -u ROS_DISCOVERY_SERVER \
    FASTRTPS_DEFAULT_PROFILES_FILE="${SUPER_CLIENT_XML}" \
    ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true)"
  # /plan is intentionally recorded, but is emitted only after a navigation
  # goal exists and therefore must never gate starting the recorder.
  for required_topic in /clock /visual_slam/status /localization/ready /cmd_vel_sim; do
    grep -Fxq "${required_topic}" <<<"${topic_list}" || \
      die "required bag topic was not discovered: ${required_topic}"
  done
  mapfile -t BAG_TOPICS < <(python3 -c 'import sys,yaml; print("\n".join(yaml.safe_load(open(sys.argv[1]))["record_topics"]))' "${PROJECT_ROOT}/config/stage10.yaml")
  setsid env -u ROS_DISCOVERY_SERVER \
    FASTRTPS_DEFAULT_PROFILES_FILE="${SUPER_CLIENT_XML}" ros2 bag record \
    --storage mcap --storage-preset-profile zstd_fast \
    --disable-keyboard-controls --output "${RUN_DIR}/rosbag" \
    --topics "${BAG_TOPICS[@]}" >"${RUN_DIR}/rosbag.log" 2>&1 & BAG_PID=$!
  sleep 1
  process_alive "${BAG_PID}" || die "rosbag recorder exited; see ${RUN_DIR}/rosbag.log"
fi

require_dynamic="false"
[[ "${EXPERIMENT_CLASS}" == "dynamic" ]] && require_dynamic="true"
fault_parameter="${FAULT_SEQUENCE}"
[[ -n "${fault_parameter}" ]] || fault_parameter="''"
printf -v GOAL_TIMEOUT_S '%.3f' "${PHASE10_GOAL_TIMEOUT_S:-150.0}"
runner_args=(--ros-args -p use_sim_time:=true -p result_path:="${NAV_REPORT}"
  -p require_rviz:="${RVIZ}" -p goal_timeout_s:="${GOAL_TIMEOUT_S}"
  -p "goal_poses:=${GOAL_POSES}" -p action_topic:=/navigate_to_pose_resilient
  -p nvblox_slice_topic:=/nvblox_node/combined_map_slice
  -p phase9_mode:=true -p force_relocalization:=false
  -p require_surround_cameras:=false -p require_dynamic_outputs:="${require_dynamic}"
  -p experiment_class:="stage10_${EXPERIMENT_CLASS}"
  -p trajectory_path:="${RUN_DIR}/trajectory.csv"
  -p command_trace_path:="${RUN_DIR}/command_trace.csv"
  -p goal_xy_tolerance_m:=0.25 -p goal_yaw_tolerance_deg:=10.0
  -p minimum_ground_truth_motion_m:=0.50
  -p "fault_sequence:=${fault_parameter}" -p fault_duration_s:=2.0
  -p fault_stop_grace_s:=1.5 -p maximum_command_while_fault:=0.02)
set +e
ros2 run nova_carter_experiments navigation_test_runner "${runner_args[@]}" \
  >>"${RUN_DIR}/ros.log" 2>&1
RUNNER_STATUS=$?
set -e

stop_group "${BAG_PID}"; BAG_PID=""
stop_group "${NAV_PID}"; NAV_PID=""
stop_simulator; SIM_PID=""
touch "${GPU_STOP}"
stop_group "${GPU_PID}"; GPU_PID=""

set +e
finalize_bag_arg=()
[[ "${RECORD_BAG}" == "true" ]] && finalize_bag_arg+=(--require-rosbag)
python3 "${PROJECT_ROOT}/tools/finalize_stage10_trial.py" "${RUN_DIR}" \
  --runner-exit-code "${RUNNER_STATUS}" --config "${PROJECT_ROOT}/config/stage10.yaml" \
  "${finalize_bag_arg[@]}" \
  >"${RUN_DIR}/finalize.log" 2>&1
FINAL_STATUS=$?
set -e
if (( FINAL_STATUS != 0 )); then
  tail -n 120 "${RUN_DIR}/ros.log" >&2 || true
  cat "${RUN_DIR}/finalize.log" >&2
  die "Stage 10 trial failed: ${RUN_DIR}"
fi
info "Stage 10 trial passed: ${RUN_DIR}/result.json"
