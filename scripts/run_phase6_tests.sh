#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

export ROS_DOMAIN_ID="${PHASE6_TEST_ROS_DOMAIN_ID:-46}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage6/${RUN_ID}"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase6"
MAP_DIR="${PROJECT_ROOT}/data/maps/phase6_validation/${RUN_ID}/nvblox"
MAPPING_DURATION="${PHASE6_MAPPING_DURATION_SIM_SECONDS:-60.0}"
SIM_REPORT="${LOG_DIR}/simulator.json"
RESULT="${REPORT_DIR}/run-${RUN_ID}.json"
SIM_LOG="${LOG_DIR}/simulator.log"
BRINGUP_LOG="${LOG_DIR}/bringup.log"
TEST_LOG="${LOG_DIR}/nvblox-test.log"
PARAM_DUMP="${LOG_DIR}/nvblox-runtime.yaml"
SIM_STOP_FILE="${LOG_DIR}/stop-simulator"
mkdir -p "${LOG_DIR}" "${REPORT_DIR}" "${MAP_DIR}"

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

info "Phase 6 isolated test domain: ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  --headless --duration 360 --stop-file "${SIM_STOP_FILE}" --report "${SIM_REPORT}" \
  >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

info "Waiting for Standalone depth and visual sensor graphs"
ready=false
for _ in {1..180}; do
  if ! process_alive "${SIM_PID}"; then
    tail -n 180 "${SIM_LOG}" >&2 || true
    die "simulator exited during Phase 6 startup"
  fi
  if grep -Fq "JACKAL_SENSORS_READY" "${SIM_LOG}" 2>/dev/null; then
    ready=true
    break
  fi
  sleep 0.5
done
[[ "${ready}" == true ]] || die "timed out waiting for sensor graphs: ${SIM_LOG}"

info "Starting control, cuVSLAM, and nvblox static TSDF bringup"
setsid ros2 launch jackal_bringup phase6.launch.py >"${BRINGUP_LOG}" 2>&1 &
BRINGUP_PID=$!

info "Waiting for cuVSLAM and nvblox services"
ready=false
for _ in {1..240}; do
  if ! process_alive "${BRINGUP_PID}"; then
    tail -n 220 "${BRINGUP_LOG}" >&2 || true
    die "Phase 6 bringup exited during startup"
  fi
  if ros2 service type /visual_slam/save_map 2>/dev/null \
      | grep -Fq "isaac_ros_visual_slam_interfaces/srv/FilePath" \
      && ros2 service type /nvblox_node/save_map 2>/dev/null \
      | grep -Fq "nvblox_msgs/srv/FilePath"; then
    ready=true
    break
  fi
  sleep 0.5
done
[[ "${ready}" == true ]] || die "timed out waiting for Phase 6 services: ${BRINGUP_LOG}"

ros2 param dump /nvblox_node >"${PARAM_DUMP}"
python3 - "${PARAM_DUMP}" <<'PY'
import sys
import yaml

params = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["/nvblox_node"]["ros__parameters"]
expected = {
    "mapping_type": "static_tsdf",
    "global_frame": "map",
    "num_cameras": 1,
    "use_tf_transforms": True,
    "use_depth": True,
    "use_color": True,
    "use_lidar": False,
    "voxel_size": 0.05,
    "integrate_depth_rate_hz": 10.0,
    "integrate_color_rate_hz": 3.0,
    "update_esdf_rate_hz": 10.0,
    "update_mesh_rate_hz": 1.0,
}
for name, value in expected.items():
    actual = params[name]
    if isinstance(value, float):
        assert abs(actual - value) < 1.0e-4, (name, actual, value)
    else:
        assert actual == value, (name, actual, value)
print("nvblox_runtime_parameters=passed")
PY

info "Running ${MAPPING_DURATION} simulated seconds of reconstruction and persistence tests"
set +e
ros2 run jackal_experiments nvblox_test_runner --ros-args \
  -p use_sim_time:=true -p result_path:="${RESULT}" -p output_dir:="${MAP_DIR}" \
  -p mapping_duration_sim_seconds:="${MAPPING_DURATION}" \
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
  tail -n 240 "${BRINGUP_LOG}" >&2 || true
  die "Phase 6 nvblox runner failed; result: ${RESULT}"
fi
if rg -n "\[(ERROR|FATAL)\]|Could not transform|queue.*dropp" "${BRINGUP_LOG}"; then
  die "Phase 6 bringup log contains reconstruction errors: ${BRINGUP_LOG}"
fi
python3 "${PROJECT_ROOT}/tools/check_phase6_nvblox.py" "${RESULT}"
install -m 0644 "${RESULT}" "${REPORT_DIR}/latest.json"
install -m 0644 "${PARAM_DUMP}" "${REPORT_DIR}/nvblox-runtime-latest.yaml"
info "Phase 6 passed; result: ${RESULT}"
