#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="kujiale_jackal_8cam"
INTERACTIVE="false"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --interactive) INTERACTIVE="true"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_mapping.sh [--map NAME] --interactive"
      echo "Starts the GUI and 8-camera mapper; drive with WASD and press Q to save."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ "${INTERACTIVE}" == "true" ]] || die "manual mapping requires --interactive"
[[ -t 0 ]] || die "manual mapping requires an interactive terminal"
[[ "${MAP_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "invalid map name"

MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/mapping/${RUN_ID}"
SIM_STOP="${LOG_DIR}/stop-simulator"
BAG_ROOT="$(mktemp -d "${PROJECT_ROOT}/data/bags/.${MAP_NAME}.XXXXXX")"
BAG_DIR="${BAG_ROOT}/capture"
export ROS_DOMAIN_ID="${MAPPING_ROS_DOMAIN_ID:-47}"

if [[ -d "${MAP_DIR}" ]] && find "${MAP_DIR}" -mindepth 1 -print -quit | grep -q .; then
  die "map directory is not empty; preserving it: ${MAP_DIR}"
fi
mkdir -p "${MAP_DIR}" "${LOG_DIR}" "${MAP_DIR}/nvblox" \
  "${MAP_DIR}/mesh" "${MAP_DIR}/occupancy" "${PROJECT_ROOT}/data/locks"
exec 9>"${PROJECT_ROOT}/data/locks/mapping-workflow.lock"
flock -n 9 || die "another mapping workflow is already running"

SIM_PID=""
BRINGUP_PID=""
BAG_PID=""
process_alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
group_alive() { [[ -n "$1" ]] && kill -0 -- "-$1" 2>/dev/null; }
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if group_alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..80}; do group_alive "${pid}" || break; sleep 0.25; done
    group_alive "${pid}" && kill -TERM -- "-${pid}" 2>/dev/null || true
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
  stop_group "${BAG_PID}"
  stop_group "${BRINGUP_PID}"
  stop_simulator
}
trap cleanup EXIT INT TERM

info "Starting Kujiale GUI with mapping_8cam in ROS domain ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  --gui --duration 0 --camera-profile mapping_8cam --reliable-sensor-qos \
  --stop-file "${SIM_STOP}" --report "${LOG_DIR}/simulator.json" \
  >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..360}; do
  grep -Fq "JACKAL_SENSORS_READY" "${LOG_DIR}/simulator.log" 2>/dev/null && break
  process_alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq "camera_profile=mapping_8cam streams=8" "${LOG_DIR}/simulator.log" || \
  die "8-camera simulator startup timed out"

BRINGUP_SHARE="$(ros2 pkg prefix jackal_bringup --share)"
setsid ros2 launch jackal_bringup phase6.launch.py \
  camera_profile:=mapping_8cam image_qos:=DEFAULT \
  visual_slam_params:="${BRINGUP_SHARE}/config/visual_slam_mapping_8cam.yaml" \
  >"${LOG_DIR}/bringup.log" 2>&1 & BRINGUP_PID=$!

info "Waiting for all eight normalized image publishers and nvblox services"
IMAGE_TOPICS=()
CAMERA_INFO_TOPICS=()
for pair in front left right back; do
  for side in left right; do
    IMAGE_TOPICS+=("/${pair}_stereo_camera/${side}/image_raw")
    CAMERA_INFO_TOPICS+=("/${pair}_stereo_camera/${side}/camera_info")
  done
done
for _ in {1..360}; do
  topic_list="$(ros2 topic list --no-daemon --spin-time 2 2>/dev/null || true)"
  topics_ready="true"
  for topic in "${IMAGE_TOPICS[@]}"; do
    grep -Fxq "${topic}" <<<"${topic_list}" || topics_ready="false"
  done
  if [[ "${topics_ready}" == "true" ]] \
    && ros2 service type /nvblox_node/save_map 2>/dev/null | grep -Fq FilePath; then
    break
  fi
  process_alive "${BRINGUP_PID}" || die "mapping bringup exited; see ${LOG_DIR}/bringup.log"
  sleep 0.5
done
topic_list="$(ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true)"
for topic in "${IMAGE_TOPICS[@]}"; do
  grep -Fxq "${topic}" <<<"${topic_list}" || die "mapping topic is missing: ${topic}"
done

BAG_TOPICS=("${IMAGE_TOPICS[@]}" "${CAMERA_INFO_TOPICS[@]}" \
  /front_stereo_imu/imu /tf /tf_static /clock)
setsid ros2 bag record --storage mcap --storage-preset-profile fastwrite \
  --disable-keyboard-controls --output "${BAG_DIR}" --topics "${BAG_TOPICS[@]}" \
  >"${LOG_DIR}/rosbag.log" 2>&1 & BAG_PID=$!
sleep 2
process_alive "${BAG_PID}" || die "rosbag recorder exited; see ${LOG_DIR}/rosbag.log"

info "Manual mapping ready: W/S forward/back, A/D rotate, Space stop, Q save and exit"
ros2 run jackal_teleop keyboard_teleop

info "Stopping and indexing the 8-camera MCAP"
stop_group "${BAG_PID}"
BAG_PID=""
ros2 bag info "${BAG_DIR}" >"${LOG_DIR}/rosbag-info.txt"
grep -Fq "storage_identifier: mcap" "${BAG_DIR}/metadata.yaml" || die "bag is not MCAP"

info "Saving nvblox binary map, mesh, rates and occupancy grid"
ros2 run jackal_experiments nvblox_map_saver --ros-args \
  -p output_dir:="${MAP_DIR}/nvblox" -p stem:=kujiale \
  >"${LOG_DIR}/save-nvblox.log" 2>&1
install -m 0644 "${MAP_DIR}/nvblox/kujiale.ply" "${MAP_DIR}/mesh/kujiale.ply"
ros2 run jackal_experiments occupancy_saver --ros-args \
  -p use_sim_time:=true -p output_dir:="${MAP_DIR}/occupancy" \
  >"${LOG_DIR}/save-occupancy.log" 2>&1

stop_group "${BRINGUP_PID}"
BRINGUP_PID=""
stop_simulator
SIM_PID=""

"${PROJECT_ROOT}/scripts/export_vgl_models.sh" "${PROJECT_ROOT}/data/models/vgl" \
  >"${LOG_DIR}/model-export.log" 2>&1
"${PROJECT_ROOT}/scripts/create_vgl_map.sh" "${BAG_DIR}" "${MAP_DIR}" \
  --topic-config "${PROJECT_ROOT}/ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml" \
  --max-sync-us "${MAPPING_MAX_SYNC_US:-40000}" \
  >"${LOG_DIR}/offline-map.log" 2>&1
python3 "${PROJECT_ROOT}/tools/write_map_manifest.py" "${MAP_DIR}" "${BAG_DIR}" \
  --run-id "${RUN_ID}"

case "${BAG_ROOT}" in
  "${PROJECT_ROOT}/data/bags/.${MAP_NAME}."*) rm -rf -- "${BAG_ROOT}" ;;
  *) die "refusing to remove unexpected bag directory: ${BAG_ROOT}" ;;
esac
trap - EXIT INT TERM
info "Map complete: ${MAP_DIR}"
info "Raw bag and offline intermediates were removed; runtime artifacts remain."
