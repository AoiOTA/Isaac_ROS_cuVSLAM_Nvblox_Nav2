#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="warehouse_v1"
FOUR_WAY="false"
FRONT_RATE_HZ=""
RELIABLE_SENSOR_QOS="false"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --four-way) FOUR_WAY="true"; shift ;;
    --front-rate-hz) FRONT_RATE_HZ="${2:?missing front rate}"; shift 2 ;;
    --reliable-sensor-qos) RELIABLE_SENSOR_QOS="true"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_mapping.sh [--map NAME] [--front-rate-hz HZ] [--reliable-sensor-qos] [--four-way]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
if [[ -n "${FRONT_RATE_HZ}" && ! "${FRONT_RATE_HZ}" =~ ^[1-9][0-9]*$ ]]; then
  die "--front-rate-hz must be a positive integer"
fi
if [[ "${FOUR_WAY}" == "true" ]]; then
  FRONT_RATE_HZ="${FRONT_RATE_HZ:-10}"
  RELIABLE_SENSOR_QOS="true"
fi
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage7/${RUN_ID}"
BAG_DIR="${PROJECT_ROOT}/data/bags/${MAP_NAME}_${RUN_ID}"
REPORT="${PROJECT_ROOT}/data/reports/phase7/mapping-${RUN_ID}.json"
SIM_STOP="${LOG_DIR}/stop-simulator"
DURATION="${PHASE7_MAPPING_DURATION_SIM_SECONDS:-40.0}"
[[ "${DURATION}" =~ ^[0-9]+$ ]] && DURATION="${DURATION}.0"
mkdir -p "${LOG_DIR}" "$(dirname "${REPORT}")" "${MAP_DIR}"
export ROS_DOMAIN_ID="${PHASE7_MAPPING_ROS_DOMAIN_ID:-47}"

SIM_PID=""; BRINGUP_PID=""; BAG_PID=""; CAMERA_GATE_PID=""
alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..120}; do alive "${pid}" || break; sleep 0.25; done
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  stop_group "${BAG_PID}"
  stop_group "${CAMERA_GATE_PID}"
  stop_group "${BRINGUP_PID}"
  if alive "${SIM_PID}"; then touch "${SIM_STOP}"; fi
  stop_group "${SIM_PID}"
}
trap cleanup EXIT INT TERM

rm -rf "${MAP_DIR}/online_cuvslam" "${MAP_DIR}/nvblox" "${MAP_DIR}/mesh" \
  "${MAP_DIR}/occupancy" "${MAP_DIR}/offline" "${BAG_DIR}"
mkdir -p "${MAP_DIR}/nvblox" "${MAP_DIR}/mesh"

info "Starting isolated map collection in ROS domain ${ROS_DOMAIN_ID}; four_way=${FOUR_WAY}"
sim_args=(--headless --duration 600 --stop-file "${SIM_STOP}" --report "${LOG_DIR}/simulator.json")
if [[ -n "${FRONT_RATE_HZ}" ]]; then
  sim_args+=(--front-image-width 1280 --front-image-height 800 \
    --front-image-rate-hz "${FRONT_RATE_HZ}")
fi
if [[ "${RELIABLE_SENSOR_QOS}" == "true" ]]; then
  sim_args+=(--reliable-sensor-qos)
fi
if [[ "${FOUR_WAY}" == "true" ]]; then
  sim_args+=(--enable-surround-cameras --surround-image-rate-hz 10)
fi
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${sim_args[@]}" \
  >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..240}; do
  grep -Fq NOVA_CARTER_SENSORS_READY "${LOG_DIR}/simulator.log" 2>/dev/null && break
  alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq NOVA_CARTER_SENSORS_READY "${LOG_DIR}/simulator.log" || die "sensor startup timeout"

bringup_args=()
if [[ -n "${FRONT_RATE_HZ}" ]]; then
  bringup_args+=(front_image_width:=1280 front_image_height:=800)
fi
if [[ "${RELIABLE_SENSOR_QOS}" == "true" ]]; then
  bringup_args+=(image_qos:=DEFAULT)
fi
if [[ "${FOUR_WAY}" == "true" ]]; then
  bringup_args+=(enable_surround_cameras:=true)
fi
setsid ros2 launch nova_carter_bringup phase6.launch.py "${bringup_args[@]}" \
  >"${LOG_DIR}/bringup.log" 2>&1 & BRINGUP_PID=$!
for _ in {1..300}; do
  if ros2 service type /visual_slam/save_map 2>/dev/null | grep -Fq FilePath \
    && ros2 service type /nvblox_node/save_map 2>/dev/null | grep -Fq FilePath; then break; fi
  alive "${BRINGUP_PID}" || die "bringup exited; see ${LOG_DIR}/bringup.log"
  sleep 0.5
done

if [[ "${FOUR_WAY}" == "true" ]]; then
  info "Enabling the three on-demand side/back stereo pairs for map collection"
  setsid ros2 topic pub --rate 2 --qos-reliability reliable \
    --qos-durability transient_local /vgl/cameras_enabled std_msgs/msg/Bool \
    '{data: true}' >"${LOG_DIR}/camera-gate.log" 2>&1 & CAMERA_GATE_PID=$!
  sleep 3
fi

info "Recording synchronized visual calibration and TF to MCAP"
bag_topics=(
  /front_stereo_camera/left/image_raw /front_stereo_camera/right/image_raw
  /front_stereo_camera/left/camera_info /front_stereo_camera/right/camera_info
  /front_stereo_imu/imu /tf /tf_static /clock
)
if [[ "${FOUR_WAY}" == "true" ]]; then
  for camera in left right back; do
    bag_topics+=(
      "/${camera}_stereo_camera/left/image_raw"
      "/${camera}_stereo_camera/right/image_raw"
      "/${camera}_stereo_camera/left/camera_info"
      "/${camera}_stereo_camera/right/camera_info"
    )
  done
fi
setsid ros2 bag record --storage mcap --output "${BAG_DIR}" \
  --storage-preset-profile fastwrite \
  "${bag_topics[@]}" \
  >"${LOG_DIR}/rosbag.log" 2>&1 & BAG_PID=$!
sleep 2

runner_args=()
if [[ -n "${FRONT_RATE_HZ}" ]]; then
  # Reliable 10 Hz Phase 9 acquisition intentionally trades throughput for
  # lossless, tightly synchronized visual-map input.
  runner_args+=(
    -p expected_camera_info_rate_hz:=8.0
    -p min_depth_callback_rate_hz:=6.0
    -p min_depth_integration_rate_hz:=6.0
    -p min_color_integration_rate_hz:=1.0
    -p min_esdf_update_rate_hz:=7.0
  )
fi
ros2 run nova_carter_experiments nvblox_test_runner --ros-args \
  -p use_sim_time:=true -p result_path:="${REPORT}" \
  -p output_dir:="${MAP_DIR}/nvblox" \
  -p mapping_duration_sim_seconds:="${DURATION}" \
  "${runner_args[@]}" \
  >"${LOG_DIR}/mapping-runner.log" 2>&1
cp "${MAP_DIR}/nvblox/warehouse.ply" "${MAP_DIR}/mesh/warehouse.ply"

info "Saving online cuVSLAM map and Nav2 occupancy grid"
ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '${MAP_DIR}/online_cuvslam'}" \
  >"${LOG_DIR}/save-cuvslam.log"
grep -Eq 'success[=:][[:space:]]*(true|True)' "${LOG_DIR}/save-cuvslam.log" || \
  die "cuVSLAM save failed"
ros2 run nova_carter_experiments occupancy_saver --ros-args \
  -p use_sim_time:=true -p output_dir:="${MAP_DIR}/occupancy" \
  >"${LOG_DIR}/save-occupancy.log" 2>&1

stop_group "${BAG_PID}"; BAG_PID=""
stop_group "${CAMERA_GATE_PID}"; CAMERA_GATE_PID=""
stop_group "${BRINGUP_PID}"; BRINGUP_PID=""
touch "${SIM_STOP}"
stop_group "${SIM_PID}"; SIM_PID=""

ros2 bag info "${BAG_DIR}" >"${LOG_DIR}/rosbag-info.txt"
grep -Fq 'storage_identifier: mcap' "${BAG_DIR}/metadata.yaml" || die "bag is not MCAP"
"${PROJECT_ROOT}/scripts/export_vgl_models.sh" "${PROJECT_ROOT}/data/models/vgl" \
  2>&1 | tee "${LOG_DIR}/model-export.log"
map_args=("${BAG_DIR}" "${MAP_DIR}")
if [[ "${FOUR_WAY}" == "true" ]]; then
  map_args+=(--topic-config "${PROJECT_ROOT}/ros2_ws/install/nova_carter_bringup/share/nova_carter_bringup/config/mapping_topics_4way.yaml")
fi
"${PROJECT_ROOT}/scripts/create_vgl_map.sh" "${map_args[@]}" \
  2>&1 | tee "${LOG_DIR}/offline-map.log"

manifest_args=("${MAP_DIR}" "${BAG_DIR}" --run-id "${RUN_ID}")
if [[ -n "${FRONT_RATE_HZ}" ]]; then
  manifest_args+=(--nominal-rate-hz "${FRONT_RATE_HZ}" \
    --generation-command './scripts/run_phase9_mapping.sh --map {map_name}')
fi
if [[ "${FOUR_WAY}" == "true" ]]; then
  manifest_args+=(--camera-count 8 --stereo-pairs 4 \
    --image-width 1280 --image-height 800 \
    --surround-image-width 1280 --surround-image-height 800 \
    --surround-rate-hz 10)
fi
python3 "${PROJECT_ROOT}/tools/write_map_manifest.py" "${manifest_args[@]}"
cp "${MAP_DIR}/manifest.json" "${PROJECT_ROOT}/data/reports/phase7/manifest-latest.json"
info "Stage 7 map complete: ${MAP_DIR}"
