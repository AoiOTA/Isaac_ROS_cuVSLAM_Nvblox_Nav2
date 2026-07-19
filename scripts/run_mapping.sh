#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

# Jazzy still honors ROS_LOCALHOST_ONLY but warns on every short-lived CLI
# probe. Use its supported replacement for this local-only mapping workflow.
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MAP_NAME="kujiale_jackal_8cam"
MAPPING_MODE=""
SIM_MODE=""
RVIZ=""
COVERAGE_CONFIG="${PROJECT_ROOT}/config/mapping_coverage.yaml"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --interactive)
      [[ -z "${MAPPING_MODE}" ]] || die "choose exactly one of --interactive or --auto"
      MAPPING_MODE="interactive"; shift ;;
    --auto)
      [[ -z "${MAPPING_MODE}" ]] || die "choose exactly one of --interactive or --auto"
      MAPPING_MODE="auto"; shift ;;
    --gui|--headless) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --coverage-config) COVERAGE_CONFIG="${2:?missing coverage config}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_mapping.sh [--map NAME] (--interactive | --auto) [--gui | --headless] [--rviz | --no-rviz]"
      echo "Interactive mode defaults to Isaac Sim GUI + RViz and uses WASD/Q."
      echo "Auto mode defaults to headless/no-RViz and follows the checked closed-loop route."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ -n "${MAPPING_MODE}" ]] || die "mapping requires exactly one of --interactive or --auto"
if [[ -z "${SIM_MODE}" ]]; then
  if [[ "${MAPPING_MODE}" == "interactive" ]]; then
    SIM_MODE="--gui"
  else
    SIM_MODE="--headless"
  fi
fi
if [[ -z "${RVIZ}" ]]; then
  if [[ "${MAPPING_MODE}" == "interactive" ]]; then
    RVIZ="true"
  else
    RVIZ="false"
  fi
fi
if [[ "${MAPPING_MODE}" == "interactive" ]]; then
  [[ -t 0 ]] || die "manual mapping requires an interactive terminal"
  [[ "${SIM_MODE}" == "--gui" ]] || die "manual mapping requires --gui"
else
  require_file "${COVERAGE_CONFIG}"
fi
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
RVIZ_PID=""
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
  stop_group "${RVIZ_PID}"
  stop_group "${BAG_PID}"
  stop_group "${BRINGUP_PID}"
  stop_simulator
}
trap cleanup EXIT INT TERM

info "Starting Kujiale ${SIM_MODE#--} with mapping_8cam in ROS domain ${ROS_DOMAIN_ID}"
setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "${SIM_MODE}" --duration 0 --camera-profile mapping_8cam --reliable-sensor-qos \
  --stop-file "${SIM_STOP}" --report "${LOG_DIR}/simulator.json" \
  >"${LOG_DIR}/simulator.log" 2>&1 & SIM_PID=$!
for _ in {1..360}; do
  grep -Fq "JACKAL_SENSORS_READY" "${LOG_DIR}/simulator.log" 2>/dev/null && break
  process_alive "${SIM_PID}" || die "simulator exited; see ${LOG_DIR}/simulator.log"
  sleep 0.5
done
grep -Fq "camera_profile=mapping_8cam streams=8 lidar=false" "${LOG_DIR}/simulator.log" || \
  die "four-Hawk/eight-stream simulator startup timed out"

BRINGUP_SHARE="$(ros2 pkg prefix jackal_bringup --share)"
setsid ros2 launch jackal_bringup phase6.launch.py \
  camera_profile:=mapping_8cam image_qos:=DEFAULT \
  visual_slam_params:="${BRINGUP_SHARE}/config/visual_slam_mapping_8cam.yaml" \
  >"${LOG_DIR}/bringup.log" 2>&1 & BRINGUP_PID=$!

info "Waiting for eight live normalized images, CameraInfo, and nvblox"
IMAGE_TOPICS=()
CAMERA_INFO_TOPICS=()
for pair in front left right back; do
  for side in left right; do
    IMAGE_TOPICS+=("/${pair}_stereo_camera/${side}/image_raw")
    CAMERA_INFO_TOPICS+=("/${pair}_stereo_camera/${side}/camera_info")
  done
done
for topic in "${IMAGE_TOPICS[@]}"; do
  process_alive "${BRINGUP_PID}" || die "mapping bringup exited; see ${LOG_DIR}/bringup.log"
  ros2 topic echo --no-daemon --once --no-arr --timeout 30 \
    "${topic}" sensor_msgs/msg/Image >/dev/null || \
    die "mapping image has no live sample: ${topic}"
done
for topic in "${CAMERA_INFO_TOPICS[@]}"; do
  process_alive "${BRINGUP_PID}" || die "mapping bringup exited; see ${LOG_DIR}/bringup.log"
  ros2 topic echo --no-daemon --once --no-arr --timeout 30 \
    "${topic}" sensor_msgs/msg/CameraInfo >/dev/null || \
    die "mapping CameraInfo has no live sample: ${topic}"
done
service_ready="false"
for _ in {1..360}; do
  if ros2 service type /nvblox_node/save_map 2>/dev/null | grep -Fq FilePath; then
    service_ready="true"
    break
  fi
  process_alive "${BRINGUP_PID}" || die "mapping bringup exited; see ${LOG_DIR}/bringup.log"
  sleep 0.5
done
[[ "${service_ready}" == "true" ]] || die "nvblox save service startup timed out"
ros2 topic echo --no-daemon --once --timeout 30 \
  /visual_slam/status isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus >/dev/null || \
  die "cuVSLAM status has no live sample"
ros2 topic echo --no-daemon --once --timeout 30 \
  /nvblox_node/static_map_slice nvblox_msgs/msg/DistanceMapSlice >/dev/null || \
  die "nvblox static map slice has no live sample"

if [[ "${RVIZ}" == "true" ]]; then
  RVIZ_CONFIG="${BRINGUP_SHARE}/rviz/mapping.rviz"
  require_file "${RVIZ_CONFIG}"
  info "Starting mapping RViz (map, robot, TF, cuVSLAM trajectory, nvblox mesh/ESDF)"
  setsid rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true \
    >"${LOG_DIR}/rviz.log" 2>&1 & RVIZ_PID=$!
  sleep 3
  process_alive "${RVIZ_PID}" || die "mapping RViz exited; see ${LOG_DIR}/rviz.log"
fi

BAG_TOPICS=("${IMAGE_TOPICS[@]}" "${CAMERA_INFO_TOPICS[@]}" \
  /front_stereo_camera/depth/image_raw \
  /front_stereo_camera/depth/camera_info \
  /front_stereo_imu/imu /tf /tf_static /clock)
# cuVGL reads the MCAP by receive timestamp. fastwrite disables MCAP
# chunking, so long multi-camera captures have no timestamp index and can be
# read out of order. zstd_fast keeps chunks/indexes at the required throughput.
setsid ros2 bag record --storage mcap --storage-preset-profile zstd_fast \
  --disable-keyboard-controls --output "${BAG_DIR}" --topics "${BAG_TOPICS[@]}" \
  >"${LOG_DIR}/rosbag.log" 2>&1 & BAG_PID=$!
sleep 2
process_alive "${BAG_PID}" || die "rosbag recorder exited; see ${LOG_DIR}/rosbag.log"

if [[ "${MAPPING_MODE}" == "interactive" ]]; then
  info "Manual mapping ready in Isaac Sim GUI + RViz"
  info "W/S forward/back, A/D rotate, Space stop, Q stop recording, validate, save and exit"
  info "Keyboard control is read from THIS TERMINAL; keep this terminal focused while driving."
  info "Press P to show travelled distance. Q is accepted only after at least 2.0 m."
  ros2 run jackal_teleop keyboard_teleop
else
  info "Automated mapping ready: following the collision-clear closed-loop coverage route"
  ros2 run jackal_experiments mapping_coverage_driver --ros-args \
    -p use_sim_time:=true -p config_path:="${COVERAGE_CONFIG}" \
    -p report_path:="${LOG_DIR}/coverage.json" \
    >"${LOG_DIR}/coverage.log" 2>&1
fi

info "Stopping and indexing the four-Hawk/eight-stream MCAP"
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
  -p obstacle_distance_m:=0.0 \
  >"${LOG_DIR}/save-occupancy.log" 2>&1

VISUAL_SAVE_ARGS=(
  -p output_dir:="${MAP_DIR}/cuvslam"
)
if [[ "${MAPPING_MODE}" == "auto" ]]; then
  read -r EXPECTED_PATH MINIMUM_POSE_DURATION < <(
    python3 - "${LOG_DIR}/coverage.json" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
path = float(report["route"]["ground_truth_path_length_m"])
duration = 0.80 * float(report["simulation_duration_s"])
print(f"{path:.9f} {duration:.9f}")
PY
  )
  VISUAL_SAVE_ARGS+=(
    -p expected_path_length_m:="${EXPECTED_PATH}"
    -p minimum_pose_count:=500
    -p minimum_duration_s:="${MINIMUM_POSE_DURATION}"
  )
fi
info "Saving the live cuVSLAM database and globally optimized map-frame trajectory"
if ! ros2 run jackal_experiments visual_map_saver --ros-args \
  "${VISUAL_SAVE_ARGS[@]}" >"${LOG_DIR}/save-cuvslam.log" 2>&1; then
  if [[ -f "${MAP_DIR}/cuvslam/save_report.json" ]]; then
    read -r SAVE_REASONS PATH_METERS CLOSURE_METERS DURATION_SECONDS < <(
      python3 - "${MAP_DIR}/cuvslam/save_report.json" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
trajectory = report.get("trajectory", {})
print(
    ",".join(report.get("failure_reasons", [])) or "unknown",
    f"{float(trajectory.get('planar_path_length_m', 0.0)):.3f}",
    f"{float(trajectory.get('closure_error_m', 0.0)):.3f}",
    f"{float(trajectory.get('duration_s', 0.0)):.1f}",
)
PY
    )
    info "Map was not promoted: cuVSLAM quality failed (${SAVE_REASONS})."
    info "Trajectory: path=${PATH_METERS}m closure_error=${CLOSURE_METERS}m duration=${DURATION_SECONDS}s."
  else
    info "Map was not promoted: cuVSLAM save failed before producing a quality report."
  fi
  info "Detailed save log: ${LOG_DIR}/save-cuvslam.log"
  info "The raw MCAP and partial artifacts were retained for diagnosis."
  die "Start a new map name; drive from THIS TERMINAL for broad coverage, return near the start, then press Q."
fi

stop_group "${BRINGUP_PID}"
BRINGUP_PID=""
stop_group "${RVIZ_PID}"
RVIZ_PID=""
stop_simulator
SIM_PID=""

mapping_validation_args=("${LOG_DIR}/simulator.json")
if [[ "${MAPPING_MODE}" == "auto" ]]; then
  mapping_validation_args+=(--coverage-report "${LOG_DIR}/coverage.json")
else
  # Manual map capture may contain a scrape while the operator explores a
  # dense room. Keep that evidence in the manifest, but reserve strict zero
  # collision promotion for the automated/acceptance mapping workflow.
  mapping_validation_args+=(--allow-physical-collisions)
fi
if ! python3 "${PROJECT_ROOT}/tools/validate_mapping_run.py" \
  "${mapping_validation_args[@]}" >"${LOG_DIR}/mapping-validation.json"; then
  read -r COLLISION_EVENTS VALIDATION_REASONS < <(
    python3 - "${LOG_DIR}/mapping-validation.json" "${LOG_DIR}/simulator.json" <<'PY'
import json
import sys

validation = json.load(open(sys.argv[1], encoding="utf-8"))
simulator = json.load(open(sys.argv[2], encoding="utf-8"))
contacts = simulator.get("robot_contacts", {})
count = contacts.get("collision_event_count", "unknown") if isinstance(contacts, dict) else "unknown"
reasons = ",".join(validation.get("failure_reasons", [])) or "unknown"
print(count, reasons)
PY
  )
  info "Map was not promoted: validation failed (${VALIDATION_REASONS}); physical collision events=${COLLISION_EVENTS}."
  info "No manifest.json was written, so navigation is intentionally blocked."
  info "Saved partial artifacts and the evidence log were retained: ${MAP_DIR}, ${LOG_DIR}/mapping-validation.json"
  die "Rebuild this manual map with a new --map name and avoid physical contacts; wait for 'Map complete:' before starting navigation."
fi

MANUAL_COLLISION_EVENTS="$(python3 - "${LOG_DIR}/mapping-validation.json" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
print(int(report.get("physical_collision_event_count", 0)))
PY
)"
if [[ "${MAPPING_MODE}" == "interactive" && "${MANUAL_COLLISION_EVENTS}" != "0" ]]; then
  info "Manual map promotion retains ${MANUAL_COLLISION_EVENTS} physical collision events in manifest.json."
  info "This map is usable for manual navigation, but is not evidence for the strict zero-collision acceptance metric."
fi

"${PROJECT_ROOT}/scripts/export_vgl_models.sh" "${PROJECT_ROOT}/data/models/vgl" \
  >"${LOG_DIR}/model-export.log" 2>&1
"${PROJECT_ROOT}/scripts/create_vgl_map.sh" "${BAG_DIR}" "${MAP_DIR}" \
  --topic-config "${PROJECT_ROOT}/ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml" \
  --max-sync-us "${MAPPING_MAX_SYNC_US:-40000}" \
  >"${LOG_DIR}/offline-map.log" 2>&1
python3 "${PROJECT_ROOT}/tools/write_map_manifest.py" "${MAP_DIR}" "${BAG_DIR}" \
  --run-id "${RUN_ID}" \
  --mapping-validation "${LOG_DIR}/mapping-validation.json" \
  --generation-command \
  "./scripts/run_mapping.sh --map {map_name} --${MAPPING_MODE} ${SIM_MODE}"

case "${BAG_ROOT}" in
  "${PROJECT_ROOT}/data/bags/.${MAP_NAME}."*) rm -rf -- "${BAG_ROOT}" ;;
  *) die "refusing to remove unexpected bag directory: ${BAG_ROOT}" ;;
esac
trap - EXIT INT TERM
info "Map complete: ${MAP_DIR}"
info "Raw bag and offline intermediates were removed; runtime artifacts remain."
