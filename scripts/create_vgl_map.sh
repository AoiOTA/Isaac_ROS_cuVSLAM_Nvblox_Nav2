#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

BAG="${1:?usage: create_vgl_map.sh BAG [MAP_DIR] [--topic-config FILE]}"
shift
MAP_DIR="${PROJECT_ROOT}/data/maps/kujiale_latest_20260719_160004"
if (($#)) && [[ "$1" != --* ]]; then
  MAP_DIR="$1"
  shift
fi
[[ -d "${BAG}" ]] || die "MCAP rosbag directory not found: ${BAG}"
TOPIC_CONFIG="${PROJECT_ROOT}/ros2_ws/install/jackal_bringup/share/jackal_bringup/config/mapping_topics_8cam.yaml"
TUM_POSE_FILE=""
MAX_SYNC_US=40000
MINIMUM_SYNCED_FRAMES=40
while (($#)); do
  case "$1" in
    --topic-config) TOPIC_CONFIG="${2:?missing topic config}"; shift 2 ;;
    --tum-pose-file) TUM_POSE_FILE="${2:?missing TUM pose file}"; shift 2 ;;
    --max-sync-us) MAX_SYNC_US="${2:?missing synchronization window}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/create_vgl_map.sh BAG [MAP_DIR] [--tum-pose-file FILE] [--topic-config FILE] [--max-sync-us N]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
require_file "${TOPIC_CONFIG}"
if [[ -z "${TUM_POSE_FILE}" ]]; then
  TUM_POSE_FILE="${MAP_DIR}/cuvslam/optimized_poses.tum"
fi
require_file "${TUM_POSE_FILE}"
[[ "${MAX_SYNC_US}" =~ ^[1-9][0-9]*$ ]] || die "--max-sync-us must be a positive integer"
WORK="$(mktemp -d "${PROJECT_ROOT}/data/bags/.vgl-work.XXXXXX")"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
mkdir -p "${WORK}" "${MAP_DIR}/config"
export ISAAC_ROS_WS="${PROJECT_ROOT}/ros2_ws"

cleanup() {
  case "${WORK}" in
    "${PROJECT_ROOT}/data/bags/.vgl-work."*) rm -rf -- "${WORK}" ;;
    *) die "refusing to remove unexpected temporary directory: ${WORK}" ;;
  esac
}
trap cleanup EXIT INT TERM

require_file "${MAP_DIR}/cuvslam/data.mdb"
info "Creating a cuVGL map aligned to the saved online cuVSLAM trajectory"
mkdir -p "${WORK}/edex"
POSE_BAG="${WORK}/cuvslam_poses"
SENSOR_BAG="${WORK}/synchronized_sensor_bag"
python3 "${PROJECT_ROOT}/tools/tum_to_pose_bag.py" \
  "${TUM_POSE_FILE}" "${POSE_BAG}"
python3 "${PROJECT_ROOT}/tools/prepare_vgl_sensor_bag.py" \
  "${BAG}" "${SENSOR_BAG}" \
  --tum-pose-file "${TUM_POSE_FILE}" \
  --topic-config "${TOPIC_CONFIG}" \
  --max-sync-us "${MAX_SYNC_US}" \
  --minimum-groups "${MINIMUM_SYNCED_FRAMES}" \
  --report "${WORK}/sensor-bag-report.json"
CONVERTER_ARGS=(
  --output_folder_path="${WORK}/edex" \
  --sensor_data_bag_file="${SENSOR_BAG}" \
  # Motion-aware selection and stationary keepalives were already applied by
  # prepare_vgl_sensor_bag.py.  Reapplying the vendor filter can permanently
  # reject all frames after an ordinary stationary interval.
  --min_inter_frame_distance=0 \
  --min_inter_frame_rotation_degrees=0 \
  --sample_sync_threshold_microseconds="${MAX_SYNC_US}" \
  --generate_edex=True \
  --image_extension=.jpg \
  --base_link_name=base_link \
  --camera_topic_config="${TOPIC_CONFIG}" \
  --pose_bag_file="${POSE_BAG}" \
  --pose_topic_name=/visual_slam/vis/slam_odometry \
  --reference_pose_frame=map \
  --rectify_images=True
)
ros2 run isaac_mapping_ros rosbag_to_mapping_data "${CONVERTER_ARGS[@]}"

# Materialize the common pose-bearing metadata before cuVGL performs its own
# stricter feature-keyframe selection. Both cuVGL and offline nvblox must branch
# from this cuVSLAM-optimized set; neither map is the pose source for the other.
SHARED_FRAMES="${WORK}/optimized_frames"
mkdir -p "${SHARED_FRAMES}"
require_file "${WORK}/edex/frames_meta.json"
install -m 0644 "${WORK}/edex/frames_meta.json" \
  "${SHARED_FRAMES}/frames_meta.json"
install -m 0644 "${WORK}/sensor-bag-report.json" \
  "${SHARED_FRAMES}/selection_report.json"
python3 "${PROJECT_ROOT}/tools/write_optimized_frames_report.py" \
  "${SHARED_FRAMES}/frames_meta.json" "${SHARED_FRAMES}/report.json" \
  --tum-pose-file "${TUM_POSE_FILE}" \
  --source-bag "${BAG}" \
  --selection-report "${SHARED_FRAMES}/selection_report.json"

rm -rf "${WORK}/cuvgl_map"
info "Creating cuVGL BoW map with the project TensorRT cache"
ros2 run isaac_ros_visual_mapping create_cuvgl_map.py \
  --map_folder="${WORK}/cuvgl_map" \
  --raw_image_folder="${WORK}/edex" \
  --extract_feature --build_bow_index --feature_type=aliked --print_mode=tail \
  --binary_folder_path="$(ros2 pkg prefix isaac_ros_visual_mapping)/lib/isaac_ros_visual_mapping" \
  --config_folder_path="$(ros2 pkg prefix isaac_ros_visual_mapping --share)/configs/isaac" \
  --model_dir="${MODEL_DIR}"

[[ -d "${WORK}/cuvgl_map" ]] || die "cuVGL map was not created"
python3 - "${WORK}/cuvgl_map/keyframes/frames_meta.json" "${MINIMUM_SYNCED_FRAMES}" <<'PY'
import json
import sys
from pathlib import Path

metadata_path = Path(sys.argv[1])
minimum = int(sys.argv[2])
if not metadata_path.is_file():
    raise RuntimeError(f"cuVGL keyframe metadata is missing: {metadata_path}")
metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
frames = metadata.get("keyframes_metadata", [])
if not isinstance(frames, list):
    raise RuntimeError("cuVGL keyframe metadata is invalid")
timestamps = {
    frame.get("timestamp_microseconds")
    for frame in frames
    if isinstance(frame, dict) and frame.get("timestamp_microseconds") is not None
}
if len(timestamps) < minimum:
    raise RuntimeError(
        "cuVGL map has only "
        f"{len(timestamps)} synchronized frame groups; require at least {minimum}. "
        "Use an indexed MCAP capture (not fastwrite) and rebuild the map."
    )
print(f"cuVGL synchronized frame groups: {len(timestamps)}")
PY
rm -rf "${MAP_DIR}/cuvgl"
cp -a "${WORK}/cuvgl_map" "${MAP_DIR}/cuvgl"
rm -rf "${MAP_DIR}/optimized_frames"
cp -a "${SHARED_FRAMES}" "${MAP_DIR}/optimized_frames"
python3 "${PROJECT_ROOT}/tools/prepare_vgl_runtime_config.py" \
  "$(ros2 pkg prefix isaac_ros_visual_mapping --share)/configs/isaac" \
  "${MAP_DIR}/config" --max-sync-us "${MAX_SYNC_US}"
install -m 0644 "${WORK}/sensor-bag-report.json" \
  "${MAP_DIR}/config/vgl_sensor_selection_report.json"

[[ -s "${MAP_DIR}/cuvslam/data.mdb" ]] || die "empty online cuVSLAM map"
require_file "${MAP_DIR}/optimized_frames/frames_meta.json"
require_file "${MAP_DIR}/optimized_frames/report.json"
[[ -n "$(find "${MAP_DIR}/cuvgl/keyframes" -type f -size +0c -print -quit)" ]] || die "empty cuVGL keyframes"
[[ -n "$(find "${MAP_DIR}/cuvgl/vocabulary" -type f -size +0c -print -quit)" ]] || die "empty BoW vocabulary"
[[ -n "$(find "${MAP_DIR}/cuvgl" -maxdepth 1 -type f -name 'bow_index*' -size +0c -print -quit)" ]] || die "empty BoW index"
info "Aligned visual maps created in ${MAP_DIR}"
