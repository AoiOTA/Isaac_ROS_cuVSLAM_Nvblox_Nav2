#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

BAG="${1:?usage: create_vgl_map.sh BAG [MAP_DIR] [--topic-config FILE]}"
shift
MAP_DIR="${PROJECT_ROOT}/data/maps/kujiale_jackal_8cam"
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
if [[ -n "${TUM_POSE_FILE}" ]]; then
  require_file "${TUM_POSE_FILE}"
fi
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
info "Creating a cuVGL map from the saved online cuVSLAM trajectory"
mkdir -p "${WORK}/edex"
CONVERTER_ARGS=(
  --output_folder_path="${WORK}/edex" \
  --sensor_data_bag_file="${BAG}" \
  --min_inter_frame_distance=0.1 \
  --min_inter_frame_rotation_degrees=2.0 \
  --sample_sync_threshold_microseconds="${MAX_SYNC_US}" \
  --generate_edex=True \
  --image_extension=.jpg \
  --base_link_name=base_link \
  --camera_topic_config="${TOPIC_CONFIG}" \
  --rectify_images=True
)
if [[ -n "${TUM_POSE_FILE}" ]]; then
  CONVERTER_ARGS+=(--tum_pose_file="${TUM_POSE_FILE}")
else
  info "Using recorded /tf poses for cuVGL conversion"
fi
ros2 run isaac_mapping_ros rosbag_to_mapping_data "${CONVERTER_ARGS[@]}"

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
python3 "${PROJECT_ROOT}/tools/prepare_vgl_runtime_config.py" \
  "$(ros2 pkg prefix isaac_ros_visual_mapping --share)/configs/isaac" \
  "${MAP_DIR}/config" --max-sync-us "${MAX_SYNC_US}"

[[ -s "${MAP_DIR}/cuvslam/data.mdb" ]] || die "empty online cuVSLAM map"
[[ -n "$(find "${MAP_DIR}/cuvgl/keyframes" -type f -size +0c -print -quit)" ]] || die "empty cuVGL keyframes"
[[ -n "$(find "${MAP_DIR}/cuvgl/vocabulary" -type f -size +0c -print -quit)" ]] || die "empty BoW vocabulary"
[[ -n "$(find "${MAP_DIR}/cuvgl" -maxdepth 1 -type f -name 'bow_index*' -size +0c -print -quit)" ]] || die "empty BoW index"
info "Aligned visual maps created in ${MAP_DIR}"
