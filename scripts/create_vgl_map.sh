#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

BAG="${1:?usage: create_vgl_map.sh BAG [MAP_DIR]}"
MAP_DIR="${2:-${PROJECT_ROOT}/data/maps/warehouse_v1}"
[[ -d "${BAG}" ]] || die "MCAP rosbag directory not found: ${BAG}"
TOPIC_CONFIG="${PROJECT_ROOT}/ros2_ws/install/nova_carter_bringup/share/nova_carter_bringup/config/mapping_topics.yaml"
WORK="${MAP_DIR}/offline"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
rm -rf "${WORK}"
mkdir -p "${WORK}" "${MAP_DIR}/config"
export ISAAC_ROS_WS="${PROJECT_ROOT}/ros2_ws"

info "Creating aligned cuVSLAM and cuVGL maps from ${BAG}"
ros2 run isaac_mapping_ros create_map_offline.py \
  --sensor_data_bag="${BAG}" \
  --map_dir="${WORK}" \
  --steps_to_run edex compute_poses \
  --camera_topic_config="${TOPIC_CONFIG}" \
  --base_link_name=base_link \
  --use_raw_image=True \
  --print_mode=tail

rm -rf "${WORK}/cuvgl_map"
info "Creating cuVGL BoW map with the project TensorRT cache"
ros2 run isaac_ros_visual_mapping create_cuvgl_map.py \
  --map_folder="${WORK}/cuvgl_map" \
  --raw_image_folder="${WORK}/map_frames/rectified" \
  --extract_feature --build_bow_index --feature_type=aliked --print_mode=tail \
  --binary_folder_path="$(ros2 pkg prefix isaac_ros_visual_mapping)/lib/isaac_ros_visual_mapping" \
  --config_folder_path="$(ros2 pkg prefix isaac_ros_visual_mapping --share)/configs/isaac" \
  --model_dir="${MODEL_DIR}"

[[ -d "${WORK}/cuvslam_map" ]] || die "offline cuVSLAM map was not created"
[[ -d "${WORK}/cuvgl_map" ]] || die "cuVGL map was not created"
rm -rf "${MAP_DIR}/cuvslam" "${MAP_DIR}/cuvgl"
cp -a "${WORK}/cuvslam_map" "${MAP_DIR}/cuvslam"
cp -a "${WORK}/cuvgl_map" "${MAP_DIR}/cuvgl"
cp -a "$(ros2 pkg prefix isaac_ros_visual_mapping --share)/configs/isaac/." \
  "${MAP_DIR}/config/"

[[ -n "$(find "${MAP_DIR}/cuvslam" -type f -size +0c -print -quit)" ]] || die "empty cuVSLAM map"
[[ -n "$(find "${MAP_DIR}/cuvgl/keyframes" -type f -size +0c -print -quit)" ]] || die "empty cuVGL keyframes"
[[ -n "$(find "${MAP_DIR}/cuvgl/vocabulary" -type f -size +0c -print -quit)" ]] || die "empty BoW vocabulary"
[[ -n "$(find "${MAP_DIR}/cuvgl" -maxdepth 1 -type f -name 'bow_index*' -size +0c -print -quit)" ]] || die "empty BoW index"
info "Aligned visual maps created in ${MAP_DIR}"
