#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

OUTPUT_DIR="${1:-${PROJECT_ROOT}/data/models/vgl}"
PACKAGE_SHARE="$(ros2 pkg prefix isaac_ros_visual_mapping --share)"
SOURCE_MODELS="${PACKAGE_SHARE}/models"
CONFIG_DIR="${PACKAGE_SHARE}/configs/isaac"
BIN_DIR="$(ros2 pkg prefix isaac_ros_visual_mapping)/bin/visual_mapping"

require_executable "${BIN_DIR}/export_extractor_engine"
require_executable "${BIN_DIR}/export_lightglue_engine"
mkdir -p "${OUTPUT_DIR}"
cp -a "${SOURCE_MODELS}/." "${OUTPUT_DIR}/"
mapfile -t cached < <(find "${OUTPUT_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c)
if (( ${#cached[@]} >= 2 )); then
  info "Reusing ${#cached[@]} cached TensorRT engines from ${OUTPUT_DIR}"
  printf '%s\n' "${cached[@]}"
  exit 0
fi

info "Exporting ALIKED TensorRT engine to ${OUTPUT_DIR}"
"${BIN_DIR}/export_extractor_engine" \
  --feature_type=aliked \
  --configure_file="${CONFIG_DIR}/keypoint_creation_config.pb.txt" \
  --model_dir="${SOURCE_MODELS}" \
  --output_model_dir="${OUTPUT_DIR}"

info "Exporting LightGlue TensorRT engine to ${OUTPUT_DIR}"
"${BIN_DIR}/export_lightglue_engine" \
  --feature_type=aliked \
  --worker_config_file="${CONFIG_DIR}/matching_task_worker_config.pb.txt" \
  --model_dir="${SOURCE_MODELS}" \
  --output_model_dir="${OUTPUT_DIR}"

mapfile -t engines < <(find "${OUTPUT_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c)
(( ${#engines[@]} >= 2 )) || die "expected at least two exported TensorRT engines"
printf '%s\n' "${engines[@]}"
