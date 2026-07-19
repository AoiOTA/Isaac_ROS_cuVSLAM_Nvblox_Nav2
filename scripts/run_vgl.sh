#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-kujiale_latest_20260719_160004}"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
[[ -d "${MAP_DIR}/cuvgl" ]] || die "cuVGL map missing: ${MAP_DIR}/cuvgl"
[[ -d "${MAP_DIR}/cuvslam" ]] || die "cuVSLAM map missing: ${MAP_DIR}/cuvslam"
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"
[[ -n "$(find "${MODEL_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c -print -quit)" ]] || \
  die "TensorRT engines missing; run ./scripts/export_vgl_models.sh"

exec ros2 launch jackal_bringup vgl.launch.py \
  vgl_map_dir:="${MAP_DIR}/cuvgl" \
  vgl_config_dir:="${MAP_DIR}/config" \
  vgl_model_dir:="${MODEL_DIR}" \
  cuvslam_map_dir:="${MAP_DIR}/cuvslam" \
  vgl_params:="$(ros2 pkg prefix jackal_bringup --share)/config/vgl_navigation_6cam.yaml"
