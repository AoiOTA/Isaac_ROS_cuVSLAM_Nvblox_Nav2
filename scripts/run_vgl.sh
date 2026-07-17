#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v1}"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
[[ -d "${MAP_DIR}/cuvgl" ]] || die "cuVGL map missing: ${MAP_DIR}/cuvgl"
[[ -d "${MAP_DIR}/cuvslam" ]] || die "cuVSLAM map missing: ${MAP_DIR}/cuvslam"
[[ -n "$(find "${MODEL_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c -print -quit)" ]] || \
  die "TensorRT engines missing; run ./scripts/export_vgl_models.sh"

exec ros2 launch nova_carter_bringup vgl.launch.py \
  vgl_map_dir:="${MAP_DIR}/cuvgl" \
  vgl_config_dir:="${MAP_DIR}/config" \
  vgl_model_dir:="${MODEL_DIR}" \
  cuvslam_map_dir:="${MAP_DIR}/cuvslam"
