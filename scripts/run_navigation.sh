#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALLER_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros
if [[ -n "${CALLER_ROS_DOMAIN_ID}" ]]; then
  export ROS_DOMAIN_ID="${CALLER_ROS_DOMAIN_ID}"
fi

MAP_NAME="warehouse_v1"
RVIZ="true"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_navigation.sh [--map NAME] [--rviz|--no-rviz]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
for path in \
  "${MAP_DIR}/occupancy/map.yaml" "${MAP_DIR}/cuvslam/data.mdb" \
  "${MAP_DIR}/cuvgl/bow_index.pb"; do
  require_file "${path}"
done
[[ -d "${MAP_DIR}/config" ]] || die "cuVGL config directory missing: ${MAP_DIR}/config"
[[ -n "$(find "${MODEL_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c -print -quit)" ]] || \
  die "cuVGL TensorRT engines missing: ${MODEL_DIR}"

exec ros2 launch nova_carter_bringup phase8.launch.py \
  map:="${MAP_DIR}/occupancy/map.yaml" \
  vgl_map_dir:="${MAP_DIR}/cuvgl" \
  vgl_config_dir:="${MAP_DIR}/config" \
  vgl_model_dir:="${MODEL_DIR}" \
  cuvslam_map_dir:="${MAP_DIR}/cuvslam" \
  rviz:="${RVIZ}"
