#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALLER_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
CALLER_ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros
if [[ -n "${CALLER_ROS_DOMAIN_ID}" ]]; then
  export ROS_DOMAIN_ID="${CALLER_ROS_DOMAIN_ID}"
fi
if [[ -n "${CALLER_ROS_DISCOVERY_SERVER}" ]]; then
  export ROS_DISCOVERY_SERVER="${CALLER_ROS_DISCOVERY_SERVER}"
  # environment.env enables simple local discovery by default. Preserve the
  # caller's loopback discovery-server mode for late-starting Nav2 processes.
  unset ROS_LOCALHOST_ONLY
fi

MAP_NAME="kujiale_latest_20260719_160004"
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
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"
[[ -n "$(find "${MODEL_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c -print -quit)" ]] || \
  die "cuVGL TensorRT engines missing: ${MODEL_DIR}"

BRINGUP_SHARE="$(ros2 pkg prefix jackal_bringup --share)"

exec ros2 launch jackal_bringup phase8.launch.py \
  map:="${MAP_DIR}/occupancy/map.yaml" \
  vgl_map_dir:="${MAP_DIR}/cuvgl" \
  vgl_config_dir:="${MAP_DIR}/config" \
  vgl_model_dir:="${MODEL_DIR}" \
  cuvslam_map_dir:="${MAP_DIR}/cuvslam" \
  camera_profile:=navigation_6cam \
  image_qos:=DEFAULT \
  visual_slam_params:="${BRINGUP_SHARE}/config/visual_slam_navigation_6cam.yaml" \
  vgl_params:="${BRINGUP_SHARE}/config/vgl_navigation_6cam.yaml" \
  rviz:="${RVIZ}"
