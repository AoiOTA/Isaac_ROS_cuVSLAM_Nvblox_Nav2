#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REQUESTED_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros
[[ -z "${REQUESTED_ROS_DOMAIN_ID}" ]] || export ROS_DOMAIN_ID="${REQUESTED_ROS_DOMAIN_ID}"

OUTPUT_DIR="${1:-${PROJECT_ROOT}/data/maps/warehouse_v1/nvblox}"
STEM="${2:-warehouse}"
exec ros2 run nova_carter_experiments nvblox_map_saver --ros-args \
  -p output_dir:="$(realpath -m "${OUTPUT_DIR}")" -p stem:="${STEM}"
