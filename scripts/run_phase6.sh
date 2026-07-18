#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REQUESTED_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros
[[ -z "${REQUESTED_ROS_DOMAIN_ID}" ]] || export ROS_DOMAIN_ID="${REQUESTED_ROS_DOMAIN_ID}"

exec ros2 launch jackal_bringup phase6.launch.py "$@"
