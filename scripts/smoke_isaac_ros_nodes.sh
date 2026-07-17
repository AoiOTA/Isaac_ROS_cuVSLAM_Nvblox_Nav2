#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

LOG_DIR="${PROJECT_ROOT}/data/logs/stage1"
mkdir -p "${LOG_DIR}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"

run_component() {
  local label="$1" package="$2" plugin="$3"
  shift 3
  local log_file="${LOG_DIR}/${label}-${RUN_ID}.log"
  info "Loading ${label}: ${plugin}"

  set +e
  timeout --signal=INT --kill-after=5s 12s \
    ros2 component standalone --no-daemon --use-sim-time \
    "${package}" "${plugin}" "$@" >"${log_file}" 2>&1
  local status=$?
  set -e

  # timeout returns 124 only when the component remained alive for the entire
  # smoke interval, which is the expected waiting-for-input state.
  if [[ ${status} -ne 124 ]]; then
    tail -n 120 "${log_file}" >&2 || true
    die "${label} exited before the smoke interval ended (status ${status})"
  fi
  info "${label} stayed alive waiting for input; log: ${log_file}"
}

run_component \
  cuvslam \
  isaac_ros_visual_slam \
  nvidia::isaac_ros::visual_slam::VisualSlamNode \
  -p enable_image_denoising:=false

run_component \
  nvblox \
  nvblox_ros \
  nvblox::NvbloxNode \
  -p num_cameras:=1 \
  -p use_depth:=true \
  -p use_lidar:=false \
  -p global_frame:=odom

run_component \
  vgl \
  isaac_ros_visual_global_localization \
  nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode \
  -p num_cameras:=2 \
  -p stereo_localizer_cam_ids:="0,1" \
  -p enable_continuous_localization:=false \
  -p publish_map_to_base_tf:=false \
  -p map_frame:=map \
  -p base_frame:=base_link

info "All three Isaac ROS core components passed the startup smoke test"
