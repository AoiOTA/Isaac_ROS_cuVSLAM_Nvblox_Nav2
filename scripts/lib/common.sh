#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
export PROJECT_ROOT

# shellcheck source=/dev/null
source "${PROJECT_ROOT}/config/environment.env"

die() {
  echo "error: $*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "required file not found: $1"
}

require_executable() {
  [[ -x "$1" ]] || die "required executable not found: $1"
}

load_ros_environment() {
  require_file "${ROS_SETUP}"
  # ROS setup files may read optional variables that are unset in a clean shell.
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"

  if [[ -f "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  fi
  set -u
}

# Backward-compatible short names used by phase-specific entry points.
load_project_environment() {
  : "${PROJECT_ROOT:?project environment is not loaded}"
}

load_ros() {
  load_ros_environment
}

info() {
  echo "[jackal] $*"
}
