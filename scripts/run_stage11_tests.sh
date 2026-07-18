#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v2_front}"
(($# <= 1)) || die "usage: ./scripts/run_stage11_tests.sh [MAP_NAME]"
"${PROJECT_ROOT}/scripts/build.sh"
set +u
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
set -u
python3 -m pytest -q \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_control/test" \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_bringup/test" \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_experiments/test" \
  "${PROJECT_ROOT}/tests"
"${PROJECT_ROOT}/scripts/run_stage11_smoke.sh" "${MAP_NAME}"
