#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v2_front}"
(($# <= 1)) || die "usage: ./scripts/run_stage11_smoke.sh [MAP_NAME]"
"${PROJECT_ROOT}/scripts/prepare_stage11_reference.sh" --map "${MAP_NAME}"

stamp="$(date -u +%Y%m%dT%H%M%S)"
PHASE11_ROS_DOMAIN_ID=221 PHASE11_DISCOVERY_PORT=13121 \
  "${PROJECT_ROOT}/scripts/run_stage11_trial.sh" \
  --class static --seed 21991 --goal-index 3 --map "${MAP_NAME}" \
  --headless --no-rviz --no-bag --run-id "${stamp}-stage11-smoke-static-long"
PHASE11_ROS_DOMAIN_ID=222 PHASE11_DISCOVERY_PORT=13122 \
  "${PROJECT_ROOT}/scripts/run_stage11_trial.sh" \
  --class dynamic --seed 31992 --goal-index 4 --map "${MAP_NAME}" \
  --headless --no-rviz --no-bag --run-id "${stamp}-stage11-smoke-dynamic-long"
PHASE11_ROS_DOMAIN_ID=223 PHASE11_DISCOVERY_PORT=13123 \
  "${PROJECT_ROOT}/scripts/run_stage11_trial.sh" \
  --class heterogeneous --seed 41993 --goal-index 5 --map "${MAP_NAME}" \
  --headless --no-rviz --record-bag --run-id "${stamp}-stage11-smoke-heterogeneous-long"
info "Stage 11 static/dynamic/heterogeneous long-distance smoke passed"
