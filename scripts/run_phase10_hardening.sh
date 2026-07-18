#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v2_front}"
(($# <= 1)) || die "usage: ./scripts/run_phase10_hardening.sh [MAP_NAME]"
RUN_ID="$(date -u +%Y%m%dT%H%M%S)-phase10-hardening-$$"
RUN_DIR="${PROJECT_ROOT}/data/runs/${RUN_ID}"
PHASE10_ROS_DOMAIN_ID="${PHASE10_HARDENING_DOMAIN_ID:-70}" \
PHASE10_DISCOVERY_PORT="${PHASE10_HARDENING_DISCOVERY_PORT:-11900}" \
  "${PROJECT_ROOT}/scripts/run_phase10_trial.sh" \
  --class dynamic --seed 424210 --goal-index 0 --all-goals \
  --fault-sequence relocalization,depth_stale,map_slice_stale \
  --map "${MAP_NAME}" --headless --no-rviz --record-bag \
  --run-id "${RUN_ID}" --run-dir "${RUN_DIR}"
mkdir -p "${PROJECT_ROOT}/data/reports/phase10"
cp "${RUN_DIR}/result.json" \
  "${PROJECT_ROOT}/data/reports/phase10/hardening-latest.json"
info "Stage 10 fault hardening passed: ${RUN_DIR}/result.json"
