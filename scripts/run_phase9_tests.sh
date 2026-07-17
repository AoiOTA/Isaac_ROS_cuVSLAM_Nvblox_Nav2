#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

case "${1:-}" in
  -h|--help)
    echo "Usage: ./scripts/run_phase9_tests.sh [MAP_NAME]"
    exit 0
    ;;
esac
(($# <= 1)) || die "usage: ./scripts/run_phase9_tests.sh [MAP_NAME]"
MAP_NAME="${1:-warehouse_v2_front}"
TRIALS="${PHASE9_TRIALS:-3}"
[[ "${TRIALS}" =~ ^[3-5]$ ]] || die "PHASE9_TRIALS must be an integer from 3 through 5"

info "Building and running Stage 9 contract tests"
"${PROJECT_ROOT}/scripts/build.sh"
# build.sh runs in a child shell; source the freshly generated overlay again
# so this entry also works when ros2_ws/install did not exist beforehand.
# shellcheck disable=SC1091
set +u
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
set -u
python3 -m pytest -q \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_control/test" \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_bringup/test" \
  "${PROJECT_ROOT}/ros2_ws/src/nova_carter_experiments/test"

RUN_ID="$(date -u +%Y%m%dT%H%M%S)-$$"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase9/${RUN_ID}"
mkdir -p "${REPORT_DIR}"
reports=()
for ((trial=1; trial<=TRIALS; trial++)); do
  report="${REPORT_DIR}/trial-${trial}.json"
  reports+=("${report}")
  rviz_arg="--no-rviz"
  if (( trial == 1 )); then rviz_arg="--rviz"; fi
  info "Running isolated Stage 9 trial ${trial}/${TRIALS} (${rviz_arg})"
  PHASE9_ROS_DOMAIN_ID="$((58 + trial))" \
    "${PROJECT_ROOT}/scripts/run_phase9.sh" --map "${MAP_NAME}" --headless \
    "${rviz_arg}" --report "${report}"
done
python3 "${PROJECT_ROOT}/tools/summarize_phase9_trials.py" \
  "${REPORT_DIR}/summary.json" "${reports[@]}"
cp "${REPORT_DIR}/summary.json" "${PROJECT_ROOT}/data/reports/phase9/summary-latest.json"
info "Stage 9 passed ${TRIALS}/${TRIALS} independent dynamic-navigation trials"
