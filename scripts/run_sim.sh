#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
load_project_environment
load_ros

usage() {
  cat <<'EOF'
Usage: ./scripts/run_sim.sh --headless|--gui [navigation_sim.py options]

Examples:
  ./scripts/run_sim.sh --headless
  ./scripts/run_sim.sh --headless --duration 60
  ./scripts/run_sim.sh --gui --duration 15

With no --duration, the simulator runs until Ctrl-C or the GUI is closed.
The script never terminates simulator processes belonging to other projects.
EOF
}

if (($# == 0)); then
  usage >&2
  exit 2
fi
case "$1" in
  --headless|--gui) ;;
  -h|--help) usage; exit 0 ;;
  *) die "first argument must be --headless or --gui" ;;
esac

require_executable "${ISAAC_SIM_PYTHON}"
require_file "${KUJIALE_USD}"
require_file "${JACKAL_USD}"
require_file "${HAWK_USD}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage4"
REPORT="${LOG_DIR}/run-${RUN_ID}.json"
LOG_FILE="${LOG_DIR}/run-${RUN_ID}.log"
mkdir -p "${LOG_DIR}"

info "Starting Kujiale/Jackal simulator with the selected camera profile"
info "Report: ${REPORT}"
info "Log: ${LOG_FILE}"
set +e
"${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
  "$@" --report "${REPORT}" 2>&1 | tee "${LOG_FILE}"
status=${PIPESTATUS[0]}
set -e

if [[ -f "${REPORT}" ]]; then
  install -m 0644 "${REPORT}" "${LOG_DIR}/latest.json"
fi
if [[ ${status} -ne 0 ]]; then
  die "simulator exited with status ${status}; inspect ${REPORT} and ${LOG_FILE}"
fi

python3 "${PROJECT_ROOT}/tools/check_stage4_sim_report.py" "${REPORT}"
info "Standalone simulator run passed"
