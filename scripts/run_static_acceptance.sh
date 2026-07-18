#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

ACCEPTANCE_CONFIG="${PROJECT_ROOT}/config/acceptance.yaml"
MAP_NAME="kujiale_jackal_8cam"
SIM_MODE="--headless"
OUTPUT_DIR=""
while (($#)); do
  case "$1" in
    --config) ACCEPTANCE_CONFIG="${2:?missing config}"; shift 2 ;;
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --output-dir) OUTPUT_DIR="${2:?missing output directory}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_static_acceptance.sh [--map NAME] [--headless|--gui] [--output-dir DIR]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
require_file "${ACCEPTANCE_CONFIG}"
MINIMUM_VALID="$(python3 -c 'import sys,yaml; print(int(yaml.safe_load(open(sys.argv[1]))["trials"]["minimum_valid_trials"]))' "${ACCEPTANCE_CONFIG}")"
MAX_INFRA="$(python3 -c 'import sys,yaml; print(int(yaml.safe_load(open(sys.argv[1]))["trials"]["maximum_consecutive_infrastructure_failures"]))' "${ACCEPTANCE_CONFIG}")"
GOAL_COUNT="$(python3 -c 'import sys,yaml; print(len(yaml.safe_load(open(sys.argv[1]))["goals"]))' "${ACCEPTANCE_CONFIG}")"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"

BATCH_ID="$(date -u +%Y%m%dT%H%M%S)-static-acceptance"
OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_ROOT}/data/reports/static-acceptance/${BATCH_ID}}"
[[ ! -e "${OUTPUT_DIR}" ]] || die "output directory already exists: ${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}/trials"
python3 "${PROJECT_ROOT}/tools/validate_acceptance_routes.py" "${MAP_DIR}" \
  --config "${ACCEPTANCE_CONFIG}" --output "${OUTPUT_DIR}/route-validation.json"

valid_count=0
attempt_count=0
consecutive_infra=0
reports=()
while (( valid_count < MINIMUM_VALID )); do
  attempt_count=$((attempt_count + 1))
  goal_index=$((valid_count % GOAL_COUNT))
  run_dir="${OUTPUT_DIR}/trials/attempt-$(printf '%03d' "${attempt_count}")"
  info "Acceptance progress: valid ${valid_count}/${MINIMUM_VALID}; starting attempt ${attempt_count} goal ${goal_index}"
  set +e
  "${PROJECT_ROOT}/scripts/run_static_trial.sh" --config "${ACCEPTANCE_CONFIG}" \
    --map "${MAP_NAME}" --goal-index "${goal_index}" \
    --attempt-index "${attempt_count}" --run-dir "${run_dir}" "${SIM_MODE}"
  trial_status=$?
  set -e
  require_file "${run_dir}/result.json"
  reports+=("${run_dir}/result.json")
  case "${trial_status}" in
    0|10)
      valid_count=$((valid_count + 1))
      consecutive_infra=0
      ;;
    20)
      consecutive_infra=$((consecutive_infra + 1))
      if (( consecutive_infra >= MAX_INFRA )); then
        python3 "${PROJECT_ROOT}/tools/summarize_static_avoidance.py" \
          --config "${ACCEPTANCE_CONFIG}" --output "${OUTPUT_DIR}/summary.json" \
          "${reports[@]}" || true
        die "${consecutive_infra} consecutive infrastructure-invalid attempts; inspect ${OUTPUT_DIR}"
      fi
      ;;
    *) die "unexpected static trial exit status ${trial_status}" ;;
  esac
done

set +e
python3 "${PROJECT_ROOT}/tools/summarize_static_avoidance.py" \
  --config "${ACCEPTANCE_CONFIG}" --output "${OUTPUT_DIR}/summary.json" \
  "${reports[@]}"
summary_status=$?
set -e
if (( summary_status != 0 )); then
  die "static avoidance acceptance failed: ${OUTPUT_DIR}/summary.json"
fi
info "Static avoidance acceptance passed: ${OUTPUT_DIR}/summary.md"
