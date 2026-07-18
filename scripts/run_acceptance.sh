#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

read_config() {
  python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1])); print(c["acceptance"]["trial_counts"][sys.argv[2]])' \
    "${PROJECT_ROOT}/config/stage11.yaml" "$1"
}
CONFIG_STATIC_TRIALS="$(read_config static)"
CONFIG_DYNAMIC_TRIALS="$(read_config dynamic)"
CONFIG_HETEROGENEOUS_TRIALS="$(read_config heterogeneous)"
STATIC_TRIALS="${CONFIG_STATIC_TRIALS}"
DYNAMIC_TRIALS="${CONFIG_DYNAMIC_TRIALS}"
HETEROGENEOUS_TRIALS="${CONFIG_HETEROGENEOUS_TRIALS}"
SEED_BASE="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["acceptance"]["seed_base"])' "${PROJECT_ROOT}/config/stage11.yaml")"
GOAL_COUNT="$(python3 -c 'import sys,yaml; print(len(yaml.safe_load(open(sys.argv[1]))["goals"]))' "${PROJECT_ROOT}/config/stage11.yaml")"
MAP_NAME="warehouse_v2_front"
MATRIX_ID="$(date -u +%Y%m%dT%H%M%S)-stage11-$$"
RECORD_BAG="true"
BUILD="true"
RESUME="false"
INFRASTRUCTURE_RETRIES="2"
while (($#)); do
  case "$1" in
    --static-trials) STATIC_TRIALS="${2:?missing count}"; shift 2 ;;
    --dynamic-trials) DYNAMIC_TRIALS="${2:?missing count}"; shift 2 ;;
    --heterogeneous-trials) HETEROGENEOUS_TRIALS="${2:?missing count}"; shift 2 ;;
    --seed-base) SEED_BASE="${2:?missing seed}"; shift 2 ;;
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --matrix-id) MATRIX_ID="${2:?missing matrix id}"; shift 2 ;;
    --record-bag) RECORD_BAG="true"; shift ;;
    --no-bag) RECORD_BAG="false"; shift ;;
    --skip-build) BUILD="false"; shift ;;
    --resume) RESUME="true"; shift ;;
    --infrastructure-retries) INFRASTRUCTURE_RETRIES="${2:?missing retry count}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_acceptance.sh [--matrix-id ID] [--resume] [--record-bag|--no-bag] [--static-trials 10 --dynamic-trials 10 --heterogeneous-trials 10]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
for value in "${STATIC_TRIALS}" "${DYNAMIC_TRIALS}" "${HETEROGENEOUS_TRIALS}" "${SEED_BASE}" "${INFRASTRUCTURE_RETRIES}"; do
  [[ "${value}" =~ ^[0-9]+$ ]] || die "trial counts and seed must be non-negative integers"
done
(( STATIC_TRIALS > 0 && DYNAMIC_TRIALS > 0 && HETEROGENEOUS_TRIALS > 0 )) || \
  die "all three Stage 11 classes require at least one trial"
[[ "${STATIC_TRIALS}" == "${CONFIG_STATIC_TRIALS}" \
  && "${DYNAMIC_TRIALS}" == "${CONFIG_DYNAMIC_TRIALS}" \
  && "${HETEROGENEOUS_TRIALS}" == "${CONFIG_HETEROGENEOUS_TRIALS}" ]] || \
  die "formal Stage 11 trial counts are fixed by config/stage11.yaml at ${CONFIG_STATIC_TRIALS}/${CONFIG_DYNAMIC_TRIALS}/${CONFIG_HETEROGENEOUS_TRIALS}"

mkdir -p "${PROJECT_ROOT}/data/locks" "${PROJECT_ROOT}/data/runs/.locks"
exec 8>"${PROJECT_ROOT}/data/locks/stage11-matrix.lock"
flock -n 8 || die "another Stage 11 acceptance matrix is already running"
exec {MATRIX_ID_FD}>"${PROJECT_ROOT}/data/runs/.locks/${MATRIX_ID}.lock"
flock -n "${MATRIX_ID_FD}" || die "matrix id is already active: ${MATRIX_ID}"

if [[ "${BUILD}" == "true" ]]; then
  "${PROJECT_ROOT}/scripts/build.sh"
  set +u
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  set -u
fi
"${PROJECT_ROOT}/scripts/prepare_stage11_reference.sh" --map "${MAP_NAME}"

REPORT_DIR="${PROJECT_ROOT}/data/reports/phase11/acceptance/${MATRIX_ID}"
if [[ -f "${REPORT_DIR}/summary.json" ]]; then
  [[ "${RESUME}" == "true" ]] || die "matrix already completed: ${MATRIX_ID}"
  summary_matches="$(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); expected={"static":int(sys.argv[2]),"dynamic":int(sys.argv[3]),"heterogeneous":int(sys.argv[4])}; classes=r.get("classes",{}); print(str(r.get("status")=="passed" and r.get("expected_trial_count")==sum(expected.values()) and all(classes.get(name,{}).get("expected_trial_count")==count for name,count in expected.items())).lower())' "${REPORT_DIR}/summary.json" "${STATIC_TRIALS}" "${DYNAMIC_TRIALS}" "${HETEROGENEOUS_TRIALS}")"
  [[ "${summary_matches}" != "true" ]] || {
    info "Stage 11 matrix already passed: ${REPORT_DIR}/summary.json"
    exit 0
  }
  info "Existing summary does not match the configured full matrix; continuing exact-identity resume"
fi
mkdir -p "${REPORT_DIR}"

reports=()
failures=0
trial_serial=0
for experiment_class in static dynamic heterogeneous; do
  case "${experiment_class}" in
    static) count="${STATIC_TRIALS}"; seed_offset=0 ;;
    dynamic) count="${DYNAMIC_TRIALS}"; seed_offset=10000 ;;
    heterogeneous) count="${HETEROGENEOUS_TRIALS}"; seed_offset=20000 ;;
  esac
  for ((index=0; index<count; index++)); do
    trial_serial=$((trial_serial + 1))
    seed=$((SEED_BASE + seed_offset + index))
    goal_index=$((index % GOAL_COUNT))
    base_id="${MATRIX_ID}-${experiment_class}-$(printf '%03d' $((index + 1)))"
    base_dir="${PROJECT_ROOT}/data/runs/${base_id}"
    reused="false"
    if [[ "${RESUME}" == "true" ]]; then
      shopt -s nullglob
      candidates=("${base_dir}/result.json" "${base_dir}"-retry*/result.json)
      shopt -u nullglob
      for existing_report in "${candidates[@]}"; do
        [[ -s "${existing_report}" ]] || continue
        existing_status="$(python3 -c 'import json,sys,pathlib
try:
 r=json.load(open(sys.argv[1])); n=json.load(open(pathlib.Path(sys.argv[1]).parent/"navigation.json"))
 valid=(r.get("status") in ("passed","failed") and r.get("stage")==11 and r.get("seed")==int(sys.argv[2]) and r.get("experiment_class")==sys.argv[3] and r.get("goal_index")==int(sys.argv[4]) and isinstance(n.get("goals"),list) and len(n["goals"])>0)
 print(r.get("status") if valid else "invalid")
except Exception:
 print("invalid")' "${existing_report}" "${seed}" "${experiment_class}" "${goal_index}")"
        if [[ "${existing_status}" == "passed" || "${existing_status}" == "failed" ]]; then
          reports+=("${existing_report}")
          [[ "${existing_status}" != "failed" ]] || failures=$((failures + 1))
          info "Resuming completed ${experiment_class} $((index + 1))/${count} (${existing_status}) from ${existing_report}"
          reused="true"
          break
        fi
      done
    fi
    [[ "${reused}" == "true" ]] && continue
    infrastructure_attempt=0
    while true; do
      run_id="${base_id}"
      run_dir="${base_dir}"
      if [[ -e "${run_dir}/scenario.yaml" ]]; then
        retry=1
        while [[ -e "${base_dir}-retry${retry}/scenario.yaml" ]]; do retry=$((retry + 1)); done
        run_id="${base_id}-retry${retry}"
        run_dir="${PROJECT_ROOT}/data/runs/${run_id}"
      fi
      domain=$((100 + (trial_serial % 120)))
      port=$((12900 + trial_serial))
      bag_arg="--record-bag"
      [[ "${RECORD_BAG}" == "true" ]] || bag_arg="--no-bag"
      info "Stage 11 ${experiment_class} $((index + 1))/${count}: seed=${seed}, goal=${goal_index}"
      set +e
      PHASE11_ROS_DOMAIN_ID="${domain}" PHASE11_DISCOVERY_PORT="${port}" \
        "${PROJECT_ROOT}/scripts/run_stage11_trial.sh" \
        --class "${experiment_class}" --seed "${seed}" --goal-index "${goal_index}" \
        --map "${MAP_NAME}" --headless --no-rviz "${bag_arg}" \
        --run-id "${run_id}" --run-dir "${run_dir}"
      trial_status=$?
      set -e
      if [[ ! -s "${run_dir}/simulator.json" ]]; then
        die "infrastructure failure: simulator produced no report for ${run_id}"
      fi
      [[ -f "${run_dir}/result.json" ]] || die "trial produced no result: ${run_id}"

      # A seed counts as one of the formal 10/10/10 trials only after the ROS
      # runner actually attempted its goal.  A process interruption between
      # simulator readiness and runner startup must be retried, not disguised
      # as a navigation failure that consumes the statistical failure budget.
      navigation_attempted="$(python3 -c 'import json,sys; p=sys.argv[1];
try:
 r=json.load(open(p)); print(str(isinstance(r.get("goals"),list) and len(r["goals"])>0).lower())
except Exception:
 print("false")' "${run_dir}/navigation.json")"
      if [[ "${navigation_attempted}" == "true" ]]; then
        (( trial_status == 0 )) || failures=$((failures + 1))
        reports+=("${run_dir}/result.json")
        break
      fi
      infrastructure_attempt=$((infrastructure_attempt + 1))
      if (( infrastructure_attempt > INFRASTRUCTURE_RETRIES )); then
        die "infrastructure failure: no navigation goal attempt after $((INFRASTRUCTURE_RETRIES + 1)) runs for ${base_id}"
      fi
      info "Infrastructure-only empty run for ${base_id}; retrying ($((infrastructure_attempt + 1))/$((INFRASTRUCTURE_RETRIES + 1)))"
    done
  done
done

expected_total=$((STATIC_TRIALS + DYNAMIC_TRIALS + HETEROGENEOUS_TRIALS))
(( ${#reports[@]} == expected_total )) || die "only ${#reports[@]}/${expected_total} reports available"
set +e
python3 "${PROJECT_ROOT}/tools/summarize_stage11_acceptance.py" \
  "${REPORT_DIR}/summary.json" "${reports[@]}" \
  --config "${PROJECT_ROOT}/config/stage11.yaml" \
  --expected-static "${STATIC_TRIALS}" \
  --expected-dynamic "${DYNAMIC_TRIALS}" \
  --expected-heterogeneous "${HETEROGENEOUS_TRIALS}" \
  >"${REPORT_DIR}/summary.log" 2>&1
summary_status=$?
set -e
cp "${REPORT_DIR}/summary.json" "${PROJECT_ROOT}/data/reports/phase11/acceptance-summary-latest.json"
cp "${REPORT_DIR}/trials.csv" "${PROJECT_ROOT}/data/reports/phase11/acceptance-trials-latest.csv"
cp "${REPORT_DIR}/report.md" "${PROJECT_ROOT}/data/reports/phase11/acceptance-report-latest.md"
cat "${REPORT_DIR}/summary.log"
(( summary_status == 0 )) || die "Stage 11 acceptance failed (${failures} trial command failures)"
info "Stage 11 formal acceptance passed: ${REPORT_DIR}/summary.json"
