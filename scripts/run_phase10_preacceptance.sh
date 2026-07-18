#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

STATIC_TRIALS="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["preacceptance"]["static_trials"])' "${PROJECT_ROOT}/config/stage10.yaml")"
DYNAMIC_TRIALS="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["preacceptance"]["dynamic_trials"])' "${PROJECT_ROOT}/config/stage10.yaml")"
SEED_BASE="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["preacceptance"]["seed_base"])' "${PROJECT_ROOT}/config/stage10.yaml")"
MAP_NAME="warehouse_v2_front"
RECORD_BAG="true"
BUILD="true"
MATRIX_ID="$(date -u +%Y%m%dT%H%M%S)-phase10-$$"
REUSE_STATIC_MATRIX=""
REUSE_PASSED_DYNAMIC_MATRIX=""
while (($#)); do
  case "$1" in
    --static-trials) STATIC_TRIALS="${2:?missing count}"; shift 2 ;;
    --dynamic-trials) DYNAMIC_TRIALS="${2:?missing count}"; shift 2 ;;
    --seed-base) SEED_BASE="${2:?missing seed}"; shift 2 ;;
    --map) MAP_NAME="${2:?missing map}"; shift 2 ;;
    --matrix-id) MATRIX_ID="${2:?missing id}"; shift 2 ;;
    --reuse-static-matrix) REUSE_STATIC_MATRIX="${2:?missing matrix id}"; shift 2 ;;
    --reuse-passed-dynamic-matrix) REUSE_PASSED_DYNAMIC_MATRIX="${2:?missing matrix id}"; shift 2 ;;
    --record-bag) RECORD_BAG="true"; shift ;;
    --no-bag) RECORD_BAG="false"; shift ;;
    --skip-build) BUILD="false"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_phase10_preacceptance.sh [--static-trials N] [--dynamic-trials N] [--seed-base N] [--record-bag|--no-bag] [--skip-build] [--reuse-static-matrix ID] [--reuse-passed-dynamic-matrix ID]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
for value in "${STATIC_TRIALS}" "${DYNAMIC_TRIALS}" "${SEED_BASE}"; do
  [[ "${value}" =~ ^[0-9]+$ ]] || die "trial counts and seed must be non-negative integers"
done
(( STATIC_TRIALS > 0 && DYNAMIC_TRIALS > 0 )) || die "both trial classes require at least one run"
mkdir -p "${PROJECT_ROOT}/data/runs/.locks"
exec {MATRIX_LOCK_FD}>"${PROJECT_ROOT}/data/runs/.locks/${MATRIX_ID}.lock"
flock -n "${MATRIX_LOCK_FD}" || die "matrix id is already active: ${MATRIX_ID}"

# A matrix owns the physical simulator/GPU for its complete lifetime. Keep a
# separate orchestration lock from navigation_sim.py's short-lived process lock
# so accidentally launching the same acceptance command twice cannot create
# overlapping trials or have two shells write the same run directory.
mkdir -p "${PROJECT_ROOT}/data/locks"
exec 8>"${PROJECT_ROOT}/data/locks/phase10-matrix.lock"
flock -n 8 || die "another Stage 10 matrix is already running"

if [[ "${BUILD}" == "true" ]]; then
  "${PROJECT_ROOT}/scripts/build.sh"
  set +u
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  set -u
fi

REPORT_DIR="${PROJECT_ROOT}/data/reports/phase10/preacceptance/${MATRIX_ID}"
[[ ! -e "${REPORT_DIR}/summary.json" ]] || \
  die "matrix id already has a completed summary: ${MATRIX_ID}"
mkdir -p "${REPORT_DIR}"
reports=()
failures=0
trial_serial=0
if [[ -n "${REUSE_STATIC_MATRIX}" ]]; then
  shopt -s nullglob
  reused_static_reports=(
    "${PROJECT_ROOT}"/data/runs/"${REUSE_STATIC_MATRIX}"-static-*/result.json
  )
  shopt -u nullglob
  (( ${#reused_static_reports[@]} == STATIC_TRIALS )) || \
    die "reuse matrix ${REUSE_STATIC_MATRIX} has ${#reused_static_reports[@]}/${STATIC_TRIALS} static reports"
  for report in "${reused_static_reports[@]}"; do
    status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${report}")"
    [[ "${status}" == "passed" ]] || die "reused static trial did not pass: ${report}"
    reports+=("${report}")
  done
  info "Reusing ${STATIC_TRIALS} passed static reports from ${REUSE_STATIC_MATRIX}"
fi
for experiment_class in static dynamic; do
  count="${STATIC_TRIALS}"
  [[ "${experiment_class}" == "dynamic" ]] && count="${DYNAMIC_TRIALS}"
  if [[ "${experiment_class}" == "static" && -n "${REUSE_STATIC_MATRIX}" ]]; then
    continue
  fi
  for ((index=0; index<count; index++)); do
    trial_serial=$((trial_serial + 1))
    seed_offset=0
    [[ "${experiment_class}" == "dynamic" ]] && seed_offset=10000
    seed=$((SEED_BASE + seed_offset + index))
    goal_index=$((index % 3))
    run_id="${MATRIX_ID}-${experiment_class}-$(printf '%02d' $((index + 1)))"
    run_dir="${PROJECT_ROOT}/data/runs/${run_id}"
    if [[ "${experiment_class}" == "dynamic" && -n "${REUSE_PASSED_DYNAMIC_MATRIX}" ]]; then
      reused_dynamic="false"
      IFS=',' read -r -a reuse_dynamic_matrices <<<"${REUSE_PASSED_DYNAMIC_MATRIX}"
      for reuse_matrix in "${reuse_dynamic_matrices[@]}"; do
        reused_run_id="${reuse_matrix}-dynamic-$(printf '%02d' $((index + 1)))"
        reused_report="${PROJECT_ROOT}/data/runs/${reused_run_id}/result.json"
        if [[ -f "${reused_report}" ]]; then
          reused_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${reused_report}")"
          if [[ "${reused_status}" == "passed" ]]; then
            reports+=("${reused_report}")
            info "Reusing passed dynamic $((index + 1))/${count} from ${reuse_matrix}"
            reused_dynamic="true"
            break
          fi
        fi
      done
      [[ "${reused_dynamic}" == "true" ]] && continue
    fi
    domain=$((70 + (trial_serial % 160)))
    port=$((11900 + trial_serial))
    bag_arg="--record-bag"
    [[ "${RECORD_BAG}" == "true" ]] || bag_arg="--no-bag"
    info "Pre-acceptance ${experiment_class} $((index + 1))/${count}: seed=${seed}, goal=${goal_index}"
    set +e
    PHASE10_ROS_DOMAIN_ID="${domain}" PHASE10_DISCOVERY_PORT="${port}" \
      "${PROJECT_ROOT}/scripts/run_phase10_trial.sh" \
      --class "${experiment_class}" --seed "${seed}" --goal-index "${goal_index}" \
      --map "${MAP_NAME}" --headless --no-rviz "${bag_arg}" \
      --run-id "${run_id}" --run-dir "${run_dir}"
    status=$?
    set -e
    (( status == 0 )) || failures=$((failures + 1))
    if (( status != 0 )) && [[ ! -s "${run_dir}/simulator.json" ]]; then
      die "infrastructure failure: simulator produced no report for ${run_id}; stop the matrix before starting another trial"
    fi
    if [[ -f "${run_dir}/result.json" ]]; then
      reports+=("${run_dir}/result.json")
    else
      failures=$((failures + 1))
      info "Trial did not produce result.json: ${run_dir}"
    fi
  done
done

(( ${#reports[@]} == STATIC_TRIALS + DYNAMIC_TRIALS )) || \
  die "only ${#reports[@]}/$((STATIC_TRIALS + DYNAMIC_TRIALS)) trial reports were produced"
set +e
python3 "${PROJECT_ROOT}/tools/summarize_stage10_trials.py" \
  "${REPORT_DIR}/summary.json" "${reports[@]}" \
  --config "${PROJECT_ROOT}/config/stage10.yaml" \
  --expected-static "${STATIC_TRIALS}" --expected-dynamic "${DYNAMIC_TRIALS}" \
  >"${REPORT_DIR}/summary.log" 2>&1
summary_status=$?
set -e
cp "${REPORT_DIR}/summary.json" "${PROJECT_ROOT}/data/reports/phase10/preacceptance-summary-latest.json"
cp "${REPORT_DIR}/trials.csv" "${PROJECT_ROOT}/data/reports/phase10/preacceptance-trials-latest.csv"
cat "${REPORT_DIR}/summary.log"
(( summary_status == 0 )) || die "Stage 10 pre-acceptance failed (${failures} trial command failures)"
info "Stage 10 pre-acceptance passed: ${REPORT_DIR}/summary.json"
