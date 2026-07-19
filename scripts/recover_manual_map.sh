#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MAP_NAME=""
BAG_DIR=""
RUN_ID=""
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --bag) BAG_DIR="${2:?missing bag directory}"; shift 2 ;;
    --run-id) RUN_ID="${2:?missing mapping run id}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/recover_manual_map.sh --map NAME [--bag BAG_DIR] [--run-id RUN_ID]"
      echo "Completes cuVGL conversion and manifest promotion from a retained manual MCAP."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ -n "${MAP_NAME}" ]] || die "--map is required"
[[ "${MAP_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "invalid map name"

MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
[[ -d "${MAP_DIR}" ]] || die "partial map directory not found: ${MAP_DIR}"
[[ ! -e "${MAP_DIR}/manifest.json" ]] || die "map is already promoted: ${MAP_DIR}/manifest.json"

if [[ -z "${BAG_DIR}" ]]; then
  shopt -s nullglob
  bag_candidates=("${PROJECT_ROOT}/data/bags/.${MAP_NAME}."*/capture)
  shopt -u nullglob
  ((${#bag_candidates[@]} == 1)) || \
    die "expected exactly one retained bag for ${MAP_NAME}; found ${#bag_candidates[@]}; pass --bag explicitly"
  BAG_DIR="${bag_candidates[0]}"
fi
[[ -f "${BAG_DIR}/metadata.yaml" ]] || die "retained rosbag metadata not found: ${BAG_DIR}"

if [[ -z "${RUN_ID}" ]]; then
  shopt -s nullglob
  run_candidates=()
  for report in "${PROJECT_ROOT}/data/logs/mapping/"*/mapping-validation.json; do
    run_dir="$(dirname -- "${report}")"
    if rg -Fq "${MAP_DIR}" "${run_dir}"/save-cuvslam.log 2>/dev/null; then
      run_candidates+=("$(basename -- "${run_dir}")")
    fi
  done
  shopt -u nullglob
  ((${#run_candidates[@]} == 1)) || \
    die "could not infer one mapping run for ${MAP_NAME}; pass --run-id explicitly"
  RUN_ID="${run_candidates[0]}"
fi
[[ "${RUN_ID}" =~ ^[0-9]{8}T[0-9]{6}Z$ ]] || die "invalid mapping run id: ${RUN_ID}"
LOG_DIR="${PROJECT_ROOT}/data/logs/mapping/${RUN_ID}"
MAPPING_VALIDATION="${LOG_DIR}/mapping-validation.json"
require_file "${MAPPING_VALIDATION}"

python3 - "${MAP_DIR}" "${MAPPING_VALIDATION}" <<'PY'
import json
import sys
from pathlib import Path

map_dir = Path(sys.argv[1])
validation_path = Path(sys.argv[2])
for relative in ("cuvslam/save_report.json",):
    path = map_dir / relative
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("status") != "passed":
        raise RuntimeError(f"partial artifact did not pass: {path}")
validation = json.loads(validation_path.read_text(encoding="utf-8"))
if validation.get("status") not in {"passed", "passed_with_warnings"}:
    raise RuntimeError(f"mapping validation did not pass: {validation_path}")
PY

mkdir -p "${PROJECT_ROOT}/data/locks"
exec 9>"${PROJECT_ROOT}/data/locks/mapping-workflow.lock"
flock -n 9 || die "another mapping workflow is already running"

info "Recovering cuVGL and manifest for ${MAP_NAME} from retained MCAP"
"${PROJECT_ROOT}/scripts/export_vgl_models.sh" "${PROJECT_ROOT}/data/models/vgl" \
  >"${LOG_DIR}/model-export-recovery.log" 2>&1
if ! "${PROJECT_ROOT}/scripts/create_vgl_map.sh" "${BAG_DIR}" "${MAP_DIR}" \
  --topic-config "${PROJECT_ROOT}/ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml" \
  --max-sync-us "${MAPPING_MAX_SYNC_US:-40000}" \
  >"${LOG_DIR}/offline-map-recovery.log" 2>&1; then
  info "cuVGL recovery failed; retained MCAP and partial map were preserved."
  info "Detailed log: ${LOG_DIR}/offline-map-recovery.log"
  tail -30 "${LOG_DIR}/offline-map-recovery.log" >&2 || true
  exit 1
fi
if ! "${PROJECT_ROOT}/scripts/create_offline_occupancy_map.sh" \
  "${BAG_DIR}" "${MAP_DIR}" \
  >"${LOG_DIR}/offline-occupancy-recovery.log" 2>&1; then
  info "Optimized occupancy recovery failed; retained MCAP and visual maps were preserved."
  info "Detailed log: ${LOG_DIR}/offline-occupancy-recovery.log"
  tail -30 "${LOG_DIR}/offline-occupancy-recovery.log" >&2 || true
  exit 1
fi

python3 "${PROJECT_ROOT}/tools/write_map_manifest.py" "${MAP_DIR}" "${BAG_DIR}" \
  --run-id "${RUN_ID}" \
  --mapping-validation "${MAPPING_VALIDATION}" \
  --capture-retention kept_local \
  --capture-archive "${BAG_DIR}" \
  --generation-command "./scripts/recover_manual_map.sh --map {map_name}"
python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" "${MAP_DIR}"
info "Map recovery complete: ${MAP_DIR}"
info "Retained MCAP remains at ${BAG_DIR} until live localization is verified."
