#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALLER_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros
if [[ -n "${CALLER_ROS_DOMAIN_ID}" ]]; then
  export ROS_DOMAIN_ID="${CALLER_ROS_DOMAIN_ID}"
fi

MAP_NAME="warehouse_v2_front"
RVIZ="true"
FAULT_INJECTION="false"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --enable-fault-injection) FAULT_INJECTION="true"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_phase10_navigation.sh [--map NAME] [--rviz|--no-rviz] [--enable-fault-injection]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
RUNTIME_CONFIG_DIR="${PROJECT_ROOT}/data/runs/phase10_runtime/${MAP_NAME}/vgl_config"
for path in \
  "${MAP_DIR}/occupancy/map.yaml" "${MAP_DIR}/cuvslam/data.mdb" \
  "${MAP_DIR}/cuvgl/bow_index.pb"; do
  require_file "${path}"
done
[[ -d "${MAP_DIR}/config" ]] || die "cuVGL config directory missing: ${MAP_DIR}/config"
[[ -n "$(find "${MODEL_DIR}" -type f \( -name '*.engine' -o -name '*.plan' \) -size +0c -print -quit)" ]] || \
  die "cuVGL TensorRT engines missing: ${MODEL_DIR}"
python3 "${PROJECT_ROOT}/tools/prepare_vgl_runtime_config.py" \
  "${MAP_DIR}/config" "${RUNTIME_CONFIG_DIR}" --max-sync-us 3000

FAILURE_FILE="${RUNTIME_CONFIG_DIR}/nav2-lifecycle-failed-${BASHPID}.json"
MAX_ATTEMPTS="${PHASE10_NAVIGATION_START_ATTEMPTS:-3}"
[[ "${MAX_ATTEMPTS}" =~ ^[1-9][0-9]*$ ]] || \
  die "PHASE10_NAVIGATION_START_ATTEMPTS must be a positive integer"
for ((attempt=1; attempt<=MAX_ATTEMPTS; attempt++)); do
  rm -f "${FAILURE_FILE}"
  force_failure="false"
  guard_delay="${PHASE10_LIFECYCLE_GUARD_DELAY:-42.0}"
  if [[ "${PHASE10_FORCE_LIFECYCLE_FAILURE_ONCE:-false}" == "true" && "${attempt}" == "1" ]]; then
    force_failure="true"
    guard_delay="1.0"
  fi
  set +e
  ros2 launch nova_carter_bringup phase10.launch.py \
    map:="${MAP_DIR}/occupancy/map.yaml" \
    vgl_map_dir:="${MAP_DIR}/cuvgl" \
    vgl_config_dir:="${RUNTIME_CONFIG_DIR}" \
    vgl_model_dir:="${MODEL_DIR}" \
    cuvslam_map_dir:="${MAP_DIR}/cuvslam" \
    enable_fault_injection:="${FAULT_INJECTION}" \
    lifecycle_failure_file:="${FAILURE_FILE}" \
    lifecycle_guard_force_failure:="${force_failure}" \
    lifecycle_guard_delay:="${guard_delay}" \
    rviz:="${RVIZ}"
  status=$?
  set -e
  if [[ -s "${FAILURE_FILE}" ]]; then
    if (( attempt < MAX_ATTEMPTS )); then
      info "Nav2 lifecycle startup failed; relaunching clean stack ($((attempt + 1))/${MAX_ATTEMPTS})"
      continue
    fi
    cat "${FAILURE_FILE}" >&2
    die "Nav2 lifecycle startup failed after ${MAX_ATTEMPTS} clean launches"
  fi
  exit "${status}"
done
