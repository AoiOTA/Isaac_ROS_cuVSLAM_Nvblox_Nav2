#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

BAG_DIR="${1:?usage: create_offline_occupancy_map.sh BAG MAP_DIR [--config FILE]}"
MAP_DIR="${2:?usage: create_offline_occupancy_map.sh BAG MAP_DIR [--config FILE]}"
shift 2
CONFIG="${PROJECT_ROOT}/config/offline_mapping.yaml"
ROUTE_CONFIG="${PROJECT_ROOT}/config/acceptance.yaml"
while (($#)); do
  case "$1" in
    --config) CONFIG="${2:?missing offline mapping config}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/create_offline_occupancy_map.sh BAG MAP_DIR [--config FILE]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

require_file "${BAG_DIR}/metadata.yaml"
OPTIMIZED_FRAMES_META="${MAP_DIR}/optimized_frames/frames_meta.json"
OPTIMIZED_FRAMES_REPORT="${MAP_DIR}/optimized_frames/report.json"
require_file "${OPTIMIZED_FRAMES_META}"
require_file "${OPTIMIZED_FRAMES_REPORT}"
require_file "${CONFIG}"
require_file "${ROUTE_CONFIG}"
for group in nvblox mesh occupancy; do
  mkdir -p "${MAP_DIR}/${group}"
  if find "${MAP_DIR}/${group}" -mindepth 1 -print -quit | grep -q .; then
    die "offline map target is not empty; preserving it: ${MAP_DIR}/${group}"
  fi
done
mkdir -p "${MAP_DIR}/config"

WORK="$(mktemp -d "${PROJECT_ROOT}/data/bags/.offline-map-work.XXXXXX")"
SUCCEEDED="false"
cleanup() {
  if [[ "${SUCCEEDED}" == "true" ]]; then
    case "${WORK}" in
      "${PROJECT_ROOT}/data/bags/.offline-map-work."*) rm -rf -- "${WORK}" ;;
      *) die "refusing to remove unexpected offline workspace: ${WORK}" ;;
    esac
  else
    info "Offline workspace retained for diagnosis: ${WORK}"
  fi
}
trap cleanup EXIT INT TERM

read -r SENSOR_NAME DEPTH_TOPIC DEPTH_INFO_TOPIC COLOR_TOPIC MIN_DEPTH MAX_DEPTH MAX_SYNC < <(
  python3 - "${CONFIG}" <<'PY'
import sys
import yaml

config = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["native_depth"]
print(
    config["sensor_name"],
    config["depth_topic"],
    config["depth_camera_info_topic"],
    config["color_topic"],
    config["minimum_depth_m"],
    config["maximum_depth_m"],
    config["maximum_sync_ms"],
)
PY
)

info "Extracting native depth at shared globally optimized cuVSLAM map frames"
python3 "${PROJECT_ROOT}/tools/prepare_native_depth_fusion.py" \
  "${BAG_DIR}" "${OPTIMIZED_FRAMES_META}" "${WORK}/input" \
  --source-pose-report "${OPTIMIZED_FRAMES_REPORT}" \
  --sensor-name "${SENSOR_NAME}" \
  --depth-topic "${DEPTH_TOPIC}" \
  --depth-info-topic "${DEPTH_INFO_TOPIC}" \
  --color-topic "${COLOR_TOPIC}" \
  --minimum-depth-m "${MIN_DEPTH}" \
  --maximum-depth-m "${MAX_DEPTH}" \
  --maximum-sync-ms "${MAX_SYNC}"

info "Running optimized-pose TSDF mesh and static occupancy fusion"
python3 "${PROJECT_ROOT}/tools/run_offline_nvblox_fusion.py" \
  "${WORK}/input" "${WORK}/fusion" --config "${CONFIG}"
python3 "${PROJECT_ROOT}/tools/promote_offline_occupancy.py" \
  "${WORK}/fusion/occupancy_candidate/map.png" \
  "${WORK}/fusion/occupancy_candidate/map.yaml" \
  "${WORK}/occupancy" --config "${CONFIG}" \
  --fusion-report "${WORK}/fusion/fusion_report.json"

# Pixel fractions alone cannot detect a plausible-looking map whose doorway
# was closed by an over-thick endpoint band. Gate the fixed Kujiale acceptance
# regions with the same footprint proxy used before live navigation trials.
STAGED_MAP="${WORK}/${MAP_DIR##*/}"
mkdir -p "${STAGED_MAP}"
ln -s "${WORK}/occupancy" "${STAGED_MAP}/occupancy"
python3 "${PROJECT_ROOT}/tools/validate_acceptance_routes.py" \
  "${STAGED_MAP}" --config "${ROUTE_CONFIG}" \
  --output "${WORK}/occupancy/route_validation.json" \
  >"${WORK}/occupancy/route_validation.stdout.json"

install -m 0644 "${WORK}/fusion/nvblox/kujiale.nvblx" \
  "${MAP_DIR}/nvblox/kujiale.nvblx"
install -m 0644 "${WORK}/fusion/nvblox/kujiale_tsdf.nvblx" \
  "${MAP_DIR}/nvblox/kujiale_tsdf.nvblx"
install -m 0644 "${WORK}/fusion/nvblox/tsdf_timings.txt" \
  "${MAP_DIR}/nvblox/tsdf_timings.txt"
install -m 0644 "${WORK}/fusion/nvblox/occupancy_timings.txt" \
  "${MAP_DIR}/nvblox/occupancy_timings.txt"
install -m 0644 "${WORK}/fusion/fusion_report.json" \
  "${MAP_DIR}/nvblox/fusion_report.json"
install -m 0644 "${WORK}/input/report.json" \
  "${MAP_DIR}/nvblox/native_depth_report.json"
install -m 0644 "${WORK}/fusion/mesh/kujiale.ply" \
  "${MAP_DIR}/mesh/kujiale.ply"
install -m 0644 "${WORK}/occupancy/map.pgm" \
  "${WORK}/occupancy/map.yaml" "${WORK}/occupancy/save_report.json" \
  "${WORK}/occupancy/route_validation.json" \
  "${MAP_DIR}/occupancy/"
install -m 0644 "${CONFIG}" "${MAP_DIR}/config/offline_mapping.yaml"
install -m 0644 "${ROUTE_CONFIG}" \
  "${MAP_DIR}/config/offline_route_acceptance.yaml"

python3 - "${MAP_DIR}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
for relative in (
    "nvblox/kujiale.nvblx",
    "nvblox/kujiale_tsdf.nvblx",
    "nvblox/fusion_report.json",
    "nvblox/native_depth_report.json",
    "mesh/kujiale.ply",
    "occupancy/map.pgm",
    "occupancy/map.yaml",
    "occupancy/save_report.json",
    "occupancy/route_validation.json",
    "config/offline_mapping.yaml",
    "config/offline_route_acceptance.yaml",
):
    path = root / relative
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"offline runtime artifact is missing: {path}")
report = json.loads((root / "occupancy/save_report.json").read_text(encoding="utf-8"))
if report.get("status") != "passed" or any(
    not passed for passed in report.get("checks", {}).values()
):
    raise RuntimeError("offline occupancy did not pass every quality gate")
routes = json.loads(
    (root / "occupancy/route_validation.json").read_text(encoding="utf-8")
)
if routes.get("status") != "passed" or not routes.get("routes") or any(
    not route.get("passed", False) for route in routes["routes"]
):
    raise RuntimeError("offline occupancy did not pass every topology route gate")
PY

SUCCEEDED="true"
info "Optimized offline occupancy complete: ${MAP_DIR}/occupancy/map.yaml"
