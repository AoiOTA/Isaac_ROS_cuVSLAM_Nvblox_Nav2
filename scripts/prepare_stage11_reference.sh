#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_project_environment

MAP_NAME="warehouse_v2_front"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/prepare_stage11_reference.sh [--map NAME]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

MAP_YAML="${PROJECT_ROOT}/data/maps/${MAP_NAME}/occupancy/map.yaml"
require_file "${MAP_YAML}"
WAREHOUSE_USD="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["warehouse"]["resolved_path"])' "${PROJECT_ROOT}/config/assets.yaml")"
require_file "${WAREHOUSE_USD}"
OUTPUT_DIR="${PROJECT_ROOT}/data/reference/warehouse_usd_005"
mkdir -p "${OUTPUT_DIR}"

"${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/tools/extract_usd_collision_geometry.py" \
  --usd "${WAREHOUSE_USD}" \
  --output "${OUTPUT_DIR}/geometry.json" \
  --spawn-world -0.5 -0.5 0.03 \
  --vertical-band 0.05 1.20
python3 "${PROJECT_ROOT}/tools/build_stage11_reference_paths.py" \
  --geometry "${OUTPUT_DIR}/geometry.json" \
  --stage11-config "${PROJECT_ROOT}/config/stage11.yaml" \
  --map-yaml "${MAP_YAML}" \
  --output "${OUTPUT_DIR}/paths.json" \
  --grid-pgm "${OUTPUT_DIR}/collision_grid.pgm"
info "Stage 11 USD reference paths ready: ${OUTPUT_DIR}/paths.json"
