#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MAP_NAME="kujiale_manual_$(date +%Y%m%d_%H%M%S)"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_manual_mapping.sh [--map NAME]"
      echo "Starts Isaac Sim GUI, mapping RViz, WASD teleop, and the guarded save pipeline."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

info "Manual map name: ${MAP_NAME}"
info "Keep this terminal focused for WASD; Q validates and saves the map."
exec "${SCRIPT_DIR}/run_mapping.sh" \
  --map "${MAP_NAME}" --interactive --gui --rviz
