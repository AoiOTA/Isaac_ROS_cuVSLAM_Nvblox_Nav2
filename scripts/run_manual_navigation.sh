#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MAP_NAME="kujiale_latest_20260719_160004"
SIM_MODE="--headless"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--no-gui) SIM_MODE="--headless"; shift ;;
    --gui) SIM_MODE="--gui"; shift ;;
    -h|--help)
      echo "Usage: ./scripts/run_manual_navigation.sh [--map NAME] [--gui|--no-gui]"
      echo "Starts manual navigation with RViz, automatic cuVSLAM + cuVGL localization, and 2D Goal Pose control."
      echo "RViz is always enabled; use --gui to show Isaac Sim window, --no-gui for headless simulator."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

info "Manual navigation map: ${MAP_NAME} (${SIM_MODE} simulation)"
exec "${SCRIPT_DIR}/run_all.sh" \
  --map "${MAP_NAME}" --manual "${SIM_MODE}" --rviz
