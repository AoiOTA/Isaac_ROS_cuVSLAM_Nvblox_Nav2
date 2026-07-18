#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MAP_NAME="kujiale_jackal_8cam"
while (($#)); do
  case "$1" in
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_manual_navigation.sh [--map NAME]"
      echo "Starts Isaac Sim GUI, navigation RViz, automatic cuVGL localization, and 2D Goal Pose control."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

info "Manual navigation map: ${MAP_NAME}"
exec "${SCRIPT_DIR}/run_all.sh" \
  --map "${MAP_NAME}" --manual --gui --rviz
