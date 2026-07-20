#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MAP_NAME="kujiale_latest_20260719_160004"
SIM_MODE="--headless"
FOLLOW_CAMERA_ARGS=()

while (($#)); do
  case "$1" in
    --map)
      MAP_NAME="${2:?missing map name}"
      shift 2
      ;;
    --headless|--no-gui)
      SIM_MODE="--headless"
      shift
      ;;
    --gui)
      SIM_MODE="--gui"
      shift
      ;;
    --disable-follow-camera)
      FOLLOW_CAMERA_ARGS+=(--disable-follow-camera)
      shift
      ;;
    --follow-camera-distance)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-distance "$2")
      shift 2
      ;;
    --follow-camera-height)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-height "$2")
      shift 2
      ;;
    --follow-camera-look-ahead)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-look-ahead "$2")
      shift 2
      ;;
    --follow-camera-look-at-height)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-look-at-height "$2")
      shift 2
      ;;
    --follow-camera-focal-length)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-focal-length "$2")
      shift 2
      ;;
    --follow-camera-smoothing-time)
      FOLLOW_CAMERA_ARGS+=(--follow-camera-smoothing-time "$2")
      shift 2
      ;;
    -h|--help)
      echo "Usage: ./scripts/run_manual_navigation.sh [--map NAME] [--gui|--headless]"
      echo "[--disable-follow-camera]"
      echo "[--follow-camera-distance M] [--follow-camera-height M]"
      echo "[--follow-camera-look-ahead M] [--follow-camera-look-at-height M]"
      echo "[--follow-camera-focal-length M] [--follow-camera-smoothing-time S]"
      echo "Starts Isaac Sim GUI, navigation RViz, automatic cuVGL localization, and 2D Goal Pose control."
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

info "Manual navigation map: ${MAP_NAME} (${SIM_MODE} simulation)"
exec "${SCRIPT_DIR}/run_all.sh" \
  --map "${MAP_NAME}" --manual "${SIM_MODE}" --rviz "${FOLLOW_CAMERA_ARGS[@]}"
