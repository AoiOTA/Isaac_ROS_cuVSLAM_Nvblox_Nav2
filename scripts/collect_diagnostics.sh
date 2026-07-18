#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

OUTPUT=""
while (($#)); do
  case "$1" in
    --output) OUTPUT="${2:?missing output path}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/collect_diagnostics.sh [--output FILE]"
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
RUN_ID="$(date -u +%Y%m%dT%H%M%S)-$$"
OUTPUT="${OUTPUT:-${PROJECT_ROOT}/data/logs/diagnostics/${RUN_ID}.txt}"
mkdir -p "$(dirname "${OUTPUT}")"

section() { printf '\n[%s]\n' "$1" >>"${OUTPUT}"; }
capture() {
  local label="$1"; shift
  section "${label}"
  timeout 15s "$@" >>"${OUTPUT}" 2>&1 || printf 'command unavailable or timed out\n' >>"${OUTPUT}"
}

printf 'Nova Carter diagnostics\ncreated_utc=%s\nproject_root=%s\n' \
  "$(date -u +%FT%TZ)" "${PROJECT_ROOT}" >"${OUTPUT}"
capture "os" bash -lc 'uname -a; lsb_release -ds; uptime'
capture "gpu" nvidia-smi
capture "disk-memory" bash -lc 'free -h; df -h / /home'
capture "git" git -C "${PROJECT_ROOT}" status --short --branch
capture "versions" bash -lc "source /opt/ros/jazzy/setup.bash; ros2 --help | head -n 2; '${ISAAC_SIM_PYTHON}' -c 'import isaacsim; print(isaacsim.__version__)'"
capture "isaac-ros-packages" bash -lc "source /opt/ros/jazzy/setup.bash; source '${PROJECT_ROOT}/ros2_ws/install/setup.bash' 2>/dev/null || true; ros2 pkg list | grep -E 'isaac_ros|nvblox|nova_carter|nav2' | sort"
capture "ros-nodes" ros2 node list
capture "ros-topics" ros2 topic list -t
capture "ros-services" ros2 service list -t
capture "ros-actions" ros2 action list -t
capture "topic-rates" bash -lc "source /opt/ros/jazzy/setup.bash; source '${PROJECT_ROOT}/ros2_ws/install/setup.bash' 2>/dev/null || true; for topic in /clock /front_stereo_camera/left/image_raw /visual_slam/status /nvblox_node/combined_map_slice /cmd_vel_sim; do echo \"topic=\${topic}\"; timeout 4s ros2 topic hz \"\${topic}\" || true; done"
capture "tf-main-chain" bash -lc "source /opt/ros/jazzy/setup.bash; source '${PROJECT_ROOT}/ros2_ws/install/setup.bash' 2>/dev/null || true; timeout 5s ros2 run tf2_ros tf2_echo map base_link || true"
capture "phase10-config" sed -n '1,260p' "${PROJECT_ROOT}/config/stage10.yaml"
capture "phase11-config" sed -n '1,320p' "${PROJECT_ROOT}/config/stage11.yaml"
capture "phase11-latest-summary" bash -lc "test -f '${PROJECT_ROOT}/data/reports/phase11/acceptance-summary-latest.json' && jq '{status,trial_count,classes,aggregate}' '${PROJECT_ROOT}/data/reports/phase11/acceptance-summary-latest.json' || echo 'no completed Stage 11 summary'"
info "Diagnostics collected without changing ROS or simulator state: ${OUTPUT}"
