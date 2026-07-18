#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

if (($#)); then
  case "$1" in
    -h|--help)
      echo "Usage: ./scripts/check_manual_navigation.sh"
      echo "Checks a running manual-navigation stack without publishing a goal."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
fi

DISCOVERY_PORT="${NAVIGATION_DISCOVERY_PORT:-11849}"
export ROS_DOMAIN_ID="${NAVIGATION_ROS_DOMAIN_ID:-49}"
PROOF_DIR="$(mktemp -d /tmp/jackal-manual-navigation-check.XXXXXX)"
PROOF_XML="${PROOF_DIR}/fastdds-super-client.xml"
cleanup() {
  rm -f -- "${PROOF_XML}"
  rmdir -- "${PROOF_DIR}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "${PROJECT_ROOT}/tools/write_fastdds_super_client.py" \
  --port "${DISCOVERY_PORT}" --output "${PROOF_XML}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROOF_XML}"
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY

ros2 run jackal_experiments manual_navigation_ready --ros-args \
  -p use_sim_time:=true -p action_topic:=/navigate_to_pose -p timeout_s:=30.0

topics="$(ros2 topic list --no-daemon --spin-time 5)"
for topic in \
  /front_stereo_camera/left/image_raw \
  /left_stereo_camera/left/image_raw \
  /right_stereo_camera/left/image_raw; do
  grep -Fxq "${topic}" <<<"${topics}" || die "missing navigation camera: ${topic}"
done
if grep -Fxq /back_stereo_camera/left/image_raw <<<"${topics}"; then
  die "rear Hawk publisher must stay disabled during navigation"
fi

info "Manual navigation diagnostics passed: map/TF/action ready, front+side Hawks on, rear Hawk off"
