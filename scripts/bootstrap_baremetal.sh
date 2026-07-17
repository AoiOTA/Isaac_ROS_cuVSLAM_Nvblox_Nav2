#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

MODE="install"
ISAAC_ROS_MIRROR="${ISAAC_ROS_MIRROR:-cn}"
PACKAGE_FILE="${PROJECT_ROOT}/config/isaac_ros_packages.txt"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage1"

usage() {
  cat <<'EOF'
Usage: ./scripts/bootstrap_baremetal.sh [--preflight|--simulate|--install]

  --preflight  Validate the machine and official repository endpoints only.
  --simulate   Configure repositories, run isaac-ros bare-metal init, and
               perform an APT dry run without installing target packages.
  --install    Complete the official Isaac ROS 4.5 bare-metal installation.

Environment:
  ISAAC_ROS_MIRROR=cn|us   Select NVIDIA Isaac ROS repository mirror (default cn).
EOF
}

while (($#)); do
  case "$1" in
    --preflight) MODE="preflight" ;;
    --simulate) MODE="simulate" ;;
    --install) MODE="install" ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
  shift
done

case "${ISAAC_ROS_MIRROR}" in
  cn) ISAAC_ROS_BASE_URL="https://isaac.download.nvidia.cn/isaac-ros" ;;
  us) ISAAC_ROS_BASE_URL="https://isaac.download.nvidia.com/isaac-ros" ;;
  *) die "ISAAC_ROS_MIRROR must be cn or us" ;;
esac

preflight() {
  info "Phase 1 preflight"
  [[ -r /etc/os-release ]] || die "/etc/os-release is unavailable"
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]] \
    || die "Ubuntu 24.04 is required; found ${PRETTY_NAME:-unknown}"
  [[ "$(dpkg --print-architecture)" == "amd64" ]] || die "amd64 architecture is required"
  locale charmap | grep -qi 'UTF-8' || die "A UTF-8 locale is required"

  load_ros
  command -v ros2 >/dev/null || die "ROS 2 CLI is unavailable"
  [[ "${ROS_DISTRO:-}" == "jazzy" ]] || die "ROS_DISTRO must be jazzy"
  command -v nvidia-smi >/dev/null || die "nvidia-smi is unavailable"

  local driver major free_gib
  driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)"
  major="${driver%%.*}"
  ((major >= 580)) || die "NVIDIA driver >= 580 is required; found ${driver}"
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader

  free_gib="$(df -BG --output=avail "${PROJECT_ROOT}" | tail -n1 | tr -dc '0-9')"
  ((free_gib >= 80)) || die "At least 80 GiB free disk is required; found ${free_gib} GiB"

  python3 - <<'PY'
import cv2
if cv2.__version__ != "4.6.0":
    raise SystemExit(f"OpenCV 4.6.0 is required; found {cv2.__version__}")
print(f"OpenCV {cv2.__version__}")
PY

  local endpoints=(
    "${ISAAC_ROS_BASE_URL}/repos.key"
    "${ISAAC_ROS_BASE_URL}/release-4.5/dists/noble/Release"
    "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"
    "https://repo.download.nvidia.com/jetson/jetson-ota-public.asc"
    "https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/release-4.5/docker/rosdep/extra_rosdeps.yaml"
  )
  local endpoint
  for endpoint in "${endpoints[@]}"; do
    curl --fail --location --silent --show-error --head \
      --connect-timeout 15 --max-time 60 --retry 3 --retry-all-errors "${endpoint}" >/dev/null
    info "Reachable: ${endpoint}"
  done
  info "Preflight passed"
}

install_key_from_url() {
  local url="$1" destination="$2" tmp="$3"
  curl --fail --location --silent --show-error "${url}" | gpg --dearmor >"${tmp}"
  sudo install -o root -g root -m 0644 "${tmp}" "${destination}"
}

configure_repositories() {
  local tmp_dir="$1"
  info "Configuring official NVIDIA repositories"
  sudo install -d -m 0755 /etc/apt/keyrings /usr/share/keyrings /etc/ros/rosdep/sources.list.d

  install_key_from_url \
    "${ISAAC_ROS_BASE_URL}/repos.key" \
    /usr/share/keyrings/nvidia-isaac-ros.gpg \
    "${tmp_dir}/nvidia-isaac-ros.gpg"
  printf '%s\n' \
    "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] ${ISAAC_ROS_BASE_URL}/release-4.5 noble main" \
    >"${tmp_dir}/nvidia-isaac-ros.list"
  sudo install -o root -g root -m 0644 \
    "${tmp_dir}/nvidia-isaac-ros.list" /etc/apt/sources.list.d/nvidia-isaac-ros.list

  curl --fail --location --silent --show-error \
    "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb" \
    --output "${tmp_dir}/cuda-keyring.deb"
  sudo dpkg -i "${tmp_dir}/cuda-keyring.deb"

  install_key_from_url \
    "https://repo.download.nvidia.com/jetson/jetson-ota-public.asc" \
    /etc/apt/keyrings/nvidia-jetson.gpg \
    "${tmp_dir}/nvidia-jetson.gpg"
  printf '%s\n' \
    "deb [signed-by=/etc/apt/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main" \
    >"${tmp_dir}/nvidia-jetson.list"
  sudo install -o root -g root -m 0644 \
    "${tmp_dir}/nvidia-jetson.list" /etc/apt/sources.list.d/nvidia-jetson.list

  curl --fail --location --silent --show-error \
    "https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/release-4.5/docker/rosdep/extra_rosdeps.yaml" \
    --output "${tmp_dir}/nvidia-isaac-ros-extra.yaml"
  sudo install -o root -g root -m 0644 \
    "${tmp_dir}/nvidia-isaac-ros-extra.yaml" /etc/ros/rosdep/nvidia-isaac-ros-extra.yaml
  printf '%s\n' "yaml file:///etc/ros/rosdep/nvidia-isaac-ros-extra.yaml" \
    >"${tmp_dir}/30-nvidia-isaac-ros.list"
  sudo install -o root -g root -m 0644 \
    "${tmp_dir}/30-nvidia-isaac-ros.list" /etc/ros/rosdep/sources.list.d/30-nvidia-isaac-ros.list
}

read_target_packages() {
  mapfile -t TARGET_PACKAGES < <(sed -E '/^[[:space:]]*(#|$)/d; s/[[:space:]]+#.*$//' "${PACKAGE_FILE}")
  ((${#TARGET_PACKAGES[@]} > 0)) || die "No packages found in ${PACKAGE_FILE}"
}

apt_dry_run() {
  local log_file="$1"
  read_target_packages
  info "APT dry run for ${#TARGET_PACKAGES[@]} target packages"
  apt-get --simulate install "${TARGET_PACKAGES[@]}" | tee "${log_file}"
  if grep -Eq '^Remv |The following packages will be REMOVED:' "${log_file}"; then
    die "APT proposed package removals; refusing to continue. Inspect ${log_file}"
  fi
  grep -Eq '^Inst cuda-toolkit-13-0 \(13\.0\.' "${log_file}" \
    || die "APT did not resolve CUDA Toolkit 13.0"
  grep -Eq '^Inst tensorrt \(10\.13\.3\.9-' "${log_file}" \
    || die "APT did not resolve TensorRT 10.13.3.9"
  local package
  for package in \
    ros-jazzy-isaac-ros-visual-slam \
    ros-jazzy-isaac-ros-nvblox \
    ros-jazzy-isaac-ros-visual-global-localization \
    ros-jazzy-isaac-ros-visual-mapping \
    ros-jazzy-isaac-mapping-ros; do
    grep -Eq "^Inst ${package} \\(4\\.5\\.0-" "${log_file}" \
      || die "APT did not resolve ${package} 4.5.0"
  done
  info "APT dry run passed without removals"
}

install_phase1() {
  mkdir -p "${LOG_DIR}"
  local run_id tmp_dir simulate_log
  run_id="$(date -u +%Y%m%dT%H%M%SZ)"
  tmp_dir="$(mktemp -d)"
  simulate_log="${LOG_DIR}/apt-simulate-${run_id}.log"
  trap 'rm -rf "${tmp_dir}"' RETURN

  info "Administrative access is required for the official bare-metal installation"
  sudo -v
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg git-lfs ccache
  configure_repositories "${tmp_dir}"
  sudo apt-get update
  sudo apt-get install -y isaac-ros-cli

  info "Running official Isaac ROS bare-metal initialization"
  sudo isaac-ros init baremetal --yes
  sudo apt-get update
  apt_dry_run "${simulate_log}"

  if [[ "${MODE}" == "simulate" ]]; then
    info "Simulation mode complete; target packages were not installed"
    return 0
  fi

  info "Installing CUDA 13.0, TensorRT, and Isaac ROS 4.5 target packages"
  sudo apt-get install -y "${TARGET_PACKAGES[@]}"
  rosdep update
  git lfs install --local

  dpkg-query -W -f='${Package}\t${Version}\n' \
    'cuda-*' 'libnvinfer*' 'tensorrt*' 'isaac-ros-cli' 'ros-jazzy-isaac-*' \
    2>/dev/null | sort -u >"${LOG_DIR}/installed-packages-${run_id}.tsv" || true
  info "Bare-metal package installation complete"
}

preflight
if [[ "${MODE}" != "preflight" ]]; then
  install_phase1
fi
