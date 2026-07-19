#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

ACCEPTANCE_CONFIG="${PROJECT_ROOT}/config/acceptance.yaml"
PROFILE="all"
MAP_NAME="kujiale_jackal_8cam"
SIM_MODE="--headless"
RVIZ="false"
OUTPUT_DIR=""
TELEMETRY_PERIOD="1.0"
MIN_WARMUP=""
MAX_WARMUP=""
WINDOW=""
STABLE_WINDOWS=""
MAX_MEAN_CHANGE=""
MAX_CV=""
MIN_SAMPLE=""
MAX_SAMPLE=""
while (($#)); do
  case "$1" in
    --config) ACCEPTANCE_CONFIG="${2:?missing config}"; shift 2 ;;
    --profile) PROFILE="${2:?missing profile}"; shift 2 ;;
    --map) MAP_NAME="${2:?missing map name}"; shift 2 ;;
    --headless|--gui) SIM_MODE="$1"; shift ;;
    --rviz) RVIZ="true"; shift ;;
    --no-rviz) RVIZ="false"; shift ;;
    --output-dir) OUTPUT_DIR="${2:?missing output directory}"; shift 2 ;;
    --min-warmup-s) MIN_WARMUP="${2:?missing value}"; shift 2 ;;
    --max-warmup-s) MAX_WARMUP="${2:?missing value}"; shift 2 ;;
    --window-s) WINDOW="${2:?missing value}"; shift 2 ;;
    --stable-windows) STABLE_WINDOWS="${2:?missing value}"; shift 2 ;;
    --max-mean-change) MAX_MEAN_CHANGE="${2:?missing value}"; shift 2 ;;
    --max-cv) MAX_CV="${2:?missing value}"; shift 2 ;;
    --min-sample-s) MIN_SAMPLE="${2:?missing value}"; shift 2 ;;
    --max-sample-s) MAX_SAMPLE="${2:?missing value}"; shift 2 ;;
    --telemetry-period-s) TELEMETRY_PERIOD="${2:?missing value}"; shift 2 ;;
    -h|--help)
      echo "Usage: ./scripts/run_performance_benchmark.sh [--profile all|mapping_8cam|navigation_6cam] [--map NAME] [--headless|--gui] [--rviz|--no-rviz] [adaptive wall-time options]"
      echo "Mapping records a temporary four-Hawk/eight-image-stream MCAP; navigation cycles real goals."
      echo "RViz is disabled by default; --rviz keeps Isaac Sim in the selected mode and starts the workload RViz configuration."
      echo "No frame-count baseline or documentation KPI gate is applied."
      exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
require_file "${ACCEPTANCE_CONFIG}"
mapfile -t PERFORMANCE_VALUES < <(python3 - "${ACCEPTANCE_CONFIG}" <<'PY'
import sys, yaml
value = yaml.safe_load(open(sys.argv[1]))["performance"]["adaptive_sampling"]
for name in (
    "minimum_warmup_s", "maximum_warmup_s", "stability_window_s",
    "stable_windows_required", "maximum_mean_change_ratio",
    "maximum_coefficient_of_variation", "minimum_sample_s", "maximum_sample_s",
):
    print(value[name])
PY
)
MIN_WARMUP="${MIN_WARMUP:-${PERFORMANCE_VALUES[0]}}"
MAX_WARMUP="${MAX_WARMUP:-${PERFORMANCE_VALUES[1]}}"
WINDOW="${WINDOW:-${PERFORMANCE_VALUES[2]}}"
STABLE_WINDOWS="${STABLE_WINDOWS:-${PERFORMANCE_VALUES[3]}}"
MAX_MEAN_CHANGE="${MAX_MEAN_CHANGE:-${PERFORMANCE_VALUES[4]}}"
MAX_CV="${MAX_CV:-${PERFORMANCE_VALUES[5]}}"
MIN_SAMPLE="${MIN_SAMPLE:-${PERFORMANCE_VALUES[6]}}"
MAX_SAMPLE="${MAX_SAMPLE:-${PERFORMANCE_VALUES[7]}}"
case "${PROFILE}" in
  all|mapping_8cam|navigation_6cam) ;;
  *) die "--profile must be all, mapping_8cam, or navigation_6cam" ;;
esac

RUN_ID="$(date -u +%Y%m%dT%H%M%S)-adaptive-performance"
OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_ROOT}/data/reports/performance/${RUN_ID}}"
[[ ! -e "${OUTPUT_DIR}" ]] || die "output directory already exists: ${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}" "${PROJECT_ROOT}/data/locks"
exec 9>"${PROJECT_ROOT}/data/locks/performance-benchmark.lock"
flock -n 9 || die "another performance benchmark is already running"

PROFILE_SIM_PID=""
PROFILE_ROS_PID=""
PROFILE_METRICS_PID=""
PROFILE_DRIVER_PID=""
PROFILE_BAG_PID=""
PROFILE_DISCOVERY_PID=""
PROFILE_RVIZ_PID=""
PROFILE_SIM_STOP=""
PROFILE_METRICS_STOP=""
PROFILE_DRIVER_STOP=""
PROFILE_BAG_ROOT=""
process_alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
group_alive() { [[ -n "$1" ]] && kill -0 -- "-$1" 2>/dev/null; }
stop_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if group_alive "${pid}"; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..60}; do group_alive "${pid}" || break; sleep 0.25; done
    group_alive "${pid}" && kill -TERM -- "-${pid}" 2>/dev/null || true
    for _ in {1..20}; do group_alive "${pid}" || break; sleep 0.25; done
    group_alive "${pid}" && kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
remove_profile_bag() {
  [[ -n "${PROFILE_BAG_ROOT}" ]] || return 0
  case "${PROFILE_BAG_ROOT}" in
    "${PROJECT_ROOT}/data/bags/.performance-"*) rm -rf -- "${PROFILE_BAG_ROOT}" ;;
    *) echo "error: refusing to remove unexpected performance bag: ${PROFILE_BAG_ROOT}" >&2 ;;
  esac
  PROFILE_BAG_ROOT=""
}
cleanup_profile() {
  [[ -z "${PROFILE_DRIVER_STOP}" ]] || touch "${PROFILE_DRIVER_STOP}" 2>/dev/null || true
  [[ -z "${PROFILE_METRICS_STOP}" ]] || touch "${PROFILE_METRICS_STOP}" 2>/dev/null || true
  stop_group "${PROFILE_DRIVER_PID}"
  stop_group "${PROFILE_BAG_PID}"
  stop_group "${PROFILE_METRICS_PID}"
  stop_group "${PROFILE_RVIZ_PID}"
  stop_group "${PROFILE_ROS_PID}"
  if process_alive "${PROFILE_SIM_PID}"; then
    [[ -z "${PROFILE_SIM_STOP}" ]] || touch "${PROFILE_SIM_STOP}" 2>/dev/null || true
  fi
  stop_group "${PROFILE_SIM_PID}"
  stop_group "${PROFILE_DISCOVERY_PID}"
  PROFILE_SIM_PID=""
  PROFILE_ROS_PID=""
  PROFILE_METRICS_PID=""
  PROFILE_DRIVER_PID=""
  PROFILE_BAG_PID=""
  PROFILE_DISCOVERY_PID=""
  PROFILE_RVIZ_PID=""
  remove_profile_bag
}
trap cleanup_profile EXIT INT TERM

BRINGUP_SHARE="$(ros2 pkg prefix jackal_bringup --share)"
STARTUP_TIMEOUT="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["startup_timeout_s"]))' "${ACCEPTANCE_CONFIG}")"
STARTUP_POLLS="$(python3 -c 'import math,sys; print(math.ceil(float(sys.argv[1])*2))' "${STARTUP_TIMEOUT}")"
PERFORMANCE_GOALS="$(python3 -c 'import sys,yaml; d=yaml.safe_load(open(sys.argv[1])); print("["+",".join(str(v) for g in d["goals"] for v in g["pose"])+"]")' "${ACCEPTANCE_CONFIG}")"
PERFORMANCE_GOAL_TIMEOUT="$(python3 -c 'import sys,yaml; print(float(yaml.safe_load(open(sys.argv[1]))["trials"]["goal_timeout_s"]))' "${ACCEPTANCE_CONFIG}")"
DISCOVERY_PORT_BASE="${PERFORMANCE_DISCOVERY_PORT_BASE:-13230}"
[[ "${DISCOVERY_PORT_BASE}" =~ ^[0-9]+$ ]] || \
  die "PERFORMANCE_DISCOVERY_PORT_BASE must be an integer"
DISCOVERY_PORT_BASE="$((10#${DISCOVERY_PORT_BASE}))"
(( DISCOVERY_PORT_BASE >= 1024 && DISCOVERY_PORT_BASE <= 65534 )) || \
  die "PERFORMANCE_DISCOVERY_PORT_BASE must be in 1024..65534"
command -v fastdds >/dev/null || die "fastdds discovery executable not found"

run_profile() {
  local camera_profile="$1"
  local workload="$2"
  local domain_id="$3"
  local profile_dir="${OUTPUT_DIR}/${camera_profile}"
  local ready_file="${profile_dir}/start-adaptive-warmup"
  local sim_report="${profile_dir}/simulator.json"
  local telemetry="${profile_dir}/telemetry.csv"
  local normalized="${profile_dir}/performance.json"
  local workload_report="${profile_dir}/workload.json"
  local capture_report="${profile_dir}/mapping-capture.json"
  local super_client_xml="${profile_dir}/fastdds-super-client.xml"
  local bag_dir=""
  local discovery_port="$((DISCOVERY_PORT_BASE + domain_id - 130))"
  mkdir -p "${profile_dir}"
  rm -f "${ready_file}"
  PROFILE_SIM_STOP="${profile_dir}/stop-simulator"
  PROFILE_METRICS_STOP="${profile_dir}/stop-telemetry"
  PROFILE_DRIVER_STOP="${profile_dir}/stop-workload"
  rm -f "${PROFILE_SIM_STOP}" "${PROFILE_METRICS_STOP}" "${PROFILE_DRIVER_STOP}"
  export ROS_DOMAIN_ID="${domain_id}"
  if ss -H -lun "sport = :${discovery_port}" 2>/dev/null | grep -q .; then
    die "Fast DDS discovery port ${discovery_port} is already in use"
  fi
  export ROS_DISCOVERY_SERVER="127.0.0.1:${discovery_port}"
  # The local-only flag forces SIMPLE discovery in this Jazzy/Fast DDS build.
  # The explicit server remains bound to loopback, including late rosbag and
  # workload participants started after the simulator publishers.
  unset ROS_LOCALHOST_ONLY
  python3 "${PROJECT_ROOT}/tools/write_fastdds_super_client.py" \
    --port "${discovery_port}" --output "${super_client_xml}"
  info "Starting ${camera_profile} Fast DDS discovery server on ${ROS_DISCOVERY_SERVER}"
  setsid fastdds discovery -i 0 -l 127.0.0.1 -p "${discovery_port}" \
    >"${profile_dir}/fastdds-discovery.log" 2>&1 & PROFILE_DISCOVERY_PID=$!
  for _ in {1..30}; do
    ss -H -lun "sport = :${discovery_port}" 2>/dev/null | grep -q . && break
    process_alive "${PROFILE_DISCOVERY_PID}" || \
      die "Fast DDS discovery server exited; see ${profile_dir}/fastdds-discovery.log"
    sleep 0.1
  done
  ss -H -lun "sport = :${discovery_port}" 2>/dev/null | grep -q . || \
    die "Fast DDS discovery server did not bind UDP port ${discovery_port}"

  sim_args=("${SIM_MODE}" --duration 0 --camera-profile "${camera_profile}"
    --benchmark-performance --performance-start-file "${ready_file}"
    --performance-min-warmup-s "${MIN_WARMUP}"
    --performance-max-warmup-s "${MAX_WARMUP}"
    --performance-window-s "${WINDOW}"
    --performance-stable-windows "${STABLE_WINDOWS}"
    --performance-max-mean-change "${MAX_MEAN_CHANGE}"
    --performance-max-cv "${MAX_CV}"
    --performance-min-sample-s "${MIN_SAMPLE}"
    --performance-max-sample-s "${MAX_SAMPLE}"
    --stop-file "${PROFILE_SIM_STOP}" --report "${sim_report}")
  # Both profiles feed GPU image-normalization and localization graphs. Keep
  # the measured workload on the same lossless camera transport as runtime.
  sim_args+=(--reliable-sensor-qos)
  info "Starting ${workload} workload with ${camera_profile}; adaptive wall-time sampling"
  setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
    "${sim_args[@]}" >"${profile_dir}/simulator.log" 2>&1 & PROFILE_SIM_PID=$!
  for ((poll=0; poll<STARTUP_POLLS; poll++)); do
    grep -Fq "JACKAL_PERFORMANCE_READY profile=${camera_profile} fixed_frames=none" \
      "${profile_dir}/simulator.log" 2>/dev/null && break
    process_alive "${PROFILE_SIM_PID}" || die "simulator exited during ${camera_profile} startup"
    sleep 0.5
  done
  grep -Fq "JACKAL_PERFORMANCE_READY profile=${camera_profile} fixed_frames=none" \
    "${profile_dir}/simulator.log" || die "performance simulator startup timeout"

  if [[ "${workload}" == "mapping" ]]; then
    setsid ros2 launch jackal_bringup phase6.launch.py \
      camera_profile:=mapping_8cam image_qos:=DEFAULT \
      visual_slam_params:="${BRINGUP_SHARE}/config/visual_slam_mapping_8cam.yaml" \
      >"${profile_dir}/ros.log" 2>&1 & PROFILE_ROS_PID=$!
    for pair in front left right back; do
      for side in left right; do
        ros2 topic echo --no-daemon --once --no-arr --timeout "${STARTUP_TIMEOUT}" \
          "/${pair}_stereo_camera/${side}/image_raw" sensor_msgs/msg/Image >/dev/null
      done
    done
  else
    python3 "${PROJECT_ROOT}/tools/check_map_manifest.py" \
      "${PROJECT_ROOT}/data/maps/${MAP_NAME}"
    setsid "${PROJECT_ROOT}/scripts/run_navigation.sh" --map "${MAP_NAME}" --no-rviz \
      >"${profile_dir}/ros.log" 2>&1 & PROFILE_ROS_PID=$!
    ros2 topic echo --no-daemon --once --timeout "${STARTUP_TIMEOUT}" \
      --qos-durability transient_local --filter 'm.data' \
      /localization/ready std_msgs/msg/Bool >/dev/null
    topic_list="$(env -u ROS_DISCOVERY_SERVER \
      FASTRTPS_DEFAULT_PROFILES_FILE="${super_client_xml}" \
      ros2 topic list --no-daemon --spin-time 3)"
    if grep -Fq '/back_stereo_camera/' <<<"${topic_list}"; then
      die "rear camera topics are present in navigation_6cam"
    fi
  fi
  process_alive "${PROFILE_ROS_PID}" || die "${workload} ROS workload exited during readiness checks"
  ros2 topic echo --no-daemon --once --timeout "${STARTUP_TIMEOUT}" \
    /visual_slam/status isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus >/dev/null
  ros2 topic echo --no-daemon --once --timeout "${STARTUP_TIMEOUT}" \
    /nvblox_node/static_map_slice nvblox_msgs/msg/DistanceMapSlice >/dev/null
  if [[ "${workload}" == "navigation" ]]; then
    nav2_active="false"
    for ((poll=0; poll<STARTUP_POLLS; poll++)); do
      if rg -q 'lifecycle_manager_navigation.*Managed nodes are active' \
        "${profile_dir}/ros.log"; then
        nav2_active="true"
        break
      fi
      process_alive "${PROFILE_ROS_PID}" || \
        die "navigation workload exited before lifecycle activation"
      sleep 0.5
    done
    [[ "${nav2_active}" == "true" ]] || die "Nav2 lifecycle activation timeout"

    action_ready="false"
    for ((poll=0; poll<STARTUP_POLLS; poll++)); do
      action_topics="$(env -u ROS_DISCOVERY_SERVER \
        FASTRTPS_DEFAULT_PROFILES_FILE="${super_client_xml}" \
        ros2 topic list --no-daemon --spin-time 3 --include-hidden-topics)"
      if grep -Fxq /navigate_to_pose/_action/status <<<"${action_topics}"; then
        action_ready="true"
        break
      fi
      process_alive "${PROFILE_ROS_PID}" || die "navigation workload exited before action readiness"
      sleep 0.5
    done
    [[ "${action_ready}" == "true" ]] || die "NavigateToPose action startup timeout"
  fi

  if [[ "${RVIZ}" == "true" ]]; then
    local rviz_config="${BRINGUP_SHARE}/rviz/${workload}.rviz"
    require_file "${rviz_config}"
    info "Starting ${workload} RViz while Isaac Sim remains ${SIM_MODE}"
    setsid rviz2 -d "${rviz_config}" --ros-args -p use_sim_time:=true \
      >"${profile_dir}/rviz.log" 2>&1 & PROFILE_RVIZ_PID=$!
    sleep 3
    process_alive "${PROFILE_RVIZ_PID}" || \
      die "${workload} RViz exited; see ${profile_dir}/rviz.log"
  fi

  if [[ "${workload}" == "mapping" ]]; then
    PROFILE_BAG_ROOT="$(mktemp -d "${PROJECT_ROOT}/data/bags/.performance-${camera_profile}.XXXXXX")"
    bag_dir="${PROFILE_BAG_ROOT}/capture"
    mapping_topics=()
    for pair in front left right back; do
      for side in left right; do
        mapping_topics+=("/${pair}_stereo_camera/${side}/image_raw")
        mapping_topics+=("/${pair}_stereo_camera/${side}/camera_info")
      done
    done
    mapping_topics+=(/front_stereo_imu/imu /tf /tf_static /clock)
    # rosbag joins after every image publisher. Fast DDS 2.14.6 needs a
    # SUPER_CLIENT profile to discover those existing server participants;
    # ROS_DISCOVERY_SERVER alone can leave a healthy recorder with zero topics.
    setsid env -u ROS_DISCOVERY_SERVER \
      FASTRTPS_DEFAULT_PROFILES_FILE="${super_client_xml}" ros2 bag record \
      --storage mcap --storage-preset-profile fastwrite --disable-keyboard-controls \
      --output "${bag_dir}" --topics "${mapping_topics[@]}" \
      >"${profile_dir}/rosbag.log" 2>&1 & PROFILE_BAG_PID=$!
    sleep 2
    process_alive "${PROFILE_BAG_PID}" || die "mapping performance MCAP recorder exited"
  fi

  driver_args=(--ros-args -p use_sim_time:=true -p mode:="${workload}"
    -p ready_file:="${ready_file}" -p stop_file:="${PROFILE_DRIVER_STOP}"
    -p report_path:="${workload_report}")
  if [[ "${workload}" == "navigation" ]]; then
    driver_args+=(-p "goal_poses:=${PERFORMANCE_GOALS}"
      -p goal_timeout_s:="${PERFORMANCE_GOAL_TIMEOUT}")
  fi
  setsid ros2 run jackal_experiments performance_workload_driver "${driver_args[@]}" \
    >"${profile_dir}/workload.log" 2>&1 & PROFILE_DRIVER_PID=$!

  metrics_args=(--output "${telemetry}" --stop-file "${PROFILE_METRICS_STOP}"
    --period "${TELEMETRY_PERIOD}" --pid "${PROFILE_SIM_PID}"
    --pid "${PROFILE_ROS_PID}" --pid "${PROFILE_DRIVER_PID}"
    --pid "${PROFILE_DISCOVERY_PID}")
  [[ -z "${PROFILE_BAG_PID}" ]] || metrics_args+=(--pid "${PROFILE_BAG_PID}")
  [[ "${RVIZ}" != "true" ]] || metrics_args+=(--pid "${PROFILE_RVIZ_PID}")
  setsid python3 "${PROJECT_ROOT}/tools/record_performance_metrics.py" \
    "${metrics_args[@]}" >"${profile_dir}/telemetry.log" 2>&1 & PROFILE_METRICS_PID=$!

  for ((poll=0; poll<STARTUP_POLLS; poll++)); do
    [[ -f "${ready_file}" ]] && break
    process_alive "${PROFILE_SIM_PID}" || die "simulator exited before active workload confirmation"
    process_alive "${PROFILE_ROS_PID}" || die "${workload} ROS workload exited before active load"
    process_alive "${PROFILE_DRIVER_PID}" || die "${workload} driver exited before active load"
    [[ -z "${PROFILE_BAG_PID}" ]] || process_alive "${PROFILE_BAG_PID}" || \
      die "mapping MCAP recorder exited before active load"
    [[ "${RVIZ}" != "true" ]] || process_alive "${PROFILE_RVIZ_PID}" || \
      die "${workload} RViz exited before active load"
    sleep 0.5
  done
  [[ -f "${ready_file}" ]] || die "${workload} workload never produced a nonzero simulator command"
  info "${camera_profile} active workload is confirmed; adaptive warmup has started"
  while process_alive "${PROFILE_SIM_PID}"; do
    process_alive "${PROFILE_ROS_PID}" || die "${workload} ROS workload exited during sampling"
    process_alive "${PROFILE_DRIVER_PID}" || die "${workload} driver exited during sampling"
    [[ -z "${PROFILE_BAG_PID}" ]] || process_alive "${PROFILE_BAG_PID}" || \
      die "mapping MCAP recorder exited during sampling"
    [[ "${RVIZ}" != "true" ]] || process_alive "${PROFILE_RVIZ_PID}" || \
      die "${workload} RViz exited during sampling"
    process_alive "${PROFILE_METRICS_PID}" || die "performance telemetry recorder exited"
    sleep 1
  done
  set +e
  wait "${PROFILE_SIM_PID}"
  sim_status=$?
  set -e
  PROFILE_SIM_PID=""
  touch "${PROFILE_DRIVER_STOP}"
  touch "${PROFILE_METRICS_STOP}"
  for _ in {1..100}; do process_alive "${PROFILE_DRIVER_PID}" || break; sleep 0.05; done
  if process_alive "${PROFILE_DRIVER_PID}"; then
    stop_group "${PROFILE_DRIVER_PID}"
  else
    set +e
    wait "${PROFILE_DRIVER_PID}"
    driver_status=$?
    set -e
    (( driver_status == 0 )) || die "${workload} active workload driver failed"
  fi
  PROFILE_DRIVER_PID=""
  stop_group "${PROFILE_METRICS_PID}"; PROFILE_METRICS_PID=""
  if [[ -n "${PROFILE_BAG_PID}" ]]; then
    stop_group "${PROFILE_BAG_PID}"; PROFILE_BAG_PID=""
    ros2 bag info "${bag_dir}" >"${profile_dir}/rosbag-info.txt"
    python3 "${PROJECT_ROOT}/tools/summarize_mapping_capture.py" \
      "${bag_dir}" --output "${capture_report}" >"${profile_dir}/capture-summary.log"
    remove_profile_bag
  fi
  stop_group "${PROFILE_RVIZ_PID}"; PROFILE_RVIZ_PID=""
  stop_group "${PROFILE_ROS_PID}"; PROFILE_ROS_PID=""
  (( sim_status == 0 )) || die "${camera_profile} performance simulator failed"
  summary_args=(--sim-report "${sim_report}" --telemetry "${telemetry}"
    --output "${normalized}" --workload "${workload}"
    --workload-report "${workload_report}")
  [[ "${RVIZ}" != "true" ]] || summary_args+=(--rviz-enabled)
  [[ "${workload}" != "mapping" ]] || summary_args+=(--capture-report "${capture_report}")
  python3 "${PROJECT_ROOT}/tools/summarize_performance.py" \
    "${summary_args[@]}" \
    >"${profile_dir}/summary.log"
  stop_group "${PROFILE_DISCOVERY_PID}"; PROFILE_DISCOVERY_PID=""
  profile_reports+=("${normalized}")
  info "Recorded ${camera_profile}: ${normalized}"
}

profile_reports=()
if [[ "${PROFILE}" == "all" || "${PROFILE}" == "mapping_8cam" ]]; then
  run_profile mapping_8cam mapping 130
fi
if [[ "${PROFILE}" == "all" || "${PROFILE}" == "navigation_6cam" ]]; then
  run_profile navigation_6cam navigation 131
fi
python3 "${PROJECT_ROOT}/tools/compare_performance_profiles.py" \
  --output "${OUTPUT_DIR}/summary.json" "${profile_reports[@]}" \
  >"${OUTPUT_DIR}/comparison.log"
trap - EXIT INT TERM
info "Performance observation complete (no preset KPI gate): ${OUTPUT_DIR}/summary.md"
