#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
load_ros

MAP_NAME="${1:-warehouse_v1}"
MAP_DIR="${PROJECT_ROOT}/data/maps/${MAP_NAME}"
MODEL_DIR="${PROJECT_ROOT}/data/models/vgl"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/data/logs/stage7/relocalization-${RUN_ID}"
REPORT_DIR="${PROJECT_ROOT}/data/reports/phase7"
export ROS_DOMAIN_ID="${PHASE7_TEST_ROS_DOMAIN_ID:-48}"
mkdir -p "${LOG_DIR}" "${REPORT_DIR}"
for path in "${MAP_DIR}/cuvslam" "${MAP_DIR}/cuvgl" "${MAP_DIR}/config" "${MODEL_DIR}"; do
  [[ -d "${path}" ]] || die "required Stage 7 artifact missing: ${path}"
done

poses=("-0.5 -0.5 0.00" "-0.5 -0.5 0.08" "-0.5 -0.5 -0.08" "-0.5 -0.5 0.16" "-0.5 -0.5 -0.16")
reports=()
for index in "${!poses[@]}"; do
  read -r x y heading <<<"${poses[$index]}"
  name="pose_${index}"
  attempt="${LOG_DIR}/${name}"
  mkdir -p "${attempt}"
  stop_file="${attempt}/stop-simulator"
  sim_pid=""; bringup_pid=""
  cleanup_attempt() {
    if [[ -n "${bringup_pid}" ]] && kill -0 "${bringup_pid}" 2>/dev/null; then
      kill -INT -- "-${bringup_pid}" 2>/dev/null || true
      wait "${bringup_pid}" 2>/dev/null || true
    fi
    if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then touch "${stop_file}"; fi
    if [[ -n "${sim_pid}" ]]; then wait "${sim_pid}" 2>/dev/null || true; fi
  }
  trap cleanup_attempt EXIT INT TERM
  info "Relocalization ${index}/4: requested x=${x} y=${y} yaw=${heading}"
  setsid "${ISAAC_SIM_PYTHON}" "${PROJECT_ROOT}/isaac_sim/navigation_sim.py" \
    --headless --duration 300 --stop-file "${stop_file}" --spawn-x "${x}" --spawn-y "${y}" \
    --spawn-yaw "${heading}" --report "${attempt}/simulator.json" \
    >"${attempt}/simulator.log" 2>&1 & sim_pid=$!
  for _ in {1..240}; do
    grep -Fq NOVA_CARTER_SENSORS_READY "${attempt}/simulator.log" 2>/dev/null && break
    kill -0 "${sim_pid}" 2>/dev/null || die "simulator failed in ${name}"
    sleep 0.5
  done
  setsid ros2 launch nova_carter_bringup phase7_localization.launch.py \
    vgl_map_dir:="${MAP_DIR}/cuvgl" vgl_config_dir:="${MAP_DIR}/config" \
    vgl_model_dir:="${MODEL_DIR}" cuvslam_map_dir:="${MAP_DIR}/cuvslam" \
    >"${attempt}/bringup.log" 2>&1 & bringup_pid=$!
  result="${attempt}/result.json"
  set +e
  ros2 run nova_carter_experiments vgl_test_runner --ros-args \
    -p use_sim_time:=true -p result_path:="${result}" -p attempt_name:="${name}" \
    >"${attempt}/test.log" 2>&1
  status=$?
  set -e
  cleanup_attempt; sim_pid=""; bringup_pid=""
  (( status == 0 )) || { tail -n 160 "${attempt}/bringup.log" >&2; cat "${attempt}/test.log" >&2; die "${name} failed"; }
  reports+=("${result}")
done
trap - EXIT INT TERM

python3 - "${REPORT_DIR}/relocalization-${RUN_ID}.json" "${reports[@]}" <<'PY'
import json, pathlib, sys
output = pathlib.Path(sys.argv[1])
attempts = [json.loads(pathlib.Path(p).read_text()) for p in sys.argv[2:]]
summary = {'status': 'passed' if len(attempts) == 5 and all(a['status']=='passed' for a in attempts) else 'failed',
           'attempt_count': len(attempts), 'success_count': sum(a['status']=='passed' for a in attempts),
           'attempts': attempts}
output.write_text(json.dumps(summary, indent=2, sort_keys=True)+'\n')
assert summary['status'] == 'passed'
PY
cp "${REPORT_DIR}/relocalization-${RUN_ID}.json" "${REPORT_DIR}/latest.json"
info "Five-pose cuVGL restart acceptance passed"
