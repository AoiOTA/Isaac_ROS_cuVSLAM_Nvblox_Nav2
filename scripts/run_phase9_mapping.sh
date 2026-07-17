#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
args=("$@")
has_map="false"
for argument in "${args[@]}"; do
  if [[ "${argument}" == "--map" ]]; then
    has_map="true"
    break
  fi
done
if [[ "${has_map}" == "false" ]]; then
  args=(--map warehouse_v2_front "${args[@]}")
fi
exec "${SCRIPT_DIR}/run_mapping.sh" "${args[@]}" \
  --front-rate-hz 10 --reliable-sensor-qos
