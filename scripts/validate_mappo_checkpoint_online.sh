#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: validate_mappo_checkpoint_online.sh <checkpoint.pt> <baseline_benchmark.json> [extra validate args...]" >&2
  exit 2
fi

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
CHECKPOINT_PATH="$1"
BASELINE_BENCHMARK_PATH="$2"
shift 2 || true

if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
  echo "error: checkpoint not found: ${CHECKPOINT_PATH}" >&2
  exit 2
fi

if [[ ! -f "${BASELINE_BENCHMARK_PATH}" ]]; then
  echo "error: baseline benchmark JSON not found: ${BASELINE_BENCHMARK_PATH}" >&2
  exit 2
fi

CHECKPOINT_DIR="$(dirname "${CHECKPOINT_PATH}")"
CHECKPOINT_STEM="$(basename "${CHECKPOINT_PATH}" .pt)"
BENCHMARK_JSON_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online.benchmark.json"
GATE_JSON_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online.gate.json"
LOG_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online.validate.log"
ONLINE_LOG_DIR="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online_logs"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "${CHECKPOINT_DIR}" "${ONLINE_LOG_DIR}"

echo "checkpoint_path=${CHECKPOINT_PATH}"
echo "baseline_benchmark_path=${BASELINE_BENCHMARK_PATH}"
echo "benchmark_json_path=${BENCHMARK_JSON_PATH}"
echo "gate_json_path=${GATE_JSON_PATH}"
echo "log_path=${LOG_PATH}"
echo "online_log_dir=${ONLINE_LOG_DIR}"

set +e
stdbuf -oL -eL ros2 run usv_rl validate_online_candidate \
  --model "${CHECKPOINT_PATH}" \
  --policy mappo \
  --baseline-benchmark "${BASELINE_BENCHMARK_PATH}" \
  --benchmark-output "${BENCHMARK_JSON_PATH}" \
  --gate-output "${GATE_JSON_PATH}" \
  --log-dir "${ONLINE_LOG_DIR}" \
  "$@" |& tee "${LOG_PATH}"
status=${PIPESTATUS[0]}
set -e

exit "${status}"