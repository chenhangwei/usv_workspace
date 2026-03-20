#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: evaluate_mappo_checkpoint.sh <checkpoint.pt> [extra evaluate args...]" >&2
  exit 2
fi

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
CHECKPOINT_PATH="$1"
shift || true

if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
  echo "error: checkpoint not found: ${CHECKPOINT_PATH}" >&2
  exit 2
fi

CHECKPOINT_DIR="$(dirname "${CHECKPOINT_PATH}")"
CHECKPOINT_STEM="$(basename "${CHECKPOINT_PATH}" .pt)"
JSON_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.eval.json"
LOG_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.eval.log"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "${CHECKPOINT_DIR}"

echo "checkpoint_path=${CHECKPOINT_PATH}"
echo "json_path=${JSON_PATH}"
echo "log_path=${LOG_PATH}"

stdbuf -oL -eL ros2 run usv_rl evaluate_mappo_residual \
  --policy mappo \
  --model "${CHECKPOINT_PATH}" \
  --episodes 9 \
  --steps-per-episode 90 \
  --device cpu \
  --output-json "${JSON_PATH}" \
  "$@" |& tee "${LOG_PATH}"
