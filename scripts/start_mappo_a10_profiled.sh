#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"
RUN_NAME="${1:-mappo_dense_a10_profiled_$(date +%Y%m%d_%H%M%S)}"

if [[ $# -gt 0 ]]; then
  shift
fi

OUTPUT_PATH="${DATA_DIR}/${RUN_NAME}.pt"
LOG_PATH="${DATA_DIR}/${RUN_NAME}.log"
PROFILE_ROLLOUT_STEPS="${PROFILE_ROLLOUT_STEPS:-128}"
PROFILE_TOTAL_TIMESTEPS="${PROFILE_TOTAL_TIMESTEPS:-1280}"

mkdir -p "${DATA_DIR}"

cd "${WORKSPACE_DIR}"

echo "run_name=${RUN_NAME}"
echo "output_path=${OUTPUT_PATH}"
echo "log_path=${LOG_PATH}"
echo "profile_rollout_steps=${PROFILE_ROLLOUT_STEPS}"
echo "profile_total_timesteps=${PROFILE_TOTAL_TIMESTEPS}"

: > "${LOG_PATH}"
stdbuf -oL -eL ./scripts/train_mappo_a10_pai.sh \
  "${OUTPUT_PATH}" \
  --rollout-steps "${PROFILE_ROLLOUT_STEPS}" \
  --total-timesteps "${PROFILE_TOTAL_TIMESTEPS}" \
  --checkpoint-interval 0 \
  --log-interval-updates 1 \
  "$@" |& tee "${LOG_PATH}"
