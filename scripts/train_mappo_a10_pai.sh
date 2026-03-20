#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${1:-${DATA_DIR}/mappo_dense_a10_${TIMESTAMP}.pt}"

if [[ $# -gt 0 ]]; then
  shift
fi

mkdir -p "${DATA_DIR}"

export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export NUMEXPR_NUM_THREADS="${NUMEXPR_NUM_THREADS:-1}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

ros2 run usv_rl train_mappo_residual \
  --output "${OUTPUT_PATH}" \
  --device auto \
  --amp auto \
  --matmul-precision high \
  --torch-num-threads 1 \
  --num-agents 5 \
  --scenario-set dense \
  --hidden-size 256 \
  --hidden-size 256 \
  --rollout-steps 256 \
  --minibatch-size 512 \
  --update-epochs 6 \
  --total-timesteps 500000 \
  --checkpoint-interval 25600 \
  "$@"