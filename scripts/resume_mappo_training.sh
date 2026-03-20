#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 2 ]]; then
  echo "usage: resume_mappo_training.sh <resume_checkpoint.pt> <new_total_timesteps> [output_path.pt] [extra train args...]"
  echo "example: ./scripts/resume_mappo_training.sh /mnt/data/checkpoints/usv_rl/run.pt 750000"
  exit 0
fi

RESUME_PATH="$1"
NEW_TOTAL_TIMESTEPS="$2"

if [[ $# -ge 3 && "${3:-}" != --* ]]; then
  OUTPUT_PATH="$3"
  shift 3
else
  mkdir -p "${DATA_DIR}"
  STEM="$(basename "${RESUME_PATH%.pt}")"
  OUTPUT_PATH="${DATA_DIR}/${STEM}_resume_$(date +%Y%m%d_%H%M%S).pt"
  shift 2
fi

export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export NUMEXPR_NUM_THREADS="${NUMEXPR_NUM_THREADS:-1}"
# Default to GPU 0 only when the caller did not explicitly choose a CUDA visibility policy.
if [[ -z "${CUDA_VISIBLE_DEVICES+x}" ]]; then
  export CUDA_VISIBLE_DEVICES="0"
fi
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

ros2 run usv_rl train_mappo_residual \
  --output "${OUTPUT_PATH}" \
  --resume-from "${RESUME_PATH}" \
  --device auto \
  --amp auto \
  --matmul-precision high \
  --torch-num-threads 1 \
  --total-timesteps "${NEW_TOTAL_TIMESTEPS}" \
  --checkpoint-interval 25600 \
  "$@"