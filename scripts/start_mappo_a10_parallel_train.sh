#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"
MAX_FASTDDS_SAFE_DOMAIN_ID=232

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "usage: start_mappo_a10_parallel_train.sh [run_name] [extra train args...]"
  echo "env: NUM_SAMPLER_WORKERS(default=2) BASE_ROS_DOMAIN_ID(default=220)"
  exit 0
fi

RUN_NAME="${1:-mappo_dense_a10_parallel_$(date +%Y%m%d_%H%M%S)}"

if [[ $# -gt 0 ]]; then
  shift
fi

NUM_SAMPLER_WORKERS="${NUM_SAMPLER_WORKERS:-2}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}"
HIGHEST_ROS_DOMAIN_ID=$((BASE_ROS_DOMAIN_ID + NUM_SAMPLER_WORKERS - 1))

if (( BASE_ROS_DOMAIN_ID < 0 )); then
  echo "error: BASE_ROS_DOMAIN_ID must be >= 0" >&2
  exit 2
fi

if (( HIGHEST_ROS_DOMAIN_ID > MAX_FASTDDS_SAFE_DOMAIN_ID )); then
  echo "error: requested ROS domain range ${BASE_ROS_DOMAIN_ID}-${HIGHEST_ROS_DOMAIN_ID} exceeds safe Fast DDS limit ${MAX_FASTDDS_SAFE_DOMAIN_ID}" >&2
  exit 2
fi

OUTPUT_PATH="${DATA_DIR}/${RUN_NAME}.pt"
LOG_PATH="${DATA_DIR}/${RUN_NAME}.log"

mkdir -p "${DATA_DIR}"

cd "${WORKSPACE_DIR}"

echo "run_name=${RUN_NAME}"
echo "output_path=${OUTPUT_PATH}"
echo "log_path=${LOG_PATH}"
echo "num_sampler_workers=${NUM_SAMPLER_WORKERS}"
echo "base_ros_domain_id=${BASE_ROS_DOMAIN_ID}"
echo "highest_ros_domain_id=${HIGHEST_ROS_DOMAIN_ID}"

: > "${LOG_PATH}"
stdbuf -oL -eL ./scripts/train_mappo_a10_pai.sh \
  "${OUTPUT_PATH}" \
  --num-sampler-workers "${NUM_SAMPLER_WORKERS}" \
  --base-ros-domain-id "${BASE_ROS_DOMAIN_ID}" \
  "$@" |& tee "${LOG_PATH}"