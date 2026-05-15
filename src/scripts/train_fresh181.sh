#!/bin/bash
# fresh181: pairwise priority-delta deconflict role after fresh180 was too conservative globally.

set -eo pipefail

cd "$(dirname "$0")/.."

export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh181_pairwise_priority_delta_from_fresh172.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh181_train.log}"
export RUN_NAME="${RUN_NAME:-fresh181}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-222}"
export EXTRA_TRAIN_ARGS="--random-deconflict-role-mode priority-delta --random-deconflict-priority-delta-yield-threshold -0.01 ${EXTRA_TRAIN_ARGS:-}"

exec bash ./scripts/train_fresh179.sh