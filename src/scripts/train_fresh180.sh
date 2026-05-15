#!/bin/bash
# fresh180: priority-yield repair after fresh179 showed prio=0 agents were still trained as stand-on.

set -eo pipefail

cd "$(dirname "$0")/.."

export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh180_priority_yield_gate_from_fresh172.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh180_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh180_train.log}"
export RUN_NAME="${RUN_NAME:-fresh180}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}"
export EXTRA_TRAIN_ARGS="--random-deconflict-priority-yield-threshold 0.10 ${EXTRA_TRAIN_ARGS:-}"

exec bash ./scripts/train_fresh179.sh