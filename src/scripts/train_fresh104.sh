#!/bin/bash
# fresh104: fresh103 near-goal auxiliary with goal-hold behavior in the trainer loss.
# Starts from the fresh102 2/3-completion checkpoint and writes a separate single MAPPO checkpoint.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh102_checkpoints/fresh102_post_conflict_finish_step_0001728.pt}"
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh104_goal_hold.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh104_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh104_train.log}"
export RUN_NAME="${RUN_NAME:-fresh104}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-9000}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}"

exec bash "$SCRIPT_DIR/train_fresh103.sh"
