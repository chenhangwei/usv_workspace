#!/bin/bash
# fresh105: short wide-finish-band continuation from the best fresh104 early checkpoint.
# Trainer-side only: the exported artifact remains one pure MAPPO checkpoint.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh104_checkpoints/fresh104_goal_hold_step_0003006.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh104_checkpoints/fresh104_goal_hold_step_0004228.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh105_wide_finish_band.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh105_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh105_train.log}"
export RUN_NAME="${RUN_NAME:-fresh105}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5000}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-229}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --checkpoint-interval 576 \
  --clip-range 0.035 \
  --learning-rate 7.0e-6 \
  --learning-rate-end 8.0e-7 \
  --entropy-coef 0.00008 \
  --entropy-coef-end 0.0 \
  --actor-log-std-init -1.80 \
  --force-actor-log-std -1.80 \
  --near-goal-finish-weight 10.0 \
  --near-goal-finish-weight-end 6.0 \
  --near-goal-finish-distance 5.0 \
  --near-goal-finish-goal-tolerance 0.8 \
  --near-goal-finish-target-speed 0.26 \
  --near-goal-finish-max-omega 0.12 \
  --near-goal-finish-omega-weight 0.40 \
  --crossing-imitation-weight 0.12 \
  --crossing-imitation-weight-end 0.03 \
  --episode-timeout 125.0 \
  --no-progress-timeout 68.0 \
  --min-progress-delta 0.018"

exec bash "$SCRIPT_DIR/train_fresh103.sh"