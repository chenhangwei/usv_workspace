#!/bin/bash
# fresh106: narrow post-conflict finish micro-repair from the best fresh104 early checkpoint.
# Trainer-side only; exported artifact remains one pure MAPPO checkpoint.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh106_post_conflict_narrow_finish.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh106_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh106_train.log}"
export RUN_NAME="${RUN_NAME:-fresh106}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-3500}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --checkpoint-interval 384 \
  --clip-range 0.018 \
  --learning-rate 2.2e-6 \
  --learning-rate-end 4.0e-7 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -2.05 \
  --force-actor-log-std -2.05 \
  --scenario-spawn-position-std 0.02 \
  --scenario-spawn-heading-std 0.008 \
  --scenario-goal-position-std 0.02 \
  --near-goal-finish-weight 3.2 \
  --near-goal-finish-weight-end 1.4 \
  --near-goal-finish-distance 3.7 \
  --near-goal-finish-goal-tolerance 0.8 \
  --near-goal-finish-phase-min 0.35 \
  --near-goal-finish-target-speed 0.18 \
  --near-goal-finish-max-omega 0.10 \
  --near-goal-finish-omega-weight 0.30 \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --episode-timeout 130.0 \
  --no-progress-timeout 82.0 \
  --min-progress-delta 0.012 \
  --goal-proximity-relief-distance 4.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 3.0"

exec bash "$SCRIPT_DIR/train_fresh103.sh"