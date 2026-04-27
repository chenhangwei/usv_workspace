#!/bin/bash
# fresh108: earlier gated partial-team finish repair from the safe fresh106 1152 checkpoint.
# The auxiliary activates only after at least one teammate has reached the goal and fleet separation is safe.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh106_checkpoints/fresh106_post_conflict_narrow_finish_step_0001152.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh104_checkpoints/fresh104_goal_hold_step_0003006.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh108_partial_lagging_finish.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh108_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh108_train.log}"
export RUN_NAME="${RUN_NAME:-fresh108}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-217}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 2 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.004 \
  --learning-rate 3.5e-7 \
  --learning-rate-end 1.0e-7 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -2.45 \
  --force-actor-log-std -2.45 \
  --freeze-observation-normalizer \
  --scenario-spawn-position-std 0.0 \
  --scenario-spawn-heading-std 0.0 \
  --scenario-goal-position-std 0.0 \
  --dr-position-noise-std 0.0 \
  --dr-heading-noise-std 0.0 \
  --dr-velocity-noise-ratio 0.0 \
  --dr-current-speed-max 0.0 \
  --dr-velocity-exec-noise 0.0 \
  --near-goal-finish-weight 0.0 \
  --near-goal-finish-weight-end 0.0 \
  --lagging-finish-weight 7.0 \
  --lagging-finish-weight-end 4.0 \
  --lagging-finish-distance 8.8 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -1.0 \
  --lagging-finish-target-speed 0.12 \
  --lagging-finish-max-omega 0.06 \
  --lagging-finish-omega-weight 0.15 \
  --lagging-finish-min-team-completion 0.30 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-min-team-separation 1.00 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --episode-timeout 135.0 \
  --no-progress-timeout 95.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"