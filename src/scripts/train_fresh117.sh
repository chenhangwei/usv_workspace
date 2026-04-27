#!/bin/bash
# fresh117: longer conservative linear-only lagging finish, waiting for update 3/4 gate activation.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh109_near_team_lagging_finish.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh106_checkpoints/fresh106_post_conflict_narrow_finish_step_0001152.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh117_long_linear_hold_finish.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh117_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh117_train.log}"
export RUN_NAME="${RUN_NAME:-fresh117}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1152}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-174}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00025 \
  --learning-rate 2.4e-8 \
  --learning-rate-end 8.0e-9 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -4.40 \
  --force-actor-log-std -4.40 \
  --freeze-observation-normalizer \
  --separate-actor-critic-grad-clip \
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
  --lagging-finish-weight 2.4 \
  --lagging-finish-weight-end 1.6 \
  --lagging-finish-distance 7.2 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -1.0 \
  --lagging-finish-target-speed 0.11 \
  --lagging-finish-min-speed-scale 0.70 \
  --lagging-finish-max-omega 0.04 \
  --lagging-finish-omega-weight 0.08 \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 12.0 \
  --lagging-finish-min-team-completion 0.30 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 1.25 \
  --lagging-finish-min-team-separation 1.02 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --policy-anchor-weight 70.0 \
  --policy-anchor-weight-end 90.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --ppo-policy-loss-scale 0.0 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.12 \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"