#!/bin/bash
# fresh118: safe-gap lagging finish from fresh109.
#
# fresh117 proved the lagging-finish gate can activate under aux-only training,
# but the target still pushed lagging agents through low-separation states and
# all tested checkpoints collided around the 0.72m threshold. This run keeps the
# single pure MAPPO checkpoint and trainer-side-only repair, while scaling the
# lagging finish target speed down when fleet separation is below a safe gap.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh118_safe_gap_lagging_finish.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh118_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh118_train.log}"
export RUN_NAME="${RUN_NAME:-fresh118}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1152}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-190}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00025 \
  --learning-rate 2.0e-8 \
  --learning-rate-end 6.0e-9 \
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
  --lagging-finish-weight 2.8 \
  --lagging-finish-weight-end 1.8 \
  --lagging-finish-distance 8.8 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -1.0 \
  --lagging-finish-target-speed 0.11 \
  --lagging-finish-min-speed-scale 0.55 \
  --lagging-finish-max-omega 0.035 \
  --lagging-finish-omega-weight 0.06 \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 4.0 \
  --lagging-finish-min-team-completion 0.30 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 1.25 \
  --lagging-finish-min-team-separation 0.70 \
  --lagging-finish-safe-team-separation 1.35 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --policy-anchor-weight 80.0 \
  --policy-anchor-weight-end 105.0 \
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