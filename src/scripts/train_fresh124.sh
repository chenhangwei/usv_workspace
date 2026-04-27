#!/bin/bash
# fresh124: conservative safe-finish repair from the fresh101 crossing-safe base.
#
# fresh123 showed that adding more crossing brake/turn pressure to a colliding
# base does not recover stable crossing.  This run returns to the historical
# fresh101_step_0008640 safety base and only trains a trainer-side final-meters
# auxiliary when post-conflict fleet separation is already safe.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh101_checkpoints/fresh101_crossing_progress_recover_step_0008640.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh101_crossing_progress_recover.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh124_safe_finish_from_fresh101.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh124_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh124_train.log}"
export RUN_NAME="${RUN_NAME:-fresh124}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1152}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.0010 \
  --learning-rate 8.0e-7 \
  --learning-rate-end 1.5e-7 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.65 \
  --force-actor-log-std -1.65 \
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
  --lagging-finish-weight 3.0 \
  --lagging-finish-weight-end 1.5 \
  --lagging-finish-distance 4.2 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min 0.12 \
  --lagging-finish-target-speed 0.18 \
  --lagging-finish-min-speed-scale 0.35 \
  --lagging-finish-max-omega 0.10 \
  --lagging-finish-omega-weight 0.20 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 1.30 \
  --lagging-finish-safe-team-separation 2.80 \
  --lagging-finish-safe-team-separation-power 1.40 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 1.5 \
  --team-safety-brake-weight 0.0 \
  --team-safety-brake-weight-end 0.0 \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --policy-anchor-weight 240.0 \
  --policy-anchor-weight-end 260.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
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