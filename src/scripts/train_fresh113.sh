#!/bin/bash
# fresh113: anchored raw-target last-meter continuation from fresh109 safe 2/3.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh113_anchor_raw_target.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh113_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh113_train.log}"
export RUN_NAME="${RUN_NAME:-fresh113}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-185}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.0006 \
  --learning-rate 4.0e-8 \
  --learning-rate-end 2.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -4.05 \
  --force-actor-log-std -4.05 \
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
  --lagging-finish-weight 2.2 \
  --lagging-finish-weight-end 1.4 \
  --lagging-finish-distance 5.2 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -1.0 \
  --lagging-finish-target-speed 0.14 \
  --lagging-finish-min-speed-scale 0.35 \
  --lagging-finish-raw-target \
  --lagging-finish-max-omega 0.12 \
  --lagging-finish-omega-weight 0.14 \
  --lagging-finish-min-team-completion 0.30 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 1.20 \
  --lagging-finish-min-team-separation 1.05 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --policy-anchor-weight 24.0 \
  --policy-anchor-weight-end 28.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"