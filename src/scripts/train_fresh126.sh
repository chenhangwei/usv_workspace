#!/bin/bash
# fresh126: wide post-conflict finish repair with activation restored.
#
# fresh125 widened the finish band but required team completion, which produced
# no active samples.  This keeps the wider post-conflict band but restores the
# fresh124-style completion gate while relying on phase and separation gates for
# safety.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh126_wide_finish_active_from_fresh101.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh126_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh126_train.log}"
export RUN_NAME="${RUN_NAME:-fresh126}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1440}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.0011 \
  --learning-rate 1.0e-6 \
  --learning-rate-end 2.5e-7 \
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
  --lagging-finish-weight 2.4 \
  --lagging-finish-weight-end 1.2 \
  --lagging-finish-distance 6.4 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min 0.08 \
  --lagging-finish-target-speed 0.22 \
  --lagging-finish-min-speed-scale 0.40 \
  --lagging-finish-max-omega 0.08 \
  --lagging-finish-omega-weight 0.12 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 1.60 \
  --lagging-finish-safe-team-separation 3.30 \
  --lagging-finish-safe-team-separation-power 1.25 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 1.2 \
  --team-safety-brake-weight 0.0 \
  --team-safety-brake-weight-end 0.0 \
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --policy-anchor-weight 240.0 \
  --policy-anchor-weight-end 270.0 \
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