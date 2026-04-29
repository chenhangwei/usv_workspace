#!/bin/bash
# fresh132: neighbor-localized separation-floor polish from fresh124 step0576.
#
# fresh131 confirmed that widening the team-safety brake gate restores gradient
# coverage, but the global team-min separation gate became too broad late in
# training.  This run keeps the wider completion/distance window while
# requiring a local neighbor inside the release radius, targeting moderate
# active coverage instead of team-wide braking.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh124_checkpoints/fresh124_safe_finish_from_fresh101_step_0000576.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh101_checkpoints/fresh101_crossing_progress_recover_step_0008640.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh132_neighbor_sep_floor_from_fresh124_step0576.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh132_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh132_train.log}"
export RUN_NAME="${RUN_NAME:-fresh132}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-222}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00045 \
  --learning-rate 1.3e-7 \
  --learning-rate-end 4.0e-8 \
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
  --crossing-imitation-weight 0.0 \
  --crossing-imitation-weight-end 0.0 \
  --near-goal-finish-weight 0.0 \
  --near-goal-finish-weight-end 0.0 \
  --lagging-finish-weight 0.0 \
  --lagging-finish-weight-end 0.0 \
  --team-safety-brake-weight 0.18 \
  --team-safety-brake-weight-end 0.08 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 12.2 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.15 \
  --team-safety-brake-release-separation 3.20 \
  --team-safety-brake-target-speed 0.17 \
  --team-safety-brake-omega-weight 0.015 \
  --team-safety-brake-target-omega 0.025 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-power 2.20 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 420.0 \
  --policy-anchor-weight-end 500.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-team-safety-brake \
  --ppo-policy-loss-scale 0.0 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.08 \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"