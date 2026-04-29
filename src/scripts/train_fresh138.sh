#!/bin/bash
# fresh138: low-omega finish continuation from fresh136_step0864.
#
# fresh137 produced only a tiny actor delta, and its best single-run goal
# signal was not attributable to training because an unchanged checkpoint saw
# the same goal completion on seed 1370. This run keeps the continuation small
# but relaxes the anchor enough to create a measurable actor update, while
# using a very low lagging-finish omega target instead of hold-only yaw loss.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh136_checkpoints/fresh136_active_finish_from_fresh128_step_0000864.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh128_balanced_finish_from_fresh124_step0576.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh138_low_omega_finish_from_fresh136_step0864.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh138_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh138_train.log}"
export RUN_NAME="${RUN_NAME:-fresh138}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-576}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-194}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00055 \
  --learning-rate 2.4e-7 \
  --learning-rate-end 5.0e-8 \
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
  --lagging-finish-weight 1.85 \
  --lagging-finish-weight-end 0.85 \
  --lagging-finish-distance 5.6 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -0.08 \
  --lagging-finish-target-speed 0.18 \
  --lagging-finish-min-speed-scale 0.34 \
  --lagging-finish-max-omega 0.040 \
  --lagging-finish-omega-weight 0.11 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 1.00 \
  --lagging-finish-safe-team-separation 2.55 \
  --lagging-finish-safe-team-separation-power 1.20 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-weight 1.45 \
  --team-safety-brake-weight 0.055 \
  --team-safety-brake-weight-end 0.022 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 9.2 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.10 \
  --team-safety-brake-release-separation 1.68 \
  --team-safety-brake-target-speed 0.10 \
  --team-safety-brake-omega-weight 0.006 \
  --team-safety-brake-target-omega 0.018 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.55 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 460.0 \
  --policy-anchor-weight-end 660.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
  --policy-anchor-exclude-team-safety-brake \
  --ppo-policy-loss-scale 0.0 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.10 \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"