#!/bin/bash
# fresh137: conservative stabilization continuation from fresh136_step0864.
#
# fresh136_step0864 is the first post-migration candidate that improved the
# 3-episode seed=1360 confirm over fresh128 while keeping separation intact.
# This run keeps the same trainer-side mechanisms but lowers the finish/brake
# pressure and strengthens the anchor to avoid overdriving omega or COLREGs.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh137_stabilize_from_fresh136_step0864.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh137_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh137_train.log}"
export RUN_NAME="${RUN_NAME:-fresh137}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-576}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-198}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00045 \
  --learning-rate 1.8e-7 \
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
  --lagging-finish-weight 1.35 \
  --lagging-finish-weight-end 0.65 \
  --lagging-finish-distance 5.4 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -0.08 \
  --lagging-finish-target-speed 0.17 \
  --lagging-finish-min-speed-scale 0.34 \
  --lagging-finish-max-omega 0.085 \
  --lagging-finish-omega-weight 0.12 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 1.00 \
  --lagging-finish-safe-team-separation 2.50 \
  --lagging-finish-safe-team-separation-power 1.15 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 1.35 \
  --team-safety-brake-weight 0.045 \
  --team-safety-brake-weight-end 0.018 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 9.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.10 \
  --team-safety-brake-release-separation 1.62 \
  --team-safety-brake-target-speed 0.10 \
  --team-safety-brake-omega-weight 0.006 \
  --team-safety-brake-target-omega 0.020 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.55 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 620.0 \
  --policy-anchor-weight-end 820.0 \
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