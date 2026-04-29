#!/bin/bash
# fresh134: finish-first continuation with a weak local-danger guard.
#
# fresh133 proved local-danger works mechanically, but its safety-only updates
# did not preserve completion.  This run returns to fresh124_step0576 and uses
# raw-target lagging finish as the main signal, while keeping local-danger as a
# low-weight guard near genuinely close neighbors.

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
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh134_finish_first_local_guard_from_fresh124_step0576.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh134_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh134_train.log}"
export RUN_NAME="${RUN_NAME:-fresh134}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 2 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00085 \
  --learning-rate 1.2e-6 \
  --learning-rate-end 2.0e-7 \
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
  --lagging-finish-weight 3.8 \
  --lagging-finish-weight-end 2.0 \
  --lagging-finish-distance 5.8 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -0.05 \
  --lagging-finish-target-speed 0.26 \
  --lagging-finish-min-speed-scale 0.55 \
  --lagging-finish-raw-target \
  --lagging-finish-max-omega 0.16 \
  --lagging-finish-omega-weight 0.08 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 0.95 \
  --lagging-finish-safe-team-separation 2.20 \
  --lagging-finish-safe-team-separation-power 0.80 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-omega-only \
  --lagging-finish-hold-weight 1.0 \
  --team-safety-brake-weight 0.08 \
  --team-safety-brake-weight-end 0.025 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 10.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 0.95 \
  --team-safety-brake-release-separation 1.45 \
  --team-safety-brake-target-speed 0.18 \
  --team-safety-brake-omega-weight 0.004 \
  --team-safety-brake-target-omega 0.015 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.20 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 200.0 \
  --policy-anchor-weight-end 280.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
  --policy-anchor-exclude-team-safety-brake \
  --ppo-policy-loss-scale 0.0 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.06 \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"