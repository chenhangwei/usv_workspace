#!/bin/bash
# fresh123: starboard-commitment safety brake from the best fresh122 checkpoint.
#
# fresh122's nearest-neighbor escape turn can produce one safe single episode,
# but three-episode confirmation still collides with high omega flip counts.
# This run keeps the loss trainer-side only and changes the yaw target to a
# crossing starboard commitment so the policy does not alternate turn sides
# as the nearest neighbor bearing changes.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh122_checkpoints/fresh122_early_brake_escape_turn_step_0000576.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh109_near_team_lagging_finish.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh123_starboard_commit.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh123_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh123_train.log}"
export RUN_NAME="${RUN_NAME:-fresh123}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00025 \
  --learning-rate 1.8e-8 \
  --learning-rate-end 6.0e-9 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -4.45 \
  --force-actor-log-std -4.45 \
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
  --lagging-finish-weight 0.0 \
  --lagging-finish-weight-end 0.0 \
  --team-safety-brake-weight 8.0 \
  --team-safety-brake-weight-end 5.8 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 12.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.12 \
  --team-safety-brake-release-separation 4.60 \
  --team-safety-brake-target-speed 0.040 \
  --team-safety-brake-omega-weight 1.05 \
  --team-safety-brake-target-omega 0.38 \
  --team-safety-brake-turn-mode starboard \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-power 0.65 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 55.0 \
  --policy-anchor-weight-end 90.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-team-safety-brake \
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