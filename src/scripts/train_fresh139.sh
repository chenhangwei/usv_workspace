#!/bin/bash
# fresh139: anticipatory close-pass guard from fresh138 step0576.
#
# SITL with fresh138_step0576 completed all waypoints but had repeated close
# passes below the 0.75 m separation floor. This branch keeps the completion
# behavior, adds earlier crossing-yield shaping, strengthens local close-pass
# braking, and lightly re-enables PPO reward shaping for anti-entanglement and
# straight-line smoothness.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh138_checkpoints/fresh138_low_omega_finish_from_fresh136_step0864_step_0000576.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh138_low_omega_finish_from_fresh136_step0864.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

export BASE_INPUT
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh139_anticipatory_guard_from_fresh138_step0576.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh139_checkpoints}"
export LOG_FILE="${LOG_FILE:-/tmp/fresh139_train.log}"
export RUN_NAME="${RUN_NAME:-fresh139}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-864}"
export BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-190}"

export EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:-} \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 96 \
  --checkpoint-interval 288 \
  --clip-range 0.00045 \
  --learning-rate 2.1e-7 \
  --learning-rate-end 4.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.75 \
  --force-actor-log-std -1.75 \
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
  --heading-omega-deadband 0.08 \
  --heading-omega-reference 0.85 \
  --angular-authority-power 1.20 \
  --angular-accel-limit 1.20 \
  --angular-decel-limit 2.80 \
  --conflict-turn-relief 0.84 \
  --angular-authority-floor 0.42 \
  --ppo-policy-loss-scale 0.030 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.08 \
  --collision-distance 0.75 \
  --near-miss-distance 3.20 \
  --near-miss-weight 96.0 \
  --near-miss-exponent 2.4 \
  --separation-recovery-weight 14.0 \
  --conflict-risk-weight 4.0 \
  --conflict-resolution-reward-weight 5.0 \
  --conflict-escalation-penalty-weight 14.0 \
  --unsafe-close-speed-penalty-weight 30.0 \
  --proximity-gradient-penalty-weight 42.0 \
  --proximity-gradient-distance 3.20 \
  --speed-distance-coupling-penalty-weight 44.0 \
  --speed-distance-coupling-threshold 3.20 \
  --entanglement-penalty-weight 7.0 \
  --entanglement-distance 4.0 \
  --entanglement-grace-steps 4 \
  --entanglement-low-speed-penalty-weight 1.20 \
  --crossing-time-separation-reward-weight 92.0 \
  --crossing-time-separation-penalty-weight 260.0 \
  --crossing-time-gap-target 9.5 \
  --crossing-starboard-turn-reward-weight 7.5 \
  --crossing-forward-reward-weight 2.0 \
  --crossing-slowdown-reward-weight 13.0 \
  --crossing-overspeed-penalty-weight 38.0 \
  --crossing-close-forward-penalty-weight 58.0 \
  --crossing-yield-speed 0.06 \
  --colregs-port-turn-penalty-weight 18.0 \
  --avoidance-turn-reward-weight 2.5 \
  --desired-conflict-speed 0.06 \
  --path-deviation-penalty-weight 1.2 \
  --path-deviation-tolerance 1.10 \
  --path-deviation-conflict-scale 0.85 \
  --heading-error-weight 1.20 \
  --heading-relief-factor 0.92 \
  --heading-convergence-reward-weight 0.36 \
  --heading-convergence-threshold-deg 14.0 \
  --heading-correction-reward-weight 0.62 \
  --action-smoothness-weight 4.2 \
  --angular-accel-penalty-weight 1.6 \
  --straight-line-omega-penalty-weight 5.8 \
  --straight-line-omega-conflict-floor 0.08 \
  --straight-line-omega-cte-gate 0.75 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 0.58 \
  --omega-flip-saturation-threshold 0.22 \
  --pure-turn-penalty-weight 0.35 \
  --pure-spin-penalty-weight 0.55 \
  --crossing-imitation-weight 0.12 \
  --crossing-imitation-weight-end 0.04 \
  --crossing-imitation-pretrain-epochs 1 \
  --crossing-imitation-clear-speed 0.30 \
  --crossing-imitation-middle-speed 0.12 \
  --crossing-imitation-yield-speed 0.03 \
  --crossing-imitation-clear-omega -0.05 \
  --crossing-imitation-middle-omega -0.09 \
  --crossing-imitation-yield-omega -0.14 \
  --crossing-imitation-eta-gate 1.05 \
  --crossing-imitation-phase-min -1.00 \
  --crossing-imitation-phase-max 0.12 \
  --near-goal-finish-weight 0.0 \
  --near-goal-finish-weight-end 0.0 \
  --lagging-finish-weight 1.05 \
  --lagging-finish-weight-end 0.45 \
  --lagging-finish-distance 5.4 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -0.08 \
  --lagging-finish-target-speed 0.16 \
  --lagging-finish-min-speed-scale 0.30 \
  --lagging-finish-max-omega 0.035 \
  --lagging-finish-omega-weight 0.16 \
  --lagging-finish-min-team-completion 0.0 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 0.0 \
  --lagging-finish-min-team-separation 0.85 \
  --lagging-finish-safe-team-separation 2.20 \
  --lagging-finish-safe-team-separation-power 1.60 \
  --lagging-finish-crossing-only \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-weight 1.45 \
  --team-safety-brake-weight 0.18 \
  --team-safety-brake-weight-end 0.07 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 12.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 0.85 \
  --team-safety-brake-release-separation 2.35 \
  --team-safety-brake-target-speed 0.03 \
  --team-safety-brake-omega-weight 0.055 \
  --team-safety-brake-target-omega 0.085 \
  --team-safety-brake-turn-mode starboard \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.20 \
  --team-safety-brake-crossing-only \
  --policy-anchor-weight 340.0 \
  --policy-anchor-weight-end 540.0 \
  --policy-anchor-crossing-only \
  --policy-anchor-exclude-lagging-finish \
  --policy-anchor-exclude-team-safety-brake \
  --episode-timeout 135.0 \
  --no-progress-timeout 105.0 \
  --min-progress-delta 0.008 \
  --goal-proximity-relief-distance 5.0 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 2.2"

exec bash "$SCRIPT_DIR/train_fresh103.sh"