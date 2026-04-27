#!/bin/bash
# fresh86: scenario-specific full post-attention actor trunk from fresh79_best
#
# fresh85 postmortem:
#   - scenario-specific final action heads were finally wired correctly, but they still only moved
#     the policy to a safer-yet-more-conservative overtaking regime (0 collision, 1.0 timeout)
#     without beating fresh83 overall.
#   - likely root cause: the shared frozen actor trunk remains too dominant; replacing only the
#     final action slice does not give the branch enough representational authority.
#
# working hypothesis:
#   Replace the entire post-attention actor trunk for the overtaking scenario while keeping the
#   attention encoder shared. This gives scenario-specific capacity earlier than fresh85 head-only,
#   but still avoids fully duplicating the encoder or retraining the base policy.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh79_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh86_overtaking_scenario_trunk.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh86_checkpoints"

mkdir -p "$CKPT_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  exit 1
fi

echo "========== fresh86 Overtaking Scenario Trunk (8K from fresh79_best) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 800 \
  --num-agents 3 \
  --total-timesteps 8000 \
  --clip-range 0.10 \
  --learning-rate 2.0e-5 \
  --learning-rate-end 5.0e-6 \
  --entropy-coef 0.0010 \
  --entropy-coef-end 0.0002 \
  --episode-timeout 70.0 \
  --no-progress-timeout 20.0 \
  --min-progress-delta 0.30 \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.8 \
  --scenario-spawn-heading-std 0.32 \
  --scenario-goal-position-std 0.8 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 6.0 \
  --goal-bonus 40.0 \
  --time-penalty 0.07 \
  --stall-penalty -50.0 \
  --heading-error-weight 5.0 \
  --heading-relief-factor 0.85 \
  --heading-correction-reward-weight 1.5 \
  --action-smoothness-weight 6.0 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --omega-flip-saturation-threshold 0.40 \
  --separation-recovery-weight 1.0 \
  --entanglement-penalty-weight 1.5 \
  --entanglement-low-speed-penalty-weight 2.5 \
  --collision-penalty -400.0 \
  --near-miss-distance 4.5 \
  --near-miss-weight 12.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 9.0 \
  --anticipation-distance 9.0 \
  --proximity-gradient-penalty-weight 4.0 \
  --proximity-gradient-distance 5.5 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 5.5 \
  --conflict-overspeed-penalty-weight 4.0 \
  --avoidance-turn-reward-weight 4.0 \
  --desired-conflict-speed 0.18 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.2 \
  --overtaking-corridor-reward-weight 2.4 \
  --overtaking-centerline-penalty-weight 2.2 \
  --overtaking-close-penalty-weight 3.4 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.25 \
  --goal-proximity-speed-relief 0.0 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --squash-actions \
  --min-forward-speed 0.05 \
  --angular-delta-limit 0.40 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 1.0 \
  --angular-accel-limit 1.2 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.50 \
  --angular-authority-floor 0.35 \
  --min-forward-speed-floor 0.0 \
  --collision-distance 0.75 \
  --sim-tau-linear 0.6 \
  --sim-tau-angular 0.35 \
  --dr-tau-linear-low 0.4 \
  --dr-tau-linear-high 1.0 \
  --dr-tau-angular-low 0.2 \
  --dr-tau-angular-high 1.0 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 208

echo "fresh86 overtaking scenario trunk complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"