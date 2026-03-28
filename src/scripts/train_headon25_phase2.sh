#!/bin/bash
# headon25 Phase 2: Scale up to 5-agent dense scenarios
#
# Resumes from Phase 1 checkpoint (3-agent smoke, 500K steps) and continues
# training on dense 5-agent scenarios for another 500K steps.
#
# The --load-weights-from mechanism loads actor/critic weights and normalizer
# state WITHOUT restoring training metadata, config, or optimizer state.
# This allows Phase 2 to use different agent count, scenarios, learning rate,
# and entropy schedule while keeping the learned policy parameters.
#
# Entropy continues to decay from Phase 1's endpoint (~0.01) down to 0.005.

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

PHASE1=/mnt/data/checkpoints/usv_rl/headon25_phase1.pt
OUTPUT=/mnt/data/checkpoints/usv_rl/headon25.pt

if [[ ! -f "$PHASE1" ]]; then
  echo "ERROR: Phase 1 checkpoint not found: $PHASE1"
  echo "Run train_headon25.sh first to complete Phase 1 training."
  exit 1
fi

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$PHASE1" \
  --num-agents 5 \
  --total-timesteps 500000 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 1e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.01 \
  --entropy-coef-end 0.005 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario-set dense \
  \
  --squash-actions \
  --min-forward-speed 0.12 \
  --normalize-observations \
  \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  \
  --progress-weight 4.0 \
  --goal-bonus 30.0 \
  --collision-penalty -50.0 \
  --near-miss-weight 8.0 \
  --conflict-risk-weight 3.0 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 3.0 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 1.5 \
  --angular-accel-penalty-weight 1.0 \
  --pure-cruise-reward-weight 2.0 \
  --pure-idle-penalty-weight 3.0 \
  --pure-turn-penalty-weight 0.5 \
  --pure-spin-penalty-weight 3.0 \
  --path-deviation-penalty-weight 1.0 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.0 \
  --stop-go-penalty-weight 1.5 \
  --goal-proximity-reward-weight 1.5 \
  \
  --checkpoint-interval 10000 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 90 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
