#!/bin/bash
# headon26: Fix late-training policy collapse via LR decay + tighter PPO updates
#
# headon25 postmortem (Phase 1):
#   - Steps 0-300K: Excellent learning curve (0% collision, 87% progress at 300K)
#   - Steps 300K-320K: Catastrophic collapse — 0% collision → 100% collision
#   - Steps 320K-500K: Never recovered, stayed at 100% collision / 50% progress
#
# Root cause: The optimizer (constant LR=3e-4) + low entropy (0.026) + standard
# clip=0.2 allowed a single bad gradient batch to push the policy over the
# "collision cliff". Once past it, progressive reward from rushing offset
# collision penalty, creating a local optimum the agent couldn't escape.
#
# Fixes applied in headon26:
#
#   1) LEARNING RATE SCHEDULE — 3e-4 → 5e-5 linear decay. At step 300K (60%),
#      LR ≈ 1.5e-4, half of constant. By 500K, LR = 5e-5. This prevents
#      catastrophic policy shifts in late training when the policy is mature.
#
#   2) HIGHER ENTROPY FLOOR — 0.04 → 0.02 (was 0.05 → 0.01). The floor is
#      2x higher. At 300K, entropy ≈ 0.028 (was 0.026). Enough exploration to
#      self-correct if the policy drifts toward risky behavior.
#
#   3) TIGHTER PPO CLIP — 0.15 (was 0.2). Limits policy ratio to [0.85, 1.15]
#      instead of [0.8, 1.2]. Prevents large single-step policy changes.
#
#   4) STRONGER COLLISION PENALTY — -80 (was -50). Makes collision NET cost
#      higher even if agent accumulates significant progress reward.
#      At 50% progress: reward ≈ 3.0×4.3 = 12.9, penalty = -80, net = -67.
#
#   5) LOWER PROGRESS WEIGHT — 3.0 (was 4.0). Reduces the payoff of rushing,
#      shifting the optimal policy further toward safe detour paths.
#
#   6) TIGHTER GRADIENT CLIP — 0.3 (was 0.5). Limits gradient magnitude
#      to prevent outlier batches from causing large parameter shifts.

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon26.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --num-agents 3 \
  --total-timesteps 500000 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --learning-rate-end 5e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.04 \
  --entropy-coef-end 0.02 \
  --max-grad-norm 0.3 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario-set smoke \
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
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --collision-penalty -80.0 \
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
