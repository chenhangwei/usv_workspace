#!/bin/bash
# headon31: Multi-scenario balanced training — fundamental improvements
#
# h30-130K cross-scenario evaluation revealed:
#   - head_on: 0% collision, 87% progress (excellent)
#   - crossing: 7% collision, worst_sep=0.75m (barely safe)
#   - overtaking: 100% collision at steps 21-27 (catastrophic)
#
# Root causes identified:
#   1. SCENARIO: overtaking spawns USV1 & USV2 only 2.5m apart, same direction,
#      same speed → collision is geometrically inevitable in ~25 steps
#   2. DETECTION: overtaking guidance requires body_vx < -0.03 (neighbor slower),
#      but with same-policy RL agents at same speed, the condition never triggers
#      → zero guidance signal for same-direction encounters
#   3. WEIGHTS: crossing guidance rewards (0.7/0.45) are ~3x weaker than head_on
#      (2.4/1.0/1.4) → policy barely learns crossing avoidance
#   4. TRAINING: h30 trained ONLY on two_usv_head_on → never exposed to crossing/
#      overtaking dynamics during training
#
# Fixes implemented (code changes in multi_agent_scenarios.py + multi_agent_env.py):
#   1. Overtaking scenario redesigned: 6m spacing, lateral offset, slow background
#      track ahead for real overtaking practice
#   2. Added co-directional close encounter detection: same-speed same-direction
#      neighbors now trigger overtaking guidance (abs(body_vx) < 0.15 check)
#   3. Crossing/overtaking guidance weights boosted ~3x via CLI args (below)
#   4. Training on ALL THREE scenarios equally (--scenarios flag)
#
# Warm start: h30-130K (best headon checkpoint: 0% collision, 1.40m worst_sep)
# Total: 300K steps (enough for multi-scenario convergence)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon31.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon30_checkpoints/headon30_step_0130176.pt \
  --num-agents 3 \
  --total-timesteps 300000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 2e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.08 \
  --entropy-coef 0.025 \
  --max-grad-norm 0.2 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario two_usv_head_on --scenario three_usv_crossing --scenario three_usv_overtaking \
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
  --near-miss-distance 6.0 \
  --near-miss-weight 5.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -150.0 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 3.0 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 3.0 \
  --conflict-distance 6.0 \
  --anticipation-distance 6.0 \
  --head-on-guidance-distance 6.0 \
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
  --crossing-starboard-turn-reward-weight 2.0 \
  --crossing-forward-reward-weight 1.2 \
  --overtaking-starboard-turn-reward-weight 1.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.0 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 2.5 \
  --colregs-port-turn-penalty-weight 1.5 \
  \
  --no-progress-timeout 20.0 \
  \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
