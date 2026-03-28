#!/bin/bash
# headon32: Enhanced near-miss penalty for overtaking collision avoidance
#
# h31 evaluation (at 185-200K) showed:
#   - overtaking: 100% collision still, BUT progress jumped 19.7% → 85.6%
#   - Agents navigate well but min_sep=0.60-0.75m (collision threshold=0.75m)
#   - Mean separation=2.5-3.0m, so it's ONE brief moment per episode
#
# Root cause: near-miss penalty gradient at collision boundary is too flat
#   Current: W*exp = 5.0*2.0 = 10.0 penalty/meter at boundary
#   The agent has almost no incentive to maintain 0.8m vs 0.7m separation
#
# Fix: increase near-miss gradient 3x
#   - near_miss_weight: 5.0 → 10.0 (2x)
#   - near_miss_exponent: 2.0 → 3.0 (penalty curve steeper near boundary)
#   - Gradient at boundary: 10.0*3.0 = 30.0 penalty/meter (3x improvement)
#   - Penalty profile: 52.5/step @0.75m, 23.3 @2m, 2.9 @4m (drops fast at distance)
#
# Resume from: h31 200K checkpoint (best available, already has multi-scenario skills)
# Total: 350K steps (additional 150K from h31-200K warm start)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon32.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon31_checkpoints/headon31_step_0200448.pt \
  --num-agents 3 \
  --total-timesteps 350000 \
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
  --near-miss-weight 10.0 \
  --near-miss-exponent 3.0 \
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
