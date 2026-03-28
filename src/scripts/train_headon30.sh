#!/bin/bash
# headon30: Quadratic near-miss penalty to enforce 3-5m minimum separation
#
# headon29 postmortem (v2 eval with TRUE closest approach metric):
#   - v2 eval discovered: previous "pairwise_min_separation" was final-step
#     distance, NOT the running minimum across the episode!
#   - TRUE closest approach for h29 (all checkpoints): ~0.87-1.20m
#   - This is only 0.12-0.45m above the 0.75m collision threshold
#   - At 80K: actually COLLIDED in ep2 (0.74m < 0.75m)
#   - The "bimodal oscillation" observed in h28/h29 (7-12m final distances)
#     was a measurement artifact — the actual encounter distances are uniform ~1m
#   - The LINEAR near_miss penalty (-1.5/m at 4.5m zone) is too gentle at 1m:
#     penalty=-5.25/step × ~8 steps in danger zone ≈ -42 total
#     vs goal progress incentive making the "quick pass" worthwhile
#
# Root cause: Linear penalty gradient is too flat near the collision boundary.
#   penalty(d) = -weight * (near_miss_distance - d)
#   At 1m: -5.25/step. At 3m: -2.25/step. At 4m: -0.75/step.
#   The agent learns to ZIP through the danger zone quickly, absorbing a modest
#   cumulative penalty that's offset by faster goal-reaching progress.
#
# Fix in headon30: QUADRATIC near-miss penalty (exponent=2.0)
#
#   penalty(d) = -weight * ((near_miss_distance - d) / band)^exponent * band
#   where band = near_miss_distance - collision_distance
#
#   With weight=5.0, distance=6.0, collision=0.75, exponent=2.0:
#     band = 6.0 - 0.75 = 5.25
#     At 1.0m: normalized=0.952, penalty = -5.0 * 0.907 * 5.25 = -23.8/step
#     At 2.0m: normalized=0.762, penalty = -5.0 * 0.580 * 5.25 = -15.2/step
#     At 3.0m: normalized=0.571, penalty = -5.0 * 0.327 * 5.25 =  -8.6/step
#     At 4.0m: normalized=0.381, penalty = -5.0 * 0.145 * 5.25 =  -3.8/step
#     At 5.0m: normalized=0.190, penalty = -5.0 * 0.036 * 5.25 =  -0.95/step
#     At 5.5m: normalized=0.095, penalty = -5.0 * 0.009 * 5.25 =  -0.24/step
#
#   Over 8 steps at 1m: -190 total (exceeds collision penalty of -150!)
#   Over 8 steps at 3m:  -69 total (significant deterrent)
#   Over 8 steps at 5m:   -8 total (mild, acceptable brush)
#
#   This creates a steep gradient that:
#   (a) Makes the 1-2m zone nearly as costly as actual collision
#   (b) Provides meaningful deterrent at 3-4m (target zone boundary)
#   (c) Is mild at 5-6m (the outer approach is acceptable)
#
# Other changes:
#   near_miss_distance: 4.5 → 6.0 (wider penalty zone to cover approach)
#   near_miss_weight: 1.5 → 5.0 (stronger base weight)
#   near_miss_exponent: 1.0 → 2.0 (quadratic scaling — the key innovation)
#
# Warm start: h29-25K (best v2 eval: 0% collision, episode_min=0.95-1.18m)
# Keep all other params same as h29.

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon30.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon30_checkpoints/headon30_step_0005184.pt \
  --num-agents 3 \
  --total-timesteps 200000 \
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
  --no-progress-timeout 20.0 \
  \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
