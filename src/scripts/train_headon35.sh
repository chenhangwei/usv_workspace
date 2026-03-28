#!/bin/bash
# headon35: Ultra-tight avoidance — target worst_sep ≈ 0.75–0.85m
#
# h34-450K head_on results: 0% collision, worst_sep=1.99m — still too conservative!
# Root cause: inherited policy bias from h33 + penalties still too strong collectively
#
# Three independent forces push agents apart:
#   1. safety near_miss: W * (norm_prox^exp) * band
#   2. team reward: -team_reward_weight * max(0, near_miss_distance - pair_min)
#   3. braking unsafe_close_speed: -weight * near_miss_ratio * speed_excess
#
# h35 strategy: MINIMAL repulsion in narrow band only
#   - near_miss_distance: 1.5 → 1.0 (only 0.25m band: 0.75–1.0m)
#   - near_miss_weight: 2.0 → 1.0
#     At 0.75m: penalty = 1.0 * 1.0^2 * 0.25 = 0.25/step (was 1.5)
#     At 0.85m: penalty = 1.0 * 0.6^2 * 0.25 = 0.09/step
#     At 1.0m+: penalty = 0
#   - team_reward_weight: 0.30 → 0.10
#     At 0.75m: 0.10 * max(0, 1.0-0.75) = 0.025/step (was 0.225)
#   - collision_penalty: -50 → -30 (still 120x the max near-miss per step)
#   - conflict_risk_weight: 1.5 → 0.8
#   - unsafe_close_speed: 1.0 → 0.5
#
# Total penalty at 0.75m: 0.25 + 0.025 = 0.275/step (was ~1.7 in h34, ~53 in h33)
# Total penalty at 0.85m: 0.09 + 0.015 = 0.105/step
# Collision penalty: -30 → collision still 109x worse than 0.75m near-miss
#
# Resume from h33-405K (fresh restart, not from h34 which may have confused gradients)
# Train to 600K (195K new steps from h33-405K for thorough policy adjustment)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon35.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon33_checkpoints/headon33_step_0405504.pt \
  --num-agents 3 \
  --total-timesteps 600000 \
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
  --near-miss-distance 1.0 \
  --near-miss-weight 1.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -30.0 \
  --team-reward-weight 0.10 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 0.8 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 0.5 \
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
