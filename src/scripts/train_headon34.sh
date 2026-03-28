#!/bin/bash
# headon34: Tight avoidance — worst_sep should approach 0.75m
#
# h33-355K results: 0% collision on all 3 scenarios, but too conservative:
#   head_on worst_sep=2.21m, crossing=1.22m, overtaking=1.58m
#   User wants worst_sep close to 0.75m (tight but safe avoidance)
#
# Changes from h33:
#   1. near_miss_weight: 10.0 → 2.0 (5x reduction)
#   2. near_miss_exponent: 3.0 → 2.0 (less steep curve)
#   3. near_miss_distance: 6.0 → 1.5 (shrink avoidance zone from 6m to 1.5m)
#      At 0.75m: penalty = 2.0 * (1.0)^2 * 0.75 = 1.5/step
#      At 1.0m: penalty = 2.0 * (0.67)^2 * 0.75 = 0.67/step → steep drop
#      At 1.5m+: penalty = 0 → no pressure to stay farther
#   4. collision_penalty: -150 → -50 (reduce over-cautious behavior)
#   5. conflict_risk_weight: 3.0 → 1.5 (less pre-emptive braking)
#   6. unsafe_close_speed_penalty_weight: 3.0 → 1.0 (allow faster near passes)
#
# Resume from h33-405K (latest, has strong multi-scenario + COLREGs foundation)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon34.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon33_checkpoints/headon33_step_0405504.pt \
  --num-agents 3 \
  --total-timesteps 550000 \
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
  --near-miss-distance 1.5 \
  --near-miss-weight 2.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -50.0 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 1.5 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 1.0 \
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
