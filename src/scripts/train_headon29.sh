#!/bin/bash
# headon29: Fix bimodal oscillation by narrowing near-miss penalty zone
#
# headon28 postmortem:
#   - Warm-started from h27_30K with near_miss_distance=6.0m, near_miss_weight=2.5
#   - ALL checkpoints (5K-55K+) maintained 0% collision — collapse prevention works
#   - BUT policy exhibits bimodal oscillation with ~10-15K step period:
#     * "Aggressive" mode: worst_sep=7.4-7.7m, mean_sep=8.4-10.5m
#     * "Conservative" mode: worst_sep=10.3-10.5m, mean_sep=10.9-11.8m
#   - Training reward flat at avg=-9.0, no improvement trend through 60+ updates
#
# Root cause: near_miss_distance=6.0m creates a penalty wall that produces two
# competing equilibria rather than one stable solution:
#
#   Below 6.0m: safety penalty = -2.5/m/step + team penalty = -0.30/m/step
#               Combined: -2.80 per meter below 6.0m, per step
#   Above 6.0m: No proximity penalty; progress/heading rewards pull agent closer
#
#   The policy alternates between:
#   (a) Getting pushed away by the 6m penalty wall → conservative mode (10m+)
#   (b) Being pulled back by progress reward → aggressive mode (7-8m)
#   Neither regime is stable because the penalty wall is too far from the
#   ~3-5m sweet spot we want.
#
# Fixes in headon29:
#
#   1) NARROWER NEAR-MISS ZONE: 6.0m → 4.5m
#      Moves the penalty wall closer to the desired avoidance radius (3-5m).
#      Conflict awareness (conflict_distance=6.0, anticipation_distance=6.0)
#      still provides soft behavioral guidance starting at 6m, but the hard
#      penalty only kicks in at 4.5m. This gives the policy more room to
#      approach without triggering penalties.
#
#   2) SOFTER NEAR-MISS GRADIENT: 2.5 → 1.5
#      Combined penalty below 4.5m: safety=-1.5/m + team=-0.30/m = -1.80/m/step
#      (vs -2.80 in h28). Softer gradient means the penalty wall is less "bouncy"
#      and the policy can settle into a stable equilibrium instead of oscillating.
#
#   3) WARM START FROM h28-25K (best aggressive checkpoint):
#      h28-25K: 0% collision, 78.8% progress, 7.43m worst_sep, 8.40m mean_sep
#      This checkpoint is already in the right ballpark — it approaches to ~7m
#      which is close to the 4.5m penalty zone. With the narrower penalty zone,
#      the policy should naturally compress further without oscillating.
#
#   Keep unchanged:
#   - LR=2e-5, clip=0.08 (proven collapse-free, don't need to change)
#   - collision_penalty=-150 (strong enough deterrent)
#   - conflict_distance=6.0, anticipation_distance=6.0 (soft awareness at 6m is good)
#   - All other reward weights and training params

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon29.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from /mnt/data/checkpoints/usv_rl/headon28_checkpoints/headon28_step_0025344.pt \
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
  --near-miss-distance 4.5 \
  --near-miss-weight 1.5 \
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
