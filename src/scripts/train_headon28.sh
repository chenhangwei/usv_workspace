#!/bin/bash
# headon28: Fix policy oscillation via slower updates + stronger collision deterrent
#
# headon27 postmortem:
#   - Warm-started from h26_400K (safe, bimodal) with wider near-miss zone (4.0m)
#   - 30K checkpoint: 100% safe, 0% collision, 89.1% progress (over-conservative, 10m+ sep)
#   - 40K checkpoint: BIMODAL — 3/5 conservative (11m+), 2/5 collision (0.73m)
#   - 50K checkpoint: 100% collision at step 58-66, 0.70m min separation
#   - Policy transitioned from all-safe → bimodal → all-collision in just 20K steps
#   - Pattern identical to h26 but compressed: safe→bimodal→collapse
#
# Root causes identified:
#
#   1) POLICY UPDATE TOO LARGE: LR=5e-5, clip=0.15 allows per-update policy shifts
#      large enough to jump from "avoid at 10m" to "ignore avoidance" in a few updates.
#      With only 576 samples/update (1-3 episodes), gradient variance is massive.
#      Update 70 (40K): best reward -3.16, immediately followed by update 71: -7.21
#      with 5 episodes (multiple early crashes). The policy swung dramatically.
#
#   2) GAE COLLISION PROPAGATION TOO WEAK: γ=0.99, λ=0.95 means collision penalty at
#      step 60 reaches step 0 with only 2% weight ((0.99*0.95)^60 = 0.0203).
#      Progress reward at step 0 (~0.27/step) dominates discounted collision penalty
#      (-80 * 0.02 = -1.6). Early actions are rewarded for going straight at target.
#
#   3) NEAR-MISS ZONE TOO NARROW FOR CLOSURE SPEED: At 0.8m/s relative closure,
#      USVs transit the 0.75-4.0m zone in ~20 steps (4 seconds). Near-miss penalty
#      accumulates ~96 total, but only in the last 20 steps of a 60-step episode.
#      The first 40 steps look "clean" and are dominated by progress incentive.
#
# Fixes in headon28:
#
#   1) MUCH SLOWER POLICY UPDATES:
#      - LR: 5e-5 → 2e-5 (2.5× slower gradient steps)
#      - clip: 0.15 → 0.08 (proxy ratio constrained tighter)
#      - max-grad-norm: 0.3 → 0.2 (tighter gradient clipping)
#      These prevent the policy from jumping past the safe separation band.
#      If it takes the policy 10 updates to move from 10m→8m avoidance,
#      that's much better than jumping from 10m→0m in 2 updates.
#
#   2) WIDER NEAR-MISS ZONE to start penalty earlier:
#      - near_miss_distance: 4.0m → 6.0m (matches conflict/guidance distances)
#      - near_miss_weight: 3.0 → 2.5 (compensate for wider zone)
#      Zone transit at 0.8m/s: (6.0-0.75)/0.16 = 33 steps (vs 20 for 4.0m)
#      Penalty starts at 6.0m — 37 steps before collision, giving GAE time to
#      propagate the signal to earlier actions. Even at step 0, the agent starts
#      feeling penalty gradient from 6m encounters.
#
#   3) STRONGER COLLISION DETERRENT:
#      - collision_penalty: -80 → -150
#      Even 2% GAE propagation of -150 = -3.0, which exceeds progress reward (~0.27).
#      Combined with near-miss gradient starting at 6m, creates a layered defense.
#
#   4) WARM START FROM h27_30K (the last 100% safe checkpoint):
#      h27_30K is over-conservative (10m+ separation, 89% progress) but zero collisions.
#      This is a better starting point than h26_400K (bimodal) because:
#      - It already learned the wider near-miss landscape (4.0m→6.0m change is incremental)
#      - It has no collision mode to amplify
#      - We just need to slowly compress the avoidance radius from 10m → 3-5m
#
#   5) MORE CONSERVATIVE PPO PARAMETERS:
#      - update_epochs: 4 → 3 (fewer passes over same data → more conservative updates)
#      - entropy_coef: 0.03 → 0.025 (slightly less exploration to reduce variance)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon28.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from /mnt/data/checkpoints/usv_rl/headon27_checkpoints/headon27_step_0030528.pt \
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
  --near-miss-weight 2.5 \
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
