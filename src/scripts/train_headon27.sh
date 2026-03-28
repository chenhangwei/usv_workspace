#!/bin/bash
# headon27: Stabilise avoidance via wider near-miss zone + warm-start from h26 best
#
# headon26 postmortem:
#   - V-shaped learning curve: safe → collapse (200K) → recover (400K) → collapse (500K)
#   - Best checkpoint: 400K — 0% collision, 88% progress (3-ep) / 6.7% col (15-ep)
#   - Policy is bimodal: 53% wide avoidance (12m+), 47% close approach (1.3m)
#   - Near-miss zone (0.75-1.2m) too thin — no gradient in 1.2-5m range
#   - Policy oscillates between over-conservative and over-aggressive
#   - No stable moderate-separation attractor exists in reward landscape
#
# Root causes identified:
#
#   1) THIN NEAR-MISS ZONE (0.45m band): Only penalises at <1.2m separation.
#      Agents have zero safety gradient at 1.3-5.0m. By the time they reach 1.2m
#      they are committed to their heading and moving too fast to avoid.
#
#   2) EPISODE TERMINATION ON COLLISION: Collision penalty is one-time terminal
#      event. With GAE (γ=0.99, λ=0.95), signal from collision at step 55 reaches
#      step 0 with only ~3% weight. Early actions dominated by progress incentive.
#
#   3) NO MODERATE-SEPARATION ATTRACTOR: Reward provides no positive feedback for
#      maintaining 2-5m separation. Only penalties for <1.2m + weak conflict_risk.
#      Policy either over-avoids (12m) or under-avoids (1.3m), nothing in between.
#
# Fixes applied in headon27:
#
#   1) WIDER NEAR-MISS ZONE — near_miss_distance: 1.2m → 4.0m
#      Creates dense per-step penalty gradient from 0.75m to 4.0m.
#      At 3.0m: penalty = 3.0 * (4.0-3.0) = 3.0/step
#      At 2.0m: penalty = 3.0 * (4.0-2.0) = 6.0/step
#      At 1.0m: penalty = 3.0 * (4.0-1.0) = 9.0/step
#      This provides gradient EARLY enough to influence steering decisions.
#
#   2) REDUCED NEAR-MISS WEIGHT — 8.0 → 3.0
#      Compensates for the wider band. Old max penalty (at 0.75m) was
#      8.0*(1.2-0.75)=3.6/step. New max: 3.0*(4.0-0.75)=9.75/step.
#      Stronger deterrent at close range, moderate gradient at 2-4m.
#
#   3) WARM START FROM headon26_400K — Load the best known safe policy.
#      Avoids relearning basic navigation + avoidance from scratch.
#      Policy already knows to turn starboard and make progress.
#
#   4) CONSTANT LOW LR — 5e-5 (no decay)
#      The h26 second collapse happened at LR 7e-5→5e-5. Starting at 5e-5
#      and NOT decaying prevents the tightening oscilation that caused collapse.
#
#   5) CONSTANT MODERATE ENTROPY — 0.03 (no decay)
#      Higher floor than h26's 0.02 endpoint. Maintains exploration to
#      prevent the bimodal split from locking into the collision mode.
#
#   6) LONGER NO-PROGRESS TIMEOUT — 10s → 20s
#      The 400K policy makes wide detours where it temporarily moves away
#      from the goal. Current 10s timeout kills these episodes prematurely.
#      20s gives the conservative avoidance path time to complete.
#
#   7) INCREASED ANTICIPATION & GUIDANCE DISTANCES — 5.0m → 6.0m
#      Now matches near_miss_distance=4.0m better. Provides guidance and
#      conflict risk signals earlier, at 6m rather than 5m.

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon27.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from /mnt/data/checkpoints/usv_rl/headon26_checkpoints/headon26_step_0400320.pt \
  --num-agents 3 \
  --total-timesteps 300000 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 5e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.03 \
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
  --near-miss-distance 4.0 \
  --near-miss-weight 3.0 \
  --collision-distance 0.75 \
  --collision-penalty -80.0 \
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
  --checkpoint-interval 10000 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 180 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
