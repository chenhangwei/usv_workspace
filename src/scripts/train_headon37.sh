#!/bin/bash
# headon37: Fix crossing collision from h35v2 while preserving tight passes
#
# h35v2-510K results (lr=1e-4, clip=0.20, near_miss_d=1.0, W=1, col=-30):
#   head_on:    0% coll, worst_sep=1.047m ✓ (target: approach 0.75m)
#   overtaking: 0% coll, worst_sep=0.818m ✓ (excellent!)
#   crossing:   100% coll, worst_sep=0.671m ✗ (crashes!)
#
# Strategy: Resume from h35v2-510K with ONLY crossing-fix adjustments:
#   - collision_penalty: -30 → -60 (2x stronger, but still much weaker than h33's -150)
#   - lr: 1e-4 → 3e-5 (stabilize — big updates caused crossing instability)
#   - clip-range: 0.20 → 0.12 (smaller policy steps for stability)
#   - entropy: 0.04 → 0.03 (slightly less random exploration)
#   - Keep near_miss_distance=1.0, W=1.0, team_reward=0.10 (these worked well!)
#
# The idea: h35v2 already learned tight avoidance; now we just need to
# make collisions more punishing so it learns to not cross the 0.75m line,
# while the slow lr preserves the tight-pass behavior in head_on/overtaking.
#
# Resume from h35v2-510K, train to 650K (140K new steps)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon37.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon35v2_checkpoints/headon35v2_step_0510336.pt \
  --num-agents 3 \
  --total-timesteps 650000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 3e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.12 \
  --entropy-coef 0.03 \
  --max-grad-norm 0.3 \
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
  --collision-penalty -60.0 \
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
