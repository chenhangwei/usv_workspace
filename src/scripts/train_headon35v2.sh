#!/bin/bash
# headon35v2: Accelerated policy adaptation for tight avoidance
#
# h35-460K (55K steps, near_miss_d=1.0, W=1, team=0.10, collision=-30):
#   head_on worst_sep=2.256m — NO improvement over h33 (2.21m)
#   Root cause: clip_range=0.08 + lr=2e-5 too conservative to overcome
#   the deeply ingrained "stay far away" policy from h33 training
#
# Solution: Increase lr and clip_range to speed up policy shift
#   - learning_rate: 2e-5 → 1e-4 (5x increase for faster value correction)
#   - clip-range: 0.08 → 0.20 (allow larger policy updates per step)
#   - entropy-coef: 0.025 → 0.04 (more exploration to discover tight passes)
#   - max-grad-norm: 0.2 → 0.5 (allow larger gradient steps)
#
# All penalty params same as h35 (near-miss-distance=1.0, W=1.0, team=0.10)
# Resume from h35-460K (already adapted 55K steps)
# Train to 600K (140K more steps)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon35v2.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon35_checkpoints/headon35_step_0460224.pt \
  --num-agents 3 \
  --total-timesteps 600000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 1e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.20 \
  --entropy-coef 0.04 \
  --max-grad-norm 0.5 \
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
