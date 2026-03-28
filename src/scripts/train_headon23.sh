#!/bin/bash
# headon23: Navigation-first smooth policy with tanh-squashed actor
#
# Design principles:
#   1) Goal-reaching is PRIMARY — progress_weight=12, goal_bonus=50
#   2) Allow minor collisions — collision_penalty=-8 (was -35)
#   3) Stay near route — path_deviation_penalty_weight=1.0
#   4) Smooth sailing — action_smoothness=2.0, angular_accel=1.5
#   5) No backward/stop/spin — min_forward_speed=0.12, spin_penalty=5.0
#   6) No S-shape/oscillation — tanh squashing eliminates bang-bang
#
# Key architectural change:
#   --squash-actions applies tanh(actor_output) * scale to produce
#   bounded, smooth action means. Eliminates the unbounded-output +
#   hard-clip pattern that caused 96% angular saturation.

set -euo pipefail
cd "$(dirname "$0")/.."
source install/setup.bash

OUTPUT=/mnt/data/checkpoints/usv_rl/headon23.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --num-agents 3 \
  --total-timesteps 500000 \
  --rollout-steps 128 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.01 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --scenario-set dense \
  \
  --squash-actions \
  --min-forward-speed 0.12 \
  \
  --progress-weight 12.0 \
  --goal-bonus 50.0 \
  --collision-penalty -8.0 \
  --near-miss-weight 1.5 \
  --conflict-risk-weight 0.8 \
  --conflict-brake-weight 0.5 \
  --conflict-progress-scale 0.3 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 2.0 \
  --angular-accel-penalty-weight 1.5 \
  --pure-cruise-reward-weight 3.0 \
  --pure-idle-penalty-weight 4.0 \
  --pure-turn-penalty-weight 0.5 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 1.0 \
  --stall-penalty -25.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 6.0 \
  --stop-go-penalty-weight 1.0 \
  --goal-proximity-reward-weight 1.5 \
  \
  --checkpoint-interval 10000 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 90 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
