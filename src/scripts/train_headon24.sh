#!/bin/bash
# headon24: Professional-grade RL training with sim-to-real robustness
#
# Improvements over headon23:
#   1) Observation normalization — RunningMeanStd z-scoring fixes 3-order
#      magnitude feature range mismatch
#   2) Domain randomization — GPS noise, compass drift, actuator jitter,
#      Ornstein-Uhlenbeck water current (sim-to-real transfer)
#   3) Scenario randomization — spawn/goal position & heading jitter
#      prevents overfitting to fixed geometry
#   4) Goal tolerance 0.8→0.5m — separates success from collision zone
#   5) Head-on guidance now geometric (all scenarios, not name-dependent)
#   6) Entropy 0.01→0.02, rollout 128→192 for better exploration
#   7) Extended to 800K steps (DR/normalization need longer convergence)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon24.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --num-agents 5 \
  --total-timesteps 800000 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.02 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --scenario-set dense \
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
