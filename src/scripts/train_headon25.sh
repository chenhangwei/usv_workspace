#!/bin/bash
# headon25: Fix collision-blind policy via reward rebalancing + curriculum learning
#
# Root-cause fixes from headon24 (0% success, 100% collision):
#
#   1) REWARD REBALANCE — headon24 used collision_penalty=-8 vs progress_weight=12,
#      making "rush and crash" optimal (25 steps × ~1.2 progress/step = +30, penalty
#      only -8 → net +22). Now collision_penalty=-50, progress_weight=4, so crashing
#      costs -50 vs ~10 cumulative progress → net -40. Crashing is unprofitable.
#
#   2) STRONGER CONTINUOUS SAFETY SIGNALS — near_miss_weight 1.5→8 makes approaching
#      another vessel within 1.2m continuously expensive (up to ~3.6/step), not just
#      a one-time terminal event. conflict_risk_weight 0.8→3.0 adds early avoidance
#      awareness. unsafe_close_speed_penalty 1.0→3.0 penalizes rushing at neighbors.
#
#   3) ENTROPY SCHEDULE — entropy_coef anneals from 0.05→0.005 linearly over training.
#      headon24 used constant 0.02 and action_std collapsed to 0.134 by 800K steps.
#      Higher initial entropy + slower decay prevents premature exploration collapse.
#
#   4) CURRICULUM LEARNING — Two-phase training:
#      Phase 1 (this script): 500K steps on smoke scenarios (2-3 agents)
#        → Learn basic navigation + simple collision avoidance
#      Phase 2 (train_headon25_phase2.sh): 500K steps on dense scenarios (5 agents)
#        → Resume from Phase 1, fine-tune multi-agent coordination

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon25_phase1.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --num-agents 3 \
  --total-timesteps 500000 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.05 \
  --entropy-coef-end 0.01 \
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
  --progress-weight 4.0 \
  --goal-bonus 30.0 \
  --collision-penalty -50.0 \
  --near-miss-weight 8.0 \
  --conflict-risk-weight 3.0 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 3.0 \
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
  --checkpoint-interval 10000 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 90 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
