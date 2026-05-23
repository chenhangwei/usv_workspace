#!/bin/bash
# fresh481: soft CTE speed decay inside the clear-ahead push band.
#
# fresh480 proved that a wider clear-ahead CTE band improves route progress but
# can push usv_02 back toward high CTE.  This branch restarts from the controlled
# fresh479 checkpoint, keeps the wider corridor, and decays clear-ahead target
# speed as |CTE| rises so forward motion fades before the boat sprints off-line.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh479_line_locked_push_from_fresh478.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh481_soft_cte_speed_from_fresh479.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh481_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 6.8e-8 \
  --learning-rate-end 3.2e-8 \
  --clip-range 0.000045 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 0.95 \
  --random-cte-recovery-weight-end 1.45 \
  --random-cte-recovery-target-speed 0.10 \
  --random-cte-recovery-min-speed 0.065 \
  --random-cte-recovery-max-omega 0.36 \
  --random-cte-recovery-omega-reference 0.92 \
  --random-cte-recovery-linear-weight 0.85 \
  --random-cte-recovery-omega-weight 1.18 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.8e-6 \
  --random-recovery-pretrain-max-grad-norm 0.32 \
  --random-recovery-pretrain-cte-scale 0.65 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 1.10 \
  --random-clear-ahead-weight-end 1.70 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.70 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 3.20 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 1.15 \
  --random-clear-ahead-speed-cte-full 3.05 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.20 \
  --random-clear-ahead-min-speed 0.070 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 4.60 \
  --policy-anchor-weight 4200.0 \
  --policy-anchor-weight-end 6500.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
