#!/bin/bash
# fresh483: balance fresh481 progress with fresh482 CTE control.
#
# fresh481: route improved to ~=0.180 but usv_02 reached |CTE| ~=3.39.
# fresh482: |CTE| dropped to ~=2.91 but route fell to ~=0.156.
# This interpolation keeps the fresh481 base, moves the speed decay earlier than
# fresh481 but later than fresh482, and uses a medium CTE recovery strength.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh481_soft_cte_speed_from_fresh479.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh483_balanced_soft_cte_from_fresh481.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh483_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 5.5e-8 \
  --learning-rate-end 2.7e-8 \
  --clip-range 0.000035 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 1.10 \
  --random-cte-recovery-weight-end 1.65 \
  --random-cte-recovery-target-speed 0.105 \
  --random-cte-recovery-min-speed 0.060 \
  --random-cte-recovery-max-omega 0.37 \
  --random-cte-recovery-omega-reference 0.90 \
  --random-cte-recovery-linear-weight 0.85 \
  --random-cte-recovery-omega-weight 1.15 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.5e-6 \
  --random-recovery-pretrain-max-grad-norm 0.30 \
  --random-recovery-pretrain-cte-scale 0.80 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 1.05 \
  --random-clear-ahead-weight-end 1.60 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.72 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 3.05 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 1.00 \
  --random-clear-ahead-speed-cte-full 2.75 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.205 \
  --random-clear-ahead-min-speed 0.060 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 4.15 \
  --policy-anchor-weight 4700.0 \
  --policy-anchor-weight-end 7200.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
