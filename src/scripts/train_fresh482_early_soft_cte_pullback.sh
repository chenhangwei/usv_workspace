#!/bin/bash
# fresh482: earlier soft-speed decay plus stronger CTE pullback.
#
# fresh481 improved the fresh480 tradeoff slightly, but usv_02 still drifted to
# |CTE| ~= 3.4 while making progress.  This continuation starts from fresh481,
# begins clear-ahead speed decay earlier, reaches low speed by |CTE| ~= 2.45,
# and gives the CTE recovery teacher a little more yaw authority.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh481_soft_cte_speed_from_fresh479.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh482_early_soft_cte_pullback_from_fresh481.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh482_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 6.2e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.00004 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 1.25 \
  --random-cte-recovery-weight-end 1.85 \
  --random-cte-recovery-target-speed 0.095 \
  --random-cte-recovery-min-speed 0.060 \
  --random-cte-recovery-max-omega 0.38 \
  --random-cte-recovery-omega-reference 0.86 \
  --random-cte-recovery-linear-weight 0.75 \
  --random-cte-recovery-omega-weight 1.30 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.7e-6 \
  --random-recovery-pretrain-max-grad-norm 0.30 \
  --random-recovery-pretrain-cte-scale 0.95 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.95 \
  --random-clear-ahead-weight-end 1.45 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.70 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.85 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.90 \
  --random-clear-ahead-speed-cte-full 2.45 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.20 \
  --random-clear-ahead-min-speed 0.055 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 4.00 \
  --policy-anchor-weight 4500.0 \
  --policy-anchor-weight-end 7000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
