#!/bin/bash
# fresh470: balance fresh468 progress against fresh469 CTE pullback.
#
# fresh468 moved usv_02 forward but drifted to |CTE| ~= 3.3.  fresh469 pulled
# CTE back to ~= 1.4 but lost progress.  This uses a middle CTE pullback while
# keeping the linear-only resume target.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh468_linear_resume_only_from_fresh464.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh470_linear_resume_balanced_cte_from_fresh468.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh470_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-218}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 7.0e-8 \
  --learning-rate-end 3.5e-8 \
  --clip-range 0.00006 \
  --update-epochs 3 \
  --random-cte-recovery-weight 1.30 \
  --random-cte-recovery-weight-end 1.90 \
  --random-cte-recovery-target-speed 0.16 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.24 \
  --random-cte-recovery-omega-reference 1.10 \
  --random-cte-recovery-omega-weight 0.90 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.4e-6 \
  --random-recovery-pretrain-max-grad-norm 0.28 \
  --random-recovery-pretrain-cte-scale 0.60 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.35 \
  --random-clear-ahead-weight-end 0.55 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.32 \
  --random-clear-ahead-max-abs-cte 2.05 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.16 \
  --random-clear-ahead-min-speed 0.10 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 2.00 \
  --policy-anchor-weight 6500.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"