#!/bin/bash
# fresh464: goal-heading action probe for seed1461 usv_02 high-CTE recovery.
#
# fresh463 proved that stronger signed-CTE omega can move the actor, but it
# pushes seed1461 usv_02 farther off route.  This probe keeps the stronger
# random-only recovery update and switches the yaw teacher back to goal-heading
# with the sign-corrected omega convention used by the earlier fresh413 line.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh464_goal_heading_action_probe_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh464_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1800}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}" \
exec bash scripts/train_fresh462_signed_cte_random_focus.sh \
  --goal-heading-omega-sign 1.0 \
  --random-cte-recovery-omega-mode goal-heading \
  --learning-rate 1.6e-7 \
  --learning-rate-end 6.0e-8 \
  --clip-range 0.00010 \
  --update-epochs 4 \
  --random-cte-recovery-weight 3.00 \
  --random-cte-recovery-weight-end 4.20 \
  --random-cte-recovery-target-speed 0.18 \
  --random-cte-recovery-min-speed 0.09 \
  --random-cte-recovery-max-omega 0.28 \
  --random-cte-recovery-omega-reference 1.00 \
  --random-cte-recovery-omega-weight 1.10 \
  --random-recovery-pretrain-epochs 4 \
  --random-recovery-pretrain-learning-rate 3.0e-6 \
  --random-recovery-pretrain-max-grad-norm 0.55 \
  --random-recovery-pretrain-cte-scale 1.20 \
  --policy-anchor-weight 2500.0 \
  --policy-anchor-weight-end 3800.0 \
  "$@"