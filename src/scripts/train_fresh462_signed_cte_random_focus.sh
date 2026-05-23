#!/bin/bash
# fresh462: signed-CTE random encounter recovery probe from fresh438.
#
# fresh461 confirmed that random-focused rollouts activate the CTE recovery
# samples, but the learned movement was too small and usv_02 still drifted to
# the CTE=5 boundary.  This keeps the same narrow scenario focus while teaching
# omega from signed CTE so negative CTE directly turns back toward the route.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh462_signed_cte_random_focus_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh462_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1800}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}" \
exec bash scripts/train_fresh461_cte_usv02_random_focus.sh \
  --learning-rate 8.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.00006 \
  --update-epochs 3 \
  --random-cte-recovery-weight 1.80 \
  --random-cte-recovery-weight-end 2.40 \
  --random-cte-recovery-target-speed 0.16 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.28 \
  --random-cte-recovery-omega-reference 1.00 \
  --random-cte-recovery-omega-weight 1.10 \
  --random-cte-recovery-omega-mode signed-cte \
  --random-recovery-pretrain-epochs 1 \
  --random-recovery-pretrain-learning-rate 1.2e-7 \
  --random-recovery-pretrain-max-grad-norm 0.25 \
  --random-recovery-pretrain-cte-scale 1.00 \
  --policy-anchor-weight 5000.0 \
  --policy-anchor-weight-end 7000.0 \
  "$@"