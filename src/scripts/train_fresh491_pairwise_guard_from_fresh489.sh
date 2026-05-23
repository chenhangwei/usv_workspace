#!/bin/bash
# fresh491: local pairwise role guard from fresh489.
#
# fresh490's all-agent low-threshold CTE recovery repaired the original usv_02
# runaway, but seed1461 still collided because usv_02/usv_03 kept forward speed
# at sub-meter separation.  This branch returns to fresh489 and adds only a
# local nearest-pair speed/yaw guard, leaving the CTE recovery scoped to usv_02.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh489_raw_cte_signed_return_from_fresh486.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh491_pairwise_guard_from_fresh489.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh491_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh489_raw_cte_signed_return_from_fresh486.sh \
  --random-pairwise-role-guard-weight 1.10 \
  --random-pairwise-role-guard-weight-end 1.75 \
  --random-pairwise-role-guard-pretrain-epochs 2 \
  --random-pairwise-role-guard-pretrain-learning-rate 6.0e-7 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.20 \
  --random-pairwise-role-guard-safe-separation 1.05 \
  --random-pairwise-role-guard-release-separation 2.30 \
  --random-pairwise-role-guard-min-danger 0.08 \
  --random-pairwise-role-guard-yield-speed 0.015 \
  --random-pairwise-role-guard-standon-close-speed 0.085 \
  --random-pairwise-role-guard-yield-omega 0.38 \
  --random-pairwise-role-guard-standon-omega 0.10 \
  --random-pairwise-role-guard-linear-weight 1.35 \
  --random-pairwise-role-guard-omega-weight 1.20 \
  --random-pairwise-role-guard-yield-weight 2.40 \
  --random-pairwise-role-guard-standon-weight 0.85 \
  --policy-anchor-exclude-random-pairwise-role-guard \
  "$@"