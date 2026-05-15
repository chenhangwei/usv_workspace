#!/bin/bash
# fresh320c: progress recovery from fresh320b's zero-collision hard-random policy.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320c_pairwise_guard_progress_from_fresh320b.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320c_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}" \
exec bash scripts/train_fresh320a.sh \
  --curriculum-seed 1460 \
  --curriculum-seed 1461 \
  --curriculum-seed 1458 \
  --curriculum-seed 1463 \
  --learning-rate 7.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.000045 \
  --random-deconflict-weight 3.40 \
  --random-deconflict-weight-end 3.80 \
  --random-deconflict-safe-separation 1.45 \
  --random-deconflict-release-separation 2.70 \
  --random-deconflict-yield-speed 0.030 \
  --random-deconflict-yield-omega 0.36 \
  --random-deconflict-omega-weight 1.20 \
  --random-deconflict-yield-weight 3.50 \
  --random-deconflict-pretrain-epochs 1 \
  --random-deconflict-pretrain-learning-rate 1.2e-6 \
  --random-role-balance-weight 2.70 \
  --random-role-balance-weight-end 3.00 \
  --random-role-balance-pretrain-epochs 1 \
  --random-role-balance-pretrain-learning-rate 1.0e-6 \
  --random-role-balance-yield-max-speed 0.045 \
  --random-pairwise-role-guard-weight 3.20 \
  --random-pairwise-role-guard-weight-end 3.60 \
  --random-pairwise-role-guard-safe-separation 0.82 \
  --random-pairwise-role-guard-release-separation 1.55 \
  --random-pairwise-role-guard-min-danger 0.04 \
  --random-pairwise-role-guard-yield-speed 0.025 \
  --random-pairwise-role-guard-standon-close-speed 0.12 \
  --random-pairwise-role-guard-yield-omega 0.34 \
  --random-pairwise-role-guard-standon-omega 0.08 \
  --random-pairwise-role-guard-linear-weight 0.75 \
  --random-pairwise-role-guard-omega-weight 1.25 \
  --random-pairwise-role-guard-yield-weight 2.10 \
  --random-pairwise-role-guard-standon-weight 0.55 \
  --random-safe-finish-weight 1.05 \
  --random-safe-finish-weight-end 1.40 \
  --random-safe-finish-yield-release-min-separation 1.20 \
  --random-safe-finish-yield-release-max-closing-speed 0.035 \
  --random-safe-finish-target-speed 0.22 \
  --random-offroute-finish-weight 2.10 \
  --random-offroute-finish-weight-end 2.60 \
  --random-cte-recovery-weight 0.34 \
  --random-cte-recovery-weight-end 0.42 \
  --policy-anchor-weight 520.0 \
  --policy-anchor-weight-end 620.0 \
  "$@"