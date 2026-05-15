#!/bin/bash
# fresh320d: conservative release from fresh320b's zero-collision random-hard policy.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320d_conservative_release_from_fresh320b.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320d_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-231}" \
exec bash scripts/train_fresh320a.sh \
  --curriculum-seed 1461 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --curriculum-seed 1463 \
  --learning-rate 4.5e-8 \
  --learning-rate-end 2.2e-8 \
  --clip-range 0.000032 \
  --random-deconflict-weight 4.70 \
  --random-deconflict-weight-end 5.05 \
  --random-deconflict-yield-speed 0.022 \
  --random-deconflict-yield-omega 0.40 \
  --random-deconflict-omega-weight 1.55 \
  --random-deconflict-yield-weight 4.60 \
  --random-deconflict-pretrain-epochs 1 \
  --random-deconflict-pretrain-learning-rate 9.0e-7 \
  --random-deconflict-pretrain-max-grad-norm 0.06 \
  --random-role-balance-weight 3.30 \
  --random-role-balance-weight-end 3.80 \
  --random-role-balance-pretrain-epochs 1 \
  --random-role-balance-pretrain-learning-rate 8.0e-7 \
  --random-role-balance-pretrain-max-grad-norm 0.05 \
  --random-role-balance-yield-max-speed 0.038 \
  --random-role-balance-yield-weight 2.50 \
  --random-role-balance-omega-weight 0.70 \
  --random-pairwise-role-guard-weight 1.20 \
  --random-pairwise-role-guard-weight-end 1.55 \
  --random-pairwise-role-guard-safe-separation 0.82 \
  --random-pairwise-role-guard-release-separation 1.40 \
  --random-pairwise-role-guard-min-danger 0.06 \
  --random-pairwise-role-guard-yield-speed 0.020 \
  --random-pairwise-role-guard-standon-close-speed 0.11 \
  --random-pairwise-role-guard-yield-omega 0.26 \
  --random-pairwise-role-guard-standon-omega 0.06 \
  --random-pairwise-role-guard-linear-weight 0.35 \
  --random-pairwise-role-guard-omega-weight 0.65 \
  --random-pairwise-role-guard-yield-weight 1.45 \
  --random-pairwise-role-guard-standon-weight 0.25 \
  --random-safe-finish-weight 0.72 \
  --random-safe-finish-weight-end 0.92 \
  --random-safe-finish-min-team-separation 2.75 \
  --random-safe-finish-full-team-separation 3.35 \
  --random-safe-finish-min-neighbor-separation 2.90 \
  --random-safe-finish-yield-release-min-separation 1.38 \
  --random-safe-finish-yield-release-max-closing-speed 0.000 \
  --random-safe-finish-yield-release-min-route-progress 0.58 \
  --random-safe-finish-target-speed 0.20 \
  --random-safe-finish-min-speed-scale 0.45 \
  --random-safe-finish-low-priority-speed-multiplier 1.00 \
  --random-safe-finish-max-omega 0.08 \
  --random-safe-finish-omega-weight 0.07 \
  --random-offroute-finish-weight 1.55 \
  --random-offroute-finish-weight-end 1.90 \
  --random-offroute-finish-min-team-separation 2.80 \
  --random-offroute-finish-min-neighbor-separation 2.95 \
  --random-offroute-finish-target-speed 0.13 \
  --random-offroute-finish-min-speed 0.055 \
  --random-offroute-finish-max-omega 0.18 \
  --random-offroute-finish-omega-weight 0.62 \
  --random-cte-recovery-weight 0.26 \
  --random-cte-recovery-weight-end 0.32 \
  --random-cte-recovery-min-neighbor-separation 1.55 \
  --random-cte-recovery-target-speed 0.11 \
  --random-cte-recovery-min-speed 0.040 \
  --random-cte-recovery-max-omega 0.18 \
  --random-cte-recovery-omega-weight 0.22 \
  --policy-anchor-weight 1180.0 \
  --policy-anchor-weight-end 1350.0 \
  "$@"