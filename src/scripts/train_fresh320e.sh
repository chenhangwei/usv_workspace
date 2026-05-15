#!/bin/bash
# fresh320e: safe-finish release from fresh320b with pairwise guard disabled.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320e_release_no_pairguard_from_fresh320b.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320e_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-232}" \
exec bash scripts/train_fresh320a.sh \
  --curriculum-seed 1461 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --curriculum-seed 1463 \
  --learning-rate 3.8e-8 \
  --learning-rate-end 1.8e-8 \
  --clip-range 0.000028 \
  --random-deconflict-weight 4.90 \
  --random-deconflict-weight-end 5.25 \
  --random-deconflict-yield-speed 0.020 \
  --random-deconflict-yield-omega 0.42 \
  --random-deconflict-omega-weight 1.70 \
  --random-deconflict-yield-weight 4.90 \
  --random-deconflict-pretrain-epochs 1 \
  --random-deconflict-pretrain-learning-rate 7.0e-7 \
  --random-deconflict-pretrain-max-grad-norm 0.05 \
  --random-role-balance-weight 3.50 \
  --random-role-balance-weight-end 3.90 \
  --random-role-balance-pretrain-epochs 1 \
  --random-role-balance-pretrain-learning-rate 7.0e-7 \
  --random-role-balance-pretrain-max-grad-norm 0.05 \
  --random-role-balance-yield-max-speed 0.036 \
  --random-role-balance-yield-weight 2.60 \
  --random-role-balance-omega-weight 0.78 \
  --random-pairwise-role-guard-weight 0.0 \
  --random-pairwise-role-guard-weight-end 0.0 \
  --random-safe-finish-weight 0.78 \
  --random-safe-finish-weight-end 0.98 \
  --random-safe-finish-min-team-separation 2.80 \
  --random-safe-finish-full-team-separation 3.45 \
  --random-safe-finish-min-neighbor-separation 2.95 \
  --random-safe-finish-yield-release-min-separation 1.55 \
  --random-safe-finish-yield-release-max-closing-speed -0.010 \
  --random-safe-finish-yield-release-min-route-progress 0.68 \
  --random-safe-finish-target-speed 0.19 \
  --random-safe-finish-min-speed-scale 0.42 \
  --random-safe-finish-low-priority-speed-multiplier 0.95 \
  --random-safe-finish-max-omega 0.07 \
  --random-safe-finish-omega-weight 0.06 \
  --random-offroute-finish-weight 1.35 \
  --random-offroute-finish-weight-end 1.65 \
  --random-offroute-finish-min-team-separation 2.85 \
  --random-offroute-finish-min-neighbor-separation 3.00 \
  --random-offroute-finish-target-speed 0.12 \
  --random-offroute-finish-min-speed 0.050 \
  --random-offroute-finish-max-omega 0.17 \
  --random-offroute-finish-omega-weight 0.58 \
  --random-cte-recovery-weight 0.22 \
  --random-cte-recovery-weight-end 0.28 \
  --random-cte-recovery-min-neighbor-separation 1.75 \
  --random-cte-recovery-target-speed 0.10 \
  --random-cte-recovery-min-speed 0.035 \
  --random-cte-recovery-max-omega 0.17 \
  --random-cte-recovery-omega-weight 0.20 \
  --policy-anchor-weight 1350.0 \
  --policy-anchor-weight-end 1550.0 \
  "$@"