#!/bin/bash
# fresh320a: pairwise role guard from the stable fresh313 overtaking baseline.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320a_pairwise_guard_from_fresh313.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320a_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}" \
exec bash scripts/train_fresh318.sh \
  --checkpoint-interval 1260 \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario single_usv_overtaking \
  --curriculum-seed 1458 \
  --curriculum-seed 1460 \
  --curriculum-seed 1461 \
  --curriculum-seed 1463 \
  --curriculum-seed 3071 \
  --curriculum-seed 3072 \
  --learning-rate 9.0e-8 \
  --learning-rate-end 3.5e-8 \
  --clip-range 0.000055 \
  --random-deconflict-weight 4.80 \
  --random-deconflict-weight-end 5.20 \
  --random-deconflict-critical-separation 0.92 \
  --random-deconflict-standon-close-separation 1.05 \
  --random-deconflict-standon-close-speed 0.10 \
  --random-deconflict-yield-speed 0.020 \
  --random-deconflict-yield-omega 0.44 \
  --random-deconflict-omega-weight 1.80 \
  --random-deconflict-standon-weight 0.08 \
  --random-deconflict-yield-weight 4.80 \
  --random-deconflict-pretrain-epochs 2 \
  --random-deconflict-pretrain-learning-rate 1.8e-6 \
  --random-deconflict-pretrain-max-grad-norm 0.08 \
  --random-role-balance-weight 3.80 \
  --random-role-balance-weight-end 4.30 \
  --random-role-balance-pretrain-epochs 1 \
  --random-role-balance-pretrain-learning-rate 1.4e-6 \
  --random-role-balance-pretrain-max-grad-norm 0.06 \
  --random-role-balance-min-danger 0.08 \
  --random-role-balance-standon-close-separation 1.05 \
  --random-role-balance-yield-max-speed 0.035 \
  --random-role-balance-yield-min-starboard-omega 0.24 \
  --random-role-balance-yield-weight 2.40 \
  --random-role-balance-omega-weight 0.80 \
  --random-pairwise-role-guard-weight 5.00 \
  --random-pairwise-role-guard-weight-end 6.00 \
  --random-pairwise-role-guard-safe-separation 0.80 \
  --random-pairwise-role-guard-release-separation 1.35 \
  --random-pairwise-role-guard-min-danger 0.08 \
  --random-pairwise-role-guard-yield-speed 0.015 \
  --random-pairwise-role-guard-standon-close-speed 0.10 \
  --random-pairwise-role-guard-yield-omega 0.42 \
  --random-pairwise-role-guard-standon-omega 0.12 \
  --random-pairwise-role-guard-linear-weight 1.00 \
  --random-pairwise-role-guard-omega-weight 2.20 \
  --random-pairwise-role-guard-yield-weight 2.60 \
  --random-pairwise-role-guard-standon-weight 0.80 \
  --random-safe-finish-weight 0.45 \
  --random-safe-finish-weight-end 0.65 \
  --random-offroute-finish-weight 1.25 \
  --random-offroute-finish-weight-end 1.75 \
  --policy-anchor-weight 780.0 \
  --policy-anchor-weight-end 900.0 \
  --policy-anchor-exclude-random-deconflict \
  --policy-anchor-exclude-random-pairwise-role-guard \
  --policy-anchor-exclude-random-cte-recovery \
  "$@"