#!/bin/bash
# fresh381: recover navigation after fresh380's pairwise safety fix.
# fresh380 proved the last-meter collision can be removed, but its pairwise
# guard was active on every random sample and suppressed recovery losses.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh380_pairwise_pretrain_wide_safety_from_fresh379.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh381_recovery_after_pairwise_from_fresh380.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh381_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-229}" \
exec bash scripts/train_fresh380_pairwise_pretrain_wide_safety.sh \
  --learning-rate 1.1e-7 \
  --learning-rate-end 4.5e-8 \
  --clip-range 0.00008 \
  --ppo-policy-loss-scale 0.0022 \
  --ppo-value-loss-scale 0.035 \
  --random-deconflict-weight 8.40 \
  --random-deconflict-weight-end 9.20 \
  --random-deconflict-pretrain-epochs 6 \
  --random-deconflict-pretrain-learning-rate 4.5e-6 \
  --random-deconflict-standon-close-separation 2.20 \
  --random-deconflict-standon-close-speed 0.025 \
  --random-deconflict-yield-speed 0.003 \
  --random-deconflict-omega-weight 4.80 \
  --random-deconflict-standon-weight 1.40 \
  --random-deconflict-yield-weight 8.40 \
  --random-role-balance-pretrain-epochs 4 \
  --random-role-balance-yield-max-speed 0.012 \
  --random-role-balance-yield-min-starboard-omega 0.32 \
  --random-pairwise-role-guard-weight 10.50 \
  --random-pairwise-role-guard-weight-end 12.00 \
  --random-pairwise-role-guard-pretrain-epochs 8 \
  --random-pairwise-role-guard-pretrain-learning-rate 4.8e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.07 \
  --random-pairwise-role-guard-safe-separation 1.18 \
  --random-pairwise-role-guard-release-separation 2.45 \
  --random-pairwise-role-guard-min-danger 0.07 \
  --random-pairwise-role-guard-yield-speed 0.002 \
  --random-pairwise-role-guard-standon-close-speed 0.024 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.38 \
  --random-pairwise-role-guard-linear-weight 3.10 \
  --random-pairwise-role-guard-omega-weight 4.80 \
  --random-pairwise-role-guard-yield-weight 4.00 \
  --random-pairwise-role-guard-standon-weight 2.20 \
  --random-cte-recovery-weight 7.20 \
  --random-cte-recovery-weight-end 8.40 \
  --random-cte-recovery-min-neighbor-separation 2.55 \
  --random-cte-recovery-target-speed 0.20 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.50 \
  --random-cte-recovery-omega-weight 5.60 \
  --random-offroute-finish-weight 6.40 \
  --random-offroute-finish-weight-end 7.40 \
  --random-offroute-finish-min-team-separation 2.85 \
  --random-offroute-finish-min-neighbor-separation 2.85 \
  --random-offroute-finish-target-speed 0.19 \
  --random-offroute-finish-min-speed 0.08 \
  --random-offroute-finish-max-omega 0.50 \
  --random-offroute-finish-omega-weight 5.00 \
  --policy-anchor-weight 120.0 \
  --policy-anchor-weight-end 180.0 \
  "$@"