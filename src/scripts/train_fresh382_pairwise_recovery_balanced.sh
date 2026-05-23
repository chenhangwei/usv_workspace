#!/bin/bash
# fresh382: balanced recovery from fresh380's safe-but-stalled policy.
# Keep the pairwise guard early enough to catch guard1463, but avoid fresh380's
# all-sample pairwise coverage and fresh381's too-late safety activation.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh380_pairwise_pretrain_wide_safety_from_fresh379.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh382_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}" \
exec bash scripts/train_fresh380_pairwise_pretrain_wide_safety.sh \
  --learning-rate 1.0e-7 \
  --learning-rate-end 4.5e-8 \
  --clip-range 0.00007 \
  --ppo-policy-loss-scale 0.0018 \
  --ppo-value-loss-scale 0.032 \
  --random-deconflict-weight 9.00 \
  --random-deconflict-weight-end 9.80 \
  --random-deconflict-pretrain-epochs 10 \
  --random-deconflict-pretrain-learning-rate 5.5e-6 \
  --random-deconflict-pretrain-max-grad-norm 0.07 \
  --random-deconflict-standon-close-separation 2.35 \
  --random-deconflict-standon-close-speed 0.016 \
  --random-deconflict-yield-speed 0.002 \
  --random-deconflict-omega-weight 5.40 \
  --random-deconflict-standon-weight 1.80 \
  --random-deconflict-yield-weight 9.60 \
  --random-role-balance-pretrain-epochs 6 \
  --random-role-balance-pretrain-learning-rate 4.5e-6 \
  --random-role-balance-yield-max-speed 0.008 \
  --random-role-balance-yield-min-starboard-omega 0.35 \
  --random-role-balance-standon-close-separation 2.35 \
  --random-role-balance-yield-weight 3.50 \
  --random-role-balance-standon-weight 0.16 \
  --random-pairwise-role-guard-weight 13.50 \
  --random-pairwise-role-guard-weight-end 15.00 \
  --random-pairwise-role-guard-pretrain-epochs 22 \
  --random-pairwise-role-guard-pretrain-learning-rate 6.5e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.07 \
  --random-pairwise-role-guard-safe-separation 1.20 \
  --random-pairwise-role-guard-release-separation 3.10 \
  --random-pairwise-role-guard-min-danger 0.03 \
  --random-pairwise-role-guard-yield-speed 0.001 \
  --random-pairwise-role-guard-standon-close-speed 0.014 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.44 \
  --random-pairwise-role-guard-linear-weight 3.80 \
  --random-pairwise-role-guard-omega-weight 6.00 \
  --random-pairwise-role-guard-yield-weight 5.40 \
  --random-pairwise-role-guard-standon-weight 3.40 \
  --random-cte-recovery-weight 5.00 \
  --random-cte-recovery-weight-end 5.80 \
  --random-cte-recovery-min-neighbor-separation 3.20 \
  --random-cte-recovery-target-speed 0.18 \
  --random-cte-recovery-min-speed 0.06 \
  --random-cte-recovery-max-omega 0.45 \
  --random-cte-recovery-omega-weight 4.20 \
  --random-offroute-finish-weight 4.80 \
  --random-offroute-finish-weight-end 5.60 \
  --random-offroute-finish-min-team-separation 3.25 \
  --random-offroute-finish-min-neighbor-separation 3.25 \
  --random-offroute-finish-target-speed 0.18 \
  --random-offroute-finish-min-speed 0.06 \
  --random-offroute-finish-max-omega 0.45 \
  --random-offroute-finish-omega-weight 4.00 \
  --policy-anchor-weight 170.0 \
  --policy-anchor-weight-end 220.0 \
  "$@"