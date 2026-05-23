#!/bin/bash
# fresh379: safety polish from the progress-bearing fresh378 step checkpoint.
# Goal: keep the recovery/progress learned by fresh378_step1260, but force the
# close-range deconflict turn sign and speed caps back into the actor.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh378_checkpoints/fresh378_recovery_fit_from_fresh377_step_0001260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh379_safety_polish_from_fresh378_step1260.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh379_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}" \
exec bash scripts/train_fresh377_pairwise_recovery.sh \
  --learning-rate 1.2e-7 \
  --learning-rate-end 5.0e-8 \
  --clip-range 0.00008 \
  --ppo-policy-loss-scale 0.0020 \
  --ppo-value-loss-scale 0.035 \
  --random-deconflict-weight 8.50 \
  --random-deconflict-weight-end 9.50 \
  --random-deconflict-pretrain-epochs 16 \
  --random-deconflict-pretrain-learning-rate 8.0e-6 \
  --random-deconflict-pretrain-max-grad-norm 0.08 \
  --random-deconflict-critical-separation 1.35 \
  --random-deconflict-standon-close-separation 2.10 \
  --random-deconflict-standon-close-speed 0.020 \
  --random-deconflict-yield-speed 0.004 \
  --random-deconflict-standon-omega 0.26 \
  --random-deconflict-yield-omega 0.50 \
  --random-deconflict-omega-weight 5.20 \
  --random-deconflict-standon-weight 1.60 \
  --random-deconflict-yield-weight 9.00 \
  --random-role-balance-weight 6.00 \
  --random-role-balance-weight-end 6.60 \
  --random-role-balance-pretrain-epochs 10 \
  --random-role-balance-pretrain-learning-rate 6.0e-6 \
  --random-role-balance-pretrain-max-grad-norm 0.08 \
  --random-role-balance-yield-max-speed 0.010 \
  --random-role-balance-yield-min-starboard-omega 0.36 \
  --random-role-balance-standon-close-separation 2.10 \
  --random-role-balance-standon-weight 0.18 \
  --random-role-balance-yield-weight 3.80 \
  --random-role-balance-omega-weight 1.60 \
  --random-pairwise-role-guard-weight 12.00 \
  --random-pairwise-role-guard-weight-end 14.00 \
  --random-pairwise-role-guard-safe-separation 1.15 \
  --random-pairwise-role-guard-release-separation 2.35 \
  --random-pairwise-role-guard-yield-speed 0.002 \
  --random-pairwise-role-guard-standon-close-speed 0.020 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.36 \
  --random-pairwise-role-guard-linear-weight 3.20 \
  --random-pairwise-role-guard-omega-weight 4.80 \
  --random-pairwise-role-guard-yield-weight 4.20 \
  --random-pairwise-role-guard-standon-weight 2.40 \
  --random-cte-recovery-weight 4.20 \
  --random-cte-recovery-weight-end 4.80 \
  --random-cte-recovery-omega-weight 3.40 \
  --random-offroute-finish-weight 3.80 \
  --random-offroute-finish-weight-end 4.40 \
  --random-offroute-finish-omega-weight 3.00 \
  --policy-anchor-weight 180.0 \
  --policy-anchor-weight-end 260.0 \
  "$@"