#!/bin/bash
# fresh391: light pairwise margin polish from fresh380.
# fresh380 keeps positive progress but has only ~0.8m min separation. This probe
# nudges pairwise safety without the broad recovery/clear changes that caused
# fresh382+ to stall or run away.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh380_pairwise_pretrain_wide_safety_from_fresh379.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh391_light_pairwise_margin_from_fresh380.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh391_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-251}" \
exec bash scripts/train_fresh380_pairwise_pretrain_wide_safety.sh \
  --learning-rate 9.0e-8 \
  --learning-rate-end 4.0e-8 \
  --clip-range 0.00006 \
  --ppo-policy-loss-scale 0.0015 \
  --ppo-value-loss-scale 0.030 \
  --random-deconflict-weight 8.80 \
  --random-deconflict-weight-end 9.40 \
  --random-deconflict-pretrain-epochs 6 \
  --random-role-balance-pretrain-epochs 4 \
  --random-pairwise-role-guard-weight 12.80 \
  --random-pairwise-role-guard-weight-end 13.80 \
  --random-pairwise-role-guard-pretrain-epochs 10 \
  --random-pairwise-role-guard-pretrain-learning-rate 4.5e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.045 \
  --random-pairwise-role-guard-safe-separation 1.12 \
  --random-pairwise-role-guard-release-separation 2.75 \
  --random-pairwise-role-guard-min-danger 0.015 \
  --random-pairwise-role-guard-yield-speed 0.002 \
  --random-pairwise-role-guard-standon-close-speed 0.018 \
  --random-pairwise-role-guard-yield-omega 0.48 \
  --random-pairwise-role-guard-standon-omega 0.38 \
  --random-pairwise-role-guard-linear-weight 3.30 \
  --random-pairwise-role-guard-omega-weight 5.20 \
  --random-pairwise-role-guard-yield-weight 4.60 \
  --random-pairwise-role-guard-standon-weight 2.50 \
  --random-cte-recovery-weight 3.20 \
  --random-cte-recovery-weight-end 3.60 \
  --random-offroute-finish-weight 3.00 \
  --random-offroute-finish-weight-end 3.40 \
  --random-clear-ahead-weight 0.0 \
  --random-clear-ahead-weight-end 0.0 \
  --policy-anchor-weight 190.0 \
  --policy-anchor-weight-end 250.0 \
  "$@"