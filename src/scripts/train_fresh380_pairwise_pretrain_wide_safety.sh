#!/bin/bash
# fresh380: wide-distance safety plus actor-only pairwise guard pretrain.
# Starts from fresh379 so the progress basin is retained, then directly fits
# the last-meter pairwise away-turn/speed-cap behavior before each PPO update.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh379_safety_polish_from_fresh378_step1260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh380_pairwise_pretrain_wide_safety_from_fresh379.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh380_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}" \
exec bash scripts/train_fresh379_safety_polish.sh \
  --learning-rate 1.0e-7 \
  --learning-rate-end 4.0e-8 \
  --clip-range 0.00006 \
  --ppo-policy-loss-scale 0.0015 \
  --ppo-value-loss-scale 0.030 \
  --random-deconflict-max-distance 80.0 \
  --random-deconflict-lookahead-distance 14.0 \
  --random-deconflict-weight 9.20 \
  --random-deconflict-weight-end 10.20 \
  --random-deconflict-pretrain-epochs 10 \
  --random-deconflict-pretrain-learning-rate 6.0e-6 \
  --random-deconflict-pretrain-max-grad-norm 0.07 \
  --random-deconflict-standon-close-separation 2.30 \
  --random-deconflict-standon-close-speed 0.018 \
  --random-deconflict-yield-speed 0.002 \
  --random-deconflict-omega-weight 5.60 \
  --random-deconflict-standon-weight 1.90 \
  --random-deconflict-yield-weight 9.80 \
  --random-role-balance-pretrain-epochs 6 \
  --random-role-balance-pretrain-learning-rate 4.5e-6 \
  --random-role-balance-yield-max-speed 0.008 \
  --random-role-balance-yield-min-starboard-omega 0.34 \
  --random-role-balance-standon-close-separation 2.30 \
  --random-role-balance-yield-weight 3.40 \
  --random-role-balance-standon-weight 0.14 \
  --random-pairwise-role-guard-weight 14.00 \
  --random-pairwise-role-guard-weight-end 16.00 \
  --random-pairwise-role-guard-pretrain-epochs 18 \
  --random-pairwise-role-guard-pretrain-learning-rate 7.0e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.07 \
  --random-pairwise-role-guard-safe-separation 1.20 \
  --random-pairwise-role-guard-release-separation 2.55 \
  --random-pairwise-role-guard-min-danger 0.00 \
  --random-pairwise-role-guard-yield-speed 0.001 \
  --random-pairwise-role-guard-standon-close-speed 0.018 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.42 \
  --random-pairwise-role-guard-linear-weight 3.80 \
  --random-pairwise-role-guard-omega-weight 6.20 \
  --random-pairwise-role-guard-yield-weight 5.40 \
  --random-pairwise-role-guard-standon-weight 3.20 \
  --random-cte-recovery-weight 3.40 \
  --random-cte-recovery-weight-end 3.80 \
  --random-cte-recovery-omega-weight 2.70 \
  --random-offroute-finish-weight 3.20 \
  --random-offroute-finish-weight-end 3.60 \
  --random-offroute-finish-omega-weight 2.50 \
  --policy-anchor-weight 150.0 \
  --policy-anchor-weight-end 220.0 \
  "$@"