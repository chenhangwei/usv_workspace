#!/bin/bash
# fresh383: restore forward motion after fresh382's safer separation.
# The guard1463 failure mode is now post-avoidance stalling/offroute drift, so
# this stage targets clear-ahead recovery while preserving the fresh382 shield.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh383_clear_recovery_from_fresh382.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh383_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-231}" \
exec bash scripts/train_fresh382_pairwise_recovery_balanced.sh \
  --learning-rate 1.1e-7 \
  --learning-rate-end 5.0e-8 \
  --clip-range 0.00008 \
  --ppo-policy-loss-scale 0.0022 \
  --ppo-value-loss-scale 0.034 \
  --random-deconflict-pretrain-epochs 4 \
  --random-role-balance-pretrain-epochs 3 \
  --random-pairwise-role-guard-pretrain-epochs 0 \
  --random-pairwise-role-guard-weight 12.50 \
  --random-pairwise-role-guard-weight-end 13.50 \
  --random-pairwise-role-guard-standon-close-speed 0.018 \
  --random-pairwise-role-guard-standon-weight 3.00 \
  --random-clear-ahead-weight 8.00 \
  --random-clear-ahead-weight-end 9.00 \
  --random-clear-ahead-max-distance 80.0 \
  --random-clear-ahead-min-neighbor-separation 3.60 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.26 \
  --random-clear-ahead-min-speed 0.20 \
  --random-clear-ahead-max-omega 0.42 \
  --random-clear-ahead-omega-reference 1.00 \
  --random-clear-ahead-omega-weight 3.20 \
  --random-cte-recovery-weight 7.00 \
  --random-cte-recovery-weight-end 7.80 \
  --random-cte-recovery-min-neighbor-separation 3.40 \
  --random-cte-recovery-target-speed 0.22 \
  --random-cte-recovery-min-speed 0.13 \
  --random-cte-recovery-max-omega 0.46 \
  --random-cte-recovery-omega-reference 0.90 \
  --random-cte-recovery-omega-weight 5.00 \
  --random-offroute-finish-weight 6.80 \
  --random-offroute-finish-weight-end 7.60 \
  --random-offroute-finish-min-team-separation 3.35 \
  --random-offroute-finish-min-neighbor-separation 3.35 \
  --random-offroute-finish-target-speed 0.22 \
  --random-offroute-finish-min-speed 0.13 \
  --random-offroute-finish-max-omega 0.46 \
  --random-offroute-finish-omega-reference 0.90 \
  --random-offroute-finish-omega-weight 4.80 \
  --policy-anchor-weight 220.0 \
  --policy-anchor-weight-end 260.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"