#!/bin/bash
# fresh388: long-rollout clear-ahead-only actor pretrain from fresh382.
# This keeps late recovery exposure from fresh387, but pretrains only the
# safe-and-clear forward target instead of CTE/offroute corrections.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh388_long_clear_only_recovery.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh388_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-239}" \
exec bash scripts/train_fresh387_long_rollout_late_recovery.sh \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.5e-6 \
  --random-recovery-pretrain-max-grad-norm 0.020 \
  --random-recovery-pretrain-offroute-scale 0.0 \
  --random-recovery-pretrain-cte-scale 0.0 \
  --random-recovery-pretrain-clear-scale 1.0 \
  --random-clear-ahead-weight 6.00 \
  --random-clear-ahead-weight-end 7.00 \
  --random-clear-ahead-scenario two_usv_random_encounter \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-distance 0.0 \
  --random-clear-ahead-max-distance 80.0 \
  --random-clear-ahead-min-neighbor-separation 4.80 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.24 \
  --random-clear-ahead-min-speed 0.16 \
  --random-clear-ahead-max-omega 0.24 \
  --random-clear-ahead-omega-reference 0.90 \
  --random-clear-ahead-omega-weight 1.60 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"