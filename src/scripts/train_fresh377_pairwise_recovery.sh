#!/bin/bash
# fresh377: keep fresh376 far-field recovery, restore close-range safety.
# Starts from fresh376 and adds a pairwise brake/away-turn guard for sub-2m
# random-encounter interactions where stand-on speed stayed too high.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh376_far_recovery_from_fresh367.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh377_pairwise_recovery_from_fresh376.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh377_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}" \
exec bash scripts/train_fresh376_far_recovery.sh \
  --random-deconflict-critical-separation 1.25 \
  --random-deconflict-standon-close-separation 1.80 \
  --random-deconflict-standon-close-speed 0.045 \
  --random-deconflict-yield-speed 0.008 \
  --random-deconflict-standon-omega 0.14 \
  --random-deconflict-yield-omega 0.50 \
  --random-deconflict-omega-weight 2.60 \
  --random-deconflict-standon-weight 0.75 \
  --random-deconflict-yield-weight 6.00 \
  --random-role-balance-yield-max-speed 0.020 \
  --random-role-balance-yield-min-starboard-omega 0.32 \
  --random-role-balance-standon-close-separation 1.80 \
  --random-role-balance-yield-weight 3.20 \
  --random-role-balance-omega-weight 1.20 \
  --random-pairwise-role-guard-weight 8.00 \
  --random-pairwise-role-guard-weight-end 9.50 \
  --random-pairwise-role-guard-safe-separation 1.05 \
  --random-pairwise-role-guard-release-separation 2.10 \
  --random-pairwise-role-guard-min-danger 0.03 \
  --random-pairwise-role-guard-yield-speed 0.004 \
  --random-pairwise-role-guard-standon-close-speed 0.040 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.28 \
  --random-pairwise-role-guard-linear-weight 2.20 \
  --random-pairwise-role-guard-omega-weight 3.20 \
  --random-pairwise-role-guard-yield-weight 3.20 \
  --random-pairwise-role-guard-standon-weight 1.60 \
  --random-recovery-safety-gate-scale 0.0 \
  --random-recovery-safety-gate-mode sample \
  --policy-anchor-exclude-random-pairwise-role-guard \
  "$@"