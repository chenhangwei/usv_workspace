#!/bin/bash
# fresh378: continue from fresh377 and fit recovery actions harder.
# The fresh377 guard restored separation, but recovery targets stayed active
# while the actor still accelerated away from the route.  This run lowers the
# anchor and increases CTE/off-route omega and speed-shaping losses.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh377_pairwise_recovery_from_fresh376.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh378_recovery_fit_from_fresh377.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh378_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}" \
exec bash scripts/train_fresh377_pairwise_recovery.sh \
  --learning-rate 4.0e-7 \
  --learning-rate-end 1.4e-7 \
  --clip-range 0.00016 \
  --ppo-policy-loss-scale 0.0040 \
  --ppo-value-loss-scale 0.050 \
  --progress-weight 6.60 \
  --team-progress-weight 2.80 \
  --team-regression-penalty-weight 6.50 \
  --path-deviation-penalty-weight 1.15 \
  --heading-error-weight 0.55 \
  --heading-correction-reward-weight 2.00 \
  --random-cte-recovery-weight 9.00 \
  --random-cte-recovery-weight-end 11.00 \
  --random-cte-recovery-target-speed 0.12 \
  --random-cte-recovery-min-speed 0.040 \
  --random-cte-recovery-max-omega 0.50 \
  --random-cte-recovery-omega-reference 0.62 \
  --random-cte-recovery-omega-weight 7.50 \
  --random-offroute-finish-weight 8.00 \
  --random-offroute-finish-weight-end 10.00 \
  --random-offroute-finish-target-speed 0.12 \
  --random-offroute-finish-min-speed 0.040 \
  --random-offroute-finish-max-omega 0.50 \
  --random-offroute-finish-omega-reference 0.62 \
  --random-offroute-finish-omega-weight 6.80 \
  --random-safe-finish-weight 0.45 \
  --random-safe-finish-weight-end 0.65 \
  --random-safe-finish-max-abs-cte 0.55 \
  --policy-anchor-weight 120.0 \
  --policy-anchor-weight-end 180.0 \
  "$@"