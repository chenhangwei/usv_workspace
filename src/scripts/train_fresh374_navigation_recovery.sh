#!/bin/bash
# fresh374: short on-policy navigation-recovery replay from fresh367.
# Goal: keep the fresh367 collision gains, but move guard1463 out of the
# safe-yet-negative-progress basin through rollout-level heading/CTE recovery.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh367_navigation_aware_avoidance_from_fresh352.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh374_navigation_recovery_from_fresh367.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh374_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-238}" \
exec bash scripts/train_fresh318.sh \
  --checkpoint-interval 420 \
  --curriculum-seed 1463 \
  --curriculum-seed 1463 \
  --curriculum-seed 1458 \
  --curriculum-seed 1456 \
  --curriculum-seed 1460 \
  --curriculum-seed 1461 \
  --learning-rate 1.8e-7 \
  --learning-rate-end 6.0e-8 \
  --clip-range 0.00010 \
  --ppo-policy-loss-scale 0.0040 \
  --ppo-value-loss-scale 0.040 \
  --value-coef 0.014 \
  --update-epochs 2 \
  --progress-weight 5.80 \
  --heading-error-weight 0.35 \
  --heading-correction-reward-weight 1.20 \
  --heading-convergence-reward-weight 0.40 \
  --heading-convergence-threshold-deg 18.0 \
  --pure-cruise-reward-weight 1.20 \
  --pure-idle-penalty-weight 1.00 \
  --pure-spin-penalty-weight 0.45 \
  --straight-line-omega-cte-gate 0.50 \
  --random-cte-recovery-weight 3.50 \
  --random-cte-recovery-weight-end 4.50 \
  --random-cte-recovery-min-abs-cte 0.45 \
  --random-cte-recovery-full-abs-cte 2.50 \
  --random-cte-recovery-min-neighbor-separation 2.50 \
  --random-cte-recovery-target-speed 0.20 \
  --random-cte-recovery-min-speed 0.12 \
  --random-cte-recovery-max-omega 0.46 \
  --random-cte-recovery-omega-reference 0.80 \
  --random-cte-recovery-omega-weight 2.20 \
  --random-offroute-finish-weight 3.00 \
  --random-offroute-finish-weight-end 4.00 \
  --random-offroute-finish-route-progress-min 0.0 \
  --random-offroute-finish-min-abs-cte 0.65 \
  --random-offroute-finish-full-abs-cte 2.50 \
  --random-offroute-finish-min-team-separation 2.80 \
  --random-offroute-finish-min-neighbor-separation 2.80 \
  --random-offroute-finish-target-speed 0.22 \
  --random-offroute-finish-min-speed 0.12 \
  --random-offroute-finish-max-omega 0.36 \
  --random-offroute-finish-omega-reference 0.80 \
  --random-offroute-finish-omega-weight 2.00 \
  --random-safe-finish-weight 1.20 \
  --random-safe-finish-weight-end 1.60 \
  --random-safe-finish-min-team-separation 2.80 \
  --random-safe-finish-full-team-separation 3.40 \
  --random-safe-finish-target-speed 0.23 \
  --random-safe-finish-min-speed-scale 0.65 \
  --random-safe-finish-max-omega 0.12 \
  --random-safe-finish-omega-weight 0.25 \
  --policy-anchor-weight 350.0 \
  --policy-anchor-weight-end 450.0 \
  "$@"
