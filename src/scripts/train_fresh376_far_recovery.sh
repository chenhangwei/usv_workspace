#!/bin/bash
# fresh376: far-field navigation recovery from fresh367.
# Goal: preserve fresh367 collision safety while teaching the random-encounter
# branch to recover after avoidance instead of running away from the goal.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh367_navigation_aware_avoidance_from_fresh352.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh376_far_recovery_from_fresh367.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh376_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}" \
exec bash scripts/train_fresh318.sh \
  --checkpoint-interval 420 \
  --curriculum-seed 1463 \
  --curriculum-seed 1463 \
  --curriculum-seed 1463 \
  --curriculum-seed 1456 \
  --curriculum-seed 1460 \
  --curriculum-seed 1461 \
  --episode-timeout 78.0 \
  --no-progress-timeout 42.0 \
  --min-progress-delta 0.012 \
  --learning-rate 2.4e-7 \
  --learning-rate-end 8.0e-8 \
  --clip-range 0.00012 \
  --ppo-policy-loss-scale 0.0060 \
  --ppo-value-loss-scale 0.055 \
  --value-coef 0.016 \
  --update-epochs 2 \
  --progress-weight 7.20 \
  --team-progress-weight 2.40 \
  --team-regression-penalty-weight 5.20 \
  --team-goal-proximity-weight 1.40 \
  --deadlock-penalty-weight 6.00 \
  --stall-penalty -22.0 \
  --heading-error-weight 0.42 \
  --heading-correction-reward-weight 1.60 \
  --heading-convergence-reward-weight 0.45 \
  --heading-convergence-threshold-deg 18.0 \
  --pure-cruise-reward-weight 1.25 \
  --pure-idle-penalty-weight 1.10 \
  --pure-spin-penalty-weight 0.50 \
  --straight-line-omega-cte-gate 0.45 \
  --random-safe-finish-weight 0.90 \
  --random-safe-finish-weight-end 1.20 \
  --random-safe-finish-max-distance 80.0 \
  --random-safe-finish-min-team-separation 2.90 \
  --random-safe-finish-full-team-separation 3.60 \
  --random-safe-finish-min-neighbor-separation 2.90 \
  --random-safe-finish-max-abs-cte 0.85 \
  --random-safe-finish-target-speed 0.24 \
  --random-safe-finish-min-speed-scale 0.70 \
  --random-safe-finish-max-omega 0.14 \
  --random-safe-finish-omega-weight 0.35 \
  --random-cte-recovery-weight 5.20 \
  --random-cte-recovery-weight-end 6.20 \
  --random-cte-recovery-max-distance 80.0 \
  --random-cte-recovery-min-abs-cte 0.38 \
  --random-cte-recovery-full-abs-cte 2.50 \
  --random-cte-recovery-min-neighbor-separation 2.60 \
  --random-cte-recovery-target-speed 0.19 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.50 \
  --random-cte-recovery-omega-reference 0.72 \
  --random-cte-recovery-omega-weight 4.20 \
  --random-offroute-finish-weight 4.60 \
  --random-offroute-finish-weight-end 5.60 \
  --random-offroute-finish-max-distance 80.0 \
  --random-offroute-finish-route-progress-min 0.0 \
  --random-offroute-finish-min-abs-cte 0.55 \
  --random-offroute-finish-full-abs-cte 2.60 \
  --random-offroute-finish-min-team-separation 2.90 \
  --random-offroute-finish-min-neighbor-separation 2.90 \
  --random-offroute-finish-target-speed 0.20 \
  --random-offroute-finish-min-speed 0.08 \
  --random-offroute-finish-max-omega 0.46 \
  --random-offroute-finish-omega-reference 0.72 \
  --random-offroute-finish-omega-weight 3.60 \
  --random-recovery-safety-gate-scale 0.35 \
  --random-recovery-safety-gate-mode sample \
  --policy-anchor-weight 300.0 \
  --policy-anchor-weight-end 420.0 \
  --policy-anchor-exclude-random-safe-finish \
  --policy-anchor-exclude-random-offroute-finish \
  --policy-anchor-exclude-random-cte-recovery \
  "$@"