#!/bin/bash
# fresh487: widen usv_02 CTE recovery coverage after fresh486.
#
# fresh486 kept seed1461 useful, but seed1460 still overflowed because the CTE
# recovery loss was active on too few rollout samples.  This branch expands the
# usv_02 recovery mask to lower CTE and farther distance-to-goal, while reducing
# clear-ahead pressure so high-CTE samples are not rewarded for sprinting.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh486_signed_cte_brake_from_fresh484.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh487_wide_cte_recovery_from_fresh486.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh487_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 3.4e-8 \
  --learning-rate-end 1.6e-8 \
  --clip-range 0.000022 \
  --update-epochs 4 \
  --random-cte-recovery-agent-index 1 \
  --random-cte-recovery-min-abs-cte 0.45 \
  --random-cte-recovery-full-abs-cte 2.20 \
  --random-cte-recovery-max-distance 18.0 \
  --random-cte-recovery-min-neighbor-separation 5.0 \
  --random-cte-recovery-omega-mode signed-cte-inverted \
  --random-cte-recovery-weight 1.75 \
  --random-cte-recovery-weight-end 2.60 \
  --random-cte-recovery-target-speed 0.080 \
  --random-cte-recovery-min-speed 0.015 \
  --random-cte-recovery-max-omega 0.44 \
  --random-cte-recovery-omega-reference 0.78 \
  --random-cte-recovery-linear-weight 1.00 \
  --random-cte-recovery-omega-weight 1.80 \
  --random-cte-recovery-speed-cte-slowdown 1.00 \
  --random-cte-recovery-speed-heading-gate 0.50 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.15e-6 \
  --random-recovery-pretrain-max-grad-norm 0.24 \
  --random-recovery-pretrain-cte-scale 1.60 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.75 \
  --random-clear-ahead-weight-end 1.10 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.70 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.45 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.70 \
  --random-clear-ahead-speed-cte-full 2.00 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.170 \
  --random-clear-ahead-min-speed 0.020 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 3.00 \
  --policy-anchor-weight 6000.0 \
  --policy-anchor-weight-end 9000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
