#!/bin/bash
# fresh473: stronger lookahead correction after fresh472.
#
# fresh472 kept usv_02 moving but let CTE grow.  This keeps the lookahead yaw
# teacher, makes the CTE bias stronger, and restores only partial CTE slowdown
# so off-route samples still move but do not sprint away from the path.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh472_goal_cte_lookahead_resume_from_fresh471.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh473_goal_cte_lookahead_stronger_from_fresh472.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh473_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 6.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.00005 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.35 \
  --random-cte-recovery-cte-lookahead 1.70 \
  --random-cte-recovery-cte-heading-scale 1.10 \
  --random-cte-recovery-weight 1.80 \
  --random-cte-recovery-weight-end 2.80 \
  --random-cte-recovery-target-speed 0.15 \
  --random-cte-recovery-min-speed 0.10 \
  --random-cte-recovery-max-omega 0.34 \
  --random-cte-recovery-omega-reference 0.80 \
  --random-cte-recovery-omega-weight 1.35 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.3e-6 \
  --random-recovery-pretrain-max-grad-norm 0.30 \
  --random-recovery-pretrain-cte-scale 1.15 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.22 \
  --random-clear-ahead-weight-end 0.34 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.34 \
  --random-clear-ahead-max-abs-cte 1.90 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.14 \
  --random-clear-ahead-min-speed 0.09 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 1.30 \
  --policy-anchor-weight 6500.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"