#!/bin/bash
# fresh474: anchor-free CTE lookahead correction with moderated resume speed.
#
# fresh473 achieved strong progress but drifted to |CTE| ~= 4.9.  The lookahead
# direction is correct; the next probe lets CTE-recovery samples move away from
# the policy anchor, while reducing clear-ahead pressure and keeping only a
# moderate forward target during large CTE.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh473_goal_cte_lookahead_stronger_from_fresh472.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh474_goal_cte_anchor_free_from_fresh473.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh474_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 5.0e-8 \
  --learning-rate-end 2.5e-8 \
  --clip-range 0.00004 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.50 \
  --random-cte-recovery-cte-lookahead 1.10 \
  --random-cte-recovery-cte-heading-scale 1.40 \
  --random-cte-recovery-weight 2.20 \
  --random-cte-recovery-weight-end 3.20 \
  --random-cte-recovery-target-speed 0.13 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.42 \
  --random-cte-recovery-omega-reference 0.70 \
  --random-cte-recovery-omega-weight 1.80 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.2e-6 \
  --random-recovery-pretrain-max-grad-norm 0.28 \
  --random-recovery-pretrain-cte-scale 1.30 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.12 \
  --random-clear-ahead-weight-end 0.20 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.34 \
  --random-clear-ahead-max-abs-cte 1.50 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.12 \
  --random-clear-ahead-min-speed 0.08 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 0.80 \
  --policy-anchor-weight 7000.0 \
  --policy-anchor-weight-end 9000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"