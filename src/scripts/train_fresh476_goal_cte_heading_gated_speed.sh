#!/bin/bash
# fresh476: CTE lookahead with recovery-heading speed gate.
#
# fresh473/fresh475 moved usv_02 but let it run along the wrong recovery heading.
# This gates the CTE recovery speed by alignment to the blended goal+CTE heading,
# so forward motion is retained only when the boat is aimed toward the corrective
# path direction.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh473_goal_cte_lookahead_stronger_from_fresh472.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh476_goal_cte_heading_gated_speed_from_fresh473.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh476_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 4.5e-8 \
  --learning-rate-end 2.2e-8 \
  --clip-range 0.00004 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.35 \
  --random-cte-recovery-speed-heading-gate 0.85 \
  --random-cte-recovery-cte-lookahead 1.25 \
  --random-cte-recovery-cte-heading-scale 1.30 \
  --random-cte-recovery-weight 1.65 \
  --random-cte-recovery-weight-end 2.45 \
  --random-cte-recovery-target-speed 0.14 \
  --random-cte-recovery-min-speed 0.045 \
  --random-cte-recovery-max-omega 0.40 \
  --random-cte-recovery-omega-reference 0.76 \
  --random-cte-recovery-omega-weight 1.55 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.0e-6 \
  --random-recovery-pretrain-max-grad-norm 0.24 \
  --random-recovery-pretrain-cte-scale 1.05 \
  --random-recovery-safety-gate-scale 0.15 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.10 \
  --random-clear-ahead-weight-end 0.16 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.34 \
  --random-clear-ahead-max-abs-cte 1.55 \
  --random-clear-ahead-min-neighbor-separation 5.8 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.12 \
  --random-clear-ahead-min-speed 0.07 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 0.55 \
  --policy-anchor-weight 7400.0 \
  --policy-anchor-weight-end 9400.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"