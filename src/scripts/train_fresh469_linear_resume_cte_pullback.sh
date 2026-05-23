#!/bin/bash
# fresh469: keep fresh468's linear resume, then pull CTE back harder.
#
# fresh468 got usv_02 moving again but let it drift outside the route band.
# This follow-up starts from fresh468, reduces the clear-ahead footprint, and
# strengthens the sign-corrected CTE recovery teacher for off-route samples.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh468_linear_resume_only_from_fresh464.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh469_linear_resume_cte_pullback_from_fresh468.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh469_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-217}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 8.0e-8 \
  --learning-rate-end 4.0e-8 \
  --clip-range 0.00006 \
  --update-epochs 3 \
  --random-cte-recovery-weight 2.20 \
  --random-cte-recovery-weight-end 3.20 \
  --random-cte-recovery-target-speed 0.13 \
  --random-cte-recovery-min-speed 0.07 \
  --random-cte-recovery-max-omega 0.34 \
  --random-cte-recovery-omega-reference 0.95 \
  --random-cte-recovery-omega-weight 1.35 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.8e-6 \
  --random-recovery-pretrain-max-grad-norm 0.35 \
  --random-recovery-pretrain-cte-scale 1.10 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.18 \
  --random-clear-ahead-weight-end 0.28 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.32 \
  --random-clear-ahead-max-abs-cte 2.10 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.14 \
  --random-clear-ahead-min-speed 0.09 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 1.40 \
  --policy-anchor-weight 6500.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"