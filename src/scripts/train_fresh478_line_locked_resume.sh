#!/bin/bash
# fresh478: resume only after the fresh469-style CTE pullback has locked the line.
#
# fresh477 showed that adding forward pressure while |CTE| is still large pushes
# usv_02 outside the route band again.  This branch starts from fresh469, keeps
# the goal-heading pullback for large CTE, and opens a stronger linear-only
# clear-ahead window only once usv_02 is close enough to the route.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh469_linear_resume_cte_pullback_from_fresh468.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh478_line_locked_resume_from_fresh469.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh478_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 5.0e-8 \
  --learning-rate-end 2.5e-8 \
  --clip-range 0.00004 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 0.75 \
  --random-cte-recovery-weight-end 1.10 \
  --random-cte-recovery-target-speed 0.12 \
  --random-cte-recovery-min-speed 0.075 \
  --random-cte-recovery-max-omega 0.30 \
  --random-cte-recovery-omega-reference 1.00 \
  --random-cte-recovery-linear-weight 1.00 \
  --random-cte-recovery-omega-weight 1.05 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.2e-6 \
  --random-recovery-pretrain-max-grad-norm 0.28 \
  --random-recovery-pretrain-cte-scale 0.45 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.42 \
  --random-clear-ahead-weight-end 0.65 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.60 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 1.65 \
  --random-clear-ahead-min-neighbor-separation 6.0 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.16 \
  --random-clear-ahead-min-speed 0.10 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 2.20 \
  --policy-anchor-weight 7200.0 \
  --policy-anchor-weight-end 9800.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
