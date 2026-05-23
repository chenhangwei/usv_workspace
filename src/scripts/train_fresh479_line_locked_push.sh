#!/bin/bash
# fresh479: push only inside the line-locked low-CTE window.
#
# fresh478 kept seed1461 usv_02 inside |CTE| ~= 1.5 but still barely moved.
# This continuation keeps CTE recovery light and makes the line-locked
# clear-ahead linear target much harder, while the clear-ahead CTE gate prevents
# pushing again once the boat drifts outside the route band.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh478_line_locked_resume_from_fresh469.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh479_line_locked_push_from_fresh478.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh479_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 7.5e-8 \
  --learning-rate-end 3.6e-8 \
  --clip-range 0.00005 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 0.55 \
  --random-cte-recovery-weight-end 0.80 \
  --random-cte-recovery-target-speed 0.10 \
  --random-cte-recovery-min-speed 0.070 \
  --random-cte-recovery-max-omega 0.30 \
  --random-cte-recovery-omega-reference 1.05 \
  --random-cte-recovery-linear-weight 0.80 \
  --random-cte-recovery-omega-weight 1.00 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 2.0e-6 \
  --random-recovery-pretrain-max-grad-norm 0.34 \
  --random-recovery-pretrain-cte-scale 0.30 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 1.00 \
  --random-clear-ahead-weight-end 1.55 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.62 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 1.58 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.18 \
  --random-clear-ahead-min-speed 0.12 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 5.00 \
  --policy-anchor-weight 4200.0 \
  --policy-anchor-weight-end 6500.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
