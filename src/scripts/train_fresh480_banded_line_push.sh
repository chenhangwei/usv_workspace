#!/bin/bash
# fresh480: extend the line-locked push band without reopening high-CTE sprinting.
#
# fresh479 finally moved seed1461 usv_02 a little, but the clear-ahead CTE gate
# closed once |CTE| reached about 1.7.  This continuation expands the push band
# to |CTE| <= 2.1 and restores a moderate CTE pullback so forward motion does
# not become another off-route sprint.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh479_line_locked_push_from_fresh478.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh480_banded_line_push_from_fresh479.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh480_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 7.0e-8 \
  --learning-rate-end 3.4e-8 \
  --clip-range 0.000045 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 0.80 \
  --random-cte-recovery-weight-end 1.20 \
  --random-cte-recovery-target-speed 0.11 \
  --random-cte-recovery-min-speed 0.075 \
  --random-cte-recovery-max-omega 0.34 \
  --random-cte-recovery-omega-reference 0.95 \
  --random-cte-recovery-linear-weight 0.85 \
  --random-cte-recovery-omega-weight 1.10 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.8e-6 \
  --random-recovery-pretrain-max-grad-norm 0.32 \
  --random-recovery-pretrain-cte-scale 0.48 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 1.15 \
  --random-clear-ahead-weight-end 1.80 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.68 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.10 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.20 \
  --random-clear-ahead-min-speed 0.12 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 5.00 \
  --policy-anchor-weight 3800.0 \
  --policy-anchor-weight-end 6000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
