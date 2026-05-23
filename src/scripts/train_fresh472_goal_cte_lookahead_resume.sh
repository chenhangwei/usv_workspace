#!/bin/bash
# fresh472: goal-heading plus CTE-lookahead resume teacher.
#
# fresh471 opened forward progress but still ran off the route band.  This keeps
# the linear-only resume pressure, disables CTE-induced speed slowdown, and uses
# a blended goal/CTE lookahead yaw target so usv_02 can keep moving while pulling
# back toward the route.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh471_linear_resume_inverted_cte_from_fresh468.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh472_goal_cte_lookahead_resume_from_fresh471.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh472_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 6.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.00005 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.0 \
  --random-cte-recovery-cte-lookahead 2.80 \
  --random-cte-recovery-cte-heading-scale 0.90 \
  --random-cte-recovery-weight 1.35 \
  --random-cte-recovery-weight-end 2.10 \
  --random-cte-recovery-target-speed 0.16 \
  --random-cte-recovery-min-speed 0.10 \
  --random-cte-recovery-max-omega 0.26 \
  --random-cte-recovery-omega-reference 0.95 \
  --random-cte-recovery-omega-weight 1.05 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.3e-6 \
  --random-recovery-pretrain-max-grad-norm 0.28 \
  --random-recovery-pretrain-cte-scale 0.80 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.34 \
  --random-clear-ahead-weight-end 0.52 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.34 \
  --random-clear-ahead-max-abs-cte 2.20 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.16 \
  --random-clear-ahead-min-speed 0.10 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 2.00 \
  --policy-anchor-weight 6500.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"