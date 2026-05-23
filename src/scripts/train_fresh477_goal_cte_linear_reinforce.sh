#!/bin/bash
# fresh477: keep the fresh476 recovery-heading gate, but reinforce forward speed.
#
# fresh476 reduced the worst CTE a little, yet seed1461 usv_02 learned to sit
# almost still while turning.  This keeps the safety-gated lookahead correction
# and raises the CTE-recovery linear loss so the actor retains forward motion
# while it converges back toward the route.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh476_goal_cte_heading_gated_speed_from_fresh473.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh477_goal_cte_linear_reinforce_from_fresh476.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh477_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 3.8e-8 \
  --learning-rate-end 1.9e-8 \
  --clip-range 0.000035 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.25 \
  --random-cte-recovery-speed-heading-gate 0.55 \
  --random-cte-recovery-cte-lookahead 1.20 \
  --random-cte-recovery-cte-heading-scale 1.35 \
  --random-cte-recovery-weight 1.75 \
  --random-cte-recovery-weight-end 2.60 \
  --random-cte-recovery-target-speed 0.15 \
  --random-cte-recovery-min-speed 0.080 \
  --random-cte-recovery-max-omega 0.42 \
  --random-cte-recovery-omega-reference 0.72 \
  --random-cte-recovery-linear-weight 2.40 \
  --random-cte-recovery-omega-weight 1.45 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.1e-6 \
  --random-recovery-pretrain-max-grad-norm 0.24 \
  --random-recovery-pretrain-cte-scale 1.15 \
  --random-recovery-safety-gate-scale 0.15 \
  --random-recovery-safety-gate-mode sample \
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
  --random-clear-ahead-max-abs-cte 1.55 \
  --random-clear-ahead-min-neighbor-separation 5.8 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.13 \
  --random-clear-ahead-min-speed 0.085 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 0.65 \
  --policy-anchor-weight 7000.0 \
  --policy-anchor-weight-end 9000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
