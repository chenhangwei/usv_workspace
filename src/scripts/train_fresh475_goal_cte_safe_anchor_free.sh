#!/bin/bash
# fresh475: safety-gated light anchor-free CTE correction.
#
# fresh474 showed anchor-free CTE correction can override safety behavior and
# collide on seed1461.  This keeps a small anchor-free CTE aperture but suppresses
# recovery losses on samples with active deconflict/role-guard signals.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh473_goal_cte_lookahead_stronger_from_fresh472.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh475_goal_cte_safe_anchor_free_from_fresh473.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh475_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 4.0e-8 \
  --learning-rate-end 2.0e-8 \
  --clip-range 0.000035 \
  --update-epochs 3 \
  --random-cte-recovery-omega-mode goal-cte-lookahead \
  --random-cte-recovery-speed-cte-slowdown 0.55 \
  --random-cte-recovery-cte-lookahead 1.35 \
  --random-cte-recovery-cte-heading-scale 1.20 \
  --random-cte-recovery-weight 1.35 \
  --random-cte-recovery-weight-end 2.05 \
  --random-cte-recovery-target-speed 0.12 \
  --random-cte-recovery-min-speed 0.07 \
  --random-cte-recovery-max-omega 0.34 \
  --random-cte-recovery-omega-reference 0.82 \
  --random-cte-recovery-omega-weight 1.25 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 9.0e-7 \
  --random-recovery-pretrain-max-grad-norm 0.22 \
  --random-recovery-pretrain-cte-scale 0.85 \
  --random-recovery-safety-gate-scale 0.15 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.05 \
  --random-clear-ahead-weight-end 0.08 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.03 \
  --random-clear-ahead-max-route-progress 0.34 \
  --random-clear-ahead-max-abs-cte 1.45 \
  --random-clear-ahead-min-neighbor-separation 5.8 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.11 \
  --random-clear-ahead-min-speed 0.07 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 0.30 \
  --policy-anchor-weight 7600.0 \
  --policy-anchor-weight-end 9800.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"