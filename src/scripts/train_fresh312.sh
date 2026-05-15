#!/bin/bash
# fresh312: branch from fresh310, keep phase-aware return targets but relax the policy anchor.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh312_single_overtake_return_loose_anchor_from_fresh310.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh312_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-7560}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-223}" \
exec bash scripts/train_fresh309.sh \
  --learning-rate 9.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.000060 \
  --policy-anchor-weight 280.0 \
  --policy-anchor-weight-end 420.0 \
  --overtaking-imitation-weight 8.00 \
  --overtaking-imitation-weight-end 11.00 \
  --overtaking-imitation-max-distance 28.0 \
  --overtaking-imitation-rear-approach-speed 0.18 \
  --overtaking-imitation-rear-close-speed 0.075 \
  --overtaking-imitation-rear-pass-speed 0.34 \
  --overtaking-imitation-close-separation 2.35 \
  --overtaking-imitation-release-separation 6.60 \
  --overtaking-imitation-target-starboard-offset 1.10 \
  --overtaking-imitation-rear-omega 0.28 \
  --overtaking-imitation-omega-weight 1.70 \
  --overtaking-imitation-single-return-progress 0.54 \
  --overtaking-imitation-single-return-rel-x -0.05 \
  --overtaking-imitation-single-return-min-separation 2.50 \
  --overtaking-imitation-single-return-cte-start 0.40 \
  --overtaking-imitation-single-return-cte-full 2.60 \
  --overtaking-imitation-single-return-speed 0.33 \
  --overtaking-imitation-single-return-omega 0.42 \
  --overtaking-imitation-single-finish-distance 4.50 \
  --overtaking-imitation-single-finish-speed 0.34 \
  --overtaking-imitation-single-finish-max-omega 0.08 \
  --route-progress-cte-gate-start 0.80 \
  --route-progress-cte-gate-width 2.20 \
  --route-progress-cte-gate-floor 0.20 \
  "$@"