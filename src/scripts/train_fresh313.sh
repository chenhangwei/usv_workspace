#!/bin/bash
# fresh313: branch from fresh310 with earlier/narrower single-overtake return gating.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh313_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-7560}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}" \
exec bash scripts/train_fresh309.sh \
  --learning-rate 9.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.000060 \
  --policy-anchor-weight 240.0 \
  --policy-anchor-weight-end 360.0 \
  --overtaking-imitation-weight 8.00 \
  --overtaking-imitation-weight-end 11.00 \
  --overtaking-imitation-max-distance 28.0 \
  --overtaking-imitation-rear-approach-speed 0.18 \
  --overtaking-imitation-rear-close-speed 0.075 \
  --overtaking-imitation-rear-pass-speed 0.34 \
  --overtaking-imitation-close-separation 2.25 \
  --overtaking-imitation-release-separation 6.40 \
  --overtaking-imitation-target-starboard-offset 0.78 \
  --overtaking-imitation-rear-omega 0.24 \
  --overtaking-imitation-omega-weight 1.85 \
  --overtaking-imitation-single-return-progress 0.46 \
  --overtaking-imitation-single-return-rel-x 0.05 \
  --overtaking-imitation-single-return-min-separation 2.05 \
  --overtaking-imitation-single-return-cte-start 0.32 \
  --overtaking-imitation-single-return-cte-full 2.40 \
  --overtaking-imitation-single-return-speed 0.34 \
  --overtaking-imitation-single-return-omega 0.50 \
  --overtaking-imitation-single-finish-distance 5.00 \
  --overtaking-imitation-single-finish-speed 0.34 \
  --overtaking-imitation-single-finish-max-omega 0.07 \
  --route-progress-cte-gate-start 0.70 \
  --route-progress-cte-gate-width 2.10 \
  --route-progress-cte-gate-floor 0.18 \
  "$@"