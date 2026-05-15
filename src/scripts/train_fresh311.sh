#!/bin/bash
# fresh311: continue from fresh310 with phase-aware single_usv_overtaking return-to-route imitation.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh311_single_overtake_return_from_fresh310.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh311_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-10080}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-222}" \
exec bash scripts/train_fresh309.sh \
  --overtaking-imitation-weight 6.50 \
  --overtaking-imitation-weight-end 8.50 \
  --overtaking-imitation-max-distance 28.0 \
  --overtaking-imitation-rear-approach-speed 0.18 \
  --overtaking-imitation-rear-close-speed 0.075 \
  --overtaking-imitation-rear-pass-speed 0.34 \
  --overtaking-imitation-close-separation 2.35 \
  --overtaking-imitation-release-separation 6.60 \
  --overtaking-imitation-target-starboard-offset 1.10 \
  --overtaking-imitation-rear-omega 0.28 \
  --overtaking-imitation-omega-weight 1.45 \
  --overtaking-imitation-single-return-progress 0.58 \
  --overtaking-imitation-single-return-rel-x -0.05 \
  --overtaking-imitation-single-return-min-separation 2.55 \
  --overtaking-imitation-single-return-cte-start 0.45 \
  --overtaking-imitation-single-return-cte-full 2.80 \
  --overtaking-imitation-single-return-speed 0.33 \
  --overtaking-imitation-single-return-omega 0.34 \
  --overtaking-imitation-single-finish-distance 4.30 \
  --overtaking-imitation-single-finish-speed 0.34 \
  --overtaking-imitation-single-finish-max-omega 0.08 \
  "$@"