#!/bin/bash
# fresh310: continue from fresh309 with single_usv_overtaking imitation enabled.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh309_single_overtake_finish_from_fresh308.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh310_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-7560}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh309.sh \
  --overtaking-imitation-weight 5.00 \
  --overtaking-imitation-weight-end 7.50 \
  --overtaking-imitation-max-distance 28.0 \
  --overtaking-imitation-rear-approach-speed 0.18 \
  --overtaking-imitation-rear-close-speed 0.08 \
  --overtaking-imitation-rear-pass-speed 0.34 \
  --overtaking-imitation-close-separation 2.35 \
  --overtaking-imitation-release-separation 6.60 \
  --overtaking-imitation-target-starboard-offset 1.15 \
  --overtaking-imitation-rear-omega 0.30 \
  --overtaking-imitation-omega-weight 1.25