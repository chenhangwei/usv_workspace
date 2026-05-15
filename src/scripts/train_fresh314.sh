#!/bin/bash
# fresh314: branch from fresh310 with goal-heading return targets for single overtaking.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh314_single_overtake_heading_return_from_fresh310.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh314_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-7560}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}" \
exec bash scripts/train_fresh313.sh \
  "$@"