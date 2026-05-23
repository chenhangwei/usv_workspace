#!/bin/bash
# fresh404: low-strength anchored late-lagging on both guard1463 and seed1460.
# fresh403 improves guard1463 with healthy margin, but seed1460 still leaves
# usv_03 stalled after teammates finish. This pass adds seed1460 to the low-noise
# curriculum and runs two rollouts so both late-stall shapes can be sampled.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh404_dual_seed_low_strength_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh404_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5400}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-279}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1460 \
  --checkpoint-interval 2700 \
  "$@"