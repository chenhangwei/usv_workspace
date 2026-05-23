#!/bin/bash
# fresh399: fresh398 with rollout exploration noise heavily reduced so the
# training trajectory follows the deterministic guard1463 evaluation path more
# closely and exposes late-lagging samples.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh399_guard1463_low_noise_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh399_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-269}" \
exec bash scripts/train_fresh398_guard1463_anchored_late_lagging.sh \
  --force-actor-log-std -5.0 \
  "$@"