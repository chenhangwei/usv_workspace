#!/bin/bash
# fresh400: second anchored low-noise late-lagging pass from fresh399.
# fresh399 moved the old lagging boat forward without collision; this pass lets
# the same narrow gate target the new lagging boat while anchoring to fresh399.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh399_guard1463_low_noise_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh400_second_late_lagging_pass.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh400_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-271}" \
exec bash scripts/train_fresh398_guard1463_anchored_late_lagging.sh \
  --force-actor-log-std -5.0 \
  --random-late-lagging-weight 0.60 \
  --random-late-lagging-weight-end 0.95 \
  --policy-anchor-weight 620.0 \
  --policy-anchor-weight-end 880.0 \
  "$@"