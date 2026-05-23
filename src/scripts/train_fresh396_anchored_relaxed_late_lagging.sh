#!/bin/bash
# fresh396: anchored relaxed late-lagging nudge from fresh387.
# Keeps late-lagging samples under the policy anchor, but restores the relaxed
# progress/separation gate that produced nonzero late samples in fresh393/394.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh396_anchored_relaxed_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh396_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-265}" \
exec bash scripts/train_fresh395_anchored_strict_late_lagging.sh \
  --random-late-lagging-weight 0.70 \
  --random-late-lagging-weight-end 1.10 \
  --random-late-lagging-max-self-progress 0.45 \
  --random-late-lagging-min-team-progress 0.45 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 5.40 \
  --random-late-lagging-min-neighbor-separation 5.40 \
  --policy-anchor-weight 520.0 \
  --policy-anchor-weight-end 760.0 \
  "$@"