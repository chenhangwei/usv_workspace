#!/bin/bash
# fresh403: fresh399 structure with weaker late-lagging and stronger anchor.
# Pairwise widening restored margin but killed progress in fresh401/402; this
# variant leaves pairwise safety unchanged and reduces only the late nudge.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh403_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-277}" \
exec bash scripts/train_fresh398_guard1463_anchored_late_lagging.sh \
  --force-actor-log-std -5.0 \
  --random-late-lagging-weight 0.45 \
  --random-late-lagging-weight-end 0.75 \
  --random-late-lagging-target-speed 0.085 \
  --random-late-lagging-min-speed 0.040 \
  --random-late-lagging-max-omega 0.20 \
  --random-late-lagging-omega-weight 0.90 \
  --policy-anchor-weight 700.0 \
  --policy-anchor-weight-end 1000.0 \
  "$@"