#!/bin/bash
# fresh406: diagnostic relaxed version of fresh405.
# Keeps the usv_03-only focus, but lowers the finished-teammate threshold so we
# can verify the focused gate can activate before tightening it again.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh406_usv03_relaxed_finished_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh406_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5400}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-283}" \
exec bash scripts/train_fresh405_usv03_finished_team_late_lagging.sh \
  --random-late-lagging-finished-progress 0.75 \
  --random-late-lagging-min-team-progress 0.75 \
  --random-late-lagging-max-self-progress 0.55 \
  --random-late-lagging-min-progress-gap 0.30 \
  --random-late-lagging-min-team-separation 5.40 \
  --random-late-lagging-min-neighbor-separation 5.40 \
  --random-late-lagging-weight 0.22 \
  --random-late-lagging-weight-end 0.38 \
  --policy-anchor-weight 1200.0 \
  --policy-anchor-weight-end 1600.0 \
  "$@"