#!/bin/bash
# fresh405: narrow seed1460 late-stall repair from fresh403.
# Only usv_03 (agent index 2) may receive the late-lagging target, and only
# after two teammates are essentially finished. This avoids the broad dual-seed
# push that broke fresh404.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh405_usv03_finished_team_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh405_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5400}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-281}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1460 \
  --checkpoint-interval 2700 \
  --random-late-lagging-agent-index 2 \
  --random-late-lagging-min-finished-teammates 2 \
  --random-late-lagging-finished-progress 0.92 \
  --random-late-lagging-max-self-progress 0.35 \
  --random-late-lagging-min-team-progress 0.92 \
  --random-late-lagging-min-progress-gap 0.55 \
  --random-late-lagging-min-team-separation 5.80 \
  --random-late-lagging-min-neighbor-separation 5.80 \
  --random-late-lagging-weight 0.30 \
  --random-late-lagging-weight-end 0.50 \
  --random-late-lagging-target-speed 0.075 \
  --random-late-lagging-min-speed 0.035 \
  --random-late-lagging-max-omega 0.18 \
  --random-late-lagging-omega-reference 0.95 \
  --random-late-lagging-omega-weight 0.75 \
  --policy-anchor-weight 1000.0 \
  --policy-anchor-weight-end 1400.0 \
  "$@"