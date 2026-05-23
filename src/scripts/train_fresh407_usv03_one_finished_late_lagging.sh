#!/bin/bash
# fresh407: usv_03-only late-stall repair with one finished teammate.
# fresh405/406 showed that waiting for two completed teammates never overlaps
# the usv_03 lagging/safety gate in training rollout. This keeps the target very
# weak and anchored, but starts once one teammate is clearly finished.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh407_usv03_one_finished_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh407_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5400}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-285}" \
exec bash scripts/train_fresh405_usv03_finished_team_late_lagging.sh \
  --random-late-lagging-min-finished-teammates 1 \
  --random-late-lagging-finished-progress 0.75 \
  --random-late-lagging-min-team-progress 0.75 \
  --random-late-lagging-max-self-progress 0.55 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 5.20 \
  --random-late-lagging-min-neighbor-separation 5.20 \
  --random-late-lagging-weight 0.12 \
  --random-late-lagging-weight-end 0.24 \
  --random-late-lagging-target-speed 0.065 \
  --random-late-lagging-min-speed 0.030 \
  --random-late-lagging-max-omega 0.16 \
  --random-late-lagging-omega-reference 1.00 \
  --random-late-lagging-omega-weight 0.55 \
  --policy-anchor-weight 1500.0 \
  --policy-anchor-weight-end 2000.0 \
  "$@"