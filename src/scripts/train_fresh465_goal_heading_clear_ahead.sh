#!/bin/bash
# fresh465: sign-corrected CTE recovery plus narrow clear-ahead resume.
#
# fresh464 brought seed1461 usv_02 back inside the CTE band but then left it
# nearly stopped with large yaw.  This keeps the recovery target and adds a
# usv_02-only clear-ahead teacher that is active only when random encounter
# traffic is far away and no deconflict threat is present.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh465_goal_heading_clear_ahead_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh465_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1800}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-233}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.12 \
  --random-clear-ahead-weight-end 0.24 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 32.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 20.0 \
  --random-clear-ahead-min-neighbor-separation 5.0 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source raw \
  --random-clear-ahead-target-speed 0.18 \
  --random-clear-ahead-min-speed 0.10 \
  --random-clear-ahead-max-omega 0.08 \
  --random-clear-ahead-omega-weight 0.25 \
  --random-recovery-pretrain-clear-scale 1.00 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"