#!/bin/bash
# fresh466: post-recapture resume probe for seed1461 usv_02.
#
# fresh464 recaptured the route but left usv_02 nearly stopped.  fresh465 used
# raw clear-ahead imitation and made progress worse.  This keeps the
# sign-corrected CTE recovery, then applies a usv_02-only constant-speed resume
# target only after CTE is back inside the route band and the random encounter
# is clear.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh466_recap_resume_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh466_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1800}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.45 \
  --random-clear-ahead-weight-end 0.80 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 32.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.02 \
  --random-clear-ahead-max-route-progress 0.35 \
  --random-clear-ahead-max-abs-cte 2.40 \
  --random-clear-ahead-min-neighbor-separation 5.0 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.18 \
  --random-clear-ahead-min-speed 0.12 \
  --random-clear-ahead-max-omega 0.08 \
  --random-clear-ahead-omega-reference 1.10 \
  --random-clear-ahead-omega-weight 0.30 \
  --random-recovery-pretrain-clear-scale 2.00 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"