#!/bin/bash
# fresh424: conservative version of fresh423.
# fresh423 restored progress on 1461/1463 but collided on seed1460. This keeps
# the random clear-ahead gate active, then lowers cruise pressure and requires
# wider all-around separation before the target can override fresh415.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh415_usv03_narrow_signed_cte_late.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh424_progress_retain_safe_clear.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh424_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1461 \
  --curriculum-seed 1463 \
  --random-cte-recovery-weight 0.0 \
  --random-cte-recovery-weight-end 0.0 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-late-lagging-weight 0.0 \
  --random-late-lagging-weight-end 0.0 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-weight 0.02 \
  --random-clear-ahead-weight-end 0.04 \
  --random-clear-ahead-distance 5.0 \
  --random-clear-ahead-bearing-deg 28.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-neighbor-separation 3.0 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source raw \
  --random-clear-ahead-target-speed 0.16 \
  --random-clear-ahead-min-speed 0.08 \
  --random-clear-ahead-max-omega 0.08 \
  --random-clear-ahead-omega-weight 0.10 \
  --policy-anchor-weight 7000.0 \
  --policy-anchor-weight-end 9000.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"