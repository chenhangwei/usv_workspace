#!/bin/bash
# fresh414: narrower seed1460 usv_03 CTE recovery probe.
# Keeps legacy finish-heading teachers intact, applies sign-corrected recovery
# only to usv_03 CTE samples, and disables the extra late-lagging nudge.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh414_usv03_signed_cte_only.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh414_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1460 \
  --checkpoint-interval 2700 \
  --goal-heading-omega-sign 1.0 \
  --random-cte-recovery-agent-index 2 \
  --random-cte-recovery-weight 0.05 \
  --random-cte-recovery-weight-end 0.10 \
  --random-cte-recovery-min-abs-cte 0.45 \
  --random-cte-recovery-full-abs-cte 3.00 \
  --random-cte-recovery-max-distance 35.0 \
  --random-cte-recovery-min-neighbor-separation 1.15 \
  --random-cte-recovery-target-speed 0.11 \
  --random-cte-recovery-min-speed 0.055 \
  --random-cte-recovery-max-omega 0.20 \
  --random-cte-recovery-omega-reference 1.00 \
  --random-cte-recovery-omega-weight 0.65 \
  --random-late-lagging-weight 0.0 \
  --random-late-lagging-weight-end 0.0 \
  --policy-anchor-weight 2600.0 \
  --policy-anchor-weight-end 3600.0 \
  "$@"