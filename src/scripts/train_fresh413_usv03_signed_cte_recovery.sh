#!/bin/bash
# fresh413: seed1460 usv_03 off-route recovery probe.
# Uses sign-corrected goal-heading omega targets and limits the new CTE
# recovery teacher to usv_03 so the existing fresh403 baseline stays anchored.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh413_usv03_signed_cte_recovery.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh413_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh407_usv03_one_finished_late_lagging.sh \
  --goal-heading-omega-sign 1.0 \
  --random-cte-recovery-agent-index 2 \
  --random-cte-recovery-weight 0.08 \
  --random-cte-recovery-weight-end 0.16 \
  --random-cte-recovery-min-abs-cte 0.35 \
  --random-cte-recovery-full-abs-cte 3.00 \
  --random-cte-recovery-max-distance 35.0 \
  --random-cte-recovery-min-neighbor-separation 1.15 \
  --random-cte-recovery-target-speed 0.12 \
  --random-cte-recovery-min-speed 0.06 \
  --random-cte-recovery-max-omega 0.22 \
  --random-cte-recovery-omega-reference 1.00 \
  --random-cte-recovery-omega-weight 0.70 \
  --random-late-lagging-weight 0.08 \
  --random-late-lagging-weight-end 0.16 \
  --random-late-lagging-target-speed 0.060 \
  --random-late-lagging-min-speed 0.030 \
  --random-late-lagging-max-omega 0.14 \
  --random-late-lagging-omega-weight 0.45 \
  --policy-anchor-weight 1800.0 \
  --policy-anchor-weight-end 2400.0 \
  "$@"