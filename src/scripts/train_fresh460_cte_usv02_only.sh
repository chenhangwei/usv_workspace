#!/bin/bash
# fresh460: narrower structural probe after fresh459 collision.
#
# Keep wider CTE visibility and true-CTE path reward, but remove the online
# route_progress CTE gate and broad all-agent offroute-finish target.  The only
# recovery auxiliary left is usv_02 CTE recovery, with a stronger anchor.

set -eo pipefail

cd "$(dirname "$0")/.."

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh460_cte_usv02_only_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh460_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-900}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}" \
exec bash scripts/train_fresh459_cte_structure.sh \
  --route-progress-cte-gate-start 0.0 \
  --route-progress-cte-gate-width 0.0 \
  --route-progress-cte-gate-floor 0.25 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-cte-recovery-weight 0.80 \
  --random-cte-recovery-weight-end 1.10 \
  --random-cte-recovery-min-neighbor-separation 3.00 \
  --learning-rate 3.5e-8 \
  --learning-rate-end 1.5e-8 \
  --clip-range 0.000025 \
  --policy-anchor-weight 8500.0 \
  --policy-anchor-weight-end 12000.0 \
  "$@"