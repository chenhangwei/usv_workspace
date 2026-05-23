#!/bin/bash
# fresh459: structural CTE recovery probe from fresh438.
#
# The fresh456-fresh458 trace-fit probes stayed safe but only moved usv_02's
# stall point past the clipped CTE boundary.  This stage keeps the actor shape
# unchanged while changing the training signal: expose a wider signed CTE range,
# gate route_progress when the boat is far off the reference route, and compute
# path-deviation reward from the true CTE.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh459_cte_structure_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh459_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-900}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}" \
exec bash scripts/train_fresh308.sh \
  --checkpoint-interval 900 \
  --rollout-steps 900 \
  --episode-timeout 240.0 \
  --no-progress-timeout 220.0 \
  --min-progress-delta 0.006 \
  --curriculum-seed 1463 \
  --cte-clip-range 5.0 \
  --route-progress-cte-gate-start 2.0 \
  --route-progress-cte-gate-width 5.0 \
  --route-progress-cte-gate-floor 0.0 \
  --path-deviation-use-unclipped-cte \
  --path-deviation-penalty-weight 0.95 \
  --path-deviation-tolerance 0.75 \
  --path-deviation-conflict-scale 1.25 \
  --random-cte-recovery-agent-index 1 \
  --random-cte-recovery-weight 1.20 \
  --random-cte-recovery-weight-end 1.60 \
  --random-cte-recovery-min-abs-cte 2.00 \
  --random-cte-recovery-full-abs-cte 5.00 \
  --random-cte-recovery-min-neighbor-separation 2.80 \
  --random-cte-recovery-target-speed 0.13 \
  --random-cte-recovery-min-speed 0.05 \
  --random-cte-recovery-max-omega 0.24 \
  --random-cte-recovery-omega-reference 0.75 \
  --random-cte-recovery-omega-weight 0.80 \
  --random-offroute-finish-weight 0.80 \
  --random-offroute-finish-weight-end 1.10 \
  --random-offroute-finish-min-abs-cte 2.00 \
  --random-offroute-finish-full-abs-cte 5.00 \
  --random-offroute-finish-min-team-separation 3.00 \
  --random-offroute-finish-min-neighbor-separation 3.00 \
  --random-offroute-finish-target-speed 0.13 \
  --random-offroute-finish-min-speed 0.05 \
  --random-offroute-finish-max-omega 0.24 \
  --random-offroute-finish-omega-reference 0.75 \
  --random-offroute-finish-omega-weight 0.70 \
  --learning-rate 5.0e-8 \
  --learning-rate-end 2.0e-8 \
  --clip-range 0.000035 \
  --ppo-policy-loss-scale 0.0010 \
  --ppo-value-loss-scale 0.012 \
  --policy-anchor-weight 6000.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-offroute-finish \
  "$@"