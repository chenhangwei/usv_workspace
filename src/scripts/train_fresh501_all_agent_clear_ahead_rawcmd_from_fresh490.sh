#!/bin/bash
# fresh501: all-agent clear-ahead adaptation from fresh490 under raw_cmd-fixed observations.
#
# fresh500 only reused the fresh490 clear-ahead teacher on agent index 1, while
# raw_cmd-fixed seed1452 stalls all three agents at near-zero RL speed.  This
# branch keeps the signed raw-CTE recovery and strong anchor, but widens the
# clear-ahead imitation gate to all agents and targets a modest forward speed in
# open, non-deconflict samples.  Runtime low-speed floors stay disabled.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh490_all_agent_raw_cte_signed_return_from_fresh486.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh501_all_agent_clear_ahead_rawcmd_from_fresh490.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh501_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2400}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}" \
exec bash scripts/train_fresh490_all_agent_raw_cte_signed_return_from_fresh486.sh \
  --learning-rate 5.0e-8 \
  --learning-rate-end 2.0e-8 \
  --clip-range 0.000030 \
  --update-epochs 4 \
  --random-clear-ahead-agent-index 0 \
  --random-clear-ahead-agent-index 2 \
  --random-clear-ahead-weight 0.90 \
  --random-clear-ahead-weight-end 1.50 \
  --random-clear-ahead-min-route-progress 0.000 \
  --random-clear-ahead-max-route-progress 0.72 \
  --random-clear-ahead-max-abs-cte 1.60 \
  --random-clear-ahead-min-neighbor-separation 4.80 \
  --random-clear-ahead-target-speed 0.180 \
  --random-clear-ahead-min-speed 0.080 \
  --random-clear-ahead-speed-cte-slowdown 0.75 \
  --random-clear-ahead-speed-cte-start 0.70 \
  --random-clear-ahead-speed-cte-full 1.60 \
  --random-recovery-pretrain-clear-scale 2.40 \
  --policy-anchor-weight 6000.0 \
  --policy-anchor-weight-end 9000.0 \
  "$@"