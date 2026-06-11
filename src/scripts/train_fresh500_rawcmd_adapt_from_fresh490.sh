#!/bin/bash
# fresh500: raw_cmd observation adaptation from fresh490.
#
# The bridge now passes the real raw_cmd into observations instead of zeros.
# fresh490 was selected before that fix, so this branch keeps the same signed
# raw-CTE recovery/clear-ahead teacher but resumes from fresh490 under the
# corrected observation distribution.  No runtime speed floor is trained in.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh490_all_agent_raw_cte_signed_return_from_fresh486.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh500_rawcmd_adapt_from_fresh490.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh500_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}" \
exec bash scripts/train_fresh490_all_agent_raw_cte_signed_return_from_fresh486.sh \
  --learning-rate 2.5e-8 \
  --learning-rate-end 1.0e-8 \
  --clip-range 0.000018 \
  --update-epochs 3 \
  --random-cte-recovery-weight 2.00 \
  --random-cte-recovery-weight-end 3.00 \
  --random-clear-ahead-weight 0.35 \
  --random-clear-ahead-weight-end 0.60 \
  --policy-anchor-weight 6000.0 \
  --policy-anchor-weight-end 8500.0 \
  "$@"