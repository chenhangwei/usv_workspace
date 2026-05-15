#!/bin/bash
# fresh178: CTE-only random recovery from fresh172 step1260.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh172_checkpoints/fresh172_offroute_recovery_from_fresh171_step_0001260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh178_cte_recovery_from_fresh172_step1260.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh178_checkpoints}" \
LOG_FILE="${LOG_FILE:-/tmp/fresh178_train.log}" \
RUN_NAME="${RUN_NAME:-fresh178}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}" \
EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:---random-cte-recovery-weight 3.80 --random-cte-recovery-weight-end 5.20 --random-cte-recovery-min-abs-cte 1.05 --random-cte-recovery-full-abs-cte 3.00 --random-cte-recovery-min-neighbor-separation 0.82 --random-cte-recovery-allow-threat-overlap --random-cte-recovery-target-speed 0.18 --random-cte-recovery-min-speed 0.065 --random-cte-recovery-max-omega 0.32 --random-cte-recovery-omega-reference 0.55 --random-cte-recovery-omega-weight 1.15 --policy-anchor-exclude-random-cte-recovery}" \
scripts/train_fresh172.sh