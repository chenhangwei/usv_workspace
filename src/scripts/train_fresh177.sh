#!/bin/bash
# fresh177: CTE-gated route_progress observation from fresh172 step1260.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh172_checkpoints/fresh172_offroute_recovery_from_fresh171_step_0001260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh177_cte_gated_progress_from_fresh172_step1260.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh177_checkpoints}" \
LOG_FILE="${LOG_FILE:-/tmp/fresh177_train.log}" \
RUN_NAME="${RUN_NAME:-fresh177}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}" \
EXTRA_TRAIN_ARGS="${EXTRA_TRAIN_ARGS:---route-progress-cte-gate-start 0.80 --route-progress-cte-gate-width 3.00 --route-progress-cte-gate-floor 0.25}" \
scripts/train_fresh172.sh