#!/bin/bash
# fresh179: action-target response correction after mask trace showed active auxiliaries were not moving the actor enough.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh172_checkpoints/fresh172_offroute_recovery_from_fresh171_step_0001260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh179_action_target_gate_from_fresh172.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh179_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh179_train.log}"
RUN_NAME="${RUN_NAME:-fresh179}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1680}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}"

FRESH179_ARGS="
  --learning-rate 7.5e-7
  --learning-rate-end 2.5e-7
  --clip-range 0.00015
  --update-epochs 4
  --random-safe-finish-weight 1.80
  --random-safe-finish-weight-end 2.40
  --random-safe-finish-local-separation-scale
  --random-safe-finish-max-abs-cte 1.65
  --random-offroute-finish-weight 4.20
  --random-offroute-finish-weight-end 5.20
  --random-cte-recovery-weight 1.40
  --random-cte-recovery-weight-end 2.00
  --random-cte-recovery-goal-tolerance 0.8
  --random-cte-recovery-max-distance 13.0
  --random-cte-recovery-min-abs-cte 1.05
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.05
  --random-cte-recovery-target-speed 0.16
  --random-cte-recovery-min-speed 0.06
  --random-cte-recovery-max-omega 0.30
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 1.00
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH179_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh