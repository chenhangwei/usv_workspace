#!/bin/bash
# fresh390: gentle near-distance clear-ahead nudge from fresh387.
# fresh387 recovers two boats but leaves usv_01 behind. This stage keeps that
# basin and adds a small, high-turn clear-ahead actor update only near the goal.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh390_gentle_clear_from_fresh387.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh390_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-249}" \
exec bash scripts/train_fresh388_long_clear_only_recovery.sh \
  --random-safe-finish-weight 0.0 \
  --random-safe-finish-weight-end 0.0 \
  --random-recovery-pretrain-epochs 1 \
  --random-recovery-pretrain-learning-rate 5.0e-7 \
  --random-recovery-pretrain-max-grad-norm 0.008 \
  --random-recovery-pretrain-offroute-scale 0.0 \
  --random-recovery-pretrain-cte-scale 0.0 \
  --random-recovery-pretrain-clear-scale 1.0 \
  --random-clear-ahead-weight 1.80 \
  --random-clear-ahead-weight-end 2.40 \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-neighbor-separation 5.50 \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.12 \
  --random-clear-ahead-min-speed 0.04 \
  --random-clear-ahead-max-omega 0.46 \
  --random-clear-ahead-omega-reference 0.70 \
  --random-clear-ahead-omega-weight 4.50 \
  --policy-anchor-weight 260.0 \
  --policy-anchor-weight-end 340.0 \
  "$@"