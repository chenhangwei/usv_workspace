#!/bin/bash
# fresh392: targeted late-lagging recovery from fresh387.
# fresh387 keeps collision safety and recovers usv_03, but leaves usv_01 at
# progress ~0 after teammates are far ahead. This stage disables broad
# CTE/offroute/clear recovery and only nudges low-progress random agents when a
# teammate is already clearly ahead and the fleet is safely separated.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh392_late_lagging_from_fresh387.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh392_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-253}" \
exec bash scripts/train_fresh387_long_rollout_late_recovery.sh \
  --random-safe-finish-weight 0.0 \
  --random-safe-finish-weight-end 0.0 \
  --random-cte-recovery-weight 0.0 \
  --random-cte-recovery-weight-end 0.0 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-clear-ahead-weight 0.0 \
  --random-clear-ahead-weight-end 0.0 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 8.0e-7 \
  --random-recovery-pretrain-max-grad-norm 0.012 \
  --random-recovery-pretrain-offroute-scale 0.0 \
  --random-recovery-pretrain-cte-scale 0.0 \
  --random-recovery-pretrain-clear-scale 0.0 \
  --random-recovery-pretrain-late-lagging-scale 1.0 \
  --random-late-lagging-weight 2.80 \
  --random-late-lagging-weight-end 3.40 \
  --random-late-lagging-max-distance 25.0 \
  --random-late-lagging-min-distance 3.0 \
  --random-late-lagging-max-self-progress 0.12 \
  --random-late-lagging-min-team-progress 0.85 \
  --random-late-lagging-min-progress-gap 0.75 \
  --random-late-lagging-min-team-separation 5.80 \
  --random-late-lagging-min-neighbor-separation 5.80 \
  --random-late-lagging-target-source goal \
  --random-late-lagging-target-speed 0.14 \
  --random-late-lagging-min-speed 0.07 \
  --random-late-lagging-max-omega 0.36 \
  --random-late-lagging-omega-reference 0.80 \
  --random-late-lagging-omega-weight 3.20 \
  --policy-anchor-weight 260.0 \
  --policy-anchor-weight-end 340.0 \
  --policy-anchor-exclude-random-late-lagging \
  "$@"