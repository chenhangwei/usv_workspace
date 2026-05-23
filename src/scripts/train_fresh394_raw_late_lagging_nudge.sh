#!/bin/bash
# fresh394: ultra-light late-lagging nudge from fresh387.
# fresh393 proved the relaxed gate can improve progress, but the goal-heading
# target plus pretrain broke the close-pair avoidance margin. This variant keeps
# the same sparse gate, uses the raw controller target with only a small forward
# floor, and disables the late-lagging actor-only pretrain.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh394_raw_late_lagging_nudge.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh394_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-261}" \
exec bash scripts/train_fresh392_late_lagging_from_fresh387.sh \
  --random-recovery-pretrain-epochs 0 \
  --random-late-lagging-weight 0.70 \
  --random-late-lagging-weight-end 1.10 \
  --random-late-lagging-max-self-progress 0.45 \
  --random-late-lagging-min-team-progress 0.45 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 5.40 \
  --random-late-lagging-min-neighbor-separation 5.40 \
  --random-late-lagging-target-source raw \
  --random-late-lagging-target-speed 0.10 \
  --random-late-lagging-min-speed 0.045 \
  --random-late-lagging-max-omega 0.24 \
  --random-late-lagging-omega-weight 0.80 \
  --policy-anchor-weight 420.0 \
  --policy-anchor-weight-end 620.0 \
  "$@"