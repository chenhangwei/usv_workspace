#!/bin/bash
# fresh393: relaxed but still sparse late-lagging recovery from fresh387.
# Probe showed the original late-lagging gate never activated; relaxed progress
# thresholds activate about 6% of samples while safety gates still filter them.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh393_relaxed_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh393_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-259}" \
exec bash scripts/train_fresh392_late_lagging_from_fresh387.sh \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 5.0e-7 \
  --random-recovery-pretrain-max-grad-norm 0.008 \
  --random-late-lagging-weight 1.80 \
  --random-late-lagging-weight-end 2.40 \
  --random-late-lagging-max-self-progress 0.45 \
  --random-late-lagging-min-team-progress 0.45 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 5.00 \
  --random-late-lagging-min-neighbor-separation 5.00 \
  --random-late-lagging-target-speed 0.12 \
  --random-late-lagging-min-speed 0.05 \
  --random-late-lagging-max-omega 0.32 \
  --random-late-lagging-omega-reference 0.85 \
  --random-late-lagging-omega-weight 2.60 \
  --policy-anchor-weight 300.0 \
  --policy-anchor-weight-end 420.0 \
  "$@"