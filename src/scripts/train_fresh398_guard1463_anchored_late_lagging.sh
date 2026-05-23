#!/bin/bash
# fresh398: train the anchored late-lagging nudge directly on the guard1463
# random-encounter seed, so the 900-step rollout contains the late stalled boat
# instead of depending on an incidental random rollout.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh398_guard1463_anchored_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh398_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-267}" \
exec bash scripts/train_fresh387_long_rollout_late_recovery.sh \
  --curriculum-seed 1463 \
  --random-safe-finish-weight 0.0 \
  --random-safe-finish-weight-end 0.0 \
  --random-cte-recovery-weight 0.0 \
  --random-cte-recovery-weight-end 0.0 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-clear-ahead-weight 0.0 \
  --random-clear-ahead-weight-end 0.0 \
  --random-recovery-pretrain-epochs 0 \
  --random-recovery-pretrain-offroute-scale 0.0 \
  --random-recovery-pretrain-cte-scale 0.0 \
  --random-recovery-pretrain-clear-scale 0.0 \
  --random-recovery-pretrain-late-lagging-scale 0.0 \
  --random-late-lagging-weight 0.75 \
  --random-late-lagging-weight-end 1.10 \
  --random-late-lagging-max-distance 25.0 \
  --random-late-lagging-min-distance 3.0 \
  --random-late-lagging-max-self-progress 0.45 \
  --random-late-lagging-min-team-progress 0.45 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 5.80 \
  --random-late-lagging-min-neighbor-separation 5.80 \
  --random-late-lagging-target-source goal \
  --random-late-lagging-target-speed 0.10 \
  --random-late-lagging-min-speed 0.045 \
  --random-late-lagging-max-omega 0.22 \
  --random-late-lagging-omega-reference 0.90 \
  --random-late-lagging-omega-weight 1.10 \
  --policy-anchor-weight 520.0 \
  --policy-anchor-weight-end 760.0 \
  "$@"