#!/bin/bash
# fresh395: anchored, strict late-lagging nudge from fresh387.
# Unlike fresh393/fresh394, this script does not exclude late-lagging samples
# from the policy anchor. The late signal is intentionally weaker and only
# active when the fleet is very well separated.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh395_anchored_strict_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh395_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-263}" \
exec bash scripts/train_fresh387_long_rollout_late_recovery.sh \
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
  --random-late-lagging-weight 0.55 \
  --random-late-lagging-weight-end 0.85 \
  --random-late-lagging-max-distance 25.0 \
  --random-late-lagging-min-distance 3.0 \
  --random-late-lagging-max-self-progress 0.42 \
  --random-late-lagging-min-team-progress 0.50 \
  --random-late-lagging-min-progress-gap 0.28 \
  --random-late-lagging-min-team-separation 7.20 \
  --random-late-lagging-min-neighbor-separation 7.20 \
  --random-late-lagging-target-source raw \
  --random-late-lagging-target-speed 0.09 \
  --random-late-lagging-min-speed 0.04 \
  --random-late-lagging-max-omega 0.22 \
  --random-late-lagging-omega-weight 0.60 \
  --policy-anchor-weight 520.0 \
  --policy-anchor-weight-end 760.0 \
  "$@"