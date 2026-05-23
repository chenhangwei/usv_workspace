#!/bin/bash
# fresh429: branch-only late-lagging teachability probe from fresh426.
# fresh428 remained safe, but action deltas versus fresh426 were almost zero.
# This keeps the same local gate and frozen shared actor while giving the
# three_usv_random_encounter branch enough LR/updates to show measurable motion.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh426_usv03_clear_short_probe.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh429_agent12_late_lagging_branch_probe.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh429_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1458 \
  --curriculum-seed 1461 \
  --checkpoint-interval 900 \
  --rollout-steps 300 \
  --learning-rate 6.0e-7 \
  --learning-rate-end 2.0e-7 \
  --clip-range 0.00025 \
  --random-cte-recovery-weight 0.0 \
  --random-cte-recovery-weight-end 0.0 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-clear-ahead-weight 0.0 \
  --random-clear-ahead-weight-end 0.0 \
  --random-late-lagging-agent-index 1 \
  --random-late-lagging-agent-index 2 \
  --random-late-lagging-min-finished-teammates 1 \
  --random-late-lagging-finished-progress 0.75 \
  --random-late-lagging-min-team-progress 0.75 \
  --random-late-lagging-max-self-progress 0.55 \
  --random-late-lagging-min-progress-gap 0.25 \
  --random-late-lagging-min-team-separation 4.80 \
  --random-late-lagging-min-neighbor-separation 4.80 \
  --random-late-lagging-max-cpa-score 0.0 \
  --random-late-lagging-max-local-score 0.0 \
  --random-late-lagging-weight 0.180 \
  --random-late-lagging-weight-end 0.360 \
  --random-late-lagging-target-source goal \
  --random-late-lagging-target-speed 0.180 \
  --random-late-lagging-min-speed 0.080 \
  --random-late-lagging-max-omega 0.10 \
  --random-late-lagging-omega-reference 1.20 \
  --random-late-lagging-omega-weight 0.20 \
  --policy-anchor-weight 3000.0 \
  --policy-anchor-weight-end 4500.0 \
  --policy-anchor-exclude-random-late-lagging \
  "$@"