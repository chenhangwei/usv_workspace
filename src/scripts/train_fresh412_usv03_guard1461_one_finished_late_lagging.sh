#!/bin/bash
# fresh412: fresh407 with an added seed1461 guard and weaker usv_03 target.
# fresh410/fresh411 blends showed that a tiny fresh407 signal is safe but too
# weak for seed1460, while stronger interpolation starts hurting seed1461. This
# keeps the one-finished usv_03 gate, cycles seed1461 during training, and raises
# the policy anchor so non-target behavior stays close to fresh403.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh403_low_strength_late_lagging.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh412_usv03_guard1461_one_finished_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh412_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-8100}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh407_usv03_one_finished_late_lagging.sh \
  --curriculum-seed 1461 \
  --checkpoint-interval 2700 \
  --random-late-lagging-weight 0.08 \
  --random-late-lagging-weight-end 0.16 \
  --random-late-lagging-target-speed 0.055 \
  --random-late-lagging-min-speed 0.025 \
  --random-late-lagging-max-omega 0.14 \
  --random-late-lagging-omega-weight 0.45 \
  --policy-anchor-weight 2200.0 \
  --policy-anchor-weight-end 3000.0 \
  "$@"