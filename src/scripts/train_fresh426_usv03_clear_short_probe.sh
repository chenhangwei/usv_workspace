#!/bin/bash
# fresh426: short-rollout probe for role-gated clear-ahead retention.
# fresh425 used one 900-step update and drifted the shared policy enough to lose
# seed1460 progress. This keeps the usv_03-only teacher but limits the update to
# a 300-step rollout/sample batch.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh415_usv03_narrow_signed_cte_late.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh426_usv03_clear_short_probe.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh426_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-900}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh425_usv03_clear_retain.sh \
  --checkpoint-interval 900 \
  --rollout-steps 300 \
  --random-clear-ahead-weight 0.03 \
  --random-clear-ahead-weight-end 0.06 \
  --policy-anchor-weight 7000.0 \
  --policy-anchor-weight-end 9000.0 \
  "$@"