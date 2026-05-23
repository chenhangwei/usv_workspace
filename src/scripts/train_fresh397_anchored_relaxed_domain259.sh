#!/bin/bash
# fresh397: fresh396 settings on the domain that produced late-lagging samples
# in fresh393, while keeping late samples under the policy anchor.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh397_anchored_relaxed_domain259.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh397_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-259}" \
exec bash scripts/train_fresh396_anchored_relaxed_late_lagging.sh "$@"