#!/bin/bash
# fresh336: short gated-recovery replay from fresh331.
#
# This keeps the fresh334 conservative replay setup, but suppresses CTE/offroute
# recovery losses for any minibatch that contains active random deconflict or
# pairwise role-guard samples.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh336_gated_recovery_from_fresh331.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh336_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-420}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}" \
exec bash scripts/train_fresh334_online_replay.sh \
  --random-recovery-safety-gate-scale 0.0 \
  --random-recovery-safety-gate-mode batch \
  "$@"