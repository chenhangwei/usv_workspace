#!/bin/bash
# fresh402: balanced margin pass between fresh399 and fresh401.
# fresh401 recovered separation but collapsed progress, so this variant keeps
# the fresh399 late-lagging strength and only mildly widens the pairwise guard.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh402_balanced_margin_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh402_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-275}" \
exec bash scripts/train_fresh398_guard1463_anchored_late_lagging.sh \
  --force-actor-log-std -5.0 \
  --random-pairwise-role-guard-weight 14.50 \
  --random-pairwise-role-guard-weight-end 16.20 \
  --random-pairwise-role-guard-pretrain-epochs 20 \
  --random-pairwise-role-guard-pretrain-learning-rate 5.3e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.060 \
  --random-pairwise-role-guard-safe-separation 1.28 \
  --random-pairwise-role-guard-release-separation 3.35 \
  --random-pairwise-role-guard-standon-close-speed 0.014 \
  --random-pairwise-role-guard-yield-speed 0.001 \
  --random-pairwise-role-guard-yield-omega 0.50 \
  --random-pairwise-role-guard-standon-omega 0.44 \
  --random-pairwise-role-guard-linear-weight 4.00 \
  --random-pairwise-role-guard-omega-weight 6.10 \
  --random-late-lagging-weight 0.75 \
  --random-late-lagging-weight-end 1.10 \
  --random-late-lagging-min-team-separation 5.80 \
  --random-late-lagging-min-neighbor-separation 5.80 \
  --policy-anchor-weight 540.0 \
  --policy-anchor-weight-end 780.0 \
  "$@"