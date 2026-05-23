#!/bin/bash
# fresh401: repeat the successful fresh399 low-noise anchored late-lagging pass,
# but make the close-pair guard start a little earlier to recover separation
# margin without stacking a second update on top of fresh399.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh401_low_noise_margin_late_lagging.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh401_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-273}" \
exec bash scripts/train_fresh398_guard1463_anchored_late_lagging.sh \
  --force-actor-log-std -5.0 \
  --random-pairwise-role-guard-weight 15.50 \
  --random-pairwise-role-guard-weight-end 17.50 \
  --random-pairwise-role-guard-pretrain-epochs 22 \
  --random-pairwise-role-guard-pretrain-learning-rate 5.0e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.055 \
  --random-pairwise-role-guard-safe-separation 1.35 \
  --random-pairwise-role-guard-release-separation 3.60 \
  --random-pairwise-role-guard-standon-close-speed 0.010 \
  --random-pairwise-role-guard-yield-speed 0.001 \
  --random-pairwise-role-guard-yield-omega 0.52 \
  --random-pairwise-role-guard-standon-omega 0.46 \
  --random-pairwise-role-guard-linear-weight 4.20 \
  --random-pairwise-role-guard-omega-weight 6.40 \
  --random-late-lagging-weight 0.65 \
  --random-late-lagging-weight-end 0.95 \
  --random-late-lagging-min-team-separation 6.10 \
  --random-late-lagging-min-neighbor-separation 6.10 \
  --policy-anchor-weight 560.0 \
  --policy-anchor-weight-end 820.0 \
  "$@"