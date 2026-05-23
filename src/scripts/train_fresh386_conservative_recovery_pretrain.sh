#!/bin/bash
# fresh386: conservative actor-only recovery pretrain from fresh382.
# Keep fresh382's safety envelope and use only a small recovery actor nudge.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh386_conservative_recovery_pretrain.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh386_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-235}" \
exec bash scripts/train_fresh382_pairwise_recovery_balanced.sh \
  --random-pairwise-role-guard-pretrain-epochs 18 \
  --random-pairwise-role-guard-pretrain-learning-rate 5.0e-6 \
  --random-pairwise-role-guard-weight 13.50 \
  --random-pairwise-role-guard-weight-end 15.00 \
  --random-recovery-pretrain-epochs 4 \
  --random-recovery-pretrain-learning-rate 1.5e-6 \
  --random-recovery-pretrain-max-grad-norm 0.025 \
  --random-cte-recovery-weight 5.00 \
  --random-cte-recovery-weight-end 5.60 \
  --random-cte-recovery-min-neighbor-separation 3.35 \
  --random-cte-recovery-target-speed 0.18 \
  --random-cte-recovery-min-speed 0.08 \
  --random-cte-recovery-max-omega 0.36 \
  --random-cte-recovery-omega-reference 0.95 \
  --random-cte-recovery-omega-weight 3.40 \
  --random-offroute-finish-weight 4.80 \
  --random-offroute-finish-weight-end 5.40 \
  --random-offroute-finish-min-team-separation 3.30 \
  --random-offroute-finish-min-neighbor-separation 3.30 \
  --random-offroute-finish-target-speed 0.18 \
  --random-offroute-finish-min-speed 0.08 \
  --random-offroute-finish-max-omega 0.36 \
  --random-offroute-finish-omega-reference 0.95 \
  --random-offroute-finish-omega-weight 3.30 \
  --random-clear-ahead-weight 0.00 \
  --random-clear-ahead-weight-end 0.00 \
  --policy-anchor-weight 220.0 \
  --policy-anchor-weight-end 250.0 \
  "$@"