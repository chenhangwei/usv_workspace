#!/bin/bash
# fresh387: long-rollout late recovery exposure from fresh382.
# The fresh382 stall appears around eval step 740-880, while previous probes only
# trained on 420-step rollouts. This stage lengthens the rollout and episode
# timeout so late safe-but-offroute samples enter the actor update.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh387_long_rollout_late_recovery.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh387_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-237}" \
exec bash scripts/train_fresh382_pairwise_recovery_balanced.sh \
  --checkpoint-interval 900 \
  --rollout-steps 900 \
  --episode-timeout 150.0 \
  --no-progress-timeout 130.0 \
  --min-progress-delta 0.006 \
  --learning-rate 1.2e-7 \
  --learning-rate-end 5.0e-8 \
  --clip-range 0.00008 \
  --ppo-policy-loss-scale 0.0022 \
  --ppo-value-loss-scale 0.035 \
  --random-deconflict-pretrain-epochs 8 \
  --random-role-balance-pretrain-epochs 5 \
  --random-pairwise-role-guard-pretrain-epochs 18 \
  --random-pairwise-role-guard-pretrain-learning-rate 5.5e-6 \
  --random-pairwise-role-guard-pretrain-max-grad-norm 0.06 \
  --random-pairwise-role-guard-weight 13.50 \
  --random-pairwise-role-guard-weight-end 15.00 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.0e-6 \
  --random-recovery-pretrain-max-grad-norm 0.015 \
  --random-cte-recovery-weight 5.00 \
  --random-cte-recovery-weight-end 5.80 \
  --random-cte-recovery-min-neighbor-separation 3.20 \
  --random-cte-recovery-target-speed 0.18 \
  --random-cte-recovery-min-speed 0.07 \
  --random-cte-recovery-max-omega 0.42 \
  --random-cte-recovery-omega-reference 0.82 \
  --random-cte-recovery-omega-weight 3.80 \
  --random-offroute-finish-weight 4.80 \
  --random-offroute-finish-weight-end 5.60 \
  --random-offroute-finish-min-team-separation 3.25 \
  --random-offroute-finish-min-neighbor-separation 3.25 \
  --random-offroute-finish-target-speed 0.18 \
  --random-offroute-finish-min-speed 0.07 \
  --random-offroute-finish-max-omega 0.42 \
  --random-offroute-finish-omega-reference 0.82 \
  --random-offroute-finish-omega-weight 3.60 \
  --random-clear-ahead-weight 0.00 \
  --random-clear-ahead-weight-end 0.00 \
  --policy-anchor-weight 180.0 \
  --policy-anchor-weight-end 230.0 \
  "$@"