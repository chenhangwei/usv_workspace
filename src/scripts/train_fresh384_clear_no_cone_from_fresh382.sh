#!/bin/bash
# fresh384: fresh382 safety plus clear recovery without the forward-cone gate.
# fresh383 showed random-clear-ahead never activated with the default cone; here
# clear means no CPA/local threat and enough all-around nearest separation.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh382_pairwise_recovery_balanced_from_fresh380.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh384_clear_no_cone_from_fresh382.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh384_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-232}" \
exec bash scripts/train_fresh382_pairwise_recovery_balanced.sh \
  --learning-rate 1.0e-7 \
  --learning-rate-end 4.5e-8 \
  --clip-range 0.00007 \
  --ppo-policy-loss-scale 0.0018 \
  --ppo-value-loss-scale 0.032 \
  --random-deconflict-pretrain-epochs 8 \
  --random-role-balance-pretrain-epochs 5 \
  --random-pairwise-role-guard-pretrain-epochs 16 \
  --random-pairwise-role-guard-pretrain-learning-rate 6.0e-6 \
  --random-pairwise-role-guard-weight 13.50 \
  --random-pairwise-role-guard-weight-end 15.00 \
  --random-pairwise-role-guard-standon-close-speed 0.015 \
  --random-pairwise-role-guard-standon-weight 3.30 \
  --random-clear-ahead-weight 7.50 \
  --random-clear-ahead-weight-end 8.50 \
  --random-clear-ahead-distance 0.0 \
  --random-clear-ahead-max-distance 80.0 \
  --random-clear-ahead-min-neighbor-separation 3.80 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.24 \
  --random-clear-ahead-min-speed 0.18 \
  --random-clear-ahead-max-omega 0.38 \
  --random-clear-ahead-omega-reference 1.00 \
  --random-clear-ahead-omega-weight 2.80 \
  --random-cte-recovery-weight 6.20 \
  --random-cte-recovery-weight-end 7.00 \
  --random-cte-recovery-min-neighbor-separation 3.40 \
  --random-cte-recovery-target-speed 0.21 \
  --random-cte-recovery-min-speed 0.12 \
  --random-cte-recovery-max-omega 0.44 \
  --random-cte-recovery-omega-reference 0.90 \
  --random-cte-recovery-omega-weight 4.60 \
  --random-offroute-finish-weight 6.00 \
  --random-offroute-finish-weight-end 6.80 \
  --random-offroute-finish-min-team-separation 3.35 \
  --random-offroute-finish-min-neighbor-separation 3.35 \
  --random-offroute-finish-target-speed 0.21 \
  --random-offroute-finish-min-speed 0.12 \
  --random-offroute-finish-max-omega 0.44 \
  --random-offroute-finish-omega-reference 0.90 \
  --random-offroute-finish-omega-weight 4.40 \
  --policy-anchor-weight 210.0 \
  --policy-anchor-weight-end 250.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"