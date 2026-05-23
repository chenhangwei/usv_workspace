#!/bin/bash
# fresh463: action-movement probe for seed1461 usv_02 high-CTE recovery.
#
# fresh462 proved the signed-CTE target is safe but still barely changes the
# actor on high-CTE seed1461 samples.  This keeps the same random-only gates and
# intentionally strengthens the actor-only recovery pass so the next decision
# can be made from delta/eval evidence instead of another tiny update.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh463_signed_cte_action_probe_from_fresh438.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh463_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1800}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}" \
exec bash scripts/train_fresh462_signed_cte_random_focus.sh \
  --learning-rate 2.0e-7 \
  --learning-rate-end 8.0e-8 \
  --clip-range 0.00012 \
  --update-epochs 4 \
  --random-cte-recovery-weight 3.50 \
  --random-cte-recovery-weight-end 5.00 \
  --random-cte-recovery-target-speed 0.18 \
  --random-cte-recovery-min-speed 0.09 \
  --random-cte-recovery-max-omega 0.32 \
  --random-cte-recovery-omega-weight 1.35 \
  --random-recovery-pretrain-epochs 6 \
  --random-recovery-pretrain-learning-rate 4.0e-6 \
  --random-recovery-pretrain-max-grad-norm 0.60 \
  --random-recovery-pretrain-cte-scale 1.50 \
  --policy-anchor-weight 2000.0 \
  --policy-anchor-weight-end 3000.0 \
  "$@"