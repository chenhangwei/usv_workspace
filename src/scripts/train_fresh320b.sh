#!/bin/bash
# fresh320b: pairwise role guard from fresh318 step1260, preserving its 1458/1461 safety.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh318_checkpoints/fresh318_random_hard_replay_from_fresh313_step_0001260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320b_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-229}" \
exec bash scripts/train_fresh320a.sh \
  --curriculum-seed 1460 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --curriculum-seed 1461 \
  --learning-rate 6.5e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.000040 \
  --random-deconflict-weight 4.40 \
  --random-deconflict-weight-end 4.90 \
  --random-pairwise-role-guard-weight 4.20 \
  --random-pairwise-role-guard-weight-end 4.80 \
  --random-pairwise-role-guard-yield-omega 0.38 \
  --random-pairwise-role-guard-standon-omega 0.10 \
  --random-pairwise-role-guard-omega-weight 1.70 \
  --random-role-balance-weight 3.20 \
  --random-role-balance-weight-end 3.60 \
  --policy-anchor-weight 1100.0 \
  --policy-anchor-weight-end 1250.0 \
  "$@"