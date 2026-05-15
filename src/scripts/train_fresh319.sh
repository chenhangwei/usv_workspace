#!/bin/bash
# fresh319: short omega-focused random replay from the best fresh318 early checkpoint.
#
# fresh318 step1260 made seeds 1458/1461 collision-free but left 1460 with a
# late yield-direction error. Keep this stage short so it stays random-only in
# the fresh318 curriculum order and does not wash out the early repair.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh318_checkpoints/fresh318_random_hard_replay_from_fresh313_step_0001260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh319_random_omega_replay_from_fresh318_step1260.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh319_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}" \
exec bash scripts/train_fresh318.sh \
  --curriculum-seed 1460 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --curriculum-seed 1461 \
  --learning-rate 8.0e-8 \
  --learning-rate-end 3.5e-8 \
  --clip-range 0.000050 \
  --random-deconflict-weight 5.80 \
  --random-deconflict-weight-end 6.10 \
  --random-deconflict-yield-speed 0.015 \
  --random-deconflict-yield-omega 0.50 \
  --random-deconflict-omega-weight 2.60 \
  --random-deconflict-standon-weight 0.06 \
  --random-deconflict-yield-weight 5.80 \
  --random-deconflict-pretrain-epochs 6 \
  --random-deconflict-pretrain-learning-rate 2.0e-6 \
  --random-deconflict-pretrain-max-grad-norm 0.08 \
  --random-role-balance-weight 5.60 \
  --random-role-balance-weight-end 6.00 \
  --random-role-balance-pretrain-epochs 4 \
  --random-role-balance-pretrain-learning-rate 1.8e-6 \
  --random-role-balance-pretrain-max-grad-norm 0.08 \
  --random-role-balance-yield-max-speed 0.025 \
  --random-role-balance-yield-min-starboard-omega 0.28 \
  --random-role-balance-yield-weight 3.30 \
  --random-role-balance-omega-weight 1.20 \
  --random-safe-finish-weight 0.45 \
  --random-safe-finish-weight-end 0.55 \
  --random-offroute-finish-weight 1.20 \
  --random-offroute-finish-weight-end 1.50 \
  --policy-anchor-weight 900.0 \
  --policy-anchor-weight-end 1050.0 \
  "$@"