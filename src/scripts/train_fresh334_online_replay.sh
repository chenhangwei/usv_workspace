#!/bin/bash
# fresh334: short online hard-random replay from fresh331.
#
# This is deliberately not another collision-point trace fit.  It keeps the
# fresh331 policy as the baseline and lets PPO see hard random rollouts under
# the real reward/termination dynamics, with a very small clip/lr and a strong
# policy anchor to avoid moving the encounter basin too far.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh334_online_replay_from_fresh331.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh334_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-420}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}" \
exec bash scripts/train_fresh318.sh \
  --checkpoint-interval 420 \
  --curriculum-seed 1456 \
  --curriculum-seed 1458 \
  --curriculum-seed 1460 \
  --curriculum-seed 1461 \
  --curriculum-seed 1464 \
  --learning-rate 2.5e-8 \
  --learning-rate-end 1.0e-8 \
  --clip-range 0.000018 \
  --ppo-policy-loss-scale 0.0012 \
  --ppo-value-loss-scale 0.020 \
  --value-coef 0.010 \
  --update-epochs 1 \
  --force-actor-log-std -2.80 \
  --random-deconflict-pretrain-epochs 0 \
  --random-role-balance-pretrain-epochs 0 \
  --random-deconflict-weight 4.80 \
  --random-deconflict-weight-end 4.90 \
  --random-deconflict-yield-weight 4.60 \
  --random-deconflict-omega-weight 1.45 \
  --random-role-balance-weight 3.40 \
  --random-role-balance-weight-end 3.60 \
  --random-role-balance-yield-max-speed 0.040 \
  --random-role-balance-yield-min-starboard-omega 0.0 \
  --random-role-balance-omega-weight 0.35 \
  --random-cte-recovery-weight 0.10 \
  --random-cte-recovery-weight-end 0.12 \
  --random-safe-finish-weight 0.35 \
  --random-safe-finish-weight-end 0.45 \
  --random-offroute-finish-weight 0.70 \
  --random-offroute-finish-weight-end 0.85 \
  --policy-anchor-weight 2200.0 \
  --policy-anchor-weight-end 2600.0 \
  "$@"