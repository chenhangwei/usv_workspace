#!/bin/bash
# fresh229: narrow 1458 repair from fresh228 while anchoring repaired 1461/1463.
#
# Rationale:
#   - fresh228 final fixed 1461/1463 but regressed 1458.
#   - 1458 trace shows role imbalance: stand-on usv_01 is too slow while the
#     give-way usv_02 turns hard but keeps too much forward speed.
#   - Replay 1458 first, then 1461/1463 as anchors, with balanced stand-on speed
#     pressure and stronger yield linear compliance.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh228_full_seed_yield_omega_from_fresh211.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh229_1458_role_balance_from_fresh228.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh229_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh229_train.log}"
RUN_NAME="${RUN_NAME:-fresh229}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-3780}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH229_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1463
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.25
  --learning-rate 1.2e-7
  --learning-rate-end 5.0e-8
  --clip-range 0.00008
  --ppo-policy-loss-scale 0.003
  --ppo-value-loss-scale 0.050
  --value-coef 0.015
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-yield-omega 0.42
  --random-deconflict-omega-weight 1.25
  --random-deconflict-standon-weight 0.55
  --random-deconflict-yield-weight 6.20
  --random-deconflict-pretrain-epochs 5
  --random-deconflict-pretrain-learning-rate 4.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.12
  --random-safe-finish-weight 0.90
  --random-safe-finish-weight-end 1.05
  --random-safe-finish-min-neighbor-separation 2.65
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-offroute-finish-weight 2.30
  --random-offroute-finish-weight-end 2.80
  --policy-anchor-weight 270.0
  --policy-anchor-weight-end 330.0
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH229_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh