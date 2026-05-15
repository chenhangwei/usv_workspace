#!/bin/bash
# fresh239: structured role-balance hinge repair from fresh228.
#
# Rationale:
#   - fresh233-fresh238 trace-fit/blend variants moved collisions among 1458,
#     1460, and 1463 instead of finding a stable joint fix.
#   - This run keeps exact deconflict regression weak and adds local hinge
#     constraints: stand-on agents must keep enough forward speed, while yield
#     agents must stay below a speed cap and commit to starboard yaw only when
#     the random encounter danger score is active.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh228_full_seed_yield_omega_from_fresh211.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh239_role_balance_hinge_from_fresh228.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh239_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh239_train.log}"
RUN_NAME="${RUN_NAME:-fresh239}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-8820}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH239_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1463
  --curriculum-seed 1461
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.30
  --learning-rate 1.5e-7
  --learning-rate-end 5.0e-8
  --clip-range 0.00008
  --ppo-policy-loss-scale 0.002
  --ppo-value-loss-scale 0.040
  --value-coef 0.015
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 2.25
  --random-deconflict-weight-end 1.65
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 0.95
  --random-deconflict-yield-omega 0.36
  --random-deconflict-omega-weight 0.45
  --random-deconflict-standon-weight 0.02
  --random-deconflict-yield-weight 2.20
  --random-deconflict-pretrain-epochs 0
  --random-deconflict-pretrain-learning-rate 0.0
  --random-deconflict-pretrain-max-grad-norm 0.0
  --random-role-balance-weight 8.0
  --random-role-balance-weight-end 10.5
  --random-role-balance-pretrain-epochs 5
  --random-role-balance-pretrain-learning-rate 1.6e-6
  --random-role-balance-pretrain-max-grad-norm 0.09
  --random-role-balance-min-danger 0.08
  --random-role-balance-standon-min-speed 0.285
  --random-role-balance-yield-max-speed 0.075
  --random-role-balance-yield-min-starboard-omega 0.22
  --random-role-balance-standon-weight 1.25
  --random-role-balance-yield-weight 1.65
  --random-role-balance-omega-weight 0.75
  --random-safe-finish-weight 0.85
  --random-safe-finish-weight-end 1.10
  --random-safe-finish-min-neighbor-separation 2.70
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.12
  --random-offroute-finish-weight 2.30
  --random-offroute-finish-weight-end 3.00
  --policy-anchor-weight 300.0
  --policy-anchor-weight-end 370.0
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH239_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
