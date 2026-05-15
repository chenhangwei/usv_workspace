#!/bin/bash
# fresh228: full hard-seed replay with yield-omega focused deconf pretrain.
#
# Rationale:
#   - fresh227 finally moved scenario_actor_trunks.5, but only the first two hard
#     seeds were seen before the final checkpoint; safe seeds were not real
#     rollout anchors.
#   - Trace diagnostics show the active give-way agent still under-commits to the
#     target starboard turn, so emphasize yield-agent omega while damping stand-on
#     contamination.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh211_checkpoints/fresh211_dual_threat_yield_from_fresh208.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh228_full_seed_yield_omega_from_fresh211.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh228_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh228_train.log}"
RUN_NAME="${RUN_NAME:-fresh228}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-8820}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH228_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1463
  --curriculum-seed 1460
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.25
  --learning-rate 2.4e-7
  --learning-rate-end 7.0e-8
  --clip-range 0.00010
  --ppo-policy-loss-scale 0.004
  --ppo-value-loss-scale 0.060
  --value-coef 0.020
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 0.98
  --random-deconflict-yield-omega 0.42
  --random-deconflict-omega-weight 1.85
  --random-deconflict-standon-weight 0.06
  --random-deconflict-yield-weight 4.80
  --random-deconflict-pretrain-epochs 6
  --random-deconflict-pretrain-learning-rate 5.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.14
  --random-safe-finish-weight 1.05
  --random-safe-finish-weight-end 1.30
  --random-safe-finish-min-neighbor-separation 2.65
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-offroute-finish-weight 2.60
  --random-offroute-finish-weight-end 3.40
  --policy-anchor-weight 230.0
  --policy-anchor-weight-end 285.0
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH228_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh