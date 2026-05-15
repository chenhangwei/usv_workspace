#!/bin/bash
# fresh255: critical-distance local danger repair from fresh254 final.
#
# Rationale:
#   - fresh254 final is broadly safer, but seed 1460 still collides because
#     local danger is diluted by closing-speed gating even below 1 m separation.
#   - This run enables the code-gated critical local danger and caps stand-on
#     close speed only inside that critical band, while anchoring to fresh254.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh254_route_eta_away_from_fresh244_step2520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh255_critical_local_from_fresh254.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh255_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh255_train.log}"
RUN_NAME="${RUN_NAME:-fresh255}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-3780}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}"

FRESH255_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1460
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --random-encounter-route-priority
  --force-actor-log-std -2.28
  --learning-rate 5.5e-8
  --learning-rate-end 2.0e-8
  --clip-range 0.000030
  --ppo-policy-loss-scale 0.0008
  --ppo-value-loss-scale 0.008
  --value-coef 0.004
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 6.00
  --random-deconflict-weight-end 6.60
  --random-deconflict-role-mode route-eta-delta
  --random-deconflict-route-eta-yield-threshold 0.02
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-critical-separation 1.05
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-standon-close-separation 1.05
  --random-deconflict-standon-close-speed 0.08
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.60
  --random-deconflict-standon-weight 0.22
  --random-deconflict-yield-weight 6.40
  --random-deconflict-pretrain-epochs 2
  --random-deconflict-pretrain-learning-rate 1.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.040
  --random-role-balance-weight 5.80
  --random-role-balance-weight-end 6.30
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 4.5e-7
  --random-role-balance-pretrain-max-grad-norm 0.040
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-standon-close-separation 1.05
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.20
  --random-role-balance-standon-weight 0.28
  --random-role-balance-yield-weight 2.80
  --random-role-balance-omega-weight 0.55
  --random-cte-recovery-weight 0.38
  --random-cte-recovery-weight-end 0.50
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.38
  --random-safe-finish-weight 0.80
  --random-safe-finish-weight-end 0.95
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 1.90
  --random-offroute-finish-weight-end 2.40
  --policy-anchor-weight 1200.0
  --policy-anchor-weight-end 1600.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH255_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
