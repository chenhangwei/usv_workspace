#!/bin/bash
# fresh258: mutually-exclusive clear-ahead route discipline from fresh254 final.
#
# Rationale:
#   - fresh257's forward-cone clear-ahead loss activated on 64% of random samples
#     and preserved most safety, but it could overlap with local deconflict.
#   - This run only applies clear-ahead when the goal-direction cone is clear,
#     CPA/local danger are absent, random-deconflict is inactive, and all-around
#     nearest-neighbor separation is comfortably outside the local conflict band.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh254_route_eta_away_from_fresh244_step2520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh258_clear_mutex_from_fresh254.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh258_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh258_train.log}"
RUN_NAME="${RUN_NAME:-fresh258}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}"

FRESH258_ARGS="
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
  --learning-rate 2.5e-8
  --learning-rate-end 1.0e-8
  --clip-range 0.000016
  --ppo-policy-loss-scale 0.0004
  --ppo-value-loss-scale 0.005
  --value-coef 0.003
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --clear-ahead-distance 5.0
  --clear-ahead-bearing-deg 35.0
  --clear-ahead-cte-weight 0.30
  --clear-ahead-heading-weight 0.14
  --clear-ahead-omega-weight 0.45
  --random-clear-ahead-weight 2.80
  --random-clear-ahead-weight-end 3.60
  --random-clear-ahead-distance 5.0
  --random-clear-ahead-bearing-deg 35.0
  --random-clear-ahead-cone-mode goal
  --random-clear-ahead-goal-tolerance 1.0
  --random-clear-ahead-max-distance 13.0
  --random-clear-ahead-min-neighbor-separation 2.95
  --random-clear-ahead-max-cpa-score 0.0
  --random-clear-ahead-max-local-score 0.0
  --random-clear-ahead-exclude-deconflict
  --random-clear-ahead-target-source raw
  --random-clear-ahead-target-speed 0.30
  --random-clear-ahead-min-speed 0.18
  --random-clear-ahead-max-omega 0.16
  --random-clear-ahead-omega-weight 1.15
  --random-deconflict-weight 5.40
  --random-deconflict-weight-end 5.80
  --random-deconflict-role-mode route-eta-delta
  --random-deconflict-route-eta-yield-threshold 0.02
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-critical-separation 0.0
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.45
  --random-deconflict-standon-weight 0.18
  --random-deconflict-yield-weight 5.80
  --random-deconflict-pretrain-epochs 1
  --random-deconflict-pretrain-learning-rate 5.0e-7
  --random-deconflict-pretrain-max-grad-norm 0.030
  --random-role-balance-weight 5.30
  --random-role-balance-weight-end 5.70
  --random-role-balance-pretrain-epochs 1
  --random-role-balance-pretrain-learning-rate 3.0e-7
  --random-role-balance-pretrain-max-grad-norm 0.030
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.20
  --random-role-balance-standon-weight 0.28
  --random-role-balance-yield-weight 2.60
  --random-role-balance-omega-weight 0.48
  --random-cte-recovery-weight 0.32
  --random-cte-recovery-weight-end 0.40
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.34
  --random-safe-finish-weight 0.80
  --random-safe-finish-weight-end 0.92
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 1.75
  --random-offroute-finish-weight-end 2.10
  --policy-anchor-weight 1600.0
  --policy-anchor-weight-end 2200.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH258_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
