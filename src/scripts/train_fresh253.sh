#!/bin/bash
# fresh253: explicit route-ETA pass-order roles from fresh244 step2520.
#
# Rationale:
#   - priority-delta under --random-encounter-route-priority still mixes discrete
#     crossing_priority ranks; route-eta-delta yields only when the *active*
#     neighbor feature route_eta_delta shows ego arrives later at the shared
#     conflict point (positive normalized own_eta - neighbor_eta).
#   - Keeps fresh244 hyperparameters and starboard deconflict turns; only the
#     role gate and route midpoint annotation change.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh244_checkpoints/fresh244_balanced_yield_from_fresh240_step_0002520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh253_route_eta_roles_from_fresh244_step2520.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh253_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh253_train.log}"
RUN_NAME="${RUN_NAME:-fresh253}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-231}"

FRESH253_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1463
  --random-encounter-route-priority
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.28
  --learning-rate 9.0e-8
  --learning-rate-end 3.5e-8
  --clip-range 0.000045
  --ppo-policy-loss-scale 0.0012
  --ppo-value-loss-scale 0.015
  --value-coef 0.007
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 5.40
  --random-deconflict-weight-end 5.85
  --random-deconflict-role-mode route-eta-delta
  --random-deconflict-route-eta-yield-threshold 0.02
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.48
  --random-deconflict-omega-weight 1.65
  --random-deconflict-standon-weight 0.18
  --random-deconflict-yield-weight 5.80
  --random-deconflict-pretrain-epochs 2
  --random-deconflict-pretrain-learning-rate 1.2e-6
  --random-deconflict-pretrain-max-grad-norm 0.045
  --random-role-balance-weight 5.60
  --random-role-balance-weight-end 6.20
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 5.0e-7
  --random-role-balance-pretrain-max-grad-norm 0.045
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.24
  --random-role-balance-standon-weight 0.28
  --random-role-balance-yield-weight 2.60
  --random-role-balance-omega-weight 0.65
  --random-cte-recovery-weight 0.40
  --random-cte-recovery-weight-end 0.55
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.40
  --random-safe-finish-weight 0.85
  --random-safe-finish-weight-end 1.00
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 2.10
  --random-offroute-finish-weight-end 2.70
  --policy-anchor-weight 620.0
  --policy-anchor-weight-end 780.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH253_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
