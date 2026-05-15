#!/bin/bash
# fresh241: release over-constrained yield/CTE behavior from fresh240 final.
#
# Rationale:
#   - fresh240 final fixed early 1458/1460/1463 collisions but became too
#     conservative and still collided late on 1461.
#   - fresh240 step_7560 restored 1460 progress but made 1461 collide earlier,
#     so this run starts from fresh240 final, not the step checkpoint.
#   - The repair relaxes yield-only role caps, prevents CTE recovery from
#     overriding close-range deconflict too aggressively, and adds a mild
#     anti-entanglement signal to reduce low-speed orbital stalls.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh240_yield_cte_from_fresh228.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh241_release_finish_from_fresh240.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh241_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh241_train.log}"
RUN_NAME="${RUN_NAME:-fresh241}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-8820}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH241_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1463
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.25
  --learning-rate 1.2e-7
  --learning-rate-end 4.0e-8
  --clip-range 0.00006
  --ppo-policy-loss-scale 0.0025
  --ppo-value-loss-scale 0.045
  --value-coef 0.016
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --entanglement-penalty-weight 1.10
  --entanglement-distance 2.80
  --entanglement-grace-steps 70
  --entanglement-low-speed-penalty-weight 1.80
  --random-deconflict-weight 4.50
  --random-deconflict-weight-end 4.85
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-yield-omega 0.38
  --random-deconflict-omega-weight 1.35
  --random-deconflict-standon-weight 0.030
  --random-deconflict-yield-weight 4.75
  --random-deconflict-pretrain-epochs 3
  --random-deconflict-pretrain-learning-rate 2.4e-6
  --random-deconflict-pretrain-max-grad-norm 0.10
  --random-role-balance-weight 1.80
  --random-role-balance-weight-end 2.40
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 4.0e-7
  --random-role-balance-pretrain-max-grad-norm 0.05
  --random-role-balance-min-danger 0.06
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.090
  --random-role-balance-yield-min-starboard-omega 0.14
  --random-role-balance-standon-weight 0.00
  --random-role-balance-yield-weight 1.00
  --random-role-balance-omega-weight 0.30
  --random-cte-recovery-weight 1.20
  --random-cte-recovery-weight-end 1.55
  --random-cte-recovery-min-abs-cte 1.55
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.15
  --random-cte-recovery-allow-threat-overlap
  --random-cte-recovery-target-speed 0.17
  --random-cte-recovery-min-speed 0.070
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.60
  --random-cte-recovery-omega-weight 0.50
  --random-safe-finish-weight 1.25
  --random-safe-finish-weight-end 1.70
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.215
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-safe-finish-yield-release-min-separation 2.95
  --random-safe-finish-yield-release-max-closing-speed 0.00
  --random-safe-finish-yield-release-min-route-progress 0.45
  --random-offroute-finish-weight 2.20
  --random-offroute-finish-weight-end 2.80
  --random-offroute-finish-min-neighbor-separation 2.95
  --random-offroute-finish-target-speed 0.16
  --random-offroute-finish-min-speed 0.075
  --random-offroute-finish-max-omega 0.16
  --random-offroute-finish-omega-weight 0.45
  --policy-anchor-weight 360.0
  --policy-anchor-weight-end 430.0
  --policy-anchor-exclude-random-cte-recovery
  --policy-anchor-exclude-random-offroute-finish
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH241_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh