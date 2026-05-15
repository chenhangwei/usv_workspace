#!/bin/bash
# fresh250: away-turn random deconflict from fresh244 step2520.
#
# Rationale:
#   - fixed starboard random-deconflict omega can be wrong in arbitrary random
#     geometry; 1458 failures show the yield side sometimes needs to turn away
#     from the active neighbour rather than always to starboard.
#   - fresh245-249 proved stronger speed caps/brakes can save one seed while
#     breaking the other, so this run avoids those extra pressures.
#   - Keep fresh244's balanced role basin and change only random-deconflict turn
#     direction to the new away mode.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh244_checkpoints/fresh244_balanced_yield_from_fresh240_step_0002520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh250_away_deconf_from_fresh244_step2520.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh250_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh250_train.log}"
RUN_NAME="${RUN_NAME:-fresh250}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-232}"

FRESH250_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1463
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.28
  --learning-rate 5.0e-8
  --learning-rate-end 2.0e-8
  --clip-range 0.000030
  --ppo-policy-loss-scale 0.0007
  --ppo-value-loss-scale 0.008
  --value-coef 0.004
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 5.30
  --random-deconflict-weight-end 5.75
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.45
  --random-deconflict-standon-weight 0.16
  --random-deconflict-yield-weight 5.65
  --random-deconflict-pretrain-epochs 2
  --random-deconflict-pretrain-learning-rate 7.0e-7
  --random-deconflict-pretrain-max-grad-norm 0.035
  --random-role-balance-weight 5.05
  --random-role-balance-weight-end 5.65
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 3.8e-7
  --random-role-balance-pretrain-max-grad-norm 0.035
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.22
  --random-role-balance-standon-weight 0.22
  --random-role-balance-yield-weight 2.65
  --random-role-balance-omega-weight 0.52
  --random-cte-recovery-weight 0.34
  --random-cte-recovery-weight-end 0.44
  --random-cte-recovery-min-abs-cte 1.80
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.85
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.34
  --random-safe-finish-weight 0.82
  --random-safe-finish-weight-end 0.96
  --random-safe-finish-min-neighbor-separation 2.90
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.08
  --random-offroute-finish-weight 1.95
  --random-offroute-finish-weight-end 2.50
  --policy-anchor-weight 880.0
  --policy-anchor-weight-end 1120.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH250_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
