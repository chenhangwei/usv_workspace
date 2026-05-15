#!/bin/bash
# fresh243: hard-yield close-range repair from fresh240 final.
#
# fresh242 confirmed the 1461 failure is not a mask coverage issue:
# usv_01/usv_02 both enter random_deconflict near 1.4m, but the yielding
# usv_02 still carries ~0.08-0.10m/s at the 0.75m collision.  This variant
# keeps the repair narrow and makes the yield target harder while avoiding
# any stand-on speed-up pressure.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh240_yield_cte_from_fresh228.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh243_hard_yield_from_fresh240.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh243_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh243_train.log}"
RUN_NAME="${RUN_NAME:-fresh243}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}"

FRESH243_ARGS="
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
  --force-actor-log-std -2.30
  --learning-rate 1.25e-7
  --learning-rate-end 5.0e-8
  --clip-range 0.000055
  --ppo-policy-loss-scale 0.0010
  --ppo-value-loss-scale 0.010
  --value-coef 0.005
  --update-epochs 3
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 7.20
  --random-deconflict-weight-end 8.60
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-safe-separation 1.70
  --random-deconflict-release-separation 3.20
  --random-deconflict-dcpa-target 2.10
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-yield-speed 0.000
  --random-deconflict-yield-omega 0.58
  --random-deconflict-omega-weight 2.10
  --random-deconflict-standon-speed 0.08
  --random-deconflict-standon-omega 0.04
  --random-deconflict-standon-weight 0.000
  --random-deconflict-yield-weight 8.50
  --random-deconflict-pretrain-epochs 5
  --random-deconflict-pretrain-learning-rate 2.2e-6
  --random-deconflict-pretrain-max-grad-norm 0.080
  --random-role-balance-weight 5.40
  --random-role-balance-weight-end 6.30
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 7.5e-7
  --random-role-balance-pretrain-max-grad-norm 0.055
  --random-role-balance-min-danger 0.03
  --random-role-balance-standon-min-speed 0.00
  --random-role-balance-yield-max-speed 0.025
  --random-role-balance-yield-min-starboard-omega 0.30
  --random-role-balance-standon-weight 0.00
  --random-role-balance-yield-weight 3.60
  --random-role-balance-omega-weight 0.80
  --random-cte-recovery-weight 0.25
  --random-cte-recovery-weight-end 0.35
  --random-cte-recovery-min-abs-cte 1.80
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.80
  --random-cte-recovery-target-speed 0.10
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.40
  --random-safe-finish-weight 0.70
  --random-safe-finish-weight-end 0.85
  --random-safe-finish-min-neighbor-separation 2.90
  --random-safe-finish-target-speed 0.18
  --random-safe-finish-low-priority-speed-multiplier 1.00
  --random-offroute-finish-weight 1.80
  --random-offroute-finish-weight-end 2.25
  --policy-anchor-weight 700.0
  --policy-anchor-weight-end 900.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH243_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh