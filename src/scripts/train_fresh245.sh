#!/bin/bash
# fresh245: narrow 1461 yield-speed repair from fresh244 step2520.
#
# Rationale:
#   - fresh244 step2520 keeps 1458 collision-free, but 1461 still collides just
#     below the 0.75m boundary.
#   - Trace diagnostics show random_deconflict is active on the 1461 yield
#     agent, but the give-way speed remains above the target deep into the close
#     pass.  Omega is already large enough, so this run strengthens speed
#     compliance without returning to fresh243's hard-yield/stand-on starvation.
#   - Keep 1458 in the curriculum as a guard seed so the repair is rejected early
#     if it reintroduces the previous stand-on regression.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh244_checkpoints/fresh244_balanced_yield_from_fresh240_step_0002520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh245_yield_speed_repair_from_fresh244_step2520.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh245_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh245_train.log}"
RUN_NAME="${RUN_NAME:-fresh245}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-224}"

FRESH245_ARGS="
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
  --learning-rate 6.0e-8
  --learning-rate-end 2.5e-8
  --clip-range 0.000035
  --ppo-policy-loss-scale 0.0009
  --ppo-value-loss-scale 0.010
  --value-coef 0.005
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 5.85
  --random-deconflict-weight-end 6.35
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-safe-separation 1.66
  --random-deconflict-release-separation 3.08
  --random-deconflict-dcpa-target 2.10
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.020
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.44
  --random-deconflict-omega-weight 1.45
  --random-deconflict-standon-weight 0.18
  --random-deconflict-yield-weight 6.40
  --random-deconflict-pretrain-epochs 3
  --random-deconflict-pretrain-learning-rate 1.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.040
  --random-role-balance-weight 5.90
  --random-role-balance-weight-end 6.60
  --random-role-balance-pretrain-epochs 2
  --random-role-balance-pretrain-learning-rate 5.5e-7
  --random-role-balance-pretrain-max-grad-norm 0.040
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.035
  --random-role-balance-yield-min-starboard-omega 0.22
  --random-role-balance-standon-weight 0.28
  --random-role-balance-yield-weight 3.00
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
  --random-safe-finish-weight 0.82
  --random-safe-finish-weight-end 0.96
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 2.05
  --random-offroute-finish-weight-end 2.60
  --policy-anchor-weight 720.0
  --policy-anchor-weight-end 920.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH245_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
