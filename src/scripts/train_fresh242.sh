#!/bin/bash
# fresh242: close-range deconflict micro-repair from fresh240 final.
#
# Rationale:
#   - fresh240 final keeps 1458/1460/1463 collision-free but still collides
#     late on 1461 with separation right at the collision boundary.
#   - fresh241 and its early checkpoints broke 1458 immediately, so this run
#     avoids the fresh241 release/entanglement changes and keeps the fresh240
#     role/off-route basin.
#   - The repair is intentionally narrow: CTE recovery no longer overlaps
#     active threats, while close-range deconflict gets a small buffer increase
#     under very small learning-rate updates.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh240_yield_cte_from_fresh228.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh242_close_deconf_from_fresh240.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh242_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh242_train.log}"
RUN_NAME="${RUN_NAME:-fresh242}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-3780}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH242_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1460
  --curriculum-seed 1463
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.25
  --learning-rate 7.0e-8
  --learning-rate-end 2.5e-8
  --clip-range 0.000035
  --ppo-policy-loss-scale 0.0015
  --ppo-value-loss-scale 0.025
  --value-coef 0.010
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 4.80
  --random-deconflict-weight-end 5.15
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-safe-separation 1.48
  --random-deconflict-release-separation 2.82
  --random-deconflict-dcpa-target 2.00
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-yield-speed 0.050
  --random-deconflict-yield-omega 0.40
  --random-deconflict-omega-weight 1.45
  --random-deconflict-standon-weight 0.030
  --random-deconflict-yield-weight 4.95
  --random-deconflict-pretrain-epochs 2
  --random-deconflict-pretrain-learning-rate 1.1e-6
  --random-deconflict-pretrain-max-grad-norm 0.055
  --random-role-balance-weight 4.00
  --random-role-balance-weight-end 4.85
  --random-role-balance-pretrain-epochs 1
  --random-role-balance-pretrain-learning-rate 3.0e-7
  --random-role-balance-pretrain-max-grad-norm 0.040
  --random-role-balance-min-danger 0.05
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.060
  --random-role-balance-yield-min-starboard-omega 0.18
  --random-role-balance-standon-weight 0.00
  --random-role-balance-yield-weight 2.20
  --random-role-balance-omega-weight 0.45
  --random-cte-recovery-weight 0.85
  --random-cte-recovery-weight-end 1.15
  --random-cte-recovery-min-abs-cte 1.55
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.35
  --random-cte-recovery-target-speed 0.12
  --random-cte-recovery-min-speed 0.045
  --random-cte-recovery-max-omega 0.20
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.55
  --random-safe-finish-weight 0.95
  --random-safe-finish-weight-end 1.15
  --random-safe-finish-min-neighbor-separation 2.70
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.10
  --random-offroute-finish-weight 2.70
  --random-offroute-finish-weight-end 3.45
  --policy-anchor-weight 520.0
  --policy-anchor-weight-end 700.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH242_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh