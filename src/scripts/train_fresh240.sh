#!/bin/bash
# fresh240: yield-only role cap plus CTE recovery from fresh228.
#
# Rationale:
#   - fresh239 delayed the 1458 collision but did not remove it, and it degraded
#     1460/1461 by pushing stand-on agents into high-CTE late states.
#   - This run keeps the fresh228 deconflict basin, removes stand-on speed-floor
#     pressure from role-balance, and adds a threat-overlap CTE recovery target
#     so late high-CTE agents slow and turn back instead of continuing off-route.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh228_full_seed_yield_omega_from_fresh211.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh240_yield_cte_from_fresh228.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh240_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh240_train.log}"
RUN_NAME="${RUN_NAME:-fresh240}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-8820}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH240_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1461
  --curriculum-seed 1463
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.25
  --learning-rate 1.8e-7
  --learning-rate-end 6.0e-8
  --clip-range 0.00008
  --ppo-policy-loss-scale 0.003
  --ppo-value-loss-scale 0.050
  --value-coef 0.018
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 4.65
  --random-deconflict-weight-end 4.20
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-yield-omega 0.40
  --random-deconflict-omega-weight 1.55
  --random-deconflict-standon-weight 0.035
  --random-deconflict-yield-weight 4.70
  --random-deconflict-pretrain-epochs 4
  --random-deconflict-pretrain-learning-rate 3.2e-6
  --random-deconflict-pretrain-max-grad-norm 0.11
  --random-role-balance-weight 4.0
  --random-role-balance-weight-end 5.2
  --random-role-balance-pretrain-epochs 3
  --random-role-balance-pretrain-learning-rate 8.5e-7
  --random-role-balance-pretrain-max-grad-norm 0.07
  --random-role-balance-min-danger 0.05
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.060
  --random-role-balance-yield-min-starboard-omega 0.18
  --random-role-balance-standon-weight 0.00
  --random-role-balance-yield-weight 2.40
  --random-role-balance-omega-weight 0.50
  --random-cte-recovery-weight 1.10
  --random-cte-recovery-weight-end 1.80
  --random-cte-recovery-min-abs-cte 1.45
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 0.90
  --random-cte-recovery-allow-threat-overlap
  --random-cte-recovery-target-speed 0.14
  --random-cte-recovery-min-speed 0.045
  --random-cte-recovery-max-omega 0.22
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.70
  --random-safe-finish-weight 1.00
  --random-safe-finish-weight-end 1.25
  --random-safe-finish-min-neighbor-separation 2.68
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.12
  --random-offroute-finish-weight 2.70
  --random-offroute-finish-weight-end 3.45
  --policy-anchor-weight 265.0
  --policy-anchor-weight-end 335.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH240_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh