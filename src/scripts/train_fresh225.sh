#!/bin/bash
# fresh225: real-rollout hard random encounter curriculum from the broader fresh211 safety baseline.
#
# Rationale:
#   - fresh222/fresh223/fresh224 showed that local trace-fit guard/deconf updates move
#     collisions between seeds instead of generalizing.
#   - Use fresh211 as the safer base and let PPO see varied random-encounter rollouts,
#     while keeping the existing strong policy anchor and conservative runtime shape.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh211_checkpoints/fresh211_dual_threat_yield_from_fresh208.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh225_hard_curriculum_from_fresh211.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh225_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh225_train.log}"
RUN_NAME="${RUN_NAME:-fresh225}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH225_ARGS="
  --checkpoint-interval 840
  --learning-rate 1.4e-7
  --learning-rate-end 5.0e-8
  --clip-range 0.00008
  --ppo-policy-loss-scale 0.004
  --ppo-value-loss-scale 0.050
  --value-coef 0.020
  --update-epochs 2
  --per-scenario-advantage-norm
  --scenario-spawn-position-std 0.14
  --scenario-spawn-heading-std 0.07
  --scenario-goal-position-std 0.08
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 0.92
  --random-deconflict-yield-omega 0.32
  --random-deconflict-omega-weight 0.34
  --random-deconflict-yield-weight 3.80
  --random-safe-finish-weight 1.05
  --random-safe-finish-weight-end 1.35
  --random-safe-finish-min-neighbor-separation 2.65
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-offroute-finish-weight 2.80
  --random-offroute-finish-weight-end 3.60
  --policy-anchor-weight 245.0
  --policy-anchor-weight-end 285.0
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario two_usv_random_encounter
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH225_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh