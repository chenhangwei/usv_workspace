#!/bin/bash
# fresh226: fixed-seed hard random encounter replay from fresh211.
#
# Rationale:
#   - fresh225 real rollout stayed on the fresh211 failure pattern: seeds 1461 and
#     1463 still collide while the safer seeds remain mostly intact.
#   - Replay the exact hard random geometries in PPO rollouts so the policy sees
#     the online failure states directly, with safe seeds kept in the cycle as anchors.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh211_checkpoints/fresh211_dual_threat_yield_from_fresh208.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh226_seed_replay_from_fresh211.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh226_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh226_train.log}"
RUN_NAME="${RUN_NAME:-fresh226}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5040}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH226_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1461
  --curriculum-seed 1463
  --curriculum-seed 1461
  --curriculum-seed 1463
  --curriculum-seed 1458
  --curriculum-seed 1459
  --curriculum-seed 1460
  --curriculum-seed 1462
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --force-actor-log-std -2.20
  --learning-rate 2.0e-7
  --learning-rate-end 6.0e-8
  --clip-range 0.00010
  --ppo-policy-loss-scale 0.010
  --ppo-value-loss-scale 0.080
  --value-coef 0.030
  --update-epochs 3
  --per-scenario-advantage-norm
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 0.95
  --random-deconflict-yield-omega 0.34
  --random-deconflict-omega-weight 0.32
  --random-deconflict-yield-weight 3.70
  --random-safe-finish-weight 1.05
  --random-safe-finish-weight-end 1.30
  --random-safe-finish-min-neighbor-separation 2.65
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-offroute-finish-weight 2.60
  --random-offroute-finish-weight-end 3.40
  --policy-anchor-weight 180.0
  --policy-anchor-weight-end 240.0
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH226_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh