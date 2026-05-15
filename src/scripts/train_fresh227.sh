#!/bin/bash
# fresh227: fixed-seed hard replay with active random-deconf actor pretrain.
#
# Rationale:
#   - fresh226 proved the seed replay plumbing works, but the scenario trunk moved
#     only ~1e-6, so 1461/1463 stayed on the fresh211 action pattern.
#   - Use the same hard-seed rollout data, then run a small actor-only deconf pass
#     on active random-encounter samples from those rollouts.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh211_checkpoints/fresh211_dual_threat_yield_from_fresh208.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh227_seed_replay_deconf_pretrain_from_fresh211.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh227_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh227_train.log}"
RUN_NAME="${RUN_NAME:-fresh227}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

FRESH227_ARGS="
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
  --force-actor-log-std -2.25
  --learning-rate 3.0e-7
  --learning-rate-end 1.0e-7
  --clip-range 0.00012
  --ppo-policy-loss-scale 0.004
  --ppo-value-loss-scale 0.060
  --value-coef 0.020
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-yield-danger-scale 0.95
  --random-deconflict-yield-omega 0.34
  --random-deconflict-omega-weight 0.34
  --random-deconflict-yield-weight 3.80
  --random-deconflict-pretrain-epochs 5
  --random-deconflict-pretrain-learning-rate 6.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.18
  --random-safe-finish-weight 1.05
  --random-safe-finish-weight-end 1.30
  --random-safe-finish-min-neighbor-separation 2.65
  --random-safe-finish-target-speed 0.205
  --random-safe-finish-low-priority-speed-multiplier 1.18
  --random-offroute-finish-weight 2.60
  --random-offroute-finish-weight-end 3.40
  --policy-anchor-weight 210.0
  --policy-anchor-weight-end 255.0
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH227_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh