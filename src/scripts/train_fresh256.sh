#!/bin/bash
# fresh256: narrow critical pair-brake repair from fresh254 final.
#
# Rationale:
#   - fresh254 final is the best current baseline: hard-seed sweep only leaves
#     1460 colliding, while most other seeds are safe timeouts.
#   - fresh255's critical local danger inside random-deconflict was unstable and
#     regressed 1464.  This run leaves route-ETA deconflict mostly unchanged and
#     adds a separate, very narrow team_safety_brake that activates from the
#     start of random encounters when a real nearest-neighbor pair is inside
#     the critical band.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh254_route_eta_away_from_fresh244_step2520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh256_pair_brake_from_fresh254.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh256_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh256_train.log}"
RUN_NAME="${RUN_NAME:-fresh256}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}"

FRESH256_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --curriculum-seed 1460
  --curriculum-seed 1458
  --curriculum-seed 1461
  --curriculum-seed 1464
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --random-encounter-route-priority
  --force-actor-log-std -2.28
  --learning-rate 3.5e-8
  --learning-rate-end 1.5e-8
  --clip-range 0.000020
  --ppo-policy-loss-scale 0.0005
  --ppo-value-loss-scale 0.006
  --value-coef 0.003
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --team-safety-brake-weight 2.40
  --team-safety-brake-weight-end 3.20
  --team-safety-brake-random-only
  --team-safety-brake-local-danger
  --team-safety-brake-require-neighbor
  --team-safety-brake-min-team-completion 0.0
  --team-safety-brake-max-team-completion 0.999
  --team-safety-brake-safe-separation 1.05
  --team-safety-brake-release-separation 1.35
  --team-safety-brake-target-speed 0.02
  --team-safety-brake-target-omega 0.32
  --team-safety-brake-omega-weight 0.55
  --team-safety-brake-turn-mode away
  --team-safety-brake-power 1.0
  --random-deconflict-weight 5.55
  --random-deconflict-weight-end 5.90
  --random-deconflict-role-mode route-eta-delta
  --random-deconflict-route-eta-yield-threshold 0.02
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-critical-separation 0.0
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.45
  --random-deconflict-standon-weight 0.18
  --random-deconflict-yield-weight 5.80
  --random-deconflict-pretrain-epochs 1
  --random-deconflict-pretrain-learning-rate 6.0e-7
  --random-deconflict-pretrain-max-grad-norm 0.030
  --random-role-balance-weight 5.50
  --random-role-balance-weight-end 5.90
  --random-role-balance-pretrain-epochs 1
  --random-role-balance-pretrain-learning-rate 3.5e-7
  --random-role-balance-pretrain-max-grad-norm 0.030
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.20
  --random-role-balance-standon-weight 0.28
  --random-role-balance-yield-weight 2.60
  --random-role-balance-omega-weight 0.48
  --random-cte-recovery-weight 0.34
  --random-cte-recovery-weight-end 0.42
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.34
  --random-safe-finish-weight 0.80
  --random-safe-finish-weight-end 0.92
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 1.80
  --random-offroute-finish-weight-end 2.20
  --policy-anchor-weight 1600.0
  --policy-anchor-weight-end 2200.0
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH256_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
