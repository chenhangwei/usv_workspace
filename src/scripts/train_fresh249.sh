#!/bin/bash
# fresh249: close-standon gate plus light local brake from fresh244 step2520.
#
# Rationale:
#   - fresh245/fresh246/fresh248 showed that stronger yield or generic local
#     braking can move the collision between 1461 and 1458.
#   - The underlying conflict is that stand-on role shaping still pushes forward
#     speed inside the near-collision band.
#   - This run uses the new close-standon gates to disable that speed pressure
#     only near neighbours, while otherwise preserving fresh244's balanced role
#     behavior and adding only a light local emergency brake.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh244_checkpoints/fresh244_balanced_yield_from_fresh240_step_0002520.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh249_close_standon_gate_from_fresh244_step2520.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh249_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh249_train.log}"
RUN_NAME="${RUN_NAME:-fresh249}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2520}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}"

FRESH249_ARGS="
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
  --learning-rate 4.8e-8
  --learning-rate-end 1.8e-8
  --clip-range 0.000028
  --ppo-policy-loss-scale 0.0007
  --ppo-value-loss-scale 0.008
  --value-coef 0.004
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --random-deconflict-weight 5.35
  --random-deconflict-weight-end 5.75
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-standon-close-separation 2.45
  --random-deconflict-standon-close-speed 0.08
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-omega-weight 1.42
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
  --random-role-balance-standon-close-separation 3.00
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.22
  --random-role-balance-standon-weight 0.22
  --random-role-balance-yield-weight 2.65
  --random-role-balance-omega-weight 0.52
  --team-safety-brake-weight 0.18
  --team-safety-brake-weight-end 0.14
  --team-safety-brake-goal-tolerance 0.8
  --team-safety-brake-max-distance 13.0
  --team-safety-brake-phase-min -1.0
  --team-safety-brake-min-team-completion 0.0
  --team-safety-brake-max-team-completion 0.999
  --team-safety-brake-near-team-tolerance 0.0
  --team-safety-brake-safe-separation 0.92
  --team-safety-brake-release-separation 1.70
  --team-safety-brake-target-speed 0.08
  --team-safety-brake-omega-weight 0.16
  --team-safety-brake-target-omega 0.20
  --team-safety-brake-turn-mode away
  --team-safety-brake-require-neighbor
  --team-safety-brake-local-danger
  --team-safety-brake-power 1.25
  --team-safety-brake-random-only
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
  --policy-anchor-exclude-team-safety-brake
  --policy-anchor-exclude-random-cte-recovery
"

export BASE_INPUT OUTPUT CKPT_DIR LOG_FILE RUN_NAME TOTAL_TIMESTEPS BASE_ROS_DOMAIN_ID
export EXTRA_TRAIN_ARGS="${FRESH249_ARGS} ${EXTRA_TRAIN_ARGS:-}"

exec ./scripts/train_fresh172.sh
