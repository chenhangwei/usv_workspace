#!/bin/bash
# fresh303: overtaking role-imitation repair from fresh301.
#
# Direction:
#   - Do not use fresh302 as base because it regressed random hard seeds.
#   - Apply an explicit overtaking target: front vessel follows raw route,
#     rear vessel slows and opens starboard corridor before passing.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh301_two_vessel_stageC_from_fresh300.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh303_overtaking_imitation_from_fresh301.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh303_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1680}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-225}"

FRESH303_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 840
  --scenario solo_navigation
  --scenario three_usv_clear_route
  --scenario two_usv_head_on
  --scenario two_usv_crossing
  --scenario two_usv_overtaking
  --scenario three_usv_crossing
  --scenario three_usv_overtaking
  --scenario two_usv_random_encounter
  --scenario three_usv_random_encounter
  --curriculum-scenario two_usv_overtaking
  --curriculum-scenario two_usv_overtaking
  --curriculum-scenario two_usv_crossing
  --curriculum-scenario three_usv_random_encounter
  --curriculum-seed 3103
  --curriculum-seed 3104
  --curriculum-seed 3102
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1461
  --scenario-spawn-position-std 0.025
  --scenario-spawn-heading-std 0.012
  --scenario-goal-position-std 0.02
  --random-encounter-route-priority
  --force-actor-log-std -2.34
  --learning-rate 1.5e-8
  --learning-rate-end 7.0e-9
  --clip-range 0.000010
  --ppo-policy-loss-scale 0.00025
  --ppo-value-loss-scale 0.003
  --value-coef 0.002
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --overtaking-imitation-weight 8.0
  --overtaking-imitation-weight-end 10.0
  --overtaking-imitation-max-distance 13.0
  --overtaking-imitation-front-min-speed 0.24
  --overtaking-imitation-front-max-omega 0.08
  --overtaking-imitation-rear-approach-speed 0.11
  --overtaking-imitation-rear-close-speed 0.035
  --overtaking-imitation-rear-pass-speed 0.22
  --overtaking-imitation-close-separation 2.40
  --overtaking-imitation-release-separation 6.00
  --overtaking-imitation-target-starboard-offset 1.25
  --overtaking-imitation-rear-omega 0.30
  --overtaking-imitation-omega-weight 1.40
  --speed-distance-coupling-penalty-weight 0.65
  --speed-distance-coupling-threshold 2.40
  --proximity-gradient-penalty-weight 0.30
  --proximity-gradient-distance 1.80
  --conflict-overspeed-penalty-weight 0.35
  --overtaking-starboard-turn-reward-weight 0.90
  --overtaking-forward-reward-weight 0.30
  --overtaking-corridor-reward-weight 1.20
  --overtaking-centerline-penalty-weight 1.30
  --overtaking-close-penalty-weight 1.40
  --crossing-starboard-turn-reward-weight 0.95
  --crossing-forward-reward-weight 0.45
  --crossing-close-forward-penalty-weight 1.0
  --crossing-time-separation-reward-weight 0.60
  --crossing-time-separation-penalty-weight 0.80
  --clear-ahead-distance 5.0
  --clear-ahead-bearing-deg 35.0
  --clear-ahead-cte-weight 0.60
  --clear-ahead-heading-weight 0.22
  --clear-ahead-omega-weight 0.80
  --straight-line-omega-penalty-weight 3.0
  --straight-line-omega-conflict-floor 0.08
  --straight-line-omega-cte-gate 0.70
  --random-clear-ahead-weight 3.80
  --random-clear-ahead-weight-end 4.80
  --random-clear-ahead-scenario three_usv_clear_route
  --random-clear-ahead-scenario solo_navigation
  --random-clear-ahead-distance 5.0
  --random-clear-ahead-bearing-deg 35.0
  --random-clear-ahead-cone-mode goal
  --random-clear-ahead-goal-tolerance 1.0
  --random-clear-ahead-max-distance 13.0
  --random-clear-ahead-min-neighbor-separation 2.75
  --random-clear-ahead-max-cpa-score 0.0
  --random-clear-ahead-max-local-score 0.0
  --random-clear-ahead-exclude-deconflict
  --random-clear-ahead-target-source raw
  --random-clear-ahead-target-speed 0.30
  --random-clear-ahead-min-speed 0.20
  --random-clear-ahead-max-omega 0.14
  --random-clear-ahead-omega-weight 1.20
  --random-deconflict-weight 4.40
  --random-deconflict-weight-end 4.90
  --random-deconflict-role-mode route-eta-delta
  --random-deconflict-route-eta-yield-threshold 0.02
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-dcpa-target 2.05
  --random-deconflict-yield-danger-scale 1.00
  --random-deconflict-standon-speed 0.22
  --random-deconflict-yield-speed 0.035
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.46
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.20
  --random-deconflict-standon-weight 0.18
  --random-deconflict-yield-weight 4.80
  --random-deconflict-pretrain-epochs 1
  --random-deconflict-pretrain-learning-rate 2.5e-7
  --random-deconflict-pretrain-max-grad-norm 0.025
  --random-role-balance-weight 4.20
  --random-role-balance-weight-end 4.70
  --random-role-balance-pretrain-epochs 1
  --random-role-balance-pretrain-learning-rate 2.2e-7
  --random-role-balance-pretrain-max-grad-norm 0.025
  --random-role-balance-min-danger 0.04
  --random-role-balance-standon-min-speed 0.18
  --random-role-balance-yield-max-speed 0.045
  --random-role-balance-yield-min-starboard-omega 0.20
  --random-role-balance-standon-weight 0.20
  --random-role-balance-yield-weight 2.20
  --random-role-balance-omega-weight 0.36
  --random-cte-recovery-weight 0.25
  --random-cte-recovery-weight-end 0.32
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.11
  --random-cte-recovery-min-speed 0.040
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.28
  --random-safe-finish-weight 0.65
  --random-safe-finish-weight-end 0.80
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-offroute-finish-weight 1.40
  --random-offroute-finish-weight-end 1.70
  --policy-anchor-weight 2600.0
  --policy-anchor-weight-end 3600.0
  --policy-anchor-exclude-random-cte-recovery
"

mkdir -p "$CKPT_DIR"
source ../install/setup.bash

exec /bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --num-agents 3 \
  --rollout-steps 420 \
  --reset-sampler-each-rollout \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.40 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 \
  --hidden-size 256 \
  --actor-log-std-init -2.34 \
  --max-agents 5 \
  --max-neighbors 4 \
  --squash-actions \
  --min-forward-speed 0.0 \
  --linear-delta-limit 0.30 \
  --angular-delta-limit 0.50 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.50 \
  --min-forward-speed-floor 0.0 \
  --heading-omega-deadband 0.09 \
  --heading-omega-reference 0.95 \
  --angular-authority-power 1.30 \
  --angular-accel-limit 1.05 \
  --angular-decel-limit 3.00 \
  --conflict-turn-relief 0.80 \
  --angular-authority-floor 0.45 \
  --episode-timeout 145.0 \
  --no-progress-timeout 108.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --freeze-observation-normalizer \
  --scenario-balanced-loss \
  --encounter-type-dropout 0.00 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --separate-actor-critic-grad-clip \
  $FRESH303_ARGS
