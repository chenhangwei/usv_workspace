#!/bin/bash
# fresh318: hard random-encounter replay from the fresh313 single-overtake checkpoint.
#
# Goal:
#   - Keep fresh313 scripted-lead single overtaking intact.
#   - Repair three_usv_random_encounter hard seeds 1458/1460/1461 with rollout-level
#     role-balance pressure instead of another local trace-fit/blend.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh318_random_hard_replay_from_fresh313.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh318_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-5040}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}"

FRESH318_ARGS="
  --num-sampler-workers 1
  --checkpoint-interval 1260
  --scenario solo_navigation
  --scenario single_usv_overtaking
  --scenario three_usv_clear_route
  --scenario two_usv_head_on
  --scenario two_usv_crossing
  --scenario two_usv_overtaking
  --scenario three_usv_crossing
  --scenario three_usv_overtaking
  --scenario two_usv_random_encounter
  --scenario three_usv_random_encounter
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario three_usv_random_encounter
  --curriculum-scenario single_usv_overtaking
  --curriculum-scenario three_usv_clear_route
  --curriculum-scenario two_usv_head_on
  --curriculum-scenario two_usv_crossing
  --curriculum-seed 1458
  --curriculum-seed 1460
  --curriculum-seed 1461
  --curriculum-seed 1459
  --curriculum-seed 1462
  --curriculum-seed 1463
  --curriculum-seed 1464
  --curriculum-seed 3071
  --curriculum-seed 3072
  --curriculum-seed 3001
  --curriculum-seed 3101
  --scenario-spawn-position-std 0.0
  --scenario-spawn-heading-std 0.0
  --scenario-goal-position-std 0.0
  --random-encounter-route-priority
  --force-actor-log-std -2.34
  --learning-rate 1.2e-7
  --learning-rate-end 4.0e-8
  --clip-range 0.000075
  --ppo-policy-loss-scale 0.0025
  --ppo-value-loss-scale 0.035
  --value-coef 0.014
  --update-epochs 2
  --minibatch-size 210
  --per-scenario-advantage-norm
  --overtaking-starboard-turn-reward-weight 1.05
  --overtaking-forward-reward-weight 0.52
  --overtaking-corridor-reward-weight 1.65
  --overtaking-centerline-penalty-weight 1.55
  --overtaking-close-penalty-weight 2.20
  --speed-distance-coupling-penalty-weight 1.00
  --speed-distance-coupling-threshold 2.6
  --proximity-gradient-penalty-weight 0.45
  --proximity-gradient-distance 1.9
  --clear-ahead-distance 5.0
  --clear-ahead-bearing-deg 35.0
  --clear-ahead-cte-weight 0.60
  --clear-ahead-heading-weight 0.22
  --clear-ahead-omega-weight 0.80
  --straight-line-omega-penalty-weight 3.40
  --straight-line-omega-conflict-floor 0.08
  --straight-line-omega-cte-gate 0.70
  --near-goal-finish-weight 0.30
  --near-goal-finish-weight-end 0.50
  --near-goal-finish-distance 3.5
  --near-goal-finish-phase-min -1.0
  --near-goal-finish-target-speed 0.20
  --near-goal-finish-max-omega 0.12
  --near-goal-finish-omega-weight 0.30
  --random-clear-ahead-weight 5.80
  --random-clear-ahead-weight-end 6.60
  --random-clear-ahead-scenario single_usv_overtaking
  --random-clear-ahead-scenario three_usv_clear_route
  --random-clear-ahead-scenario solo_navigation
  --random-clear-ahead-distance 5.0
  --random-clear-ahead-bearing-deg 35.0
  --random-clear-ahead-cone-mode goal
  --random-clear-ahead-goal-tolerance 1.0
  --random-clear-ahead-max-distance 13.0
  --random-clear-ahead-min-neighbor-separation 0.0
  --random-clear-ahead-max-cpa-score 0.0
  --random-clear-ahead-max-local-score 0.0
  --random-clear-ahead-exclude-deconflict
  --random-clear-ahead-target-source raw
  --random-clear-ahead-target-speed 0.32
  --random-clear-ahead-min-speed 0.22
  --random-clear-ahead-max-omega 0.12
  --random-clear-ahead-omega-weight 1.35
  --random-deconflict-weight 5.20
  --random-deconflict-weight-end 5.60
  --random-deconflict-role-mode priority-delta
  --random-deconflict-priority-delta-yield-threshold -0.01
  --random-deconflict-goal-tolerance 0.8
  --random-deconflict-max-distance 13.0
  --random-deconflict-lookahead-distance 12.0
  --random-deconflict-time-horizon 40.0
  --random-deconflict-dcpa-target 2.10
  --random-deconflict-local-danger
  --random-deconflict-safe-separation 1.58
  --random-deconflict-release-separation 2.95
  --random-deconflict-yield-danger-scale 1.05
  --random-deconflict-standon-speed 0.24
  --random-deconflict-yield-speed 0.025
  --random-deconflict-standon-omega 0.04
  --random-deconflict-yield-omega 0.44
  --random-deconflict-turn-mode away
  --random-deconflict-omega-weight 1.70
  --random-deconflict-standon-weight 0.10
  --random-deconflict-yield-weight 5.20
  --random-deconflict-pretrain-epochs 4
  --random-deconflict-pretrain-learning-rate 3.0e-6
  --random-deconflict-pretrain-max-grad-norm 0.10
  --random-role-balance-weight 4.80
  --random-role-balance-weight-end 5.30
  --random-role-balance-pretrain-epochs 3
  --random-role-balance-pretrain-learning-rate 2.5e-6
  --random-role-balance-pretrain-max-grad-norm 0.10
  --random-role-balance-min-danger 0.08
  --random-role-balance-standon-min-speed 0.20
  --random-role-balance-standon-close-separation 0.90
  --random-role-balance-yield-max-speed 0.035
  --random-role-balance-yield-min-starboard-omega 0.24
  --random-role-balance-standon-weight 0.10
  --random-role-balance-yield-weight 2.80
  --random-role-balance-omega-weight 0.80
  --random-cte-recovery-weight 0.22
  --random-cte-recovery-weight-end 0.28
  --random-cte-recovery-min-abs-cte 1.75
  --random-cte-recovery-full-abs-cte 3.00
  --random-cte-recovery-min-neighbor-separation 1.75
  --random-cte-recovery-target-speed 0.10
  --random-cte-recovery-min-speed 0.035
  --random-cte-recovery-max-omega 0.18
  --random-cte-recovery-omega-reference 0.55
  --random-cte-recovery-omega-weight 0.24
  --random-safe-finish-weight 0.65
  --random-safe-finish-weight-end 0.90
  --random-safe-finish-min-team-separation 2.75
  --random-safe-finish-full-team-separation 3.30
  --random-safe-finish-min-neighbor-separation 2.85
  --random-safe-finish-yield-release-min-separation 1.35
  --random-safe-finish-yield-release-max-closing-speed 0.02
  --random-safe-finish-yield-release-min-route-progress 0.45
  --random-safe-finish-target-speed 0.19
  --random-safe-finish-min-speed-scale 0.45
  --random-safe-finish-low-priority-speed-multiplier 1.05
  --random-safe-finish-max-omega 0.08
  --random-safe-finish-omega-weight 0.08
  --random-offroute-finish-weight 1.60
  --random-offroute-finish-weight-end 2.10
  --random-offroute-finish-route-progress-min 0.80
  --random-offroute-finish-min-abs-cte 1.20
  --random-offroute-finish-full-abs-cte 3.00
  --random-offroute-finish-min-team-separation 2.75
  --random-offroute-finish-min-neighbor-separation 2.90
  --random-offroute-finish-target-speed 0.12
  --random-offroute-finish-min-speed 0.050
  --random-offroute-finish-max-omega 0.20
  --random-offroute-finish-omega-reference 0.55
  --random-offroute-finish-omega-weight 0.75
  --policy-anchor-weight 520.0
  --policy-anchor-weight-end 700.0
  --policy-anchor-exclude-random-deconflict
  --policy-anchor-exclude-random-cte-recovery
"

mkdir -p "$CKPT_DIR"
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

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
  --max-grad-norm 0.45 \
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
  $FRESH318_ARGS \
  "$@"