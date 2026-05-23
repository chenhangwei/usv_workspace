#!/bin/bash
# fresh461: random-encounter focused structural CTE recovery from fresh438.
#
# fresh459/fresh460 wrapped train_fresh308, whose short 900-step curriculum stayed
# in single_usv_overtaking and never activated random_cte recovery.  This script
# invokes train_mappo_policy directly so the rollout actually samples the target
# random encounter seeds.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh461_cte_usv02_random_focus_from_fresh438.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh461_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-900}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

mkdir -p "$CKPT_DIR"

exec /bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --checkpoint-interval 900 \
  --num-agents 3 \
  --rollout-steps 900 \
  --reset-sampler-each-rollout \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.35 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 \
  --hidden-size 256 \
  --actor-log-std-init -2.34 \
  --force-actor-log-std -2.34 \
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
  --episode-timeout 240.0 \
  --no-progress-timeout 220.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  --scenario solo_navigation \
  --scenario single_usv_overtaking \
  --scenario three_usv_clear_route \
  --scenario two_usv_head_on \
  --scenario two_usv_crossing \
  --scenario two_usv_overtaking \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario two_usv_random_encounter \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-seed 1461 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --scenario-spawn-position-std 0.035 \
  --scenario-spawn-heading-std 0.015 \
  --scenario-goal-position-std 0.03 \
  --random-encounter-route-priority \
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
  --learning-rate 4.0e-8 \
  --learning-rate-end 1.8e-8 \
  --clip-range 0.00003 \
  --ppo-policy-loss-scale 0.0012 \
  --ppo-value-loss-scale 0.014 \
  --value-coef 0.003 \
  --update-epochs 2 \
  --minibatch-size 210 \
  --per-scenario-advantage-norm \
  --cte-clip-range 5.0 \
  --route-progress-cte-gate-start 0.0 \
  --route-progress-cte-gate-width 0.0 \
  --route-progress-cte-gate-floor 0.25 \
  --path-deviation-use-unclipped-cte \
  --path-deviation-penalty-weight 0.95 \
  --path-deviation-tolerance 0.75 \
  --path-deviation-conflict-scale 1.25 \
  --conflict-risk-weight 1.5 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 0.9 \
  --conflict-escalation-penalty-weight 1.2 \
  --unsafe-close-speed-penalty-weight 1.0 \
  --speed-distance-coupling-penalty-weight 1.00 \
  --speed-distance-coupling-threshold 2.6 \
  --proximity-gradient-penalty-weight 0.45 \
  --proximity-gradient-distance 1.9 \
  --straight-line-omega-penalty-weight 3.0 \
  --straight-line-omega-conflict-floor 0.08 \
  --straight-line-omega-cte-gate 0.70 \
  --random-deconflict-weight 4.40 \
  --random-deconflict-weight-end 4.90 \
  --random-deconflict-role-mode route-eta-delta \
  --random-deconflict-route-eta-yield-threshold 0.02 \
  --random-deconflict-safe-separation 1.58 \
  --random-deconflict-release-separation 2.95 \
  --random-deconflict-dcpa-target 2.05 \
  --random-deconflict-yield-danger-scale 1.00 \
  --random-deconflict-standon-speed 0.22 \
  --random-deconflict-yield-speed 0.035 \
  --random-deconflict-standon-omega 0.04 \
  --random-deconflict-yield-omega 0.46 \
  --random-deconflict-turn-mode away \
  --random-deconflict-omega-weight 1.20 \
  --random-deconflict-standon-weight 0.18 \
  --random-deconflict-yield-weight 4.80 \
  --random-role-balance-weight 4.20 \
  --random-role-balance-weight-end 4.70 \
  --random-role-balance-min-danger 0.04 \
  --random-role-balance-standon-min-speed 0.18 \
  --random-role-balance-yield-max-speed 0.045 \
  --random-role-balance-yield-min-starboard-omega 0.20 \
  --random-role-balance-standon-weight 0.20 \
  --random-role-balance-yield-weight 2.20 \
  --random-role-balance-omega-weight 0.36 \
  --random-cte-recovery-agent-index 1 \
  --random-cte-recovery-weight 1.20 \
  --random-cte-recovery-weight-end 1.60 \
  --random-cte-recovery-min-abs-cte 2.00 \
  --random-cte-recovery-full-abs-cte 5.00 \
  --random-cte-recovery-min-neighbor-separation 2.80 \
  --random-cte-recovery-target-speed 0.13 \
  --random-cte-recovery-min-speed 0.05 \
  --random-cte-recovery-max-omega 0.24 \
  --random-cte-recovery-omega-reference 0.75 \
  --random-cte-recovery-omega-weight 0.80 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-clear-ahead-weight 0.0 \
  --random-clear-ahead-weight-end 0.0 \
  --random-safe-finish-weight 0.0 \
  --random-safe-finish-weight-end 0.0 \
  --policy-anchor-weight 8500.0 \
  --policy-anchor-weight-end 12000.0 \
  --policy-anchor-exclude-random-cte-recovery \
  "$@"