#!/bin/bash
# fresh614: ARRIVAL-FIRST rebalance. Warm-start from fresh605 (eval 7/7, the most
# complete goal-reaching baseline). This is a DELIBERATE, COHERENT reward-priority
# flip per the user's 3 explicit needs (2026-06-28), validated against the fresh613
# 3-USV SITL where arrival failed badly (usv_06 spent the whole run wandering and
# never got within 4.4m of its first goal; usv_04 took a 3.27x-detour to goal 1;
# heading-to-goal error swung +/-160deg the entire time even with NO neighbour
# nearby -> the policy never locks onto the goal).
#
# ROOT CAUSE (confirmed from SITL): the fresh605..613 line is AVOIDANCE-FIRST.
# The goal-locking reward knobs were OFF (heading-error-weight=0,
# heading-correction-reward-weight=0), so in open water the boats wander on
# residual avoidance turning instead of pointing at the goal. Plus turn radius
# is physically too large (R=v/omega; cmd_vx pinned ~0.34 and max omega 0.5 ->
# min radius 0.68m, observed median 1.2-2.6m).
#
# THE CHANGES vs fresh613 (all serve ONE of the user's 3 needs):
#   NEED 1 - reliable arrival (goal-locking):
#     --heading-error-weight            0.0 -> 0.20  (penalize pointing away; has
#                                                     conflict + goal-proximity relief)
#     --heading-correction-reward-weight 0.0 -> 0.30 (reward turning TOWARD goal;
#                                                     the core anti-wander lever)
#   NEED 2 - allow occasional collisions, stop fear-circling:
#     --collision-penalty             -100.0 -> -30.0
#     --near-miss-weight                20.0 -> 6.0
#     --team-safety-brake-weight    0.5/0.15 -> 0.25/0.05  (brake-induced slow+turn-away
#                                                     was causing circling)
#   NEED 3 - tighter turn radius (0.3-0.5m) + natural transitions, no in-place spin:
#     --max-angular-velocity           0.50 -> 0.70  (R=0.34/0.70~=0.49m at cruise)
#     --angular-delta-limit            0.50 -> 0.70
#     --straight-line-omega-penalty    3.0  -> 1.5   (let it turn tight when aligned)
#     --pure-spin-penalty-weight       3.0   KEPT     (still forbid in-place spin)
#
# DELIBERATELY KEPT (graceful avoidance the user still wants + anti-forget):
#   - COLREGS turn rewards (head-on/crossing/overtaking starboard 3.0) UNCHANGED:
#     the user wants "graceful avoidance with some action", not zero avoidance.
#   - balanced anti-forget curriculum (solo x1, waypoint x2, crossing x2,
#     overtaking x2, clear_route x1, random x1) == fresh613.
#   - policy-anchor 1.0->0.35, LR 5e-5->2e-5, near-goal-finish 0.30/1.3.
#
# Eval reminder: overtaking MUST use the long profile (STEPS>=900, EPISODE_TIMEOUT>=240).
# Real arbiter is the 3-USV SITL: arrival rate, detour ratio, loops, turn radius.

set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh605_overtake_reward_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh614_arrival_first_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh614_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-200000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-294}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh614] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh614] OUTPUT=$OUTPUT"
echo "[fresh614] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh614] ARRIVAL-FIRST: heading-lock ON, collision/near-miss DOWN, turn radius TIGHTER"

exec /bin/python3 -m usv_rl.train_mappo_policy \
  --load-weights-from "$WARMSTART_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 16384 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  --action-mode speed_scale \
  --action-speed-scale-min "$ACTION_SPEED_SCALE_MIN" \
  --num-agents 3 \
  --max-agents 5 \
  --max-neighbors 4 \
  --scenario solo_navigation \
  --scenario single_usv_overtaking \
  --scenario waypoint_turn \
  --scenario two_usv_head_on \
  --scenario two_usv_crossing \
  --scenario two_usv_overtaking \
  --scenario three_usv_clear_route \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_random_encounter \
  --scenario-spawn-position-std 0.10 \
  --scenario-spawn-heading-std 0.05 \
  --scenario-goal-position-std 0.08 \
  --episode-timeout 110.0 \
  --no-progress-timeout 60.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.20 \
  --scenario-neighbor-speed 0.34 \
  --squash-actions \
  --min-forward-speed 0.0 \
  --min-forward-speed-floor 0.05 \
  --linear-delta-limit 0.30 \
  --angular-delta-limit 0.70 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.70 \
  --heading-omega-deadband 0.09 \
  --heading-omega-reference 0.95 \
  --angular-authority-power 1.30 \
  --angular-accel-limit 1.05 \
  --angular-decel-limit 3.00 \
  --angular-authority-floor 0.45 \
  --conflict-turn-relief 0.80 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  --hidden-size 256 \
  --hidden-size 256 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --actor-log-std-init -1.20 \
  --normalize-observations \
  --learning-rate 5.0e-5 \
  --learning-rate-end 2.0e-5 \
  --clip-range 0.2 \
  --ppo-policy-loss-scale 1.0 \
  --ppo-value-loss-scale 0.5 \
  --value-coef 0.5 \
  --update-epochs 10 \
  --minibatch-size 256 \
  --max-grad-norm 0.5 \
  --entropy-coef 0.01 \
  --entropy-coef-end 0.003 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --device cpu \
  --torch-num-threads 1 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -30.0 \
  --near-miss-weight 6.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.25 \
  --path-deviation-conflict-scale 1.0 \
  --path-deviation-exclude-overtaking \
  --conflict-risk-weight 0.0 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 0.0 \
  --conflict-resolution-reward-weight 0.0 \
  --conflict-escalation-penalty-weight 0.0 \
  --unsafe-close-speed-penalty-weight 0.0 \
  --speed-distance-coupling-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 0.0 \
  --straight-line-omega-penalty-weight 1.5 \
  --heading-error-weight 0.20 \
  --heading-correction-reward-weight 0.30 \
  --action-smoothness-weight 0.10 \
  --angular-accel-penalty-weight 0.15 \
  --head-on-corridor-reward-weight 0.0 \
  --head-on-centerline-penalty-weight 0.0 \
  --head-on-forward-reward-weight 0.0 \
  --head-on-speed-drop-penalty-weight 0.0 \
  --head-on-close-penalty-weight 0.0 \
  --head-on-no-turn-penalty-weight 0.0 \
  --crossing-forward-reward-weight 0.0 \
  --overtaking-forward-reward-weight 0.35 \
  --overtaking-corridor-reward-weight 0.9 \
  --overtaking-centerline-penalty-weight 1.1 \
  --overtaking-close-penalty-weight 1.2 \
  --team-reward-weight 0.0 \
  --team-progress-weight 0.0 \
  --coordination-reward-weight 0.0 \
  --team-completion-bonus 0.0 \
  --deadlock-penalty-weight 0.0 \
  --pure-cruise-reward-weight 0.5 \
  --pure-idle-penalty-weight 0.0 \
  --pure-turn-penalty-weight 0.0 \
  --pure-spin-penalty-weight 3.0 \
  --crossing-imitation-weight 0 \
  --crossing-imitation-weight-end 0 \
  --overtaking-imitation-weight 0 \
  --overtaking-imitation-weight-end 0 \
  --random-deconflict-weight 0 \
  --random-deconflict-weight-end 0 \
  --random-role-balance-weight 0 \
  --random-role-balance-weight-end 0 \
  --random-pairwise-role-guard-weight 0 \
  --random-pairwise-role-guard-weight-end 0 \
  --random-safe-finish-weight 0 \
  --random-safe-finish-weight-end 0 \
  --random-goal-hold-weight 0 \
  --random-goal-hold-weight-end 0 \
  --random-offroute-finish-weight 0 \
  --random-offroute-finish-weight-end 0 \
  --random-cte-recovery-weight 0 \
  --random-cte-recovery-weight-end 0 \
  --random-clear-ahead-weight 0 \
  --random-clear-ahead-weight-end 0 \
  --random-late-lagging-weight 0 \
  --random-late-lagging-weight-end 0 \
  --random-close-formation-escape-weight 0 \
  --random-close-formation-escape-weight-end 0 \
  --near-goal-finish-weight 0.30 \
  --near-goal-finish-weight-end 0.30 \
  --near-goal-finish-distance 1.3 \
  --near-goal-finish-exclude-overtaking \
  --lagging-finish-weight 0 \
  --lagging-finish-weight-end 0 \
  --team-safety-brake-weight 0.25 \
  --team-safety-brake-weight-end 0.05 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-goal-tolerance 2.0 \
  --team-safety-brake-max-distance 14.0 \
  --team-safety-brake-safe-separation 0.85 \
  --team-safety-brake-release-separation 0.95 \
  --team-safety-brake-target-speed 0.20 \
  --team-safety-brake-omega-weight 0.15 \
  --team-safety-brake-target-omega 0.25 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.5 \
  --team-safety-brake-cpa-danger \
  --team-safety-brake-cpa-lookahead-distance 4.0 \
  --team-safety-brake-cpa-time-horizon 8.0 \
  --team-safety-brake-cpa-dcpa-target 1.2 \
  --team-safety-brake-cpa-closing-speed-min 0.05 \
  --policy-anchor-weight 1.0 \
  --policy-anchor-weight-end 0.35 \
  --policy-anchor-exclude-team-safety-brake
