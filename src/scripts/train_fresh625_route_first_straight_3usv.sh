#!/bin/bash
# fresh625: ROUTE-FIRST REBALANCE. User directive (2026-07-08, after fresh624
# SITL star-route retest): "碰撞不是大问题 / 起点还是会绕圈 / 航线还是绕弯弯 /
# 不要害怕碰撞，要走出直线来，尽量贴着原始航线走" -- collisions are acceptable;
# straight lines hugging the original route are the priority.
#
# WHERE fresh624 STANDS (why warm-start fresh624):
#   SITL star route 2026-07-08 vs fresh622: first-leg net turn 647-1339 deg
#   (was 992-6100, -78%), first-leg time 45-82s (was 66-283s), loop events
#   12 (was 17, weaker: mostly ~361deg grazes vs 448-772), CTE p90 in the
#   2-5m neighbour band 0.35-0.69 (was 0.85-0.99), all 3 boats 6/6 waypoints.
#   Offline gate: cluster 9/9 (fastest ever 89-100 steps), crossing 3/3
#   success 0 collisions. Regressions: overtaking 3/3 collisions (fresh622
#   0/3) -- traced to the fresh624 head_on corridor suite firing in
#   overtaking-adjacent geometry; head_on still 3/3 (suite did NOT fix the
#   cut-back). Remaining SITL complaints = the user directive above: wide
#   arcs (10/18 legs detour >1.4x) and residual start circling.
#
# DESIGN DISCIPLINE (risk register from the 2026-07-06 project review):
#   RISK-1 (reward complexity near the edge): fresh625 adds ZERO new reward
#     terms. It ONLY rebalances existing weights, and REMOVES six terms (the
#     head_on corridor suite goes back to 0 -- it caused the overtaking 3/3
#     regression, didn't fix head_on, and adds turn/offset pressure exactly
#     against "walk straight"). Net reward complexity DOWN.
#   RISK-2 (train/deploy structural gap): exposure-based fix only --
#     max-waypoints-per-episode 4 -> 6 so episodes match the 6-leg star
#     mission structure (continuous multi-waypoint transit, not single-goal
#     sprints). pentagram_convergence x2 (dense centre-crossing transit)
#     stays. sim-tau / obs-delay domain randomization deferred (mid-term).
#   RISK-3 (speed floor 0.30 forbids yield-by-slowing): DEFERRED to fresh626
#     as a single-variable run. Changing action bounds on a warm-start is
#     high-blast-radius (metadata/anchor operate in action space); do not
#     stack it on a reward rebalance. Floor 0.30 still allows slowing to
#     ~0.10 m/s which is adequate yielding for this rebalance test.
#   RISK-4 (warm-start drift): chain is now fresh616 -> 622 -> 624 -> 625
#     (3rd chained generation). fresh616/622/624 are ANCESTOR CHECKPOINTS --
#     never delete. If fresh625 regresses on gate/SITL, the next run must be
#     a CONSOLIDATION: warm-start fresh622 and rebuild 624+625 recipes in
#     one run instead of chaining a 4th generation.
#
# THE REBALANCE (values only; every term already existed):
#   ROUTE/STRAIGHT UP -- make hugging the line the dominant signal:
#     path-deviation-penalty-weight     0.45 -> 0.90  (hug original route)
#     path-deviation-conflict-scale     1.0  -> 0.4   (CTE tolerance no longer
#       balloons during conflict -- conflict was licensing the wide arcs)
#     clear-ahead-cte-weight            0.30 -> 0.45  \
#     clear-ahead-heading-weight        0.30 -> 0.45   } route discipline trio
#     clear-ahead-omega-weight          0.30 -> 0.45  /  (predictive gate, 624)
#     straight-line-omega-penalty       1.5  -> 2.2   (no idle swinging)
#     heading-convergence-reward        0.20 -> 0.30  (existing, conflict-gated)
#     pure-cruise-reward                0.5  -> 0.7   (keep speed up, straight)
#     turn-speed-coupling               0.6  -> 0.8, floor 0.5 -> 0.4
#       (sharp turns must slow -> tighter radius -> no wide arcs; R = v/omega)
#   AVOIDANCE DOWN -- user explicitly accepts contact risk:
#     collision-penalty                 -30  -> -15
#     near-miss-weight                  6.0  -> 3.0
#     team-safety-brake-weight          0.25 -> 0.10 (end 0.05 -> 0.02)
#     conflict-turn-relief              0.80 -> 0.60  (turn penalties stay
#       more active during conflict -> less reflex swerving)
#     head_on corridor suite            2.4/2.8/1.4/1.4/2.2/1.4 -> ALL 0
#       (basic COLREGS head-on-turn-reward 3.0 kept)
#   EVERYTHING ELSE fresh624 VERBATIM: predictive route discipline gate,
#   per-agent entanglement (1.0 / 1.5m / 40 steps / low-speed 0.5),
#   separation-recovery 0.5, dense asymmetric cluster spawns, 16-slot
#   curriculum (solo x1, waypoint x2, clear_route x2, cluster x2,
#   pentagram x2, head_on x2, crossing x2, overtaking x2, random x1),
#   CRI horizon 15s / safe DCPA 1.5, guidance 7.5m, 400k steps,
#   anchor 1.0 -> 0.35 anchored to fresh624.
#
# Deployment target: Raspberry Pi 5 (3.3MB MLP+attention actor, CPU ok).
#
# ARBITER for fresh625 (user-priority order, collisions de-weighted):
#   SITL star route PRIMARY: detour ratio per leg (want most legs <1.3x,
#   fresh624 had 10/18 >1.4x), net turn per leg, loop events (want <5,
#   fresh624 12), CTE p90 <2m and 2-5m bands, 6/6 waypoints all boats.
#   Offline gate SECONDARY (regression guard): cluster 9/9 reach kept,
#   crossing success kept; collision counts REPORTED but not blocking
#   unless catastrophic (entanglement-style pileups).

set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh624_dense_transit_route_hold_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh625_route_first_straight_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh625_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-400000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-294}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh625] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh625] OUTPUT=$OUTPUT"
echo "[fresh625] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh625] ROUTE-FIRST rebalance: hug route + straight lines dominant, avoidance de-weighted (user 2026-07-08), zero new reward terms, head_on suite removed, 6-waypoint episodes"

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
  --scenario cluster_escape \
  --scenario pentagram_convergence \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_random_encounter \
  --max-waypoints-per-episode 6 \
  --waypoint-bonus 10.0 \
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
  --conflict-turn-relief 0.60 \
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
  --collision-penalty -15.0 \
  --near-miss-weight 3.0 \
  --separation-recovery-weight 0.5 \
  --entanglement-penalty-weight 1.0 \
  --entanglement-distance 1.5 \
  --entanglement-grace-steps 40 \
  --entanglement-low-speed-penalty-weight 0.5 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.90 \
  --path-deviation-conflict-scale 0.4 \
  --path-deviation-exclude-overtaking \
  --conflict-risk-weight 0.0 \
  --conflict-risk-time-horizon 15.0 \
  --conflict-risk-safe-dcpa 1.5 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 0.0 \
  --conflict-resolution-reward-weight 0.0 \
  --conflict-escalation-penalty-weight 0.0 \
  --unsafe-close-speed-penalty-weight 0.0 \
  --speed-distance-coupling-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 0.0 \
  --straight-line-omega-penalty-weight 2.2 \
  --turn-speed-coupling-penalty-weight 0.8 \
  --turn-speed-coupling-floor 0.4 \
  --turn-speed-coupling-deadband-deg 25.0 \
  --heading-error-weight 0.20 \
  --heading-correction-reward-weight 0.30 \
  --heading-convergence-reward-weight 0.30 \
  --heading-convergence-threshold-deg 12.0 \
  --clear-ahead-distance 5.0 \
  --clear-ahead-bearing-deg 35.0 \
  --clear-ahead-cte-weight 0.45 \
  --clear-ahead-heading-weight 0.45 \
  --clear-ahead-omega-weight 0.45 \
  --action-smoothness-weight 0.10 \
  --angular-accel-penalty-weight 0.15 \
  --head-on-guidance-distance 7.5 \
  --crossing-guidance-distance 7.5 \
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
  --pure-cruise-reward-weight 0.7 \
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
  --team-safety-brake-weight 0.10 \
  --team-safety-brake-weight-end 0.02 \
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
