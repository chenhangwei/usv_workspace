#!/bin/bash
# fresh617: fresh616 PLUS a dedicated START-CONGESTION ESCAPE scenario + separation
# recovery, per the 2026-06-29 SITL finding: in the star-route mission all 3 USVs
# spawn ~0.8m apart at the arena centre and got stuck in a low-speed orbital
# DEADLOCK -- 100% of the run within 2m of each other, headings 70-90deg off their
# goals, none ever departing toward its (different-direction) goal.
#
# ROOT CAUSE (the user's own diagnosis, confirmed): NO training scenario spawns 3
# USVs this tight. clear_route ~4.6m, crossing ~8.5m, random_encounter ~5m+,
# pentagram starts on a ~6m-radius ring. So "face-to-face at start -> who yields,
# who goes -> break apart and each take its own route" is a state distribution the
# policy NEVER saw. fresh613..616 only tuned rewards; the data gap remained.
#
# THE CHANGES vs fresh616:
#   (A) NEW SCENARIO cluster_escape (3 USVs on a ~0.5m-radius circle => ~0.7-0.9m
#       initial separation, goals fanned ~120deg apart at goal_distance). Added to
#       curriculum x2 so start-deadlock escape gets real rounds.
#       Curriculum (11): solo x3, waypoint x3, clear_route x2, cluster_escape x2,
#       crossing x1, overtaking x1, random x1  (route 67%, escape 18%, encounter 15%).
#   (B) --separation-recovery-weight 0.0 -> 0.30  (reward INCREASING pairwise
#       separation while inside the near-miss band -> "actively break apart").
#   (C) --entanglement-penalty-weight 0.0 -> 0.15  (penalise sustained <ent_dist
#       proximity -> anti orbital-lock; ramps after a grace period).
#   (D) gentle avoidance restore so they actually yield to disperse (we cut these
#       too hard in fresh614): --near-miss-weight 6 -> 10, --team-safety-brake 0.25
#       -> 0.35 (end 0.05 -> 0.08). collision-penalty stays -30 (user allows bumps).
#
# INHERITED from fresh616 (all KEPT): turn-speed coupling 0.6/floor0.5/db25,
# heading-lock 0.20/0.30, max-omega/angular 0.70, straight-omega 1.5, multi-waypoint
# 4, route-dominant base, warm-start fresh605, policy-anchor 1.0->0.35, LR 5e-5->2e-5.
#
# (fresh616 lineage) explicit TURN-SPEED COUPLING: obtuse turn -> keep speed; acute
# turn -> slow for a tight radius (R=v/omega).
#
# WHY (gap fresh615 didn't guarantee): the speed/turn coupling was only INDIRECT
# (reward target-speed heading_gate + near-goal slowing) and the warm-start policy
# kept cmd_vx pinned ~0.34 in SITL -> wide 1.2-2.6m arcs. fresh616 adds a DIRECT
# penalty on forward speed above a turn-appropriate cap:
#   cap = cruise * (floor + (1-floor)*cos(min(|heading_err|,90deg)))
#   aligned (obtuse leg): cap=cruise -> full speed free; 45deg: cap~=0.29;
#   90deg (acute): cap~=0.17 -> must slow -> tight radius. Conflict-relieved.
# Deadband 25deg so gentle course corrections keep full speed (natural transition).
#
# THE CHANGES vs fresh615 (ONE coherent addition):
#   --turn-speed-coupling-penalty-weight  0.0 -> 0.6
#   --turn-speed-coupling-floor           0.5
#   --turn-speed-coupling-deadband-deg    25.0
# Everything else identical to fresh615 (route-dominant curriculum, multi-waypoint=4,
# arrival-first rewards, max-omega 0.7, warm-start fresh605, anchor, LR).
#
# (fresh615 lineage) ROUTE-FIRST + MULTI-WAYPOINT for the user's 3 needs (2026-06-28):
# dedicated scenarios AND many rounds for (1) reach goal, (2) hug the original route
# (low CTE), (3) no S-curve oscillation.
#
# KEY DISCOVERY: the env ALREADY supports multi-waypoint routes natively via
# --max-waypoints-per-episode (no new scenario code needed). When an agent reaches
# its goal it gets a NEW goal at goal_distance away on a heading offset +/-90deg
# from current heading; the route line + CTE recompute against the new leg and a
# waypoint_bonus is paid. So solo_navigation / waypoint_turn become CONTINUOUS
# multi-leg routes with sharp transitions -- exactly the deployment star route,
# but randomized (generalizes better than memorizing one fixed polyline).
#
# THE CHANGES vs fresh614:
#   (A) MULTI-WAYPOINT ON:
#       --max-waypoints-per-episode  1 -> 4   (each episode is a 4-leg route)
#       --waypoint-bonus             10.0     (reward each waypoint arrival -> need 1)
#   (B) ROUTE-DOMINANT curriculum (was encounter-dominant 56%):
#       solo x3, waypoint x3, clear_route x2, crossing x1, overtaking x1, random x1
#       -> single-agent route 60%, +clear_route = 80% route, encounter 30%.
#       Serves needs 1/2/3 with dedicated rounds; 30% encounter + anchor keep
#       avoidance from being forgotten (collisions now allowed, so mild ok).
#   (C) --path-deviation-penalty-weight 0.25 -> 0.45  (hug route harder -> need 2;
#       still below config default 0.55).
#
# INHERITED from fresh614 (arrival-first, all KEPT):
#   heading-error 0.20, heading-correction 0.30 (goal-lock), collision -30,
#   near-miss 6, team-safety-brake 0.25/0.05, max-omega/angular-delta 0.70,
#   straight-line-omega 1.5, pure-spin 3.0 (no in-place spin), COLREGS turns 3.0,
#   warm-start fresh605, policy-anchor 1.0->0.35, LR 5e-5->2e-5.
#
# Eval reminder: overtaking MUST use the long profile (STEPS>=900, EPISODE_TIMEOUT>=240).
# Real arbiter is the 3-USV SITL: arrival rate, detour ratio, loops, turn radius, CTE.

set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh605_overtake_reward_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh617_cluster_escape_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh617_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-200000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-294}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh617] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh617] OUTPUT=$OUTPUT"
echo "[fresh617] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh617] +cluster_escape scenario x2, separation-recovery 0.30, entanglement 0.15, near-miss/team-brake restored"

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
  --curriculum-scenario solo_navigation \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_random_encounter \
  --max-waypoints-per-episode 4 \
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
  --near-miss-weight 10.0 \
  --separation-recovery-weight 0.30 \
  --entanglement-penalty-weight 0.15 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.45 \
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
  --turn-speed-coupling-penalty-weight 0.6 \
  --turn-speed-coupling-floor 0.5 \
  --turn-speed-coupling-deadband-deg 25.0 \
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
  --team-safety-brake-weight 0.35 \
  --team-safety-brake-weight-end 0.08 \
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
