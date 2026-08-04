#!/bin/bash
# fresh621: fresh620 recipe PLUS "look further ahead / avoid earlier" tuning of
# the predictive CRI, after fresh620's offline gate (36 ep) showed head_on FIXED
# (0/3 collisions, was 3/3) and cluster_escape best-yet (9/9 reach, 5/9<=0.5m),
# but three_usv_crossing STILL 3/3 collided and three_usv_overtaking regressed
# to 3/3.
#
# ROOT CAUSE (reconstructed the s5241 crossing collision trajectory frame by
# frame from distance_to_goal + cross_track_error + route geometry, then
# recomputed conflict_risk/clear_ahead per step): usv_01 (east) and usv_03
# (west) are HEAD-ON on y=0 while usv_02 crosses. usv_03 sits dead-ahead
# (bearing -2deg) on a collision course (predicted DCPA 0.3-1.3 m) from 12 m
# out, BUT its TCPA stays > the 8 s CRI horizon until the boats are only ~5 m
# apart. So conflict_risk = 0 for the entire 12->5 m approach, which keeps
# low_conflict_gate = 1 and clear_ahead_gate = 0.78-1.0 -- i.e. the reward was
# actively TEACHING usv_01 to hold course + low yaw straight into the head-on
# threat. By the time TCPA finally dropped under 8 s (~5 m) clear_ahead
# released and risk woke, but there was no room left for a clean starboard
# turn -> 95/-179/-89 deg panic spin -> collision at ~1 m. The COLREGS turn
# rewards also only wake at 5 m (their guidance distance), so nothing rewarded
# turning during the golden 12->5 m warning window.
#
# THE FIX (reward-code + 4 CLI knobs; validated on the s5241 trajectory:
# risk now lights 0.11@6.4m and 0.46@5m -> clear_ahead OFF ~1.4 m / 2 s
# EARLIER, inside the window where a clean starboard turn still fits):
#   (1) --conflict-risk-time-horizon 15.0  (was implicit 8.0): predictive CRI
#       looks ~2x further ahead. NOT "more afraid" -- the DCPA gate still zeroes
#       comfortable / parallel / time-separated passes; only genuine
#       collision-course tracks light up, just sooner.
#   (2) --conflict-risk-safe-dcpa 1.5  (was 1.20): treat a predicted pass
#       closer than 1.5 m (2x collision radius) as graded risk, so marginal
#       converging tracks (DCPA ~1.3) get early awareness instead of a hard 0.
#   (3) --head-on-guidance-distance 7.5  (was 5.0) and
#   (4) --crossing-guidance-distance 7.5 (new; was implicit 5.0): the COLREGS
#       starboard-turn rewards now wake at 7.5 m too, in step with the CRI, so
#       guidance and risk agree on "start the avoidance turn sooner".
# WARM-START stays fresh620 (keeps its head_on + cluster_escape gains).
#
# fresh620 provenance (kept): fresh619's recipe (identical CLI) PLUS a
# foundational reward-code upgrade: conflict_risk is a PREDICTIVE DCPA/TCPA
# collision-risk index (CRI) instead of the old proximity x range_rate x
# forward_factor heuristic.
#
# WHY (user directive 2026-07-01): the policy behaved like a "frightened bird"
# -- it slowed / turned whenever any neighbour got close-and-closing-ahead,
# regardless of whether the two tracks actually converge. The user wants a
# vessel that PREDICTS: extrapolate relative motion, compute how close the two
# hulls will actually pass (DCPA) and how soon (TCPA); risk is high only for a
# soon-AND-close encounter and rises as TCPA shrinks ("higher risk -> earlier
# avoidance"); parallel / diverging / time-separated crossings score ~0 so the
# boat keeps cruising the route without fear.
#
# THE CODE CHANGE (multi_agent_env.py, no CLI change): _compute_conflict_risk
# rewritten as risk = dcpa_gate * time_gate, where dcpa_gate ramps 1->0 over
# [collision_distance .. safe_dcpa] and time_gate ramps 1->0 over [0 .. horizon].
# safe_dcpa = max(near_miss, anticipatory_dcpa_target)=1.20; horizon =
# anticipatory_cpa_time_horizon=8.0 (existing config knobs reused, no new args).
# A small near-contact proximity floor is kept for numerical safety. Validated
# offline (/tmp/cri_check.py): dead-ahead collision escalates 0.08@5m ->
# 0.45@3m -> 0.82@1m; safe-offset / time-separated / parallel / diverging all
# score 0.000; near-contact keeps a 0.1 floor. This conflict_risk drives ~8
# downstream gates (clear_ahead, heading_convergence, straight-omega, omega-
# flip, turn-speed-coupling, heading_scale), so ALL of them become predictive
# at once. Hard collision/near-miss penalties use measured pair_min and are
# independent, so the safety net is unchanged.
#
# ALSO INHERITED FROM fresh619: heading_convergence_reward is hard-gated by
# low_conflict_gate (zero once risk>=0.45) so it never fights avoidance turns.
#
# ---- fresh619 provenance (kept) ----
# fresh619: same recipe as fresh618 (clear-ahead route discipline + heading
# convergence bonus + cluster_escape/crossing exposure, warm-start fresh616),
# but fresh618's offline gate eval (2026-07-01, 12 episodes: cluster_escape,
# three_usv_crossing, two_usv_head_on, three_usv_overtaking, 3 seeds each)
# found a SEVERE SAFETY REGRESSION: fresh618 collided in 8/9 episodes that had
# a real converging encounter (head_on 3/3, crossing 3/3, overtaking 2/3),
# vs 0 collisions for fresh616 on the same evals. cluster_escape itself (no
# real path conflict, goals diverge) was fine and much improved (0/9 -> 9/9
# reached). Frame-by-frame trace of two_usv_head_on showed both USVs holding
# ~0.34 m/s and near-zero omega/heading-error ALL THE WAY to collision --
# not a hesitation/oscillation, a straight-up failure to ever start the
# avoidance turn.
#
# ROOT CAUSE (code-level, confirmed against multi_agent_env.py): all of
# clear_ahead_*, head_on_turn_reward and conflict_risk itself already share
# the SAME 5.0 m activation distance (clear_ahead_distance ==
# anticipation_distance == conflict_distance == head_on_guidance_distance ==
# 5.0, unchanged from fresh616) and clear_ahead is ALREADY hard-gated by
# low_conflict_gate (= 0 once conflict_risk >= 0.45) -- so clear_ahead is not
# the culprit. The one NEW term fresh618 added that had NO hard conflict gate
# was --heading-convergence-reward-weight: it only decayed via heading_scale,
# which floors at 0.1 and NEVER reaches zero, so it kept paying the agent for
# holding |heading_error| < 12 deg even while a neighbour was actively
# closing -- i.e. it directly rewarded NOT turning, fighting the COLREGS
# avoidance-turn rewards at exactly the moment they needed to dominate.
#
# THE FIX (multi_agent_env.py, code-only, no CLI weight changes needed): the
# heading_convergence_reward term is now ALSO multiplied by low_conflict_gate
# (hard zero once conflict_risk >= 0.45), matching clear_ahead's gating
# discipline. fresh619 re-runs the exact fresh618 recipe (same warm-start,
# same weights, same curriculum) on top of this one-line safety fix so the
# terminal-capture/anti-circling gains from fresh618 are re-validated WITHOUT
# the collision regression.
#
# fresh618 header (unchanged CLI, kept for reference):
# WARM-START fresh616 (the best of fresh605/616/617 under FIXED 10Hz SITL
# telemetry, 2026-06-30 re-test):
#   RE-TEST EVIDENCE (3 models, same star-route mission, clean 10Hz pose):
#     model     first-leg net-turn   reach<=0.5m   min-sep   mean spd / cruise%
#     fresh617  2228/2469 deg         1/18          0.45 m    0.29 / 70-81
#     fresh605  2026/1920 deg         0/10          0.56 m    0.19-0.26 / 28-62
#     fresh616  1099/1056/831 deg     1/18          0.16 m    0.31 / 80-88   <-- best
#   (A) TERMINAL CAPTURE + ANTI-GRATUITOUS-CIRCLING via CLEAR-AHEAD route
#       discipline: --clear-ahead-distance 5.0 --clear-ahead-bearing-deg 35
#       --clear-ahead-cte-weight 0.30 --clear-ahead-heading-weight 0.30
#       --clear-ahead-omega-weight 0.30
#   (B) HEADING CONVERGENCE bonus (now conflict-gated, see fix above):
#       --heading-convergence-reward-weight 0.20 --heading-convergence-threshold-deg 12
#   (C) CONVERGENCE EXPOSURE: cluster_escape x2 + crossing x2 in curriculum.
#
# INHERITED from fresh616 (KEPT verbatim): turn-speed coupling 0.6/floor0.5/db25,
# heading-lock 0.20/0.30, near-miss 6, team-safety-brake 0.25/0.05 (NOT inflated),
# max-omega/angular 0.70, straight-omega 1.5, pure-spin 3.0, multi-waypoint 4,
# route-dominant curriculum, COLREGS turns 3.0, LR 5e-5->2e-5, anchor 1.0->0.35.
# WARM-START is fresh616 (NOT fresh618 -- fresh618's weights already learned
# the "don't turn" habit and are not a safe base to continue from).
#
# Real arbiter: offline gate MUST include cluster_escape AND at least one real
# encounter (two_usv_head_on / three_usv_crossing / three_usv_overtaking) with
# an explicit collision-rate check before this is allowed anywhere near SITL.
# Then 3-USV SITL: net-turn (loops), detour ratio, reach<=0.5 m, min separation.

set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh620_cpa_predictive_risk_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh621_early_predictive_avoid_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh621_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-200000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-294}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh621] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh621] OUTPUT=$OUTPUT"
echo "[fresh621] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh621] EARLY predictive avoid: CRI horizon 15s + safe_dcpa 1.5 + head-on/crossing guidance 7.5m (look further, avoid sooner)"

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
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario three_usv_crossing \
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
  --near-miss-weight 6.0 \
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
  --conflict-risk-time-horizon 15.0 \
  --conflict-risk-safe-dcpa 1.5 \
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
  --heading-convergence-reward-weight 0.20 \
  --heading-convergence-threshold-deg 12.0 \
  --clear-ahead-distance 5.0 \
  --clear-ahead-bearing-deg 35.0 \
  --clear-ahead-cte-weight 0.30 \
  --clear-ahead-heading-weight 0.30 \
  --clear-ahead-omega-weight 0.30 \
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
