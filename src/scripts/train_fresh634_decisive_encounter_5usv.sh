#!/bin/bash
# fresh634: DECISIVE-ENCOUNTER -- eliminate the brake-first standoff loop
# confirmed in the 2026-08-03 SITL batch. Builds on fresh633 (weights
# VERBATIM unless listed). Warmstart fresh633 final, anchor to fresh633,
# 800k steps.
#
# MOTIVATION (SITL diagnosis 2026-08-03, see
#   memories/repo/fresh633_sitl_encounter_diagnosis_20260803.md):
#   Two-boat reciprocal transit (usv_02 vs usv_03) reproduced the full
#   standoff mechanism second-by-second:
#     - t=211-217 (sep 3.0->1.4m): BOTH boats brake rl_vx 0.33->0.14 while
#       turning only 30% of the 0.70 omega bound. Lateral offset is never
#       built; they keep closing on the collision line.
#     - t=218-225: speeds collapse to 0.02-0.10 -> steerage lost -> in-place
#       bow rotation sweeps each bow across the opponent bearing -> bows
#       lock mutually-facing (relA/relB ~ 0). rl_om dithers -0.29 <-> +0.40.
#     - min separation 0.40m (and 0.14m elsewhere in the session); escape
#       only when noise breaks symmetry (B accelerates first).
#     - Session 131747: both boats settle at ~0.23-0.25 m/s ~= the give-way
#       speed 0.26 -> MUTUAL yield, no stand-on keeps course/speed.
#   Gate eval 2026-07-30 (logs/eval_fresh633_gate): 3/11 pass, mean |omega|
#   usage only ~3% of authority in failing scenarios -- confirms turning
#   was never learned, geometry luck decided outcomes.
#
# ROOT CAUSE:
#   R1 braking is FREE in conflict: conflict-brake-weight was explicitly
#      0.0 (default 1.2), so dropping below desired_conflict_speed costs
#      nothing while straight-line-omega/path-deviation still tax turning.
#      Learned optimum = brake instead of turn; braking kills steerage and
#      creates the in-place-rotation bow-lock geometry.
#   R2 no early-turn signal: avoidance_turn_reward_weight and ALL
#      anticipatory_avoidance_turn_* knobs were 0.0 -- nothing ever paid
#      the policy for opening the bearing on a CBDR contact before the
#      near-miss band.
#   R3 role symmetry: role-speed-asymmetry-weight 0.8 is too weak; both
#      boats act as give-way (speeds converge to role_giveway_speed 0.26),
#      micro-turns cancel, deadlock resolves only by noise.
#   R4 deadlock costs nothing: deadlock-penalty-weight 0.0.
#
# CHANGES (fresh634 = fresh633 + 8 CLI values, no code changes):
#   C1 (R1) conflict-brake-weight 0.0 -> 0.8: speed deficit below
#      desired_conflict_speed (0.26) now taxed in conflict; keeps steerage
#      so the avoidance turn actually translates the hull.
#   C2 (R2) anticipatory-avoidance-turn-reward-weight 0 -> 0.8,
#      anticipatory-avoidance-turn-penalty-weight 0 -> 0.6,
#      anticipatory-avoidance-turn-distance 0 -> 3.5: CBDR contacts closer
#      than 3.5m reward turning away / punish holding the bow on; wakes
#      BEFORE the 1.5-2.0m decision-gap band found in fresh632/633 SITL.
#   C3 (R2) avoidance-turn-reward-weight 0 -> 0.6: inside the near-miss
#      band the escape turn (away from nearest neighbour) earns reward,
#      complementing COLREGS starboard shaping.
#   C4 (R3) role-speed-asymmetry-weight 0.8 -> 2.0: stand-on keeping
#      speed / give-way slowing becomes a first-order term so the pair
#      stops mutually yielding.
#   C5 (R4) deadlock-penalty-weight 0.0 -> 2.0: every step without team
#      goal-distance improvement now bleeds reward (0.04/step) -- standing
#      pressure against mutual-stall equilibria.
#   C6 anti-dither: action-smoothness-weight 0.10 -> 0.18 (P1 bang-bang
#      metrics must not regress; SITL showed +-0.3 omega sign flips at
#      sub-second scale during the standoff).
#   NOT touched: conflict-risk 1.2 / conflict-turn-relief 0.75 /
#      path-deviation-conflict-scale 0.25 / wrong-heading-conflict-relief
#      1.0 (the fresh633 turn-first set), colregs turn shaping, team
#      safety brake schedule, all scenario/curriculum branches.
#
# ANTI-FORGETTING: warmstart fresh633 final, anchor 1.0->0.35, 800k steps.
# Generation 11 chain (616->...->632->633->634); regression => rebuild
# from 633.
#
# ARBITER (priority order for fresh634 acceptance):
#   P0 standoff gate (2-boat reciprocal SITL, the 2026-08-03 setup): no
#      mutual-facing window (both |rel bearing|<35 deg, sep<3m, both
#      spd<0.12) lasting >2s; min separation >0.75m; no boat below
#      0.10 m/s for >3s while conflict is active.
#   P1 keep fresh630/631 win: bang-bang reversals stay 0, omega saturation
#      stays <10%.
#   P2 role split visible: in two-boat encounters the faster boat's mean
#      speed >=85% of cruise while the slower's is <=0.26 give-way target
#      (no more twin ~0.25/0.25 profiles).
#   P3 repeated-seed gate (scripts/eval_repeated_gate.sh, REPEATS=3): must
#      beat fresh633's 3/11 pass; pentagram_convergence must not regress.
#   P4 five_usv_dense_* checkpoint-eval must not regress vs fresh633.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh633_turn_first_5usv.pt}"
# RESUME_FROM: set to a fresh634 checkpoint scaffold (.pt) to resume an
# interrupted run with full training state instead of a bare warmstart.
RESUME_FROM="${RESUME_FROM:-}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh634_decisive_encounter_5usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh634_checkpoints}"
EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh634_eval}"
EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh634}"
EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-DECISIVE-ENCOUNTER: conflict-brake 0->0.8, anticipatory-turn NEW (0.8/0.6 @3.5m), avoidance-turn NEW 0.6, role-asymmetry 0.8->2.0, deadlock 0->2.0, smoothness 0.10->0.18}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-800000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"
TWO_USV_CROSSING_CURRICULUM_REPEATS="${TWO_USV_CROSSING_CURRICULUM_REPEATS:-0}"
TWO_USV_HEAD_ON_CURRICULUM_REPEATS="${TWO_USV_HEAD_ON_CURRICULUM_REPEATS:-0}"
ANTICIPATORY_CPA_TIME_HORIZON="${ANTICIPATORY_CPA_TIME_HORIZON:-8.0}"
ROLE_STAND_ON_COURSE_HOLD_WEIGHT="${ROLE_STAND_ON_COURSE_HOLD_WEIGHT:-0.0}"
ANTICIPATORY_TURN_REWARD_WEIGHT="${ANTICIPATORY_TURN_REWARD_WEIGHT:-0.8}"
ANTICIPATORY_TURN_PENALTY_WEIGHT="${ANTICIPATORY_TURN_PENALTY_WEIGHT:-0.6}"
ANTICIPATORY_TURN_DISTANCE="${ANTICIPATORY_TURN_DISTANCE:-3.5}"
ANTICIPATORY_CPA_DISTANCE="${ANTICIPATORY_CPA_DISTANCE:-0.0}"
ANTICIPATORY_DCPA_TARGET="${ANTICIPATORY_DCPA_TARGET:-1.15}"
ANTICIPATORY_DCPA_DEFICIT_WEIGHT="${ANTICIPATORY_DCPA_DEFICIT_WEIGHT:-0.0}"
ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT="${ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT:-0.0}"
ANTICIPATORY_CLOSING_REDUCTION_WEIGHT="${ANTICIPATORY_CLOSING_REDUCTION_WEIGHT:-0.0}"
ANTICIPATORY_CPA_GIVEWAY_ONLY="${ANTICIPATORY_CPA_GIVEWAY_ONLY:-0}"
CONFLICT_SPEED_RELIEF="${CONFLICT_SPEED_RELIEF:-0.55}"
CONFLICT_SPEED_FLOOR="${CONFLICT_SPEED_FLOOR:-0.18}"
ACTION_SMOOTHNESS_WEIGHT="${ACTION_SMOOTHNESS_WEIGHT:-0.18}"
ANGULAR_ACCEL_PENALTY_WEIGHT="${ANGULAR_ACCEL_PENALTY_WEIGHT:-0.15}"
FORWARD_SPEED_CHANGE_PENALTY_WEIGHT="${FORWARD_SPEED_CHANGE_PENALTY_WEIGHT:-0.0}"
EVAL_TWO_USV_CROSSING="${EVAL_TWO_USV_CROSSING:-0}"
UPDATE_EPOCHS="${UPDATE_EPOCHS:-10}"
MINIBATCH_SIZE="${MINIBATCH_SIZE:-256}"
POLICY_ANCHOR_WEIGHT="${POLICY_ANCHOR_WEIGHT:-1.0}"
POLICY_ANCHOR_WEIGHT_END="${POLICY_ANCHOR_WEIGHT_END:-0.35}"
POLICY_ANCHOR_EXCLUDE_CPA_CONFLICT="${POLICY_ANCHOR_EXCLUDE_CPA_CONFLICT:-0}"
ENTROPY_COEF="${ENTROPY_COEF:-0.01}"
ENTROPY_COEF_END="${ENTROPY_COEF_END:-0.003}"

CURRICULUM_EXTRA_ARGS=()
for ((i = 0; i < TWO_USV_CROSSING_CURRICULUM_REPEATS; i++)); do
  CURRICULUM_EXTRA_ARGS+=(--curriculum-scenario two_usv_crossing)
done
for ((i = 0; i < TWO_USV_HEAD_ON_CURRICULUM_REPEATS; i++)); do
  CURRICULUM_EXTRA_ARGS+=(--curriculum-scenario two_usv_head_on)
done
CPA_ROLE_FLAGS=()
if [[ "$ANTICIPATORY_CPA_GIVEWAY_ONLY" == "1" ]]; then
  CPA_ROLE_FLAGS+=(--anticipatory-cpa-giveway-only)
fi
ANCHOR_EXTRA_FLAGS=()
if [[ "$POLICY_ANCHOR_EXCLUDE_CPA_CONFLICT" == "1" ]]; then
  ANCHOR_EXTRA_FLAGS+=(--policy-anchor-exclude-cpa-conflict)
fi
CHECKPOINT_EXTRA_EVAL_ARGS=()
if [[ "$EVAL_TWO_USV_CROSSING" == "1" ]]; then
  CHECKPOINT_EXTRA_EVAL_ARGS+=(--checkpoint-eval-scenario two_usv_crossing)
fi

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR" "$EVAL_DIR"

if [[ -n "$RESUME_FROM" ]]; then
  WEIGHT_FLAGS=(--resume-from "$RESUME_FROM")
  echo "[$EXPERIMENT_NAME] RESUME_FROM=$RESUME_FROM (full resume: weights+optimizer+schedule)"
else
  WEIGHT_FLAGS=(--load-weights-from "$WARMSTART_FROM")
  echo "[$EXPERIMENT_NAME] WARMSTART_FROM=$WARMSTART_FROM"
fi
echo "[$EXPERIMENT_NAME] OUTPUT=$OUTPUT"
echo "[$EXPERIMENT_NAME] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[$EXPERIMENT_NAME] $EXPERIMENT_SUMMARY"

exec /bin/python3 -m usv_rl.train_mappo_policy \
  "${WEIGHT_FLAGS[@]}" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 16384 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  --action-mode speed_scale \
  --action-speed-scale-min "$ACTION_SPEED_SCALE_MIN" \
  --num-agents 5 \
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
  --scenario same_goal_queue \
  --scenario five_usv_dense_head_on \
  --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario same_goal_queue \
  --curriculum-scenario same_goal_queue \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario five_usv_dense_head_on \
  --curriculum-scenario five_usv_dense_head_on \
  --curriculum-scenario five_usv_dense_crossing \
  --curriculum-scenario five_usv_dense_crossing \
  --curriculum-scenario five_usv_dense_overtaking \
  --curriculum-scenario five_usv_dense_overtaking \
  "${CURRICULUM_EXTRA_ARGS[@]}" \
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
  --conflict-turn-relief 0.75 \
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
  --update-epochs "$UPDATE_EPOCHS" \
  --minibatch-size "$MINIBATCH_SIZE" \
  --max-grad-norm 0.5 \
  --entropy-coef "$ENTROPY_COEF" \
  --entropy-coef-end "$ENTROPY_COEF_END" \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --device cpu \
  --torch-num-threads 1 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -15.0 \
  --near-miss-weight 1.0 \
  --head-on-near-miss-distance 0.90 \
  --separation-recovery-weight 1.6 \
  --entanglement-penalty-weight 1.6 \
  --entanglement-distance 1.5 \
  --entanglement-grace-steps 14 \
  --entanglement-low-speed-penalty-weight 1.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  --path-deviation-penalty-weight 1.10 \
  --path-deviation-conflict-scale 0.25 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-exclude-overtaking \
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
  --role-speed-asymmetry-weight 2.0 \
  --role-giveway-speed 0.26 \
  --role-stand-on-course-hold-weight "$ROLE_STAND_ON_COURSE_HOLD_WEIGHT" \
  --wrong-heading-speed-penalty-weight 1.5 \
  --wrong-heading-speed-threshold-deg 35.0 \
  --wrong-heading-conflict-relief 1.0 \
  --far-side-turn-penalty-weight 1.5 \
  --far-side-turn-threshold-deg 45.0 \
  --goal-queue-weight 1.2 \
  --conflict-risk-weight 1.2 \
  --conflict-risk-time-horizon 15.0 \
  --conflict-risk-safe-dcpa 1.5 \
  --conflict-brake-weight 0.8 \
  --desired-conflict-speed 0.26 \
  --conflict-speed-relief "$CONFLICT_SPEED_RELIEF" \
  --conflict-speed-floor "$CONFLICT_SPEED_FLOOR" \
  --conflict-progress-scale 0.0 \
  --conflict-resolution-reward-weight 0.9 \
  --conflict-escalation-penalty-weight 0.0 \
  --unsafe-close-speed-penalty-weight 0.0 \
  --speed-distance-coupling-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 0.0 \
  --avoidance-turn-reward-weight 0.6 \
  --anticipatory-avoidance-turn-reward-weight "$ANTICIPATORY_TURN_REWARD_WEIGHT" \
  --anticipatory-avoidance-turn-penalty-weight "$ANTICIPATORY_TURN_PENALTY_WEIGHT" \
  --anticipatory-avoidance-turn-distance "$ANTICIPATORY_TURN_DISTANCE" \
  --anticipatory-cpa-distance "$ANTICIPATORY_CPA_DISTANCE" \
  --anticipatory-cpa-time-horizon "$ANTICIPATORY_CPA_TIME_HORIZON" \
  --anticipatory-dcpa-target "$ANTICIPATORY_DCPA_TARGET" \
  --anticipatory-dcpa-deficit-penalty-weight "$ANTICIPATORY_DCPA_DEFICIT_WEIGHT" \
  --anticipatory-dcpa-improvement-reward-weight "$ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT" \
  --anticipatory-closing-reduction-reward-weight "$ANTICIPATORY_CLOSING_REDUCTION_WEIGHT" \
  "${CPA_ROLE_FLAGS[@]}" \
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
  --action-smoothness-weight "$ACTION_SMOOTHNESS_WEIGHT" \
  --angular-accel-penalty-weight "$ANGULAR_ACCEL_PENALTY_WEIGHT" \
  --forward-speed-change-penalty-weight "$FORWARD_SPEED_CHANGE_PENALTY_WEIGHT" \
  --head-on-guidance-distance 7.5 \
  --crossing-guidance-distance 7.5 \
  --head-on-corridor-reward-weight 0.80 \
  --head-on-centerline-penalty-weight 0.60 \
  --head-on-forward-reward-weight 0.40 \
  --head-on-speed-drop-penalty-weight 0.80 \
  --head-on-close-penalty-weight 0.0 \
  --head-on-no-turn-penalty-weight 0.0 \
  --head-on-target-starboard-offset 0.6 \
  --crossing-forward-reward-weight 0.35 \
  --overtaking-forward-reward-weight 0.35 \
  --overtaking-corridor-reward-weight 0.9 \
  --overtaking-centerline-penalty-weight 1.1 \
  --overtaking-close-penalty-weight 1.2 \
  --team-reward-weight 0.0 \
  --team-progress-weight 0.0 \
  --coordination-reward-weight 0.0 \
  --team-completion-bonus 0.0 \
  --deadlock-penalty-weight 2.0 \
  --pure-cruise-reward-weight 0.7 \
  --pure-idle-penalty-weight 0.5 \
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
  --policy-anchor-weight "$POLICY_ANCHOR_WEIGHT" \
  --policy-anchor-weight-end "$POLICY_ANCHOR_WEIGHT_END" \
  --policy-anchor-exclude-team-safety-brake \
  "${ANCHOR_EXTRA_FLAGS[@]}" \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 12 \
  --checkpoint-eval-steps 150 \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario cluster_escape \
  --checkpoint-eval-scenario five_usv_dense_head_on \
  --checkpoint-eval-scenario five_usv_dense_crossing \
  --checkpoint-eval-scenario five_usv_dense_overtaking \
  "${CHECKPOINT_EXTRA_EVAL_ARGS[@]}" \
  --checkpoint-ranking-json "$CKPT_DIR/${EXPERIMENT_NAME}_ranking.json" \
  --checkpoint-eval-json-dir "$EVAL_DIR"
