#!/bin/bash
# fresh633: TURN-FIRST -- eliminate bow-to-bow standoffs found in the
# 2026-07-24 08:22 5-USV SITL batch. Builds on fresh632 (weights VERBATIM
# unless listed). Warmstart fresh632 final, anchor to fresh632, 800k steps.
#
# MOTIVATION (SITL diagnosis 2026-07-24, see
#   memories/repo/sitl_20260724_5usv_encounter_hesitation_diagnosis.md):
#   Deployed fresh632 with 5 concurrent USVs. User-priority finding: vessels
#   approach BOW-TO-BOW, stall, then squeeze past ("头对头" standoffs; user
#   explicitly deprioritized the residual slowdown itself). Evidence:
#     - 20 mutual-facing windows (|rel bearing| both <60 deg, sep<5m, >2s),
#       132.2s total; worst pair usv_04<->usv_05 reached minsep 0.66m
#       (strict <35 deg standoff), usv_02<->usv_03 minsep 0.25m.
#     - Bow stays goal-locked at close range: at neighbor sep<3m the heading
#       error is <15 deg for 80-93% of samples while mean |omega| is only
#       12-25% of the 0.70 bound. Two goal-locked bows on crossing routes
#       ARE the head-to-head geometry.
#     - CBDR blindness: 49.3% of approaching samples are constant-bearing/
#       decreasing-range (collision course) with mean max|omega| only 26%
#       of authority at mean sep 3.66m -- plenty of room, no turn.
#     - Turn direction is wrong more often than right: toward-neighbor
#       37.6% vs away-from-neighbor 33.0% (rest ~no turn).
#     - Decision gap: |omega| usage bottoms out (14.2-14.7%) exactly in the
#       1.5-2.0m band where a decisive turn should happen; speed is already
#       cut 35% there. Braking substitutes for turning.
#
# ROOT CAUSE:
#   R1 conflict-risk-weight=0.0 (fresh632): the ONLY predictive DCPA/TCPA
#      pressure was disabled, so nothing rewards opening the bearing early.
#      Observation already carries tcpa/dcpa per neighbor (slots 6..9).
#   R2 wrong-heading speed tax (1.5 @ 35 deg, unconditional) binds "turn
#      hard" to "slow down": any avoidance turn >35 deg off-goal while
#      keeping speed is taxed, so the policy learned the cheaper option --
#      keep bow on goal and brake. Root of the standoff geometry.
#   R3 path/heading tracking keeps too much authority in conflict:
#      path-deviation-conflict-scale 0.6 leaves 66% of the CTE penalty
#      active mid-encounter; conflict-turn-relief 0.60 caps yaw authority.
#
# CHANGES (fresh633 = fresh632 + 1 code knob + 3 CLI values):
#   C1 (R2, CODE) NEW --wrong-heading-conflict-relief 1.0: the fresh628
#      wrong-heading speed tax is now scaled by
#      (1 - relief*min(conflict_risk,1)) -- fully waived at max conflict,
#      untouched in open water (waypoint-switch fix retained verbatim).
#   C2 (R1) conflict-risk-weight 0.0 -> 1.2 (default 1.5 damped: warmstart
#      policy has never seen this term; horizon 15s / safe-dcpa 1.5m kept,
#      so risk wakes at ~4-5m and rewards opening the bearing BEFORE 2m).
#   C3 (R3) path-deviation-conflict-scale 0.6 -> 0.25 (CTE penalty nearly
#      free mid-encounter; full discipline returns when conflict clears).
#   C4 (R3) conflict-turn-relief 0.60 -> 0.75 (more yaw authority retained
#      at full conflict; pairs with C1 so the freed authority gets used).
#   NOT touched: role-giveway-speed 0.26, entanglement/separation weights,
#      turn-reward / straight-line-omega knobs (P1 anti-oscillation set),
#      colregs-port-turn-penalty 5.0, all scenario/curriculum branches.
#
# ANTI-FORGETTING: warmstart fresh632 final, anchor 1.0->0.35, 800k steps.
# Generation 10 chain (616->...->632->633); regression => rebuild from 632.
#
# ARBITER (priority order for fresh633 acceptance):
#   P0 NEW head-to-head gate: no mutual-facing window (both |rel bearing|
#      <35 deg) with sep<3m lasting >2s in a 5-USV SITL batch; CBDR share
#      of approaching samples <25% (from 49.3%); away-turn share > toward-
#      turn share at 0.8-3m (from 33.0% vs 37.6%).
#   P1 keep fresh630/631 win: bang-bang reversals stay 0, omega saturation
#      stays <10% (fresh632 SITL: PASS, must not regress).
#   P2 decisiveness unchanged targets: give-way mean speed >=75% of cruise;
#      <0.15 m/s time share <15%.
#   P3 no frozen-separation window >3s (|d(sep)/dt|<0.04, sep<1.2m).
#   P4 five_usv_dense_* checkpoint-eval must not regress vs fresh632.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh632_scale_decisive_5usv.pt}"
# RESUME_FROM: set to a fresh633 checkpoint scaffold (.pt) to resume an
# interrupted run with full training state (optimizer momentum, LR/entropy
# schedule position, completed_timesteps) instead of a bare weight warmstart.
# Takes priority over WARMSTART_FROM/--load-weights-from when set.
RESUME_FROM="${RESUME_FROM:-}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh633_turn_first_5usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh633_checkpoints}"
EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh633_eval}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-800000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR" "$EVAL_DIR"

if [[ -n "$RESUME_FROM" ]]; then
  WEIGHT_FLAGS=(--resume-from "$RESUME_FROM")
  echo "[fresh633] RESUME_FROM=$RESUME_FROM (full resume: weights+optimizer+schedule)"
else
  WEIGHT_FLAGS=(--load-weights-from "$WARMSTART_FROM")
  echo "[fresh633] WARMSTART_FROM=$WARMSTART_FROM"
fi
echo "[fresh633] OUTPUT=$OUTPUT"
echo "[fresh633] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh633] TURN-FIRST: conflict-risk-weight 0.0->1.2, wrong-heading-conflict-relief NEW 1.0, path-deviation-conflict-scale 0.6->0.25, conflict-turn-relief 0.60->0.75; warmstart+anchor fresh632"

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
  --role-speed-asymmetry-weight 0.8 \
  --role-giveway-speed 0.26 \
  --wrong-heading-speed-penalty-weight 1.5 \
  --wrong-heading-speed-threshold-deg 35.0 \
  --wrong-heading-conflict-relief 1.0 \
  --far-side-turn-penalty-weight 1.5 \
  --far-side-turn-threshold-deg 45.0 \
  --goal-queue-weight 1.2 \
  --conflict-risk-weight 1.2 \
  --conflict-risk-time-horizon 15.0 \
  --conflict-risk-safe-dcpa 1.5 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 0.0 \
  --conflict-resolution-reward-weight 0.9 \
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
  --deadlock-penalty-weight 0.0 \
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
  --policy-anchor-weight 1.0 \
  --policy-anchor-weight-end 0.35 \
  --policy-anchor-exclude-team-safety-brake \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 12 \
  --checkpoint-eval-steps 150 \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario cluster_escape \
  --checkpoint-eval-scenario five_usv_dense_head_on \
  --checkpoint-eval-scenario five_usv_dense_crossing \
  --checkpoint-eval-scenario five_usv_dense_overtaking \
  --checkpoint-ranking-json "$CKPT_DIR/fresh633_ranking.json" \
  --checkpoint-eval-json-dir "$EVAL_DIR"
