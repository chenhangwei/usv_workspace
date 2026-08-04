#!/bin/bash
# fresh628: STRICT MASTER/SLAVE ROLES + EFFECTIVE TURN. Builds on fresh627
# (all fresh627 weights VERBATIM unless listed). 400k steps, full curriculum,
# anchor to fresh627.
#
# MOTIVATION (fresh627 SITL star route x2 2026-07-11, sess 083054/084818):
#   FIXED by fresh627: loops 8->1; encounter CTE == free CTE (no more
#   run-away avoidance); final-arrival quality 0.14-0.67m; start-cluster
#   separation 110s->25s. REMAINING (user 5-point directive 2026-07-11):
#     P-A switch-turn 100% inefficient (30/30): need only 6-85 deg but boats
#         hold FULL 0.34 speed with he 135-173 deg for ~10s (omega~0.05),
#         sail away, then U-arc back: 106-242 deg net turn, 2.4-9.8m and
#         12-24s wasted per switch; ~50% far-side initial turn. This is THE
#         detour driver (detour>1.3x rose to 30/33 legs).
#     P-B no role asymmetry: encounter speeds nearly identical (0.26-0.33).
#         User: roles MUST be strict master/slave per COLREGS -- one stand-on
#         passes FAST, one give-way yields lightly, never both-master or
#         both-slave. And: do NOT fear collisions (allow them).
#
# CODE CHANGES (this run's variables; defaults 0 keep old scripts intact):
#   1) classify_encounter_role REWRITE (multi_agent_types.py):
#      - STRICT COMPLEMENT: every engaged pair labels exactly one GIVE_WAY +
#        one STAND_ON. Overtaking role = longitudinal order (fixes body_x<=0
#        -> NONE bug where the overtaken boat never knew it was stand-on).
#        Crossing role = starboard rule (Rule 15) both directions. Head-on +
#        near-reciprocal ambiguous band = deterministic ID tie-break
#        (usv_01 < usv_02 => 01 stands on; user operational rule overrides
#        Rule 14 both-give-way).
#      - WORLD/BODY FRAME FIX: production callers pass world-frame vectors;
#        classifier now takes own_yaw and rotates internally. The old code
#        treated world vectors as body vectors => silent mislabeling for any
#        heading != east, in TRAINING and SITL alike. Bridge +
#        policy_inference_node + evaluate_mappo_policy all pass yaw + ids now
#        (train/deploy/eval parity, verified by 8/8 symmetric-pair unit
#        checks incl. north/oblique headings).
#   2) --role-speed-asymmetry-weight 0.8, --role-giveway-speed 0.14 (NEW):
#      risk-gated: stand-on rewarded for HOLDING cruise; give-way rewarded
#      for slowing toward 0.14 (+/- symmetric penalty for overspeed);
#      gate ~ conflict_risk/0.30 so the give-way boat resumes fast as the
#      predicted risk decays ("风险降低后快速通过").
#   3) --wrong-heading-speed-penalty-weight 1.5, threshold 60 deg (NEW):
#      taxes forward speed by (1-cos(he)) beyond 60 deg. he=135 deg at
#      v=0.34 costs ~0.87/step vs progress ~0.34 -> the sail-away transient
#      becomes strictly unprofitable; slow-spin-then-go wins. Targets P-A.
#   4) --near-miss-weight 3.0 -> 1.5 (user: "不要担心碰撞，允许碰撞发生"):
#      halves proximity fear so the stand-on boat actually holds course and
#      speed through close passes. Collision penalty -15 unchanged.
#
# ANTI-FORGETTING: 400k steps, full 16-slot curriculum, warmstart fresh627
# final, anchor 1.0->0.35 anchored to fresh627 (keeps loop fix + route
# discipline + arrival quality). Generation 7 chain; if fresh628 regresses,
# consolidation rebuild from fresh622 is REQUIRED next.
#
# ARBITER (user 2026-07-11, priority order):
#   P1 switch-turn efficiency: net turn per switch < 2x required turn,
#      no sail-away >2m past the vertex, switch settle < 8s.
#   P2 role asymmetry visible: in every conflict window the pair shows a
#      clear speed split (stand-on >= 0.28, give-way <= 0.20).
#   P3 keep fresh627 wins: loops <= 1, encounter CTE p90 <= 1.0m, arrivals
#      36/36, no runaway. Detour>1.3x legs back down toward <= 8/33.
#   P4 collisions REPORT-ONLY (explicitly allowed by user).
#   Offline gate SECONDARY: compare with logs/eval_fresh627_gate/.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh627_path_economy_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh628_role_asymmetry_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh628_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-400000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh628] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh628] OUTPUT=$OUTPUT"
echo "[fresh628] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh628] ROLE ASYMMETRY: strict master/slave + wrong-heading speed tax 1.5 + role split 0.8; near-miss 1.5; 400k steps, anchor to fresh627"

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
  --near-miss-weight 1.5 \
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
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
  --role-speed-asymmetry-weight 0.8 \
  --role-giveway-speed 0.14 \
  --wrong-heading-speed-penalty-weight 1.5 \
  --wrong-heading-speed-threshold-deg 60.0 \
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
  --policy-anchor-exclude-team-safety-brake
