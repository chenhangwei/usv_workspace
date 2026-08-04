#!/bin/bash
# fresh629: RULE INTERNALISATION + ROUTE-TIGHT + SAME-GOAL QUEUE.
# Builds on fresh628 (weights VERBATIM unless listed). 600k steps (1.5x
# fresh628, user: "把减少偏离航线再加大训练量"), full curriculum + new
# same_goal_queue scenario, anchor to fresh628.
#
# MOTIVATION (fresh628 + deployment-assist stack, SITL 2026-07-12 104604):
#   The assist stack (turn governor / short-side assist / clear-ahead route
#   gate / role-aware scaling / role latch / master push-through) reached ALL
#   user acceptance lines: 15/15 legs detour-free, 0 loops, pair min
#   separations 3.99/3.86/1.62m. USER DIRECTIVES for this run (2026-07-12):
#     D1 "减少规则依赖" -- internalise the assists into the policy;
#     D2 "减少偏离航线+加大训练量; 预测没有很大碰撞也过渡避让, 不要过渡避让";
#     D3 "同时同方向到达同一目标点 -> 同航点排队" (the fresh628 near-contact
#        at the shared final vertex, sess 102710, had NO COLREGS class).
#
# CODE CHANGES (fresh629, defaults 0/off keep older scripts intact):
#   1) far-side-turn penalty (config+env): tax the omega component that turns
#      the LONG way while |he|>45 deg -> internalises the deployment
#      short-side turn assist (D1).
#   2) wrong-heading tax threshold 60->35 deg (existing knob): SITL switch
#      transients measured he0 22-62 deg, mostly BELOW the old 60 threshold;
#      35 makes the speed tax actually engage at real switches ->
#      internalises the turn governor (D1).
#   3) same_goal_queue scenario (multi_agent_scenarios.py): 2-3 boats sail
#      PARALLEL SAME-direction to goals clustered 0.5-0.9m apart -- exactly
#      the shared-waypoint convergence with no COLREGS class (D3).
#   4) goal-queue shaping (config+env): near a shared goal cluster the boat
#      FARTHER from the goal is the queue follower -- rewarded for holding
#      <=0.12 m/s and >=1.2m separation, penalised for pressing in; leader
#      keeps normal progress rewards. Deterministic leader choice (distance,
#      then ID) mirrors the deployment master election (D3).
#   5) TRAINING-BRIDGE ROLE LATCH (multi_agent_bridge.py): same latch the
#      deployment node got after sess 102710 (role kept inside 2.5m until
#      separation >3.5m) -- train/deploy observation parity (D1).
#   6) ROUTE-TIGHT + ANTI-OVER-AVOIDANCE (D2):
#      --path-deviation-tolerance 0.8 -> 0.3  (free corridor halved twice:
#        the policy learnt "0.5m parallel offset is free"; now it is not)
#      --path-deviation-penalty-weight 0.90 -> 1.10
#      --near-miss-weight 1.5 -> 1.0 (user allows collisions; near-miss fear
#        is what pushes boats off-route on LOW predicted risk)
#      --path-inefficiency stays 2.0; conflict-scale stays 0.4 so risk-gated
#        corridor relief still exists for REAL conflicts only.
#
# ANTI-FORGETTING: 600k steps, warmstart fresh628 final, anchor 1.0->0.35
# ANCHORED TO fresh628, full 16-slot curriculum + same_goal_queue slots.
# Generation 8 chain (616->...->628->629); regression => consolidation
# rebuild from fresh622 REQUIRED next.
#
# ARBITER (user 2026-07-12, priority order):
#   P1 route adherence: free-sail CTE p90 <= 0.5m WITHOUT the clear-ahead
#      gate; no over-avoidance swings on low predicted risk (CRI<0.15).
#   P2 same-goal queue: shared-vertex approach -> ordered arrival, pair min
#      separation >= 1.0m, zero mutual-yield stalls.
#   P3 keep fresh628+assists wins with assists progressively DISABLED in
#      SITL A/B (turn assist off first, then route gate off).
#   P4 collisions REPORT-ONLY.
#   Offline gate SECONDARY: compare with logs/eval_fresh628_gate/.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh628_role_asymmetry_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh629_route_tight_queue_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh629_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-600000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh629] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh629] OUTPUT=$OUTPUT"
echo "[fresh629] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh629] ROUTE-TIGHT+QUEUE: tolerance 0.3, near-miss 1.0, far-side-turn 1.5, wrong-heading@35deg, goal-queue 1.2, same_goal_queue scenario; 600k steps, anchor to fresh628"

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
  --scenario same_goal_queue \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario pentagram_convergence \
  --curriculum-scenario same_goal_queue \
  --curriculum-scenario same_goal_queue \
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
  --near-miss-weight 1.0 \
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
  --path-deviation-penalty-weight 1.10 \
  --path-deviation-conflict-scale 0.4 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-exclude-overtaking \
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
  --role-speed-asymmetry-weight 0.8 \
  --role-giveway-speed 0.14 \
  --wrong-heading-speed-penalty-weight 1.5 \
  --wrong-heading-speed-threshold-deg 35.0 \
  --far-side-turn-penalty-weight 1.5 \
  --far-side-turn-threshold-deg 45.0 \
  --goal-queue-weight 1.2 \
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
