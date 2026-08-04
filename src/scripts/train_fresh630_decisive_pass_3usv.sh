#!/bin/bash
# fresh630: DECISIVE PASS -- kill the mutual-crawl in encounters.
# Builds on fresh629 (weights VERBATIM unless listed). Warmstart fresh629
# final, anchor to fresh629, 600k steps, same curriculum + 1 extra
# two_usv_head_on slot.
#
# MOTIVATION (fresh629 SITL-5 2026-07-14 150108, Z-mirror 9-leg route):
#   All control-layer fixes landed (sync barrier, rotation dedup,
#   route_omega sign) -- 9/9 waypoints reached, both boats. BUT encounters
#   are HESITANT: user "相遇还是不果断，总是害怕碰撞，不够流畅".
#   Quantified: full-run mean speed 0.215/0.222 (63% of 0.34 cruise), 28%
#   of samples < 0.15 m/s; 9 encounter windows of 13-31s each, min speeds
#   0.01-0.18 m/s; min separations STILL 0.18-0.28m -- slowing bought no
#   safety, boats crawl past each other instead of committing to a pass.
#
# ROOT CAUSE (fresh629 reward audit):
#   R1 head-on speed shaping ALL ZERO (forward/speed-drop/corridor/
#      centerline) -- only turn-reward 3.0 survives; slowing in a head-on
#      is unpenalised and holding speed is unrewarded.
#   R2 route-tight corridor (tol 0.3, conflict-scale 0.4 -> max 0.7m in
#      full conflict) makes the lateral dodge EXPENSIVE (1.10/m) while
#      slowing is nearly free (time penalty 0.01) -> policy picks "slow".
#   R3 give-way shaping pays the follower to crawl at 0.14 m/s (weight
#      0.8) -- observed usv_03 encounter means 0.13-0.21 m/s.
#   R4 conflict-resolution-reward 0 -- no bonus for resolving fast, no
#      cost for dragging an encounter out 30s.
#
# CHANGES (fresh630, all existing knobs, NO code changes):
#   C1 head-on shaping ON (R1): corridor 0.8, centerline-penalty 0.6,
#      forward-reward 0.40, speed-drop-penalty 0.80, starboard offset
#      1.0 -> 0.6 (mirror pass = 2*offset = 1.2m, fits the conflict
#      corridor); crossing-forward-reward 0 -> 0.35 (SITL classifier
#      labels many mirror passes as crossing).
#   C2 conflict corridor relief (R2): path-deviation-conflict-scale
#      0.4 -> 0.9 (full-conflict tolerance 0.3+0.9=1.2m). Free-sail
#      corridor UNCHANGED at 0.3 -- route-tight stays for clear water.
#   C3 give-way flow (R3): role-giveway-speed 0.14 -> 0.20 -- follower
#      still yields but keeps flowing.
#   C4 decisiveness (R4): conflict-resolution-reward-weight 0 -> 0.5;
#      entanglement-low-speed-penalty 0.5 -> 1.0, grace 40 -> 20 --
#      mutual crawl inside 1.5m now taxed early and hard.
#   C5 head-on-near-miss-distance override 0.90 (two_usv_head_on only):
#      a clean committed 1.0-1.2m pass is NOT punished as a near-miss, so
#      the fear gradient cannot re-learn "slow = fewer penalties".
#
# ANTI-FORGETTING: 600k steps, warmstart fresh629 final, anchor 1.0->0.35.
# Generation 8 chain (616->...->629->630); regression => consolidation
# rebuild from fresh622 REQUIRED next.
#
# ARBITER (user 2026-07-14, priority order):
#   P1 decisive pass: encounter windows (sep<2.5m) mean speed >= 0.25 m/s
#      per boat, no sample < 0.10 m/s outside terminal approach; window
#      duration <= 15s (SITL-5 baseline 13-31s).
#   P2 pass quality: pair min separation >= 0.9m on mirror legs (SITL-5
#      baseline 0.18-0.28m).
#   P3 keep fresh629 wins: free-sail CTE p90 <= 0.5m, 0 loops, all
#      waypoints reached, same-goal queue ordered arrival.
#   P4 collisions REPORT-ONLY.
#   Offline gate SECONDARY: compare with logs/eval_fresh629_gate/.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh629_route_tight_queue_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh630_decisive_pass_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh630_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-600000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh630] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh630] OUTPUT=$OUTPUT"
echo "[fresh630] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh630] DECISIVE PASS: head-on shaping ON (fwd 0.40/drop 0.80/corridor 0.80/offset 0.6), conflict-scale 0.9, giveway 0.20, resolution 0.5, entangle-low-speed 1.0@20, head-on near-miss 0.90; anchor to fresh629"

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
  --head-on-near-miss-distance 0.90 \
  --separation-recovery-weight 0.5 \
  --entanglement-penalty-weight 1.0 \
  --entanglement-distance 1.5 \
  --entanglement-grace-steps 20 \
  --entanglement-low-speed-penalty-weight 1.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  --path-deviation-penalty-weight 1.10 \
  --path-deviation-conflict-scale 0.9 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-exclude-overtaking \
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
  --role-speed-asymmetry-weight 0.8 \
  --role-giveway-speed 0.20 \
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
  --conflict-resolution-reward-weight 0.5 \
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
  --policy-anchor-exclude-team-safety-brake
