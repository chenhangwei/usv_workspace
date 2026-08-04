#!/bin/bash
# fresh631: UNTANGLE -- kill the orbit lock and the conflict-corridor abuse.
# Builds on fresh630 (weights VERBATIM unless listed). Warmstart fresh630
# final, anchor to fresh630, 600k steps, +1 cluster_escape curriculum slot.
#
# MOTIVATION (fresh630 SITL 2026-07-15 174403/174944/175946, 3 courses):
#   Pentagram 3USV: clean (min pair sep 2.54m+). Z-mirror 2USV: 9/9 done,
#   min sep 0.21m. Figure-8 2USV: 25/25 done BUT first encounter locked
#   into a BINARY ORBIT for 135s -- separation FROZEN at 0.42+/-0.02m,
#   opposite near-constant omegas (-0.5/+0.6), both creeping ~0.11 m/s,
#   encounter classifier unset at that range. Straight legs: CTE>0.3 for
#   36-44% of samples, 40-54% of those with neighbour >=5m -- policy
#   learnt "any conflict = free 1.2m corridor" from fresh630 C2.
#
# ROOT CAUSE (fresh630 reward audit):
#   R1 conflict corridor TOO GENEROUS: tolerance 0.3 + 0.9*conflict ->
#      1.2m free offset under ANY conflict; mirror courses keep a mild
#      conflict level alive nearly the whole leg -> mid-leg drift 0.4-0.9m.
#   R2 orbit lock unpunished: separation-rate never enters the reward --
#      separation_recovery 0.5 is too weak to matter against the stable
#      two-body rotation (sep frozen => recovery term = 0 forever, no
#      gradient pushes OUT of the orbit).
#
# CHANGES (fresh631, all existing knobs, NO code changes):
#   C1 conflict corridor tightened (R1): path-deviation-conflict-scale
#      0.9 -> 0.6 (full-conflict tolerance 0.3+0.6=0.9m -- still fits the
#      0.6m starboard-offset dodge, no longer pays a 1.2m wander).
#   C2 active disengagement (R2): separation-recovery-weight 0.5 -> 1.2
#      -- growing the pair separation while inside 1.5*near-miss now pays
#      HARD, making "turn out and leave" strictly better than orbiting.
#   C3 +1 cluster_escape curriculum slot (spawns pairwise 0.7-0.9m --
#      exactly the orbit-lock geometry) -> 3 slots total.
#
# DEPLOYMENT PAIRING: the online node got an ORBIT-LOCK BREAKER +
# push-through steering 0.85 + route gate relaxation (conflict 0.30,
# separation 2.0) on 2026-07-15; fresh631 internalises the same behaviour
# so the breaker becomes a dormant safety net.
#
# ANTI-FORGETTING: 600k steps, warmstart fresh630 final, anchor 1.0->0.35.
# Generation 8 chain (616->...->630->631); regression => consolidation
# rebuild from fresh622 REQUIRED next.
#
# ARBITER (user 2026-07-15, priority order):
#   P1 no orbit lock: every encounter window (sep<1.5m) must show a
#      separation-rate sign change within <=10s; no frozen-separation
#      (|d sep/dt|<0.04 for >5s) segments.
#   P2 route adherence: straight-leg CTE p90 <= 0.4m INCLUDING mild
#      mutual-approach phases (SITL-fresh630 baseline: CTE>0.3 for 36-44%).
#   P3 keep fresh630 wins: decisive pass speed, 9/9 + 25/25 completion,
#      pentagram cleanliness.
#   P4 collisions REPORT-ONLY.
#   Offline gate SECONDARY: compare with logs/eval_fresh630_gate/.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh630_decisive_pass_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh631_untangle_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh631_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-600000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh631] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh631] OUTPUT=$OUTPUT"
echo "[fresh631] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh631] UNTANGLE: conflict-scale 0.9->0.6, separation-recovery 0.5->1.2, +1 cluster_escape slot; keeps fresh630 decisive-pass shaping; anchor to fresh630"

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
  --separation-recovery-weight 1.2 \
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
  --path-deviation-conflict-scale 0.6 \
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
