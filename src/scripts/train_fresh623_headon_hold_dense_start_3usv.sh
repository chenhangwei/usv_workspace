#!/bin/bash
# fresh623: fresh622 + TWO targeted fixes from the 2026-07-06 evidence pair
# (36-ep offline gate + first-ever full 3-USV SITL star-route completion).
#
# WHERE fresh622 STANDS (why warm-start fresh622, not fresh616):
#   offline gate (36 ep, /tmp/diag_622): cluster_escape 9/9 reach (best ever),
#   three_usv_crossing 0/3 collisions, three_usv_overtaking 0/3 + 9/9 reach.
#   SITL star route (~/usv_logs 2026-07-06): ALL 3 USVs completed 6/6
#   waypoints and arrived (first time any model finished the mission);
#   mid-route legs clean (detour 1.0-2.0x, CTE mean 0.14-0.20m, cruise
#   0.25-0.31 m/s). Only TWO failure modes remain, both understood:
#
# FIX A -- two_usv_head_on 3/3 collisions (offline gate): frame-by-frame
#   trace shows usv_01 DID start the starboard turn early (CTE -0.76 by
#   step 30) but then CUT BACK to the centerline mid-pass (CTE -0.59 by
#   step 40, heading error back to 0) while usv_02 barely offset; combined
#   lateral clearance <1m -> minsep 0.58-0.74 -> collision. fresh620 (which
#   passed head_on 3/3) HELD CTE ~-0.97 until the pass completed.
#   ROOT CAUSE: fresh622 (like all recent runs) zeroed the entire designed
#   head_on corridor suite (config.py defaults corridor 2.4 / centerline 2.8
#   / close 2.2 / no_turn 1.4 / forward 1.4 / speed_drop 1.4) keeping only
#   turn_reward 3.0 -- so the reward paid for STARTING the turn but nothing
#   paid for HOLDING the starboard offset until the opposing boat is past;
#   route-hugging terms then won the tug-of-war mid-pass. This is the exact
#   same failure pattern as the fresh605 overtaking fix (suite designed,
#   scripts zeroed it, restoring it fixed the behavior).
#   THE FIX (CLI-only): restore the designed head_on suite verbatim:
#     --head-on-corridor-reward-weight 2.4   (pay for holding starboard offset)
#     --head-on-centerline-penalty-weight 2.8 (punish drifting back to center)
#     --head-on-close-penalty-weight 2.2     (punish closing while centered)
#     --head-on-no-turn-penalty-weight 1.4   (punish not turning when urgent)
#     --head-on-forward-reward-weight 1.4    (keep driving through the pass)
#     --head-on-speed-drop-penalty-weight 1.4 (no stalling mid-avoidance)
#   Scope: _compute_head_on_guidance_reward only fires for an opposing-flow
#   neighbour dead ahead within 7.5m -- crossing/overtaking/cluster shaping
#   untouched.
#
# FIX B -- dense-start spiral entanglement (SITL): boats started with
#   pairwise gaps ~0.9-1.5m in an ASYMMETRIC almost-line configuration;
#   usv_02/03 spun in place for 219-283s (net turn 4976-6100 deg = 14-17
#   full circles, detour 6.5-10.0x) before dispersing, with minsep 0.17-0.56m
#   -- a direct violation of the no-entanglement requirement. After escape,
#   everything was clean, so the deficit is ONLY the ultra-dense uneven
#   start. cluster_escape trained a FIXED symmetric ring (radius 0.5,
#   pairwise always 0.87m), so lopsided clusters were out of distribution.
#   THE FIX (multi_agent_scenarios.py): cluster_escape now randomizes ring
#   radius 0.45-0.70 with +/-30deg spawn-angle jitter (validated over 500
#   draws: tightest pair 0.78-1.18m, never below collision_distance+margin
#   so no t=0 terminations) and +/-25deg PER-AGENT goal-bearing jitter
#   (min goal gap >=72deg -- still dispersal, not a crossing drill).
#
# EVERYTHING ELSE IS fresh622 VERBATIM: balanced 15-slot curriculum
# (head_on x2, crossing x2, overtaking x2, cluster x2, solo/waypoint/clear
# x6, random x1), anti-entanglement suite, CRI horizon 15s / safe DCPA 1.5,
# head-on+crossing guidance 7.5m, clear-ahead trio, conflict-gated heading
# convergence, turn-speed coupling, 400k steps, anchor 1.0->0.35 (anchored
# to the warm-start = fresh622, protecting its proven cluster/crossing/
# overtaking/terminal-capture skills for the whole run).
#
# Deployment target: Raspberry Pi 5 -- model is a small MLP+attention actor
# (3.3MB, obs 82, act 2), already runs on CPU in SITL; no size change here.
#
# Gate for fresh623 (must pass ALL simultaneously, 36-ep offline):
#   cluster_escape 9/9 reach, head_on 0/3 coll, crossing 0/3, overtaking 0/3.
#   Then 3-USV SITL star route: first-leg net turn should collapse from
#   ~5000-6000deg toward <1500deg, no multi-minute start spiral.

set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh622_balanced_no_entangle_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh623_headon_hold_dense_start_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh623_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-400000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-294}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh623] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh623] OUTPUT=$OUTPUT"
echo "[fresh623] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh623] fresh622 recipe + head_on corridor suite restored (hold the offset) + cluster_escape dense asymmetric spawns"

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
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario waypoint_turn \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario three_usv_clear_route \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario cluster_escape \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
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
  --separation-recovery-weight 0.5 \
  --entanglement-penalty-weight 1.0 \
  --entanglement-distance 2.0 \
  --entanglement-grace-steps 25 \
  --entanglement-low-speed-penalty-weight 0.5 \
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
  --head-on-corridor-reward-weight 2.4 \
  --head-on-centerline-penalty-weight 2.8 \
  --head-on-forward-reward-weight 1.4 \
  --head-on-speed-drop-penalty-weight 1.4 \
  --head-on-close-penalty-weight 2.2 \
  --head-on-no-turn-penalty-weight 1.4 \
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
