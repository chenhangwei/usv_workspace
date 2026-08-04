#!/bin/bash
# fresh627: PATH ECONOMY + ARRIVAL QUALITY. Single reward-class change on top
# of fresh626 (all fresh626/625 weights VERBATIM otherwise). FULL-VOLUME run:
# 400k steps (2x fresh626) with the complete 16-slot curriculum and anchor to
# fresh626 -- per user directive 2026-07-10: "训练量要足够，不要遗忘，保证训练质量".
#
# MOTIVATION (fresh626 SITL star route x2, 2026-07-10, sess 185947/190654):
#   FIXED by fresh626: final-waypoint runaway GONE -- 36/36 waypoints passed,
#   zero mission cuts. KEPT: mid-route straightness.
#   REMAINING failure modes (user 4-point arbiter, priority order):
#     P1 waypoint effectiveness: ALL passes graze the acceptance-radius edge
#        (34/36 at 1.44-1.50m of the 1.5m radius, 2 at 3.0m fallback) and
#        detour>1.3x on 12/16 legs (worst 3.2x) -- corner-cutting arcs
#        instead of driving through the waypoint.
#     P2 route adherence during encounters: policy DOES slow (low-speed share
#        45-68% in encounters) but drifts sideways while doing so (max 3.24m
#        off-route); free-sail CTE p90 0.7-1.4m.
#     P3 loops: 5+3 loop events; usv_01 goal100004 (144-deg vertex) looped in
#        BOTH sessions (net turn 1270/916 deg) with NO neighbour around --
#        far-side turn choice at big heading error persists.
#     P4 entanglement: start-cluster crawl 110s at 0.52m (s1); mid-route
#        parallel lock ~96s at 1.1m (s2). No collision (never <0.5m).
#
# THE CHANGE (one reward class: path economy / arrival quality, 2 knobs):
#   1) --path-inefficiency-penalty-weight 2.0  (NEW, config.py + env)
#      Per-step penalty = w * max(0, travel_dist - progress_delta).
#      Straight-to-goal motion costs 0; lateral drift pays 1x travel; moving
#      AWAY pays ~2x; STANDING STILL PAYS 0 -> "slow down and wait" becomes
#      strictly cheaper than "swing wide around", which directly targets
#      P1 (detour arcs), P2 (sideways drift), P3 (loops burn travel with
#      zero progress -> heavily taxed).
#   2) --waypoint-pass-quality-bonus 6.0  (NEW, config.py + env)
#      Extra bonus at EVERY waypoint pass and the final arrival scaled by
#      (1 - pass_dist/goal_tolerance): center-threading earns the full 6.0,
#      edge-grazing earns ~0 -> targets P1 (effective arrival).
#   3) --pure-idle-penalty-weight 0.0 -> 0.5  (existing knob, was disabled)
#      USER CLARIFICATION 2026-07-10 19:45: "减速等待不能全部都不动，而是
#      有效的让路，绝对不能都不动". The inefficiency tax makes standing
#      still FREE, which risks a mutual-freeze equilibrium. The idle penalty
#      closes it: _target_forward_speed floors at 0.18 m/s under FULL
#      conflict, so a steerage-speed crawl-yield (~0.18) pays ZERO idle
#      penalty while a DEAD STOP pays 0.5*speed_deficit per step; when the
#      predicted risk drops, the target snaps back to cruise 0.34 (plus the
#      open-water crawl term) so the boat MUST resume promptly. Combined
#      effect: crawl-yield on-route = the unique cheap avoidance mode --
#      exactly "减速让路、风险降低后立即前进、绝不集体停止".
#   Implementation: usv_rl/config.py (2 fields), multi_agent_env.py
#   (_previous_positions tracking + 3 reward hook sites), CLI wiring in
#   train_mappo_policy.py + mappo_parallel_sampler.py. Defaults 0.0 => all
#   older run scripts reproduce their original reward surfaces exactly.
#
# ANTI-FORGETTING / QUALITY MEASURES (user directive):
#   - 400k total steps (2x fresh626'''s 200k): the new penalty reshapes the
#     dense progress signal, so the policy needs full re-equilibration, not
#     a targeted top-up.
#   - WARMSTART fresh626 final; policy anchor 1.0 -> 0.35 ANCHORED TO
#     fresh626 (preserves wide-turn transit + straightness + no-runaway).
#   - Complete 16-slot curriculum incl. all encounter scenarios (head_on,
#     crossing, overtaking, random_encounter, cluster_escape, pentagram)
#     so avoidance skills keep being rehearsed while path economy tightens.
#   - Waypoint-transition angle_offset stays at fresh626'''s +-150 deg.
#
# RISK REGISTER:
#   RISK-1 (reward complexity): +2 knobs, but one coherent class; monitor
#     mean_reward drop in first updates (expected: more negative baseline).
#   RISK-2 (avoidance suppression): inefficiency tax on evasive arcs could
#     discourage necessary avoidance. Mitigation: weight 2.0 gives ~0.07/step
#     tax at full lateral cruise vs near-miss penalty 3.0/collision -15 --
#     safety terms still dominate by >40x. Watch encounter minsep on gate.
#   RISK-4 (warm-start drift): generation 6 (616->622->624->625->626->627).
#     If fresh627 regresses, consolidation rebuild from fresh622 is REQUIRED
#     next (rebuild 624+625+626+627 recipes in one run). Ancestors kept.
#
# ARBITER (user 2026-07-10, priority order):
#   P1 waypoint effectiveness: SITL pass distances well inside the radius
#      (target: median <1.0m of the 1.5m radius), detour>1.3x legs cut to
#      <=6/16, NO mission cuts.
#   P2 route adherence: encounter CTE p90 <=1.0m, no >3m excursions; slowing
#      (not swinging) remains the avoidance mode.
#   P3 loops: zero loop events at 144-deg vertices.
#   P4 entanglement: start-cluster separation <60s, no >60s pair locks.
#   Offline gate SECONDARY: cluster 9/9 reach kept; collisions REPORT-ONLY.
#   Compare with logs/eval_fresh626_gate/.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh626_wide_turn_transit_3usv.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh627_path_economy_3usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh627_checkpoints}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-400000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

echo "[fresh627] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh627] OUTPUT=$OUTPUT"
echo "[fresh627] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh627] PATH ECONOMY: inefficiency tax 2.0 + pass-quality bonus 6.0; 400k steps, anchor to fresh626, full curriculum"

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
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
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
