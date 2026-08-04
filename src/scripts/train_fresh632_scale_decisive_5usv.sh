#!/bin/bash
# fresh632: SCALE + DECISIVE -- fix multi-vessel scaling regression and residual
# creeping/convoy clamping found in the 2026-07-17 13:12-13:58 SITL batch.
# Builds on fresh631 (weights VERBATIM unless listed). Warmstart fresh631
# final, anchor to fresh631, 800k steps, +5-agent dense curriculum.
#
# MOTIVATION (SITL diagnosis 2026-07-17, see
#   memories/repo/sitl_20260717_1312_1358_avoidance_smoothness_diagnosis.md):
#   Deployed fresh631 (3-agent trained, num_agents=3) was field-tested this
#   time with 4-5 concurrent USVs (pentagram 01/02/04/05, figure-8 with
#   01/02/03/04/05 all in the neighbor graph). Findings:
#     - Bang-bang / omega-saturation ("钳制" in the oscillation sense) is
#       SOLVED: saturation <8%, 0 bang-bang reversals in every scenario.
#       fresh630/631's decisive-pass fix holds. DO NOT touch turn-reward /
#       straight-line-omega-penalty knobs.
#     - "犹豫" (creeping / not committing) is NOT solved: usv_02 mean speed
#       in Z-mirror/figure-8 is only 63-64% of cruise (0.227-0.230 vs 0.35-
#       0.36 proxy), 29% of samples <0.15 m/s -- almost IDENTICAL to the
#       pre-fresh630 fresh629 baseline (63%, 28%). role-giveway-speed=0.20
#       (fresh628/630) is the direct driver: give-way vessels target only
#       59% of cruise and linger there for tens of seconds per encounter.
#     - Figure-8 (now run with 5 concurrent USVs instead of fresh631's 2-
#       USV validation) shows the residual orbit-lock/entanglement problem
#       AMPLIFIED, not fixed, at higher agent density: usv_02<->usv_03
#       stuck within 3m for 220.2s (minsep 0.269m), usv_02<->usv_04 stuck
#       for 150.3s (minsep 0.249m) -- both longer than the 93.8s fresh631
#       recorded for the 2-USV case.
#     - Pentagram (cleanest scenario, 4 USV) still shows a mild same-
#       direction "convoy" clamping between usv_04/usv_05: 4 separate
#       encounter windows covering almost the whole 476s run, longest 75s
#       at minsep 1.11-1.44m with 5.4s meeting the frozen-separation test.
#       This is the OLD convoy-clamping root cause (fresh520), only
#       partially damped by the existing distance-only entanglement penalty.
#
# ROOT CAUSE:
#   R1 give-way creep floor too low/too sticky: role-giveway-speed=0.20 is
#      only 59% of cruise (0.34) and conflict-resolution-reward-weight=0.5
#      is too weak to make "resolve and re-accelerate" beat "sit at 0.20".
#   R2 entanglement anti-clamp too weak for convoy geometry: distance-only
#      penalty (weight 1.0, grace 20 steps) was tuned against orbit-lock
#      (low relative speed) and only partially suppresses same-direction
#      "escort" proximity where both vessels keep moving forward together.
#   R3 no training exposure above 3 concurrent agents: fresh630/631 curric-
#      ulum tops out at three_usv_*; the deployed field tests now regularly
#      run 4-5 USVs simultaneously (existing five_usv_dense_* scenarios were
#      never included because num-agents was pinned to 3).
#
# CHANGES (fresh632, all existing knobs + curriculum, NO code changes):
#   D1 decisive give-way (R1): role-giveway-speed 0.20 -> 0.26 (76% of
#      cruise, still meaningfully slower than stand-on to preserve COLREGS
#      asymmetry); conflict-resolution-reward-weight 0.5 -> 0.9 (reward
#      actively shrinking conflict risk instead of parking at giveway speed).
#   D2 stronger anti-clamp (R2): entanglement-penalty-weight 1.0 -> 1.6,
#      entanglement-grace-steps 20 -> 14 (react sooner to sustained close
#      proximity regardless of relative speed -- covers convoy AND orbit
#      geometries); separation-recovery-weight 1.2 -> 1.6 (push harder once
#      inside the near-miss band so pairs don't drift back into it).
#   D3 scale generalization (R3): num-agents 3 -> 5 (max-agents stays 5,
#      unchanged architecture/global-state size -- no checkpoint remap
#      needed); add five_usv_dense_head_on/_crossing/_overtaking to both
#      --scenario (deployable branch set) and --curriculum-scenario
#      (oversampled) so the policy gets real gradient on >=5 concurrent
#      agents. All fresh631 scenario branches kept verbatim; smaller
#      scenarios auto-fill unused agent slots with spectators.
#   D4 real per-checkpoint eval instead of the broken offline-gate stub:
#      --auto-evaluate-checkpoints across the two problem scenarios
#      (two_usv_head_on, cluster_escape) plus the three new five_usv_dense_*
#      scenarios, ranked and dumped to JSON every checkpoint.
#
# ANTI-FORGETTING: 800k steps (up from 600k -- more scenario branches need
# exposure), warmstart fresh631 final, anchor 1.0->0.35. Generation 9 chain
# (616->...->631->632); regression => consolidation rebuild from fresh622.
#
# ARBITER (priority order for fresh632 acceptance):
#   P1 keep fresh630/631 win: bang-bang reversals stay 0, omega saturation
#      stays <10% across all scenarios (do not reintroduce oscillation).
#   P2 decisiveness: give-way mean speed during encounters >=75% of cruise
#      (up from 63-64%); <0.15 m/s time share <15% (down from 29%).
#   P3 no clamping/orbit-lock/convoy: no encounter window may sustain
#      |d(sep)/dt|<0.04 with sep<1.2m for >3s (down from up to 5.4s
#      pentagram / 3.3s figure-8 observed this batch).
#   P4 scale generalization: five_usv_dense_* checkpoint-eval completion
#      rate and collision rate must not regress vs fresh631 baseline on the
#      three_usv_* scenarios; report-only for collisions elsewhere.
#   Offline gate: SKIPPED (known broken stub, see fresh631 session notes).
#      Real validation is the built-in --auto-evaluate-checkpoints JSON
#      output plus a follow-up manual SITL batch (pentagram/Z/figure-8)
#      after training completes.
set -eo pipefail
cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh631_untangle_3usv.pt}"
# RESUME_FROM: set to a fresh632 checkpoint scaffold (.pt) to resume an
# interrupted run with full training state (optimizer momentum, LR/entropy
# schedule position, completed_timesteps) instead of a bare weight warmstart.
# Takes priority over WARMSTART_FROM/--load-weights-from when set.
RESUME_FROM="${RESUME_FROM:-}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh632_scale_decisive_5usv.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh632_checkpoints}"
EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh632_eval}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-800000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-296}"
ACTION_SPEED_SCALE_MIN="${ACTION_SPEED_SCALE_MIN:-0.30}"

mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR" "$EVAL_DIR"

if [[ -n "$RESUME_FROM" ]]; then
  WEIGHT_FLAGS=(--resume-from "$RESUME_FROM")
  echo "[fresh632] RESUME_FROM=$RESUME_FROM (full resume: weights+optimizer+schedule)"
else
  WEIGHT_FLAGS=(--load-weights-from "$WARMSTART_FROM")
  echo "[fresh632] WARMSTART_FROM=$WARMSTART_FROM"
fi
echo "[fresh632] OUTPUT=$OUTPUT"
echo "[fresh632] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh632] SCALE+DECISIVE: num-agents 3->5 (+five_usv_dense_*), role-giveway-speed 0.20->0.26, conflict-resolution 0.5->0.9, entanglement-weight 1.0->1.6/grace 20->14, separation-recovery 1.2->1.6; anchor to fresh631"

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
  --path-deviation-conflict-scale 0.6 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-exclude-overtaking \
  --path-inefficiency-penalty-weight 2.0 \
  --waypoint-pass-quality-bonus 6.0 \
  --role-speed-asymmetry-weight 0.8 \
  --role-giveway-speed 0.26 \
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
  --checkpoint-ranking-json "$CKPT_DIR/fresh632_ranking.json" \
  --checkpoint-eval-json-dir "$EVAL_DIR"
