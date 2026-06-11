#!/usr/bin/env bash
# =============================================================================
# fresh506: 10× COLREGS reward shaping, warm-start from fresh505 stage-A pilot.
#
# fresh505 result: 15/15 zero-collision + 1.0 success across head_on/crossing/
# overtaking, BUT mean_colregs_violation_ratio = 0.55–1.00 (real violations).
# Diagnosis: shaping weights 0.3–0.5 dominated by progress(1.0) + goal(80).
#
# fresh506 change list (config-only, no source edits):
#   1. 10× all four COLREGS shaping weights:
#        head_on_turn_reward            0.3 -> 3.0
#        crossing_starboard_turn        0.3 -> 3.0
#        overtaking_starboard_turn      0.3 -> 3.0
#        colregs_port_turn_penalty      0.5 -> 5.0
#   2. Warm-start from fresh505_stagea_pilot.pt (already avoids all 3 scenarios).
#   3. Zero out BC teachers — fresh505 already learned crossing/overtaking
#      avoidance; teachers only tightened min_sep from 3.0 -> 1.9. Let PPO
#      explore the COLREGS gradient cleanly now.
#   4. Budget: 100k steps / ~50 updates / ~2.5h CPU.
#   5. Domain id 231 (avoid clash with any lingering 230).
#   6. Log -> logs/fresh506_train.log (NOT /tmp, which got swept).
#
# Decision gate (3 scenarios × 5 seeds = 15 episodes):
#   - PASS  : colP >= 0.5 AND each scenario coll == 0 AND succ >= 0.7
#             -> proceed to Stage A.2 (scale to 3-USV + denser scenes).
#   - WEAK  : colP 0.3–0.5 with no regression -> consider 20× or longer budget.
#   - FAIL  : colP < 0.3 OR avoidance regressed -> shaping alone insufficient,
#             escalate to Stage A.2 obs change (per-neighbor role one-hot).
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

RESUME_FROM="${RESUME_FROM:-/mnt/data/checkpoints/usv_rl/fresh505_stagea_pilot.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh506_colregs_10x.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh506_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-300000}"  # cumulative: fresh505 finished at 200000, so this adds 100k new steps
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-231}"

echo "[fresh506] RESUME_FROM=$RESUME_FROM"
echo "[fresh506] OUTPUT=$OUTPUT"
echo "[fresh506] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh506] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

exec python3 -m usv_rl.train_mappo_policy \
  --resume-from "$RESUME_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: same 2-agent multi-scenario pool as fresh505 -----` \
  --num-agents 2 \
  --max-agents 2 \
  --max-neighbors 1 \
  --scenario two_usv_head_on \
  --scenario two_usv_crossing \
  --scenario two_usv_overtaking \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario two_usv_crossing \
  --curriculum-scenario two_usv_overtaking \
  --scenario-spawn-position-std 0.05 \
  --scenario-spawn-heading-std 0.02 \
  --scenario-goal-position-std 0.05 \
  \
  `# ----- Episode timing (unchanged) -----` \
  --episode-timeout 90.0 \
  --no-progress-timeout 60.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  \
  `# ----- Controller / kinematics (unchanged) -----` \
  --squash-actions \
  --min-forward-speed 0.0 \
  --min-forward-speed-floor 0.0 \
  --linear-delta-limit 0.30 \
  --angular-delta-limit 0.50 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.50 \
  --heading-omega-deadband 0.09 \
  --heading-omega-reference 0.95 \
  --angular-authority-power 1.30 \
  --angular-accel-limit 1.05 \
  --angular-decel-limit 3.00 \
  --angular-authority-floor 0.45 \
  --conflict-turn-relief 0.80 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  \
  `# ----- Network (unchanged) -----` \
  --hidden-size 256 --hidden-size 256 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --actor-log-std-init -1.20 \
  \
  `# ----- Observation normalization on (continues from fresh505) -----` \
  --normalize-observations \
  \
  `# ----- PPO core (smaller LR, smaller entropy — fine-tune phase) -----` \
  --learning-rate 1.0e-4 \
  --learning-rate-end 3.0e-5 \
  --clip-range 0.2 \
  --ppo-policy-loss-scale 1.0 \
  --ppo-value-loss-scale 0.5 \
  --value-coef 0.5 \
  --update-epochs 10 \
  --minibatch-size 256 \
  --max-grad-norm 0.5 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.002 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --device cpu \
  --torch-num-threads 1 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  \
  `# ----- Reward base (preserve fresh504/505 hardened terms) -----` \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -400.0 \
  --near-miss-weight 10.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  \
  `# ----- COLREGS reward shaping: 10× fresh505 -----` \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  \
  `# ----- All other reward shaping still 0 -----` \
  --path-deviation-penalty-weight 0.0 \
  --path-deviation-conflict-scale 0.0 \
  --conflict-risk-weight 0.0 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 0.0 \
  --conflict-resolution-reward-weight 0.0 \
  --conflict-escalation-penalty-weight 0.0 \
  --unsafe-close-speed-penalty-weight 0.0 \
  --speed-distance-coupling-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 0.0 \
  --straight-line-omega-penalty-weight 0.0 \
  --heading-error-weight 0.0 \
  --action-smoothness-weight 0.0 \
  --head-on-corridor-reward-weight 0.0 \
  --head-on-centerline-penalty-weight 0.0 \
  --head-on-forward-reward-weight 0.0 \
  --head-on-speed-drop-penalty-weight 0.0 \
  --head-on-close-penalty-weight 0.0 \
  --head-on-no-turn-penalty-weight 0.0 \
  --crossing-forward-reward-weight 0.0 \
  --overtaking-forward-reward-weight 0.0 \
  --overtaking-corridor-reward-weight 0.0 \
  --overtaking-centerline-penalty-weight 0.0 \
  --overtaking-close-penalty-weight 0.0 \
  --team-reward-weight 0.0 \
  --team-progress-weight 0.0 \
  --coordination-reward-weight 0.0 \
  --team-completion-bonus 0.0 \
  --deadlock-penalty-weight 0.0 \
  --pure-cruise-reward-weight 0.0 \
  --pure-idle-penalty-weight 0.0 \
  --pure-turn-penalty-weight 0.0 \
  --pure-spin-penalty-weight 0.0 \
  \
  `# ----- All BC teachers OFF (fresh505 already taught avoidance; let PPO follow COLREGS gradient cleanly) -----` \
  --crossing-imitation-weight 0 --crossing-imitation-weight-end 0 \
  --overtaking-imitation-weight 0 --overtaking-imitation-weight-end 0 \
  --random-deconflict-weight 0 --random-deconflict-weight-end 0 \
  --random-role-balance-weight 0 --random-role-balance-weight-end 0 \
  --random-pairwise-role-guard-weight 0 --random-pairwise-role-guard-weight-end 0 \
  --random-safe-finish-weight 0 --random-safe-finish-weight-end 0 \
  --random-goal-hold-weight 0 --random-goal-hold-weight-end 0 \
  --random-offroute-finish-weight 0 --random-offroute-finish-weight-end 0 \
  --random-cte-recovery-weight 0 --random-cte-recovery-weight-end 0 \
  --random-clear-ahead-weight 0 --random-clear-ahead-weight-end 0 \
  --random-late-lagging-weight 0 --random-late-lagging-weight-end 0 \
  --random-close-formation-escape-weight 0 --random-close-formation-escape-weight-end 0 \
  --near-goal-finish-weight 0 --near-goal-finish-weight-end 0 \
  --lagging-finish-weight 0 --lagging-finish-weight-end 0 \
  --team-safety-brake-weight 0 --team-safety-brake-weight-end 0 \
  --policy-anchor-weight 0 --policy-anchor-weight-end 0
