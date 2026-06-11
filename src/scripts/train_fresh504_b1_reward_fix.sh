#!/usr/bin/env bash
# =============================================================================
# fresh504-b1: "Harden the safety signal" — reward-misspec fix on top of fresh503.
#
# Purpose
# -------
# fresh503 PROVED that PPO learns when teachers are off and reward is minimal,
# but it also exposed a reward bug: collision_penalty=-50 was TRADEABLE against
# accumulated progress reward, so PPO converged to a deterministic "crash for
# cash" policy (5/5 head-on collisions, 0% COLREGS compliance).
#
# B1 fix = identical recipe to fresh503 with ONLY 3 reward params changed:
#   --collision-penalty   -50  -> -400  (make collision strictly negative-EV)
#   --near-miss-weight     4   -> 10    (steepen the close-quarters gradient)
#   --goal-bonus           30  -> 80    (avoid the trivial "freeze in place"
#                                        local optimum after we punish dashing)
#
# Everything else (teachers off, normalize-obs on, PPO canonical hypers, single
# head-on scenario, 50k steps) is byte-identical to fresh503 so the only
# variable changed is the reward function.
#
# Decision gate (after eval):
#   - PASS  if success_rate >= 0.7 AND collision_rate == 0 over >=5 head-on seeds
#           -> green light Stage A (curriculum + colregs role obs).
#   - WEAK  if collision_rate drops but success < 0.7 (likely freeze/timeout)
#           -> tune goal_bonus / add gentle starboard nudge, not full overhaul.
#   - FAIL  if collision_rate still > 0.2 -> escalate to B2 (zero collided-ep
#           progress) or B3 (PPO-Lagrangian).
#
# IMPORTANT: this is a *pilot*, not a deployable policy. Output should never be
# evaluated against the multi-USV core5 acceptance gate — that gate applies to
# 3-USV random_encounter, not single-scenario head-on.
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# --- Pre-flight: source ROS workspace so PYTHONPATH is correct ---------------
# shellcheck disable=SC1091
source ../install/setup.bash
# Prepend live src so source edits take effect without rebuild.
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

# --- Outputs ----------------------------------------------------------------
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh504_b1_reward_fix.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh504_b1_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-50000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-229}"

echo "[fresh504-b1] OUTPUT=$OUTPUT"
echo "[fresh504-b1] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh504-b1] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

# --- Launch the trainer -----------------------------------------------------
exec python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: single scenario, 2 agents -----` \
  --num-agents 2 \
  --max-agents 2 \
  --max-neighbors 1 \
  --scenario two_usv_head_on \
  --curriculum-scenario two_usv_head_on \
  --scenario-spawn-position-std 0.05 \
  --scenario-spawn-heading-std 0.02 \
  --scenario-goal-position-std 0.05 \
  \
  `# ----- Episode timing -----` \
  --episode-timeout 90.0 \
  --no-progress-timeout 60.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  \
  `# ----- Controller / kinematics (unchanged from baseline) -----` \
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
  `# ----- Network -----` \
  --hidden-size 256 --hidden-size 256 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --actor-log-std-init -1.20 \
  \
  `# ----- Observation normalization (LEARNING, not frozen) -----` \
  --normalize-observations \
  \
  `# ----- PPO core hyperparameters (CANONICAL, not crippled) -----` \
  --learning-rate 3.0e-4 \
  --learning-rate-end 1.0e-4 \
  --clip-range 0.2 \
  --ppo-policy-loss-scale 1.0 \
  --ppo-value-loss-scale 0.5 \
  --value-coef 0.5 \
  --update-epochs 10 \
  --minibatch-size 256 \
  --max-grad-norm 0.5 \
  --entropy-coef 0.02 \
  --entropy-coef-end 0.005 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --device cpu \
  --torch-num-threads 1 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  \
  `# ----- Reward: minimalist RL signal -----` \
  `# Keep alive: progress, goal, collision, light near-miss, light time` \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -400.0 \
  --near-miss-weight 10.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  \
  `# Zero out ALL reward shaping (let RL discover its own behavior)` \
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
  --head-on-turn-reward-weight 0.0 \
  --head-on-forward-reward-weight 0.0 \
  --head-on-speed-drop-penalty-weight 0.0 \
  --head-on-close-penalty-weight 0.0 \
  --head-on-no-turn-penalty-weight 0.0 \
  --crossing-starboard-turn-reward-weight 0.0 \
  --crossing-forward-reward-weight 0.0 \
  --overtaking-starboard-turn-reward-weight 0.0 \
  --overtaking-forward-reward-weight 0.0 \
  --overtaking-corridor-reward-weight 0.0 \
  --overtaking-centerline-penalty-weight 0.0 \
  --overtaking-close-penalty-weight 0.0 \
  --colregs-port-turn-penalty-weight 0.0 \
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
  `# ----- ALL TEACHERS OFF -----` \
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
  --crossing-imitation-weight 0 --crossing-imitation-weight-end 0 \
  --overtaking-imitation-weight 0 --overtaking-imitation-weight-end 0 \
  --near-goal-finish-weight 0 --near-goal-finish-weight-end 0 \
  --lagging-finish-weight 0 --lagging-finish-weight-end 0 \
  --team-safety-brake-weight 0 --team-safety-brake-weight-end 0 \
  --policy-anchor-weight 0 --policy-anchor-weight-end 0
