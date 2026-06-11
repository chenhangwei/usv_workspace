#!/usr/bin/env bash
# =============================================================================
# fresh503-pilot: "Unleash PPO" — strategic reset experiment.
#
# Purpose
# -------
# Prove that the current trainer can do real PPO learning when:
#   1) all BC teachers are turned OFF (their distillation losses dominated grads),
#   2) reward shaping is reduced to the minimum (progress, goal, collision,
#      light near-miss, light time pressure) so the optimisation target is
#      unambiguous,
#   3) PPO hyperparameters return to canonical values (LR 3e-4, clip 0.2,
#      policy-loss-scale 1.0, value-loss-scale 0.5, update-epochs 10),
#   4) the actor base is NOT frozen, no policy anchor, no warm-start.
#
# Scope: only the `two_usv_head_on` scenario, 2 agents, ~50k env-steps.
# Decision gate (after this run completes):
#   - if mean_reward rises monotonically AND entropy decays from ~init to <0.5
#     AND eval head-on success >= 0.7 with collision == 0 → green light for the
#     full Stage-A reset (fresh504+).
#   - else → diagnose (likely reward signal sparsity or env reset bug); do not
#     proceed to fresh504 with the same recipe.
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
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh503_pilot_unleash_ppo.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh503_pilot_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-50000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-2048}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}"

echo "[fresh503-pilot] OUTPUT=$OUTPUT"
echo "[fresh503-pilot] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh503-pilot] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

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
  --goal-bonus 30.0 \
  --collision-penalty -50.0 \
  --near-miss-weight 4.0 \
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
