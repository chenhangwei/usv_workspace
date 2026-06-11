#!/usr/bin/env bash
# =============================================================================
# fresh505: Stage A pilot — "rules-aware avoidance" on top of fresh504-b1.
#
# fresh504-b1 proved 5/5 head-on success + 0 collisions, but COLREGS compliance
# is still 0 (both USVs converge to the same deterministic side, not Rule 14).
# It also has never seen crossing or overtaking encounters.
#
# Stage A pilot adds, with NO source changes:
#   1. Scenario pool: head_on + crossing + overtaking (round-robin per reset).
#   2. Modest COLREGS reward shaping (was all 0 in fresh504):
#        head_on_turn_reward=0.3,  crossing_starboard_turn_reward=0.3,
#        overtaking_starboard_turn_reward=0.3, colregs_port_turn_penalty=0.5
#   3. Three light cosine-decay BC teachers (head-on left bare; already solved):
#        crossing_imitation        0.10 -> 0
#        overtaking_imitation      0.10 -> 0
#        random_deconflict         0.05 -> 0
#   4. Warm-start: --resume-from fresh504_b1 ckpt (preserve head-on policy).
#   5. Budget: 200k steps / ~50 updates / ~5h CPU.
#
# Decision gate (3 scenarios × 5 seeds = 15 episodes):
#   - PASS  : per-scenario success >= 0.7, collision == 0,
#             AND mean COLREGS compliance >= 0.5
#             -> Stage A.2 (per-neighbor role obs + 3-USV scenarios).
#   - WEAK  : avoidance OK but COLREGS < 0.3 -> needs per-neighbor role obs
#             (jump to Stage A.2 early).
#   - FAIL  : any scenario collision > 0.2 -> teachers too aggressive,
#             halve all teacher weights and retry.
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

RESUME_FROM="${RESUME_FROM:-/mnt/data/checkpoints/usv_rl/fresh504_b1_reward_fix.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh505_stagea_pilot.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh505_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-200000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-230}"

echo "[fresh505] RESUME_FROM=$RESUME_FROM"
echo "[fresh505] OUTPUT=$OUTPUT"
echo "[fresh505] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh505] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

exec python3 -m usv_rl.train_mappo_policy \
  --resume-from "$RESUME_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: 2 agents, multi-scenario pool -----` \
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
  `# ----- Controller / kinematics (unchanged from fresh504) -----` \
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
  `# ----- Observation normalization on (will continue learning) -----` \
  --normalize-observations \
  \
  `# ----- PPO core hyperparameters (canonical; smaller LR vs fresh504 because warmstart) -----` \
  --learning-rate 1.5e-4 \
  --learning-rate-end 5.0e-5 \
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
  \
  `# ----- Reward: keep fresh504 hardened base, add COLREGS shaping -----` \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -400.0 \
  --near-miss-weight 10.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  \
  `# ----- COLREGS reward shaping (NEW: was 0 in fresh504) -----` \
  --head-on-turn-reward-weight 0.3 \
  --crossing-starboard-turn-reward-weight 0.3 \
  --overtaking-starboard-turn-reward-weight 0.3 \
  --colregs-port-turn-penalty-weight 0.5 \
  \
  `# ----- All other reward shaping still 0 (don't reintroduce fresh503 noise) -----` \
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
  `# ----- Light cosine-decay BC teachers (head-on left bare: already solved) -----` \
  --crossing-imitation-weight 0.10 --crossing-imitation-weight-end 0.0 \
  --overtaking-imitation-weight 0.10 --overtaking-imitation-weight-end 0.0 \
  --random-deconflict-weight 0.05 --random-deconflict-weight-end 0.0 \
  \
  `# ----- All other teachers stay OFF -----` \
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
