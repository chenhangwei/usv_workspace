#!/usr/bin/env bash
# =============================================================================
# fresh509a (Stage 1 of curriculum): 2-USV warmup with 3-USV architecture.
#
# Goal: rebuild the AttentionCritic's mlp.0 layer (which gained ~50% zero
# columns when migrating fresh507 obs37 -> 3-USV layout) on FAMILIAR 2-USV
# dynamics, before facing the harder 3-USV scenarios in Stage 2.
#
# Why fresh508 failed:
#   - 100k steps + LR decay 1e-4 -> 3e-5 was not enough for critic to learn
#     the new 3-USV global state layout. Loss stayed >150, reward all-negative,
#     crossing degenerated to "drift away" (timeout 5/5), overtaking/random
#     lost avoidance (coll 5/5 and 4/5).
#
# Strategy:
#   - Architecture: num_agents=2 (only two actors), max_agents=3 + max_neighbors=2
#     so critic sees the SAME 3-USV layout it will need in Stage 2 (slot 3 = zero).
#   - Scenarios: only 2-USV (head_on / crossing / overtaking). The actor sees the
#     same scenario branches it was trained on as fresh507; only the critic needs
#     to relearn the wider global state.
#   - LR held constant at 5e-5 (no decay): smaller than the original 1e-4 to
#     avoid wrecking the actor, larger than fresh508's terminal 3e-5 so critic
#     can actually move.
#   - 50k steps, ~1.5h CPU. Decision gate next: critic loss must drop below
#     20 and mean_reward must turn POSITIVE before Stage 2.
#
# Stage 2 (fresh509b) will warmstart from this checkpoint, expand num_agents to
# 3, swap to 3-USV scenarios, and run ~200k steps.
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/baselines/colregs_2usv_v1_obs37.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh509a2_criticonly.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh509a2_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-24000}"
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-228}"

echo "[fresh509a] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh509a] OUTPUT=$OUTPUT"
echo "[fresh509a] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh509a] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

exec python3 -m usv_rl.train_mappo_policy \
  --load-weights-from "$WARMSTART_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: 2 actors, 3-USV arch (critic sees max_agents=3, max_nb=2) -----` \
  --num-agents 2 \
  --max-agents 3 \
  --max-neighbors 2 \
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
  --episode-timeout 90.0 \
  --no-progress-timeout 60.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  \
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
  --hidden-size 256 --hidden-size 256 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --actor-log-std-init -1.20 \
  \
  --normalize-observations \
  --freeze-observation-normalizer \
  \
  `# ----- PPO core: LR HELD at 5e-5 (no decay) so critic can actually move -----` \
  --learning-rate 2.0e-4 \
  --learning-rate-end 2.0e-4 \
  --clip-range 0.2 \
  --ppo-policy-loss-scale 0.0 \
  --ppo-value-loss-scale 1.0 \
  --value-coef 0.5 \
  --update-epochs 10 \
  --minibatch-size 256 \
  --max-grad-norm 0.5 \
  --entropy-coef 0.0 \
  --entropy-coef-end 0.0 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --device cpu \
  --torch-num-threads 1 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -400.0 \
  --near-miss-weight 10.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  \
  --head-on-turn-reward-weight 3.0 \
  --crossing-starboard-turn-reward-weight 3.0 \
  --overtaking-starboard-turn-reward-weight 3.0 \
  --colregs-port-turn-penalty-weight 5.0 \
  \
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
