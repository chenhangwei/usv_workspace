#!/usr/bin/env bash
# =============================================================================
# fresh508: A.2 step5-a — scale to 3-USV scenarios on the obs37 baseline.
#
# Goal: warmstart from fresh507 (colregs_2usv_v1_obs37.pt, neighbor_dim=15,
# max_neighbors=1) into a 3-USV training pool, exercising the new 3-hot
# encounter + 2-hot role per-neighbor channels under denser interaction.
#
# Why step5-a (not jumping straight to 5-USV):
#   - obs migration here only changes max_neighbors 1 -> 2 (attention seq
#     dim, no feature-dim change), so --load-weights-from is the safe path.
#   - keeps a single dimension of change vs fresh507 to attribute regressions
#     cleanly. fresh509 will scale to 5-USV after this gates PASS.
#
# Change set vs fresh507:
#   1. --num-agents 3  --max-agents 3  --max-neighbors 2
#   2. Scenario pool: three_usv_crossing, three_usv_overtaking,
#      three_usv_random_encounter (all 3-USV native scenarios).
#   3. TOTAL_TIMESTEPS = 100000 (~3-4h CPU; 3-USV rollout slower than 2-USV).
#   4. BASE_ROS_DOMAIN_ID = 222 (eval subprocess = 223; safe vs ceiling 232,
#      and clear of fresh507's 220/221).
#   5. Warmstart via --load-weights-from /baselines/colregs_2usv_v1_obs37.pt
#      (obs feature dim unchanged at 15; max_neighbors widening handled by
#      the attention migration helpers).
#   6. OUTPUT = fresh508_3usv_obs37.pt.
#   7. Reward shaping: identical to fresh506/507 (10x COLREGS terms only).
#      Per user note (min_sep ~1m is the sweet spot, >2m wastes path), we
#      do NOT widen safe-distance terms here. A dedicated corridor reward
#      pass is deferred to a follow-up sweep once 3-USV is stable.
#
# Decision gate (3 scenarios x 5 seeds = 15 episodes via eval scripts):
#   - PASS  : per-scenario coll == 0 AND succ >= 0.7 AND overall colP >= 0.5
#             -> proceed to fresh509 (5-USV dense scenarios).
#   - WEAK  : minor regression on one scenario (e.g. random_encounter colP
#             0.3-0.5, no collision) -> extend 100-200k more steps.
#   - FAIL  : collisions reappear OR overall colP < 0.3 -> revisit attention
#             seq-length migration OR add curriculum (head_on warmstart ->
#             crossing -> random) before scaling further.
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/baselines/colregs_2usv_v1_obs37.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh508_3usv_obs37.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh508_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-100000}"  # direct; --load-weights-from resets completed counter
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-222}"

echo "[fresh508] WARMSTART_FROM=$WARMSTART_FROM"
echo "[fresh508] OUTPUT=$OUTPUT"
echo "[fresh508] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh508] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

exec python3 -m usv_rl.train_mappo_policy \
  --load-weights-from "$WARMSTART_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: 3 controlled agents, native 3-USV scenarios -----` \
  --num-agents 3 \
  --max-agents 3 \
  --max-neighbors 2 \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_random_encounter \
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
  `# ----- Observation normalization on (migrated from obs37 baseline) -----` \
  --normalize-observations \
  \
  `# ----- PPO core (same as fresh506/507 fine-tune phase) -----` \
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
  `# ----- Reward base (preserve fresh506/507) -----` \
  --progress-weight 1.0 \
  --goal-bonus 80.0 \
  --collision-penalty -400.0 \
  --near-miss-weight 10.0 \
  --time-penalty 0.01 \
  --stall-penalty -5.0 \
  \
  `# ----- COLREGS reward shaping: 10x (fresh506 PASS config) -----` \
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
  `# ----- All BC teachers OFF (same as fresh506/507) -----` \
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
