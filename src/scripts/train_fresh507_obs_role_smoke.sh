#!/usr/bin/env bash
# =============================================================================
# fresh507: A.2-β observation smoke test.
#
# Goal: verify that extending neighbor observation 10 -> 15 dims (3-hot
# encounter + 2-hot role per neighbor) does NOT regress the fresh506 baseline
# when warm-starting from the locked colregs_2usv_v1 checkpoint.
#
# Change set vs fresh506 (config-only; obs change comes from code in A.2-β):
#   1. Load weights only from colregs_2usv_v1.pt (locked fresh506 baseline,
#      neighbor_dim=10). We MUST use --load-weights-only (not --resume-from)
#      because the resume path does not migrate attention key/value projection
#      shapes when neighbor_feature_dim changes; load-weights-only invokes the
#      _migrate_attention_actor_layout / _migrate_attention_critic_layout
#      helpers that zero-pad the new 5 dims in attention K/V projections plus
#      the critic first-layer columns. Optimizer state is intentionally
#      discarded (would shape-mismatch anyway).
#   2. TOTAL_TIMESTEPS = 50000. With --load-weights-only there is no resume
#      training state, so completed_timesteps starts at 0 and 50000 means a
#      direct 50k smoke (~1.5h CPU). Different semantics from --resume-from
#      runs (fresh506 used cumulative 300000 against a 200000 resume baseline).
#   3. BASE_ROS_DOMAIN_ID = 220. Trainer also spawns an eval subprocess at
#      base + num_sampler_workers (=221), so we keep margin below the ROS 2
#      Linux ceiling of 232.
#   4. OUTPUT = fresh507_obs_role_smoke.pt (separate from baseline).
#   5. Reward/shaping/scenarios/network: IDENTICAL to fresh506. Only the obs
#      layout differs.
#
# Decision gate (3 scenarios x 5 seeds = 15 episodes via eval scripts):
#   - PASS  : colP >= 0.5 AND each scenario coll == 0 AND succ >= 0.7
#             -> proceed to Stage A.2 step 5 (scale to 3/5-USV scenarios).
#   - WEAK  : minor regression (colP 0.4-0.5, no collision regression) ->
#             extend warmstart to 100-200k more steps; obs change is fine but
#             policy needs more updates to exploit new signals.
#   - FAIL  : collisions reappear OR colP < 0.3 OR succ drops below 0.5 ->
#             revisit migration code (likely actor weight padding) before
#             proceeding.
# =============================================================================
set -eo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

RESUME_FROM="${RESUME_FROM:-/mnt/data/checkpoints/usv_rl/baselines/colregs_2usv_v1.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh507_obs_role_smoke.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh507_checkpoints}"
mkdir -p "$(dirname "$OUTPUT")" "$CKPT_DIR"

TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-50000}"  # direct (no resume state); 50k smoke
ROLLOUT_STEPS="${ROLLOUT_STEPS:-2048}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}"

echo "[fresh507] RESUME_FROM=$RESUME_FROM"
echo "[fresh507] OUTPUT=$OUTPUT"
echo "[fresh507] TOTAL_TIMESTEPS=$TOTAL_TIMESTEPS  ROLLOUT_STEPS=$ROLLOUT_STEPS"
echo "[fresh507] BASE_ROS_DOMAIN_ID=$BASE_ROS_DOMAIN_ID"

exec python3 -m usv_rl.train_mappo_policy \
  --load-weights-from "$RESUME_FROM" \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval "$CHECKPOINT_INTERVAL" \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --rollout-steps "$ROLLOUT_STEPS" \
  \
  `# ----- Topology: same 2-agent multi-scenario pool as fresh506 -----` \
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
  `# ----- Observation normalization on (migrated from fresh506 baseline) -----` \
  --normalize-observations \
  \
  `# ----- PPO core (same as fresh506 fine-tune phase) -----` \
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
  `# ----- Reward base (preserve fresh506) -----` \
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
  `# ----- All BC teachers OFF (same as fresh506) -----` \
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
