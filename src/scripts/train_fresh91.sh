#!/bin/bash
# fresh91: full-scenario branch-only repair after sampler/env config fixes.
#
# Rationale:
#   - fresh87 was the first full-scenario scenario-trunk run, but it trained before
#     the parallel sampler reward/env config parity fixes.
#   - fresh88/fresh89/fresh90 unfroze the shared actor/attention and degraded the
#     strong fresh86 overtaking trunk into high-collision hard-scenario behaviour.
#   - train_mappo_policy now remaps scenario branch tensors by scenario name, so a
#     single-scenario trunk checkpoint can safely seed the matching branch inside a
#     five-scenario deployable checkpoint.
#
# Goal:
#   Keep one pure-RL MAPPO checkpoint with trainer-side scenario trunks.  Freeze the
#   shared base again, preserve the fresh86 overtaking trunk by name, initialize the
#   other scenario trunks from the shared base policy, and train only scenario trunks
#   with hard-scenario oversampling under the fixed sampler config.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh86_checkpoints/fresh86_overtaking_scenario_trunk_step_0001152.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh91_full_scenario_trunk_branch_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh91_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh91_train.log}"
RUN_NAME="${RUN_NAME:-fresh91}"

mkdir -p "$CKPT_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  exit 1
fi

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

{
  echo "========== ${RUN_NAME} Full-Scenario Scenario-Trunk Branch Repair (24K from fresh86 best) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1200 \
  --num-agents 3 \
  --total-timesteps 24000 \
  --clip-range 0.06 \
  --learning-rate 8.0e-6 \
  --learning-rate-end 2.0e-6 \
  --entropy-coef 0.0006 \
  --entropy-coef-end 0.00005 \
  --actor-log-std-init -0.8 \
  --force-actor-log-std -0.8 \
  --episode-timeout 72.0 \
  --no-progress-timeout 22.0 \
  --min-progress-delta 0.24 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.9 \
  --scenario-spawn-heading-std 0.36 \
  --scenario-goal-position-std 0.9 \
  --encounter-type-dropout 0.08 \
  --cte-clip-range 5.0 \
  --progress-weight 5.4 \
  --goal-bonus 42.0 \
  --time-penalty 0.06 \
  --stall-penalty -45.0 \
  --heading-error-weight 4.0 \
  --heading-relief-factor 0.90 \
  --heading-correction-reward-weight 1.0 \
  --action-smoothness-weight 7.5 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --forward-speed-change-penalty-weight 1.5 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.30 \
  --team-reward-weight 0.55 \
  --team-progress-weight 1.65 \
  --team-goal-proximity-weight 0.30 \
  --team-regression-penalty-weight 0.90 \
  --separation-recovery-weight 2.5 \
  --team-dispersion-penalty-weight 0.20 \
  --team-dispersion-margin 0.90 \
  --entanglement-penalty-weight 2.8 \
  --entanglement-distance 4.5 \
  --entanglement-grace-steps 7 \
  --entanglement-low-speed-penalty-weight 3.5 \
  --collision-penalty -750.0 \
  --near-miss-distance 5.5 \
  --near-miss-weight 22.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 5.8 \
  --conflict-distance 10.0 \
  --anticipation-distance 10.5 \
  --head-on-guidance-distance 6.0 \
  --head-on-target-starboard-offset 0.60 \
  --head-on-phase-gate-strength 0.45 \
  --crossing-starboard-turn-reward-weight 2.2 \
  --crossing-forward-reward-weight 1.1 \
  --colregs-port-turn-penalty-weight 3.0 \
  --proximity-gradient-penalty-weight 8.0 \
  --proximity-gradient-distance 6.4 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 6.5 \
  --unsafe-close-speed-penalty-weight 2.5 \
  --conflict-overspeed-penalty-weight 8.0 \
  --avoidance-turn-reward-weight 7.0 \
  --desired-conflict-speed 0.13 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.65 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.30 \
  --goal-proximity-speed-relief 0.0 \
  --near-goal-idle-penalty-weight 1.0 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.5 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --squash-actions \
  --min-forward-speed 0.05 \
  --angular-delta-limit 0.40 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.12 \
  --dr-heading-noise-std 0.025 \
  --dr-velocity-noise-ratio 0.04 \
  --dr-current-speed-max 0.05 \
  --dr-velocity-exec-noise 0.06 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 1.0 \
  --angular-accel-limit 1.1 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.55 \
  --angular-authority-floor 0.35 \
  --min-forward-speed-floor 0.0 \
  --collision-distance 0.75 \
  --sim-tau-linear 0.6 \
  --sim-tau-angular 0.35 \
  --dr-tau-linear-low 0.4 \
  --dr-tau-linear-high 1.0 \
  --dr-tau-angular-low 0.2 \
  --dr-tau-angular-high 1.0 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 213 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} full-scenario branch repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
