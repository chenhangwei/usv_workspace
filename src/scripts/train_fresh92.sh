#!/bin/bash
# fresh92: crossing-only branch repair from the best fresh91 full-scenario trunk.
#
# Rationale:
#   - fresh91 fixed branch remapping/env parity and modestly improved random encounter,
#     but three_usv_crossing still stayed at 100% collision in selected eval.
#   - Because scenario trunks are routed by scenario id, a rollout curriculum containing only
#     three_usv_crossing updates the crossing trunk while keeping the other deployed scenario
#     trunks in the same single checkpoint.
#   - Observation-normalizer statistics are frozen so this crossing-only repair does not shift
#     the input scaling for solo/head_on/overtaking/random branches.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh91_checkpoints/fresh91_full_scenario_trunk_branch_repair_step_0015870.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh92_crossing_branch_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh92_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh92_train.log}"
RUN_NAME="${RUN_NAME:-fresh92}"

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
  echo "========== ${RUN_NAME} Crossing Branch Repair (12K from fresh91 selected best) =========="
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
  --total-timesteps 12000 \
  --clip-range 0.05 \
  --learning-rate 6.0e-6 \
  --learning-rate-end 1.2e-6 \
  --entropy-coef 0.0005 \
  --entropy-coef-end 0.00005 \
  --actor-log-std-init -0.85 \
  --force-actor-log-std -0.85 \
  --episode-timeout 72.0 \
  --no-progress-timeout 24.0 \
  --min-progress-delta 0.22 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --scenario-spawn-position-std 0.75 \
  --scenario-spawn-heading-std 0.30 \
  --scenario-goal-position-std 0.75 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 4.8 \
  --goal-bonus 42.0 \
  --time-penalty 0.055 \
  --stall-penalty -45.0 \
  --heading-error-weight 3.2 \
  --heading-relief-factor 0.95 \
  --heading-correction-reward-weight 0.8 \
  --action-smoothness-weight 5.8 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --forward-speed-change-penalty-weight 1.2 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.18 \
  --team-reward-weight 0.60 \
  --team-progress-weight 1.50 \
  --team-goal-proximity-weight 0.25 \
  --team-regression-penalty-weight 0.80 \
  --separation-recovery-weight 3.5 \
  --team-dispersion-penalty-weight 0.35 \
  --team-dispersion-margin 0.80 \
  --entanglement-penalty-weight 3.2 \
  --entanglement-distance 4.8 \
  --entanglement-grace-steps 6 \
  --entanglement-low-speed-penalty-weight 3.0 \
  --collision-penalty -900.0 \
  --near-miss-distance 5.8 \
  --near-miss-weight 28.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 5.8 \
  --conflict-distance 12.0 \
  --anticipation-distance 12.0 \
  --head-on-guidance-distance 6.0 \
  --head-on-target-starboard-offset 0.60 \
  --head-on-phase-gate-strength 0.45 \
  --crossing-starboard-turn-reward-weight 5.5 \
  --crossing-forward-reward-weight 2.4 \
  --colregs-port-turn-penalty-weight 5.0 \
  --proximity-gradient-penalty-weight 10.0 \
  --proximity-gradient-distance 7.0 \
  --speed-distance-coupling-penalty-weight 8.0 \
  --speed-distance-coupling-threshold 7.2 \
  --unsafe-close-speed-penalty-weight 3.0 \
  --conflict-overspeed-penalty-weight 10.0 \
  --avoidance-turn-reward-weight 9.0 \
  --desired-conflict-speed 0.10 \
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
  --freeze-observation-normalizer \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.035 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 0.95 \
  --angular-accel-limit 1.0 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.60 \
  --angular-authority-floor 0.45 \
  --min-forward-speed-floor 0.0 \
  --collision-distance 0.75 \
  --sim-tau-linear 0.6 \
  --sim-tau-angular 0.35 \
  --dr-tau-linear-low 0.4 \
  --dr-tau-linear-high 1.0 \
  --dr-tau-angular-low 0.2 \
  --dr-tau-angular-high 1.0 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 217 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} crossing branch repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"