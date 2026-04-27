#!/bin/bash
# fresh94: crossing yield reward decoupled from turn progress + stronger anti-charge training.
#
# fresh93 selected eval showed every inspected checkpoint still had
# three_usv_crossing collision_rate=1.0. The most likely reward conflict was:
#   - slowdown reward only paid after a starboard turn was already present;
#   - close-forward penalty mostly disappeared once turning began;
#   - default stop-go penalty still punished the braking needed for yielding.
# fresh94 keeps a single deployable full-5 scenario-trunk MAPPO checkpoint and
# trains only the crossing trunk with frozen shared actor base / frozen obs normalizer.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh93_checkpoints/fresh93_crossing_yield_branch_repair_step_0009792.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh94_crossing_yield_decouple_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh94_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh94_train.log}"
RUN_NAME="${RUN_NAME:-fresh94}"

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
  echo "========== ${RUN_NAME} Crossing Decoupled Yield Repair (24K from fresh93 selected) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 2400 \
  --num-agents 3 \
  --total-timesteps 24000 \
  --clip-range 0.08 \
  --learning-rate 1.2e-5 \
  --learning-rate-end 1.5e-6 \
  --entropy-coef 0.0006 \
  --entropy-coef-end 0.00004 \
  --actor-log-std-init -0.85 \
  --force-actor-log-std -0.85 \
  --episode-timeout 82.0 \
  --no-progress-timeout 28.0 \
  --min-progress-delta 0.16 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --scenario-spawn-position-std 0.65 \
  --scenario-spawn-heading-std 0.26 \
  --scenario-goal-position-std 0.65 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 3.8 \
  --goal-bonus 42.0 \
  --time-penalty 0.04 \
  --stall-penalty -36.0 \
  --heading-error-weight 2.6 \
  --heading-relief-factor 0.98 \
  --heading-correction-reward-weight 0.55 \
  --pure-cruise-reward-weight 0.10 \
  --pure-idle-penalty-weight 0.04 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 1.75 \
  --action-smoothness-weight 4.2 \
  --saturated-omega-flip-penalty-weight 7.0 \
  --forward-speed-change-penalty-weight 0.55 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.12 \
  --team-reward-weight 0.80 \
  --team-progress-weight 1.10 \
  --team-goal-proximity-weight 0.15 \
  --team-regression-penalty-weight 0.75 \
  --separation-recovery-weight 7.0 \
  --team-dispersion-penalty-weight 0.35 \
  --team-dispersion-margin 0.95 \
  --entanglement-penalty-weight 5.0 \
  --entanglement-distance 5.4 \
  --entanglement-grace-steps 4 \
  --entanglement-low-speed-penalty-weight 1.2 \
  --collision-penalty -1400.0 \
  --near-miss-distance 6.8 \
  --near-miss-weight 45.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.2 \
  --conflict-distance 13.5 \
  --anticipation-distance 13.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.70 \
  --head-on-phase-gate-strength 0.55 \
  --crossing-starboard-turn-reward-weight 7.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 16.0 \
  --crossing-overspeed-penalty-weight 32.0 \
  --crossing-close-forward-penalty-weight 26.0 \
  --crossing-yield-speed 0.04 \
  --colregs-port-turn-penalty-weight 7.0 \
  --proximity-gradient-penalty-weight 18.0 \
  --proximity-gradient-distance 9.0 \
  --speed-distance-coupling-penalty-weight 20.0 \
  --speed-distance-coupling-threshold 9.0 \
  --unsafe-close-speed-penalty-weight 6.0 \
  --conflict-overspeed-penalty-weight 22.0 \
  --avoidance-turn-reward-weight 10.0 \
  --desired-conflict-speed 0.04 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 1.3 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.65 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.25 \
  --goal-proximity-speed-relief 0.0 \
  --near-goal-idle-penalty-weight 0.8 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.5 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --squash-actions \
  --min-forward-speed 0.0 \
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
  --dr-position-noise-std 0.08 \
  --dr-heading-noise-std 0.018 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.035 \
  --dr-velocity-exec-noise 0.045 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 0.95 \
  --angular-accel-limit 1.0 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.65 \
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
  --base-ros-domain-id 221 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} crossing decoupled yield repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
