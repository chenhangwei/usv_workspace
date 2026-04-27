#!/bin/bash
# fresh96: crossing ETA/priority structural repair.
#
# fresh95 still had three_usv_crossing collision_rate=1.0 for every selected
# checkpoint. fresh96 changes the observation and reward structure instead of
# more global speed shaping: each agent observes route progress, signed conflict
# phase, ETA-to-conflict, dynamic priority, and ETA gap. The reward then gives
# asymmetric pure-RL signals for "clear first" vs "yield/hold gap" inside the
# single scenario-trunk MAPPO checkpoint.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh95_checkpoints/fresh95_crossing_multiway_starboard_repair_step_0028800.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh96_crossing_eta_priority_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh96_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh96_train.log}"
RUN_NAME="${RUN_NAME:-fresh96}"

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
  echo "========== ${RUN_NAME} Crossing ETA/Priority Structural Repair (42K from fresh95 selected) =========="
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
  --total-timesteps 42000 \
  --clip-range 0.08 \
  --learning-rate 2.2e-5 \
  --learning-rate-end 1.8e-6 \
  --entropy-coef 0.0010 \
  --entropy-coef-end 0.00005 \
  --actor-log-std-init -0.65 \
  --force-actor-log-std -0.65 \
  --episode-timeout 92.0 \
  --no-progress-timeout 32.0 \
  --min-progress-delta 0.12 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --scenario-spawn-position-std 0.25 \
  --scenario-spawn-heading-std 0.10 \
  --scenario-goal-position-std 0.25 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 3.4 \
  --goal-bonus 44.0 \
  --time-penalty 0.04 \
  --stall-penalty -38.0 \
  --heading-error-weight 2.2 \
  --heading-relief-factor 0.98 \
  --heading-correction-reward-weight 0.50 \
  --pure-cruise-reward-weight 0.05 \
  --pure-idle-penalty-weight 0.03 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 1.60 \
  --action-smoothness-weight 3.0 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --forward-speed-change-penalty-weight 0.40 \
  --omega-flip-saturation-threshold 0.32 \
  --straight-line-omega-conflict-floor 0.10 \
  --team-reward-weight 0.85 \
  --team-progress-weight 1.00 \
  --team-goal-proximity-weight 0.10 \
  --team-regression-penalty-weight 0.70 \
  --separation-recovery-weight 7.5 \
  --team-dispersion-penalty-weight 0.25 \
  --team-dispersion-margin 0.95 \
  --entanglement-penalty-weight 4.0 \
  --entanglement-distance 5.2 \
  --entanglement-grace-steps 4 \
  --entanglement-low-speed-penalty-weight 0.8 \
  --collision-penalty -1600.0 \
  --near-miss-distance 6.8 \
  --near-miss-weight 48.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.2 \
  --conflict-distance 13.5 \
  --anticipation-distance 13.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.75 \
  --head-on-phase-gate-strength 0.55 \
  --crossing-starboard-turn-reward-weight 14.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 12.0 \
  --crossing-overspeed-penalty-weight 28.0 \
  --crossing-close-forward-penalty-weight 26.0 \
  --crossing-yield-speed 0.04 \
  --crossing-time-separation-reward-weight 36.0 \
  --crossing-time-separation-penalty-weight 78.0 \
  --crossing-time-gap-target 7.0 \
  --colregs-port-turn-penalty-weight 16.0 \
  --proximity-gradient-penalty-weight 20.0 \
  --proximity-gradient-distance 9.0 \
  --speed-distance-coupling-penalty-weight 22.0 \
  --speed-distance-coupling-threshold 9.0 \
  --unsafe-close-speed-penalty-weight 7.5 \
  --conflict-overspeed-penalty-weight 24.0 \
  --avoidance-turn-reward-weight 1.5 \
  --desired-conflict-speed 0.04 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 1.2 \
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
  --angular-delta-limit 0.50 \
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
  --dr-position-noise-std 0.04 \
  --dr-heading-noise-std 0.012 \
  --dr-velocity-noise-ratio 0.02 \
  --dr-current-speed-max 0.025 \
  --dr-velocity-exec-noise 0.030 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.50 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 0.95 \
  --angular-accel-limit 1.6 \
  --angular-decel-limit 3.0 \
  --conflict-turn-relief 0.75 \
  --angular-authority-floor 0.50 \
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

echo "${RUN_NAME} crossing ETA/priority repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
