#!/bin/bash
# fresh95: crossing multi-way starboard/right-turn repair.
#
# fresh94 aggregate improved, but every selected checkpoint still had
# three_usv_crossing collision_rate=1.0. A single-episode diagnostic showed all
# three vessels charging into the intersection at vx≈0.36 and colliding around
# step 56. The code change for fresh95 broadens crossing guidance inside the
# trainer/env reward: in the annotated three_usv_crossing scenario, every
# forward/near closing conflict receives yield + starboard-turn shaping, not
# only classical starboard-side crossing pairs. This stays pure RL: one full-5
# scenario-trunk MAPPO checkpoint, no evaluator router/blend, no ORCA/APF/MPC.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh94_checkpoints/fresh94_crossing_yield_decouple_repair_step_0017280.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh95_crossing_multiway_starboard_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh95_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh95_train.log}"
RUN_NAME="${RUN_NAME:-fresh95}"

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
  echo "========== ${RUN_NAME} Crossing Multi-way Starboard Repair (30K from fresh94 selected) =========="
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
  --total-timesteps 30000 \
  --clip-range 0.08 \
  --learning-rate 1.6e-5 \
  --learning-rate-end 1.5e-6 \
  --entropy-coef 0.0008 \
  --entropy-coef-end 0.00004 \
  --actor-log-std-init -0.75 \
  --force-actor-log-std -0.75 \
  --episode-timeout 88.0 \
  --no-progress-timeout 30.0 \
  --min-progress-delta 0.14 \
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
  --scenario-spawn-position-std 0.70 \
  --scenario-spawn-heading-std 0.28 \
  --scenario-goal-position-std 0.70 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 3.6 \
  --goal-bonus 42.0 \
  --time-penalty 0.04 \
  --stall-penalty -36.0 \
  --heading-error-weight 2.4 \
  --heading-relief-factor 0.98 \
  --heading-correction-reward-weight 0.55 \
  --pure-cruise-reward-weight 0.06 \
  --pure-idle-penalty-weight 0.035 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 1.55 \
  --action-smoothness-weight 3.2 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 0.45 \
  --omega-flip-saturation-threshold 0.32 \
  --straight-line-omega-conflict-floor 0.10 \
  --team-reward-weight 0.80 \
  --team-progress-weight 1.05 \
  --team-goal-proximity-weight 0.15 \
  --team-regression-penalty-weight 0.75 \
  --separation-recovery-weight 7.5 \
  --team-dispersion-penalty-weight 0.32 \
  --team-dispersion-margin 0.95 \
  --entanglement-penalty-weight 4.5 \
  --entanglement-distance 5.4 \
  --entanglement-grace-steps 4 \
  --entanglement-low-speed-penalty-weight 1.0 \
  --collision-penalty -1500.0 \
  --near-miss-distance 6.8 \
  --near-miss-weight 45.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.2 \
  --conflict-distance 13.5 \
  --anticipation-distance 13.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.75 \
  --head-on-phase-gate-strength 0.55 \
  --crossing-starboard-turn-reward-weight 18.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 18.0 \
  --crossing-overspeed-penalty-weight 34.0 \
  --crossing-close-forward-penalty-weight 30.0 \
  --crossing-yield-speed 0.04 \
  --colregs-port-turn-penalty-weight 18.0 \
  --proximity-gradient-penalty-weight 20.0 \
  --proximity-gradient-distance 9.0 \
  --speed-distance-coupling-penalty-weight 22.0 \
  --speed-distance-coupling-threshold 9.0 \
  --unsafe-close-speed-penalty-weight 7.0 \
  --conflict-overspeed-penalty-weight 24.0 \
  --avoidance-turn-reward-weight 1.5 \
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
  --dr-position-noise-std 0.08 \
  --dr-heading-noise-std 0.018 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.035 \
  --dr-velocity-exec-noise 0.045 \
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
  --base-ros-domain-id 231 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} crossing multi-way starboard repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
