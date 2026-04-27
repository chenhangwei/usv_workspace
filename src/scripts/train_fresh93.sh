#!/bin/bash
# fresh93: crossing yield/slowdown branch repair after fresh92 failed to reduce crossing collisions.
#
# fresh92 diagnosis:
#   - branch routing, sampler/env parity, and frozen obs-normalizer all worked.
#   - three_usv_crossing still stayed at collision=1.0 in every inspected checkpoint.
#   - crossing metrics showed progress_efficiency≈1.0 and controller logs kept vx≈0.36,
#     meaning the crossing branch learned to keep charging through the intersection.
#   - fresh92 also used --crossing-forward-reward-weight 2.4, which rewarded forward
#     motion while turning and fought the desired slow/yield behavior.
#
# fresh93 change:
#   - uses the new trainer-side crossing slowdown/yield reward terms.
#   - sets crossing_forward_reward=0 so crossing no longer explicitly rewards speed.
#   - lowers conflict-speed incentives and strongly penalizes crossing overspeed/close forward motion.
#   - keeps single deployable MAPPO checkpoint with full 5 scenario trunks; only crossing rollout
#     curriculum is sampled and shared actor/obs-normalizer remain frozen.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh92_crossing_branch_repair.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh93_crossing_yield_branch_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh93_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh93_train.log}"
RUN_NAME="${RUN_NAME:-fresh93}"

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
  echo "========== ${RUN_NAME} Crossing Yield Branch Repair (16K from fresh92) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1600 \
  --num-agents 3 \
  --total-timesteps 16000 \
  --clip-range 0.05 \
  --learning-rate 7.0e-6 \
  --learning-rate-end 1.0e-6 \
  --entropy-coef 0.0004 \
  --entropy-coef-end 0.00003 \
  --actor-log-std-init -0.9 \
  --force-actor-log-std -0.9 \
  --episode-timeout 76.0 \
  --no-progress-timeout 26.0 \
  --min-progress-delta 0.18 \
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
  --progress-weight 4.2 \
  --goal-bonus 42.0 \
  --time-penalty 0.05 \
  --stall-penalty -42.0 \
  --heading-error-weight 2.8 \
  --heading-relief-factor 0.96 \
  --heading-correction-reward-weight 0.6 \
  --pure-cruise-reward-weight 0.25 \
  --pure-idle-penalty-weight 0.20 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 1.15 \
  --action-smoothness-weight 5.0 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --forward-speed-change-penalty-weight 0.8 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.16 \
  --team-reward-weight 0.60 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.20 \
  --team-regression-penalty-weight 0.70 \
  --separation-recovery-weight 4.0 \
  --team-dispersion-penalty-weight 0.45 \
  --team-dispersion-margin 0.90 \
  --entanglement-penalty-weight 3.4 \
  --entanglement-distance 5.0 \
  --entanglement-grace-steps 5 \
  --entanglement-low-speed-penalty-weight 2.4 \
  --collision-penalty -950.0 \
  --near-miss-distance 6.0 \
  --near-miss-weight 30.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 5.8 \
  --conflict-distance 12.5 \
  --anticipation-distance 12.5 \
  --head-on-guidance-distance 6.0 \
  --head-on-target-starboard-offset 0.60 \
  --head-on-phase-gate-strength 0.45 \
  --crossing-starboard-turn-reward-weight 6.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 9.0 \
  --crossing-overspeed-penalty-weight 14.0 \
  --crossing-close-forward-penalty-weight 10.0 \
  --crossing-yield-speed 0.08 \
  --colregs-port-turn-penalty-weight 6.0 \
  --proximity-gradient-penalty-weight 12.0 \
  --proximity-gradient-distance 7.5 \
  --speed-distance-coupling-penalty-weight 10.0 \
  --speed-distance-coupling-threshold 7.8 \
  --unsafe-close-speed-penalty-weight 3.5 \
  --conflict-overspeed-penalty-weight 12.0 \
  --avoidance-turn-reward-weight 8.0 \
  --desired-conflict-speed 0.08 \
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
  --near-goal-idle-penalty-weight 0.8 \
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
  --base-ros-domain-id 219 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} crossing yield branch repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
