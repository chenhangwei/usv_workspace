#!/bin/bash
# fresh98: structural pairwise-conflict observation repair.
#
# fresh97 still had three_usv_crossing collision_rate=1.0 for every candidate.
# fresh98 keeps a single pure MAPPO checkpoint and trainer-side scenario trunk,
# but expands each neighbor block with TCPA/DCPA plus pairwise route ETA/priority
# deltas so the attention module can reason about crossing order per neighbor.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_0033984.pt"
AGGREGATE_BASE="/mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_0040896.pt"
FINAL_BASE="/mnt/data/checkpoints/usv_rl/fresh97_unfrozen_eta_temporal_repair.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  elif [[ -f "$AGGREGATE_BASE" ]]; then
    BASE_INPUT="$AGGREGATE_BASE"
  elif [[ -f "$FINAL_BASE" ]]; then
    BASE_INPUT="$FINAL_BASE"
  else
    BASE_INPUT="$(ls -1t /mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_*.pt 2>/dev/null | head -n 1 || true)"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh98_pairwise_conflict_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh98_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh98_train.log}"
RUN_NAME="${RUN_NAME:-fresh98}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh97 first."
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
  echo "========== ${RUN_NAME} Pairwise Conflict Observation Repair (72K) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 2400 \
  --num-agents 3 \
  --total-timesteps 72000 \
  --clip-range 0.055 \
  --learning-rate 5.0e-5 \
  --learning-rate-end 2.0e-6 \
  --entropy-coef 0.0018 \
  --entropy-coef-end 0.00008 \
  --actor-log-std-init -0.38 \
  --force-actor-log-std -0.38 \
  --episode-timeout 104.0 \
  --no-progress-timeout 42.0 \
  --min-progress-delta 0.08 \
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
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --scenario-spawn-position-std 0.20 \
  --scenario-spawn-heading-std 0.08 \
  --scenario-goal-position-std 0.20 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 2.6 \
  --goal-bonus 44.0 \
  --time-penalty 0.05 \
  --stall-penalty -48.0 \
  --heading-error-weight 1.8 \
  --heading-relief-factor 0.98 \
  --heading-correction-reward-weight 0.44 \
  --pure-cruise-reward-weight 0.03 \
  --pure-idle-penalty-weight 0.04 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 2.35 \
  --action-smoothness-weight 4.2 \
  --saturated-omega-flip-penalty-weight 11.0 \
  --forward-speed-change-penalty-weight 0.65 \
  --omega-flip-saturation-threshold 0.30 \
  --straight-line-omega-conflict-floor 0.13 \
  --team-reward-weight 0.85 \
  --team-progress-weight 0.75 \
  --team-goal-proximity-weight 0.08 \
  --team-regression-penalty-weight 0.85 \
  --separation-recovery-weight 12.0 \
  --team-dispersion-penalty-weight 0.20 \
  --team-dispersion-margin 1.00 \
  --entanglement-penalty-weight 6.2 \
  --entanglement-distance 5.8 \
  --entanglement-grace-steps 3 \
  --entanglement-low-speed-penalty-weight 0.7 \
  --collision-penalty -2100.0 \
  --near-miss-distance 7.8 \
  --near-miss-weight 72.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.4 \
  --conflict-distance 15.8 \
  --anticipation-distance 16.2 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.75 \
  --head-on-phase-gate-strength 0.55 \
  --crossing-starboard-turn-reward-weight 9.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 24.0 \
  --crossing-overspeed-penalty-weight 56.0 \
  --crossing-close-forward-penalty-weight 50.0 \
  --crossing-yield-speed 0.03 \
  --crossing-time-separation-reward-weight 96.0 \
  --crossing-time-separation-penalty-weight 220.0 \
  --crossing-time-gap-target 10.0 \
  --colregs-port-turn-penalty-weight 20.0 \
  --proximity-gradient-penalty-weight 32.0 \
  --proximity-gradient-distance 10.0 \
  --speed-distance-coupling-penalty-weight 46.0 \
  --speed-distance-coupling-threshold 10.0 \
  --unsafe-close-speed-penalty-weight 14.0 \
  --conflict-overspeed-penalty-weight 54.0 \
  --avoidance-turn-reward-weight 1.8 \
  --desired-conflict-speed 0.04 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 1.0 \
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
  --max-grad-norm 0.45 \
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
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.035 \
  --dr-heading-noise-std 0.010 \
  --dr-velocity-noise-ratio 0.02 \
  --dr-current-speed-max 0.020 \
  --dr-velocity-exec-noise 0.025 \
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
  --base-ros-domain-id 223 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} pairwise-conflict repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
