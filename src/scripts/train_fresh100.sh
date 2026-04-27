#!/bin/bash
# fresh100: stronger trainer-side crossing role imitation with actor-only
# pretrain epochs after each PPO rollout. The deployed artifact is still one
# pure MAPPO checkpoint: no evaluator router, checkpoint blend, ORCA/APF/MPC, or
# deployment action override is introduced.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh98_checkpoints/fresh98_pairwise_conflict_repair_step_0062784.pt"
FINAL_FRESH98="/mnt/data/checkpoints/usv_rl/fresh98_pairwise_conflict_repair.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  elif [[ -f "$FINAL_FRESH98" ]]; then
    BASE_INPUT="$FINAL_FRESH98"
  else
    BASE_INPUT="$(ls -1t /mnt/data/checkpoints/usv_rl/fresh98_checkpoints/fresh98_pairwise_conflict_repair_step_*.pt 2>/dev/null | head -n 1 || true)"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh100_crossing_pretrain_role.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh100_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh100_train.log}"
RUN_NAME="${RUN_NAME:-fresh100}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh98 first."
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
  echo "========== ${RUN_NAME} Crossing Pretrain Role Repair (43.2K) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1800 \
  --num-agents 3 \
  --total-timesteps 43200 \
  --clip-range 0.045 \
  --learning-rate 5.5e-5 \
  --learning-rate-end 1.0e-6 \
  --entropy-coef 0.0005 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.60 \
  --force-actor-log-std -1.60 \
  --episode-timeout 108.0 \
  --no-progress-timeout 46.0 \
  --min-progress-delta 0.06 \
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
  --scenario-spawn-position-std 0.08 \
  --scenario-spawn-heading-std 0.03 \
  --scenario-goal-position-std 0.08 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --crossing-imitation-weight 18.0 \
  --crossing-imitation-weight-end 8.0 \
  --crossing-imitation-pretrain-epochs 12 \
  --crossing-imitation-clear-speed 0.24 \
  --crossing-imitation-middle-speed 0.02 \
  --crossing-imitation-yield-speed 0.0 \
  --crossing-imitation-clear-omega -0.12 \
  --crossing-imitation-middle-omega -0.24 \
  --crossing-imitation-yield-omega -0.36 \
  --crossing-imitation-eta-gate 1.05 \
  --crossing-imitation-phase-min -1.05 \
  --crossing-imitation-phase-max 0.55 \
  --progress-weight 1.4 \
  --goal-bonus 48.0 \
  --time-penalty 0.055 \
  --stall-penalty -56.0 \
  --heading-error-weight 1.6 \
  --heading-relief-factor 0.99 \
  --heading-correction-reward-weight 0.38 \
  --pure-cruise-reward-weight 0.02 \
  --pure-idle-penalty-weight 0.03 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 2.2 \
  --action-smoothness-weight 4.8 \
  --saturated-omega-flip-penalty-weight 12.5 \
  --forward-speed-change-penalty-weight 0.75 \
  --omega-flip-saturation-threshold 0.28 \
  --straight-line-omega-conflict-floor 0.14 \
  --team-reward-weight 0.95 \
  --team-progress-weight 0.42 \
  --team-goal-proximity-weight 0.05 \
  --team-regression-penalty-weight 1.0 \
  --separation-recovery-weight 16.0 \
  --team-dispersion-penalty-weight 0.18 \
  --team-dispersion-margin 1.00 \
  --entanglement-penalty-weight 8.5 \
  --entanglement-distance 6.2 \
  --entanglement-grace-steps 2 \
  --entanglement-low-speed-penalty-weight 0.8 \
  --collision-penalty -2800.0 \
  --near-miss-distance 8.2 \
  --near-miss-weight 96.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.8 \
  --conflict-distance 17.0 \
  --anticipation-distance 17.0 \
  --head-on-guidance-distance 7.0 \
  --head-on-target-starboard-offset 0.80 \
  --head-on-phase-gate-strength 0.65 \
  --crossing-starboard-turn-reward-weight 10.5 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 38.0 \
  --crossing-overspeed-penalty-weight 92.0 \
  --crossing-close-forward-penalty-weight 92.0 \
  --crossing-yield-speed 0.0 \
  --crossing-time-separation-reward-weight 150.0 \
  --crossing-time-separation-penalty-weight 380.0 \
  --crossing-time-gap-target 14.0 \
  --colregs-port-turn-penalty-weight 28.0 \
  --proximity-gradient-penalty-weight 46.0 \
  --proximity-gradient-distance 10.5 \
  --speed-distance-coupling-penalty-weight 76.0 \
  --speed-distance-coupling-threshold 10.5 \
  --unsafe-close-speed-penalty-weight 24.0 \
  --conflict-overspeed-penalty-weight 90.0 \
  --avoidance-turn-reward-weight 1.8 \
  --desired-conflict-speed 0.02 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 1.0 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.65 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.20 \
  --goal-proximity-speed-relief 0.0 \
  --near-goal-idle-penalty-weight 0.8 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.75 \
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
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.020 \
  --dr-heading-noise-std 0.006 \
  --dr-velocity-noise-ratio 0.012 \
  --dr-current-speed-max 0.010 \
  --dr-velocity-exec-noise 0.012 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.50 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 0.95 \
  --angular-accel-limit 1.6 \
  --angular-decel-limit 3.0 \
  --conflict-turn-relief 0.78 \
  --angular-authority-floor 0.52 \
  --min-forward-speed-floor 0.0 \
  --collision-distance 0.75 \
  --sim-tau-linear 0.6 \
  --sim-tau-angular 0.35 \
  --dr-tau-linear-low 0.4 \
  --dr-tau-linear-high 0.9 \
  --dr-tau-angular-low 0.2 \
  --dr-tau-angular-high 0.9 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 225 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} crossing-pretrain role repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"