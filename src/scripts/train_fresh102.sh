#!/bin/bash
# fresh102: post-conflict completion recovery from the best fresh101 crossing candidate.
# Keeps the final artifact as one pure MAPPO checkpoint. All role guidance here is
# trainer-side shaping/auxiliary loss only; there is no evaluator/deployment router.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh101_checkpoints/fresh101_crossing_progress_recover_step_0008640.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh100_checkpoints/fresh100_crossing_pretrain_role_step_0002304.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh102_post_conflict_finish.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh102_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh102_train.log}"
RUN_NAME="${RUN_NAME:-fresh102}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-24000}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh101 first."
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
  echo "========== ${RUN_NAME} Post-Conflict Finish Recovery (${TOTAL_TIMESTEPS}) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1200 \
  --num-agents 3 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --clip-range 0.050 \
  --learning-rate 2.4e-5 \
  --learning-rate-end 8.0e-7 \
  --entropy-coef 0.00025 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.55 \
  --force-actor-log-std -1.55 \
  --episode-timeout 120.0 \
  --no-progress-timeout 58.0 \
  --min-progress-delta 0.025 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.08 \
  --scenario-spawn-heading-std 0.03 \
  --scenario-goal-position-std 0.08 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --crossing-imitation-weight 0.70 \
  --crossing-imitation-weight-end 0.00 \
  --crossing-imitation-pretrain-epochs 0 \
  --crossing-imitation-clear-speed 0.34 \
  --crossing-imitation-middle-speed 0.20 \
  --crossing-imitation-yield-speed 0.06 \
  --crossing-imitation-clear-omega -0.04 \
  --crossing-imitation-middle-omega -0.10 \
  --crossing-imitation-yield-omega -0.16 \
  --crossing-imitation-eta-gate 0.68 \
  --crossing-imitation-phase-min -0.95 \
  --crossing-imitation-phase-max 0.02 \
  --progress-weight 3.8 \
  --goal-bonus 78.0 \
  --team-completion-bonus 42.0 \
  --coordination-reward-weight 0.36 \
  --time-penalty 0.038 \
  --stall-penalty -92.0 \
  --heading-error-weight 1.25 \
  --heading-relief-factor 0.99 \
  --heading-correction-reward-weight 0.52 \
  --pure-cruise-reward-weight 0.04 \
  --pure-idle-penalty-weight 0.05 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 2.2 \
  --action-smoothness-weight 3.8 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --forward-speed-change-penalty-weight 0.48 \
  --omega-flip-saturation-threshold 0.28 \
  --straight-line-omega-conflict-floor 0.12 \
  --team-reward-weight 1.05 \
  --team-progress-weight 1.15 \
  --team-goal-proximity-weight 0.22 \
  --team-regression-penalty-weight 1.4 \
  --deadlock-penalty-weight 6.5 \
  --separation-recovery-weight 10.0 \
  --team-dispersion-penalty-weight 0.10 \
  --team-dispersion-margin 1.10 \
  --entanglement-penalty-weight 5.0 \
  --entanglement-distance 6.0 \
  --entanglement-grace-steps 3 \
  --entanglement-low-speed-penalty-weight 0.72 \
  --collision-penalty -3200.0 \
  --near-miss-distance 8.2 \
  --near-miss-weight 84.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.8 \
  --conflict-distance 17.0 \
  --anticipation-distance 17.0 \
  --head-on-guidance-distance 7.0 \
  --head-on-target-starboard-offset 0.80 \
  --head-on-phase-gate-strength 0.65 \
  --crossing-starboard-turn-reward-weight 7.2 \
  --crossing-forward-reward-weight 2.0 \
  --crossing-slowdown-reward-weight 14.0 \
  --crossing-overspeed-penalty-weight 38.0 \
  --crossing-close-forward-penalty-weight 42.0 \
  --crossing-yield-speed 0.06 \
  --crossing-time-separation-reward-weight 95.0 \
  --crossing-time-separation-penalty-weight 230.0 \
  --crossing-time-gap-target 10.0 \
  --colregs-port-turn-penalty-weight 16.0 \
  --proximity-gradient-penalty-weight 34.0 \
  --proximity-gradient-distance 10.5 \
  --speed-distance-coupling-penalty-weight 40.0 \
  --speed-distance-coupling-threshold 10.5 \
  --unsafe-close-speed-penalty-weight 18.0 \
  --conflict-overspeed-penalty-weight 40.0 \
  --avoidance-turn-reward-weight 1.8 \
  --desired-conflict-speed 0.06 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.8 \
  --overtaking-close-penalty-weight 4.5 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 3.4 \
  --goal-proximity-heading-relief 0.82 \
  --goal-proximity-smoothness-relief 0.82 \
  --goal-proximity-conflict-relief 0.55 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 1.6 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.60 \
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
  --dr-position-noise-std 0.018 \
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
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} post-conflict finish recovery complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
