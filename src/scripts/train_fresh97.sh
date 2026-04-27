#!/bin/bash
# fresh97: unfrozen ETA/priority temporal coordination repair.
#
# fresh96 added route timing features, but interim probes showed the crossing
# trunk still barely used the new ETA/priority columns. fresh97 keeps the same
# single MAPPO checkpoint architecture and pure-RL policy, but unfreezes the
# attention/base path and observation normalizer so the new timing features can
# reshape the policy. Reward weights are shifted toward temporal separation and
# yield/clear role differentiation for three_usv_crossing.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

DEFAULT_BASE="/mnt/data/checkpoints/usv_rl/fresh96_crossing_eta_priority_repair.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$DEFAULT_BASE" ]]; then
    BASE_INPUT="$DEFAULT_BASE"
  else
    BASE_INPUT="$(ls -1t /mnt/data/checkpoints/usv_rl/fresh96_checkpoints/fresh96_crossing_eta_priority_repair_step_*.pt 2>/dev/null | head -n 1 || true)"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh97_unfrozen_eta_temporal_repair.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh97_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh97_train.log}"
RUN_NAME="${RUN_NAME:-fresh97}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh96 first."
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
  echo "========== ${RUN_NAME} Unfrozen ETA/Temporal Crossing Repair (56K) =========="
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
  --total-timesteps 56000 \
  --clip-range 0.06 \
  --learning-rate 4.0e-5 \
  --learning-rate-end 3.0e-6 \
  --entropy-coef 0.0012 \
  --entropy-coef-end 0.00008 \
  --actor-log-std-init -0.45 \
  --force-actor-log-std -0.45 \
  --episode-timeout 98.0 \
  --no-progress-timeout 38.0 \
  --min-progress-delta 0.10 \
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
  --scenario-spawn-position-std 0.18 \
  --scenario-spawn-heading-std 0.08 \
  --scenario-goal-position-std 0.18 \
  --encounter-type-dropout 0.0 \
  --cte-clip-range 5.0 \
  --progress-weight 2.5 \
  --goal-bonus 42.0 \
  --time-penalty 0.05 \
  --stall-penalty -44.0 \
  --heading-error-weight 1.8 \
  --heading-relief-factor 0.98 \
  --heading-correction-reward-weight 0.42 \
  --pure-cruise-reward-weight 0.03 \
  --pure-idle-penalty-weight 0.04 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --conflict-progress-scale 2.20 \
  --action-smoothness-weight 4.0 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --forward-speed-change-penalty-weight 0.60 \
  --omega-flip-saturation-threshold 0.30 \
  --straight-line-omega-conflict-floor 0.12 \
  --team-reward-weight 0.80 \
  --team-progress-weight 0.70 \
  --team-goal-proximity-weight 0.08 \
  --team-regression-penalty-weight 0.80 \
  --separation-recovery-weight 10.0 \
  --team-dispersion-penalty-weight 0.18 \
  --team-dispersion-margin 0.95 \
  --entanglement-penalty-weight 5.5 \
  --entanglement-distance 5.6 \
  --entanglement-grace-steps 3 \
  --entanglement-low-speed-penalty-weight 0.6 \
  --collision-penalty -1900.0 \
  --near-miss-distance 7.2 \
  --near-miss-weight 60.0 \
  --near-miss-exponent 2.0 \
  --head-on-near-miss-distance 6.4 \
  --conflict-distance 14.5 \
  --anticipation-distance 15.0 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.75 \
  --head-on-phase-gate-strength 0.55 \
  --crossing-starboard-turn-reward-weight 12.0 \
  --crossing-forward-reward-weight 0.0 \
  --crossing-slowdown-reward-weight 20.0 \
  --crossing-overspeed-penalty-weight 42.0 \
  --crossing-close-forward-penalty-weight 42.0 \
  --crossing-yield-speed 0.03 \
  --crossing-time-separation-reward-weight 72.0 \
  --crossing-time-separation-penalty-weight 160.0 \
  --crossing-time-gap-target 8.5 \
  --colregs-port-turn-penalty-weight 18.0 \
  --proximity-gradient-penalty-weight 28.0 \
  --proximity-gradient-distance 9.5 \
  --speed-distance-coupling-penalty-weight 40.0 \
  --speed-distance-coupling-threshold 9.5 \
  --unsafe-close-speed-penalty-weight 12.0 \
  --conflict-overspeed-penalty-weight 48.0 \
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
  --base-ros-domain-id 225 \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} unfrozen ETA/temporal repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"