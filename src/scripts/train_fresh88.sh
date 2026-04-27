#!/bin/bash
# fresh88: full-scenario scenario trunks with shared attention unfrozen.
#
# fresh87 proved that trainer-side scenario trunks route correctly, but the frozen shared
# attention encoder kept crossing/random collision high.  This continuation keeps the
# single-checkpoint pure-RL scenario-trunk architecture and unfreezes the full actor so the
# shared attention encoder can adapt while each scenario trunk remains branch-specific.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

RUN_NAME="${RUN_NAME:-fresh88}"
BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh87_full_scenario_trunk.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh88_full_scenario_trunk_unfrozen.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh88_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh88_train.log}"

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

echo "========== ${RUN_NAME} Full-Scenario Scenario Trunk Unfrozen Attention (16K continuation) ==========" | tee "$LOG_FILE"
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1000 \
  --num-agents 3 \
  --total-timesteps 16000 \
  --clip-range 0.06 \
  --learning-rate 6.0e-6 \
  --learning-rate-end 1.5e-6 \
  --entropy-coef 0.0005 \
  --entropy-coef-end 0.00005 \
  --episode-timeout 70.0 \
  --no-progress-timeout 22.0 \
  --min-progress-delta 0.26 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --scenario-spawn-position-std 0.9 \
  --scenario-spawn-heading-std 0.36 \
  --scenario-goal-position-std 0.9 \
  --encounter-type-dropout 0.08 \
  --cte-clip-range 5.0 \
  --progress-weight 5.2 \
  --goal-bonus 40.0 \
  --time-penalty 0.06 \
  --stall-penalty -45.0 \
  --heading-error-weight 4.2 \
  --heading-relief-factor 0.90 \
  --heading-correction-reward-weight 1.2 \
  --action-smoothness-weight 7.0 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --omega-flip-saturation-threshold 0.38 \
  --team-reward-weight 0.50 \
  --team-progress-weight 1.60 \
  --team-goal-proximity-weight 0.30 \
  --team-regression-penalty-weight 0.80 \
  --separation-recovery-weight 2.0 \
  --entanglement-penalty-weight 2.4 \
  --entanglement-distance 4.3 \
  --entanglement-grace-steps 8 \
  --entanglement-low-speed-penalty-weight 3.0 \
  --collision-penalty -650.0 \
  --near-miss-distance 5.2 \
  --near-miss-weight 18.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 9.5 \
  --anticipation-distance 10.0 \
  --head-on-guidance-distance 5.5 \
  --head-on-target-starboard-offset 0.55 \
  --head-on-phase-gate-strength 0.45 \
  --colregs-port-turn-penalty-weight 2.4 \
  --proximity-gradient-penalty-weight 7.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 5.0 \
  --speed-distance-coupling-threshold 6.0 \
  --conflict-overspeed-penalty-weight 6.0 \
  --avoidance-turn-reward-weight 6.0 \
  --desired-conflict-speed 0.14 \
  --overtaking-starboard-turn-reward-weight 2.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.6 \
  --overtaking-centerline-penalty-weight 2.5 \
  --overtaking-close-penalty-weight 4.0 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.65 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.30 \
  --goal-proximity-speed-relief 0.0 \
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
  --base-ros-domain-id 211 \
  ${EXTRA_TRAIN_ARGS:-} \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} full-scenario trunk unfrozen complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"