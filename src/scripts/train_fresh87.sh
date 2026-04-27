#!/bin/bash
# fresh87: full-scenario scenario-specific actor trunks from fresh79_best
#
# Rationale:
#   - fresh80-82 evaluator routing/blending only moved along collision-vs-timeout frontiers.
#   - fresh83-86 proved branch-specific training is the right structural direction, but those
#     runs were single-scenario and fresh85/86 were affected by missing branch warm-start.
#   - train_mappo_policy.py now initializes missing scenario heads/trunks from the loaded
#     shared actor MLP, so each scenario trunk starts as the base policy instead of random.
#
# Goal:
#   Train one deployable pure-RL checkpoint with trainer-side scenario-conditioned trunks.
#   This is not evaluator-side router/blend; it remains a single MAPPO checkpoint.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh79_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh87_full_scenario_trunk.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh87_checkpoints"
LOG_FILE="/tmp/fresh87_train.log"

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

echo "========== fresh87 Full-Scenario Scenario Trunk (24K from fresh79_best) ==========" | tee "$LOG_FILE"
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1200 \
  --num-agents 3 \
  --total-timesteps 24000 \
  --clip-range 0.08 \
  --learning-rate 1.2e-5 \
  --learning-rate-end 3.0e-6 \
  --entropy-coef 0.0008 \
  --entropy-coef-end 0.0001 \
  --episode-timeout 70.0 \
  --no-progress-timeout 20.0 \
  --min-progress-delta 0.30 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --scenario-spawn-position-std 0.8 \
  --scenario-spawn-heading-std 0.32 \
  --scenario-goal-position-std 0.8 \
  --encounter-type-dropout 0.10 \
  --cte-clip-range 5.0 \
  --progress-weight 6.0 \
  --goal-bonus 40.0 \
  --time-penalty 0.07 \
  --stall-penalty -50.0 \
  --heading-error-weight 5.0 \
  --heading-relief-factor 0.85 \
  --heading-correction-reward-weight 1.5 \
  --action-smoothness-weight 6.0 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --omega-flip-saturation-threshold 0.40 \
  --separation-recovery-weight 1.0 \
  --entanglement-penalty-weight 1.5 \
  --entanglement-distance 4.0 \
  --entanglement-grace-steps 10 \
  --entanglement-low-speed-penalty-weight 2.5 \
  --collision-penalty -400.0 \
  --near-miss-distance 4.5 \
  --near-miss-weight 12.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 9.0 \
  --anticipation-distance 9.0 \
  --head-on-guidance-distance 5.0 \
  --head-on-target-starboard-offset 0.45 \
  --head-on-phase-gate-strength 0.35 \
  --colregs-port-turn-penalty-weight 1.5 \
  --proximity-gradient-penalty-weight 4.0 \
  --proximity-gradient-distance 5.5 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 5.5 \
  --conflict-overspeed-penalty-weight 4.0 \
  --avoidance-turn-reward-weight 4.0 \
  --desired-conflict-speed 0.18 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.2 \
  --overtaking-corridor-reward-weight 2.4 \
  --overtaking-centerline-penalty-weight 2.2 \
  --overtaking-close-penalty-weight 3.4 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.25 \
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
  --freeze-actor-base \
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 1.0 \
  --angular-accel-limit 1.2 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.50 \
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
  --base-ros-domain-id 209 \
  2>&1 | tee -a "$LOG_FILE"

echo "fresh87 full-scenario trunk complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
echo "Run: bash src/scripts/fresh87_quickeval.sh" | tee -a "$LOG_FILE"
