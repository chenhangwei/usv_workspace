#!/bin/bash
# fresh152: true random-first local safety brake repair.
#
# fresh151's brake loss never activated because the short run stayed in the
# first curriculum scenario.  This run samples random encounters first and uses
# an auxiliary-only repair so the random scenario trunk learns the local brake
# and away-turn response without touching the shared actor base.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh151_random_safety_gate_from_fresh150.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh150_cpa_margin_soft_clear_from_fresh142.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh152_random_first_safety_brake_from_fresh151.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh152_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh152_train.log}"
RUN_NAME="${RUN_NAME:-fresh152}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-720}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-218}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh151/fresh150 first."
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
  echo "========== ${RUN_NAME} Random-First Safety Brake (${TOTAL_TIMESTEPS}) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 240 \
  --num-agents 3 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --clip-range 0.00020 \
  --learning-rate 1.8e-7 \
  --learning-rate-end 6.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.90 \
  --force-actor-log-std -1.90 \
  --ppo-policy-loss-scale 0.000 \
  --ppo-value-loss-scale 0.000 \
  --value-coef 0.00 \
  --rollout-steps 96 \
  --update-epochs 2 \
  --minibatch-size 192 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.45 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 --hidden-size 256 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario two_usv_random_encounter \
  --scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario two_usv_random_encounter \
  --max-agents 5 \
  --max-neighbors 4 \
  --squash-actions \
  --min-forward-speed 0.0 \
  --linear-delta-limit 0.30 \
  --angular-delta-limit 0.50 \
  --cruise-speed 0.34 \
  --max-angular-velocity 0.50 \
  --min-forward-speed-floor 0.0 \
  --heading-omega-deadband 0.09 \
  --heading-omega-reference 0.95 \
  --angular-authority-power 1.30 \
  --angular-accel-limit 1.05 \
  --angular-decel-limit 3.00 \
  --conflict-turn-relief 0.74 \
  --angular-authority-floor 0.38 \
  --episode-timeout 145.0 \
  --no-progress-timeout 108.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  --team-safety-brake-weight 0.72 \
  --team-safety-brake-weight-end 0.52 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 13.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.05 \
  --team-safety-brake-release-separation 3.00 \
  --team-safety-brake-target-speed 0.035 \
  --team-safety-brake-omega-weight 0.85 \
  --team-safety-brake-target-omega 0.30 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 0.90 \
  --team-safety-brake-random-only \
  --policy-anchor-weight 420.0 \
  --policy-anchor-weight-end 520.0 \
  --policy-anchor-exclude-team-safety-brake \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --freeze-observation-normalizer \
  --scenario-balanced-loss \
  --domain-randomization \
  --scenario-spawn-position-std 0.035 \
  --scenario-spawn-heading-std 0.015 \
  --scenario-goal-position-std 0.030 \
  --dr-position-noise-std 0.010 \
  --dr-heading-noise-std 0.003 \
  --dr-velocity-noise-ratio 0.006 \
  --dr-current-speed-max 0.004 \
  --dr-velocity-exec-noise 0.006 \
  --dr-tau-linear-low 0.45 \
  --dr-tau-linear-high 0.80 \
  --dr-tau-angular-low 0.24 \
  --dr-tau-angular-high 0.70 \
  --encounter-type-dropout 0.02 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  --num-sampler-workers 2 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --separate-actor-critic-grad-clip \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} random-first safety brake complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"