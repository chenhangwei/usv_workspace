#!/bin/bash
# fresh169: weak-CPA safe finish and stronger near-goal hold from fresh168 final.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh168_random_hold_lowprio_from_fresh163_step1152.pt}"
FALLBACK_INPUT="${FALLBACK_INPUT:-/mnt/data/checkpoints/usv_rl/fresh163_checkpoints/fresh163_priority_obs_random_fit_from_fresh159_step1728_step_0001152.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh169_weak_cpa_hold_from_fresh168.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh169_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh169_train.log}"
RUN_NAME="${RUN_NAME:-fresh169}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}"

mkdir -p "$CKPT_DIR"

if [[ ! -f "$BASE_INPUT" && -f "$FALLBACK_INPUT" ]]; then
  echo "WARN: fresh168 base missing, falling back to fresh163: $FALLBACK_INPUT"
  BASE_INPUT="$FALLBACK_INPUT"
fi
if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh168/fresh163 first."
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
  echo "========== ${RUN_NAME} Weak CPA Hold (${TOTAL_TIMESTEPS}) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 1260 \
  --num-agents 3 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --clip-range 0.00006 \
  --learning-rate 2.8e-7 \
  --learning-rate-end 1.2e-7 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -2.35 \
  --force-actor-log-std -2.35 \
  --ppo-policy-loss-scale 0.000 \
  --ppo-value-loss-scale 0.000 \
  --value-coef 0.00 \
  --rollout-steps 420 \
  --reset-sampler-each-rollout \
  --update-epochs 2 \
  --minibatch-size 420 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.65 \
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
  --conflict-turn-relief 0.80 \
  --angular-authority-floor 0.45 \
  --episode-timeout 145.0 \
  --no-progress-timeout 108.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  --lagging-finish-weight 0.00 \
  --lagging-finish-weight-end 0.00 \
  --team-safety-brake-weight 0.00 \
  --team-safety-brake-weight-end 0.00 \
  --random-deconflict-weight 5.40 \
  --random-deconflict-weight-end 5.10 \
  --random-deconflict-pretrain-epochs 0 \
  --random-deconflict-pretrain-learning-rate 0.0 \
  --random-deconflict-pretrain-max-grad-norm 0.0 \
  --random-deconflict-role-mode priority \
  --random-deconflict-priority-yield-threshold -0.10 \
  --random-deconflict-goal-tolerance 0.8 \
  --random-deconflict-max-distance 13.0 \
  --random-deconflict-lookahead-distance 12.40 \
  --random-deconflict-time-horizon 42.0 \
  --random-deconflict-dcpa-target 2.08 \
  --random-deconflict-closing-speed-min 0.001 \
  --random-deconflict-local-danger \
  --random-deconflict-safe-separation 1.42 \
  --random-deconflict-release-separation 2.82 \
  --random-deconflict-power 0.70 \
  --random-deconflict-standon-speed 0.33 \
  --random-deconflict-yield-speed 0.072 \
  --random-deconflict-yield-danger-scale 0.82 \
  --random-deconflict-standon-omega 0.04 \
  --random-deconflict-yield-omega 0.32 \
  --random-deconflict-omega-weight 0.31 \
  --random-deconflict-standon-weight 0.45 \
  --random-deconflict-yield-weight 3.95 \
  --random-safe-finish-weight 1.70 \
  --random-safe-finish-weight-end 2.10 \
  --random-safe-finish-goal-tolerance 0.8 \
  --random-safe-finish-max-distance 13.0 \
  --random-safe-finish-phase-min -1.0 \
  --random-safe-finish-min-team-separation 2.70 \
  --random-safe-finish-full-team-separation 3.25 \
  --random-safe-finish-min-neighbor-separation 2.95 \
  --random-safe-finish-max-cpa-score 0.12 \
  --random-safe-finish-max-local-score 0.02 \
  --random-safe-finish-target-speed 0.24 \
  --random-safe-finish-min-speed-scale 0.62 \
  --random-safe-finish-hold-distance 1.10 \
  --random-safe-finish-hold-weight 4.00 \
  --random-safe-finish-low-priority-threshold -0.50 \
  --random-safe-finish-low-priority-speed-multiplier 1.55 \
  --random-safe-finish-max-omega 0.055 \
  --random-safe-finish-omega-weight 0.12 \
  --policy-anchor-weight 230.0 \
  --policy-anchor-weight-end 260.0 \
  --policy-anchor-exclude-team-safety-brake \
  --policy-anchor-exclude-random-deconflict \
  --policy-anchor-exclude-random-safe-finish \
  --policy-anchor-exclude-lagging-finish \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --freeze-observation-normalizer \
  --scenario-balanced-loss \
  --encounter-type-dropout 0.00 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  --num-sampler-workers 2 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --separate-actor-critic-grad-clip \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} weak-cpa hold fit complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"