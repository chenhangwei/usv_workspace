#!/bin/bash
# fresh172: recover off-route random finish after fresh171 exposed route-progress saturation.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh171_inside_goal_hold_from_fresh168.pt}"
FALLBACK_INPUT="${FALLBACK_INPUT:-/mnt/data/checkpoints/usv_rl/fresh168_random_hold_lowprio_from_fresh163_step1152.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh172_offroute_recovery_from_fresh171.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh172_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh172_train.log}"
RUN_NAME="${RUN_NAME:-fresh172}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1260}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-214}"

mkdir -p "$CKPT_DIR"

if [[ ! -f "$BASE_INPUT" && -f "$FALLBACK_INPUT" ]]; then
  echo "WARN: fresh171 base missing, falling back to fresh168: $FALLBACK_INPUT"
  BASE_INPUT="$FALLBACK_INPUT"
fi
if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh171/fresh168 first."
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
  echo "========== ${RUN_NAME} Off-Route Finish Recovery (${TOTAL_TIMESTEPS}) =========="
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
  --clip-range 0.00005 \
  --learning-rate 2.2e-7 \
  --learning-rate-end 9.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -2.40 \
  --force-actor-log-std -2.40 \
  --ppo-policy-loss-scale 0.000 \
  --ppo-value-loss-scale 0.000 \
  --value-coef 0.00 \
  --rollout-steps 420 \
  --reset-sampler-each-rollout \
  --update-epochs 2 \
  --minibatch-size 420 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.60 \
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
  --random-deconflict-weight 4.95 \
  --random-deconflict-weight-end 4.65 \
  --random-deconflict-pretrain-epochs 0 \
  --random-deconflict-pretrain-learning-rate 0.0 \
  --random-deconflict-pretrain-max-grad-norm 0.0 \
  --random-deconflict-role-mode priority \
  --random-deconflict-priority-yield-threshold -0.10 \
  --random-deconflict-goal-tolerance 0.8 \
  --random-deconflict-max-distance 13.0 \
  --random-deconflict-lookahead-distance 12.00 \
  --random-deconflict-time-horizon 40.0 \
  --random-deconflict-dcpa-target 1.95 \
  --random-deconflict-closing-speed-min 0.001 \
  --random-deconflict-local-danger \
  --random-deconflict-safe-separation 1.38 \
  --random-deconflict-release-separation 2.60 \
  --random-deconflict-power 0.68 \
  --random-deconflict-standon-speed 0.33 \
  --random-deconflict-yield-speed 0.055 \
  --random-deconflict-yield-danger-scale 1.00 \
  --random-deconflict-standon-omega 0.04 \
  --random-deconflict-yield-omega 0.30 \
  --random-deconflict-omega-weight 0.28 \
  --random-deconflict-standon-weight 0.42 \
  --random-deconflict-yield-weight 3.55 \
  --random-safe-finish-weight 1.15 \
  --random-safe-finish-weight-end 1.40 \
  --random-safe-finish-goal-tolerance 0.8 \
  --random-safe-finish-max-distance 13.0 \
  --random-safe-finish-phase-min -1.0 \
  --random-safe-finish-min-team-separation 2.62 \
  --random-safe-finish-full-team-separation 3.20 \
  --random-safe-finish-min-neighbor-separation 2.80 \
  --random-safe-finish-max-cpa-score 0.0 \
  --random-safe-finish-max-local-score 0.0 \
  --random-safe-finish-target-speed 0.21 \
  --random-safe-finish-min-speed-scale 0.58 \
  --random-safe-finish-hold-distance 0.0 \
  --random-safe-finish-hold-weight 1.00 \
  --random-safe-finish-low-priority-threshold -0.50 \
  --random-safe-finish-low-priority-speed-multiplier 1.25 \
  --random-safe-finish-max-omega 0.060 \
  --random-safe-finish-omega-weight 0.06 \
  --random-goal-hold-weight 0.00 \
  --random-goal-hold-weight-end 0.00 \
  --random-goal-hold-distance 0.0 \
  --random-offroute-finish-weight 3.10 \
  --random-offroute-finish-weight-end 4.20 \
  --random-offroute-finish-goal-tolerance 0.8 \
  --random-offroute-finish-max-distance 13.0 \
  --random-offroute-finish-min-distance 1.25 \
  --random-offroute-finish-route-progress-min 0.82 \
  --random-offroute-finish-min-abs-cte 1.10 \
  --random-offroute-finish-full-abs-cte 3.00 \
  --random-offroute-finish-phase-min -1.0 \
  --random-offroute-finish-min-team-separation 2.70 \
  --random-offroute-finish-min-neighbor-separation 2.86 \
  --random-offroute-finish-max-cpa-score 0.0 \
  --random-offroute-finish-max-local-score 0.0 \
  --random-offroute-finish-target-speed 0.13 \
  --random-offroute-finish-min-speed 0.055 \
  --random-offroute-finish-max-omega 0.22 \
  --random-offroute-finish-omega-reference 0.55 \
  --random-offroute-finish-omega-weight 0.80 \
  --policy-anchor-weight 190.0 \
  --policy-anchor-weight-end 235.0 \
  --policy-anchor-exclude-team-safety-brake \
  --policy-anchor-exclude-random-deconflict \
  --policy-anchor-exclude-random-safe-finish \
  --policy-anchor-exclude-random-goal-hold \
  --policy-anchor-exclude-random-offroute-finish \
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
  ${EXTRA_TRAIN_ARGS:-} \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} off-route finish recovery complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"