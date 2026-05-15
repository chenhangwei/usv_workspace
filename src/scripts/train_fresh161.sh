#!/bin/bash
# fresh161: direct actor-only random deconflict fitting.
#
# fresh160 proved that local danger gating activates the right samples, but the
# pretrain path reused a tiny PPO learning rate. This continuation goes back to
# the fresh159 best checkpoint and uses a separate actor-only optimizer for the
# random-deconflict target.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh159_checkpoints/fresh159_random_speed_split_from_fresh158_step2304_step_0001728.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh161_direct_random_fit_from_fresh159_step1728.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh161_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh161_train.log}"
RUN_NAME="${RUN_NAME:-fresh161}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1152}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-227}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh159 first."
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
  echo "========== ${RUN_NAME} Direct Random Fit (${TOTAL_TIMESTEPS}) =========="
  echo "Base  : $BASE_INPUT"
  echo "Output: $OUTPUT"
  echo "Ckpts : $CKPT_DIR"
} | tee "$LOG_FILE"

/bin/python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-interval 576 \
  --num-agents 3 \
  --total-timesteps "$TOTAL_TIMESTEPS" \
  --clip-range 0.00010 \
  --learning-rate 7.5e-7 \
  --learning-rate-end 3.0e-7 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -2.35 \
  --force-actor-log-std -2.35 \
  --ppo-policy-loss-scale 0.000 \
  --ppo-value-loss-scale 0.000 \
  --value-coef 0.00 \
  --rollout-steps 192 \
  --reset-sampler-each-rollout \
  --update-epochs 2 \
  --minibatch-size 192 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.75 \
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
  --team-safety-brake-weight 1.10 \
  --team-safety-brake-weight-end 0.85 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 13.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.45 \
  --team-safety-brake-release-separation 3.10 \
  --team-safety-brake-target-speed 0.00 \
  --team-safety-brake-omega-weight 0.35 \
  --team-safety-brake-target-omega 0.30 \
  --team-safety-brake-turn-mode starboard \
  --team-safety-brake-head-on-starboard-threshold 0.18 \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 0.70 \
  --team-safety-brake-cpa-danger \
  --team-safety-brake-cpa-lookahead-distance 12.00 \
  --team-safety-brake-cpa-time-horizon 40.0 \
  --team-safety-brake-cpa-dcpa-target 1.90 \
  --team-safety-brake-cpa-closing-speed-min 0.002 \
  --team-safety-brake-random-only \
  --team-safety-brake-random-yield-agents-only \
  --team-safety-brake-random-yield-agent-weight 1.65 \
  --team-safety-brake-random-standon-agent-weight 0.00 \
  --random-deconflict-weight 5.50 \
  --random-deconflict-weight-end 4.50 \
  --random-deconflict-pretrain-epochs 140 \
  --random-deconflict-pretrain-learning-rate 1.0e-4 \
  --random-deconflict-pretrain-max-grad-norm 2.0 \
  --random-deconflict-goal-tolerance 0.8 \
  --random-deconflict-max-distance 13.0 \
  --random-deconflict-lookahead-distance 12.40 \
  --random-deconflict-time-horizon 42.0 \
  --random-deconflict-dcpa-target 2.10 \
  --random-deconflict-closing-speed-min 0.001 \
  --random-deconflict-local-danger \
  --random-deconflict-safe-separation 1.45 \
  --random-deconflict-release-separation 2.80 \
  --random-deconflict-power 0.50 \
  --random-deconflict-standon-speed 0.32 \
  --random-deconflict-yield-speed 0.00 \
  --random-deconflict-standon-omega 0.06 \
  --random-deconflict-yield-omega 0.34 \
  --random-deconflict-omega-weight 0.35 \
  --random-deconflict-standon-weight 0.04 \
  --random-deconflict-yield-weight 4.80 \
  --policy-anchor-weight 90.0 \
  --policy-anchor-weight-end 130.0 \
  --policy-anchor-exclude-team-safety-brake \
  --policy-anchor-exclude-random-deconflict \
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

echo "${RUN_NAME} direct random fit complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"
