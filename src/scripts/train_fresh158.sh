#!/bin/bash
# fresh158: random encounter role-staggered deconfliction.
#
# fresh157's best fixed-seed random checkpoint still collided at 0.746m.
# This run starts from that closest checkpoint and adds a trainer-side
# random-deconflict auxiliary: lower-index stand-on keeps moving, while
# higher-index give-way agents slow and commit starboard under CPA threat.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh157_checkpoints/fresh157_role_weighted_guard_from_fresh153_step_0001728.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh158_random_deconflict_from_fresh157_step1728.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh158_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh158_train.log}"
RUN_NAME="${RUN_NAME:-fresh158}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2304}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh157 first."
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
  echo "========== ${RUN_NAME} Random Deconflict (${TOTAL_TIMESTEPS}) =========="
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
  --clip-range 0.00020 \
  --learning-rate 2.4e-7 \
  --learning-rate-end 7.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.95 \
  --force-actor-log-std -1.95 \
  --ppo-policy-loss-scale 0.000 \
  --ppo-value-loss-scale 0.000 \
  --value-coef 0.00 \
  --rollout-steps 192 \
  --reset-sampler-each-rollout \
  --update-epochs 4 \
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
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_crossing \
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
  --conflict-turn-relief 0.74 \
  --angular-authority-floor 0.38 \
  --episode-timeout 145.0 \
  --no-progress-timeout 108.0 \
  --min-progress-delta 0.006 \
  --collision-distance 0.75 \
  --near-miss-distance 1.06 \
  --scenario-neighbor-speed 0.34 \
  --near-goal-finish-weight 0.34 \
  --near-goal-finish-weight-end 0.20 \
  --near-goal-finish-distance 4.20 \
  --near-goal-finish-goal-tolerance 0.8 \
  --near-goal-finish-phase-min 0.0 \
  --near-goal-finish-target-speed 0.24 \
  --near-goal-finish-max-omega 0.12 \
  --near-goal-finish-omega-weight 0.32 \
  --near-goal-finish-crossing-only \
  --lagging-finish-weight 0.66 \
  --lagging-finish-weight-end 0.36 \
  --lagging-finish-distance 6.00 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min 0.0 \
  --lagging-finish-target-speed 0.22 \
  --lagging-finish-min-speed-scale 0.45 \
  --lagging-finish-raw-target \
  --lagging-finish-max-omega 0.10 \
  --lagging-finish-omega-weight 0.24 \
  --lagging-finish-min-team-completion 0.15 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 1.00 \
  --lagging-finish-min-team-separation 1.18 \
  --lagging-finish-safe-team-separation 1.90 \
  --lagging-finish-safe-team-separation-power 1.25 \
  --lagging-finish-crossing-only \
  --team-safety-brake-weight 0.90 \
  --team-safety-brake-weight-end 0.55 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 13.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 1.26 \
  --team-safety-brake-release-separation 3.20 \
  --team-safety-brake-target-speed 0.05 \
  --team-safety-brake-omega-weight 1.20 \
  --team-safety-brake-target-omega 0.46 \
  --team-safety-brake-turn-mode starboard \
  --team-safety-brake-head-on-starboard-threshold 0.18 \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.05 \
  --team-safety-brake-cpa-danger \
  --team-safety-brake-cpa-lookahead-distance 10.80 \
  --team-safety-brake-cpa-time-horizon 30.0 \
  --team-safety-brake-cpa-dcpa-target 1.45 \
  --team-safety-brake-cpa-closing-speed-min 0.004 \
  --team-safety-brake-random-only \
  --team-safety-brake-random-yield-agents-only \
  --team-safety-brake-random-yield-agent-weight 0.70 \
  --team-safety-brake-random-standon-agent-weight 0.18 \
  --random-deconflict-weight 2.35 \
  --random-deconflict-weight-end 1.35 \
  --random-deconflict-goal-tolerance 0.8 \
  --random-deconflict-max-distance 13.0 \
  --random-deconflict-lookahead-distance 11.80 \
  --random-deconflict-time-horizon 34.0 \
  --random-deconflict-dcpa-target 1.65 \
  --random-deconflict-closing-speed-min 0.004 \
  --random-deconflict-power 0.85 \
  --random-deconflict-standon-speed 0.31 \
  --random-deconflict-yield-speed 0.03 \
  --random-deconflict-standon-omega 0.10 \
  --random-deconflict-yield-omega 0.50 \
  --random-deconflict-omega-weight 1.15 \
  --random-deconflict-standon-weight 0.36 \
  --random-deconflict-yield-weight 1.00 \
  --policy-anchor-weight 320.0 \
  --policy-anchor-weight-end 430.0 \
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
  --domain-randomization \
  --scenario-spawn-position-std 0.045 \
  --scenario-spawn-heading-std 0.020 \
  --scenario-goal-position-std 0.035 \
  --dr-position-noise-std 0.012 \
  --dr-heading-noise-std 0.004 \
  --dr-velocity-noise-ratio 0.007 \
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

echo "${RUN_NAME} random deconflict repair complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"