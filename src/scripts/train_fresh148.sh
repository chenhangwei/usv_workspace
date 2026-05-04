#!/bin/bash
# fresh148: side-aware conflict risk with soft clear-to-route gate from fresh142.
#
# Direction:
#   - do not continue fresh145's global strong CTE/omega punishment
#   - keep close-margin/finish behavior from fresh142/fresh144
#   - raise side/oblique closing-neighbour risk so random encounters react before collision range

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

PREFERRED_BASE="/mnt/data/checkpoints/usv_rl/fresh142_close_margin_finish_route_from_fresh140.pt"
FALLBACK_BASE="/mnt/data/checkpoints/usv_rl/fresh139_checkpoints/fresh139_anticipatory_guard_from_fresh138_step0576_step_0000864.pt"
if [[ -z "${BASE_INPUT:-}" ]]; then
  if [[ -f "$PREFERRED_BASE" ]]; then
    BASE_INPUT="$PREFERRED_BASE"
  else
    BASE_INPUT="$FALLBACK_BASE"
  fi
fi

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh148_side_risk_soft_clear_from_fresh142.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh148_checkpoints}"
LOG_FILE="${LOG_FILE:-/tmp/fresh148_train.log}"
RUN_NAME="${RUN_NAME:-fresh148}"
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-480}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}"

mkdir -p "$CKPT_DIR"

if [[ -z "$BASE_INPUT" || ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found. Set BASE_INPUT or finish fresh142 first."
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
  echo "========== ${RUN_NAME} Side-Aware Conflict Risk (${TOTAL_TIMESTEPS}) =========="
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
  --clip-range 0.00032 \
  --learning-rate 5.0e-8 \
  --learning-rate-end 1.0e-8 \
  --entropy-coef 0.00000 \
  --entropy-coef-end 0.00000 \
  --actor-log-std-init -1.82 \
  --force-actor-log-std -1.82 \
  --ppo-policy-loss-scale 0.014 \
  --ppo-value-loss-scale 0.0 \
  --value-coef 0.08 \
  --rollout-steps 96 \
  --update-epochs 1 \
  --minibatch-size 160 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.55 \
  --device auto \
  --torch-num-threads 1 \
  --hidden-size 256 --hidden-size 256 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario two_usv_random_encounter \
  --scenario three_usv_random_encounter \
  --curriculum-scenario solo_navigation \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_crossing \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario three_usv_random_encounter \
  --curriculum-scenario two_usv_random_encounter \
  --curriculum-scenario two_usv_head_on \
  --curriculum-scenario three_usv_overtaking \
  --curriculum-scenario solo_navigation \
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
  --max-waypoints-per-episode 1 \
  --progress-weight 4.4 \
  --goal-bonus 112.0 \
  --team-completion-bonus 76.0 \
  --coordination-reward-weight 0.50 \
  --time-penalty 0.040 \
  --stall-penalty -118.0 \
  --collision-distance 0.75 \
  --collision-penalty -3600.0 \
  --near-miss-distance 1.06 \
  --near-miss-weight 220.0 \
  --near-miss-exponent 2.45 \
  --head-on-near-miss-distance 1.20 \
  --conflict-distance 10.0 \
  --anticipation-distance 10.0 \
  --conflict-risk-weight 2.6 \
  --conflict-bearing-floor 0.58 \
  --conflict-progress-scale 0.95 \
  --conflict-resolution-reward-weight 5.0 \
  --conflict-escalation-penalty-weight 12.0 \
  --unsafe-close-speed-penalty-weight 22.0 \
  --conflict-overspeed-penalty-weight 18.0 \
  --desired-conflict-speed 0.11 \
  --conflict-brake-weight 0.0 \
  --stop-go-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 8.4 \
  --proximity-gradient-distance 1.18 \
  --speed-distance-coupling-penalty-weight 14.0 \
  --speed-distance-coupling-threshold 1.16 \
  --separation-recovery-weight 1.6 \
  --avoidance-turn-reward-weight 3.4 \
  --team-reward-weight 0.72 \
  --team-progress-weight 1.44 \
  --team-goal-proximity-weight 0.44 \
  --team-regression-penalty-weight 2.0 \
  --team-dispersion-penalty-weight 0.06 \
  --team-dispersion-margin 0.55 \
  --deadlock-penalty-weight 9.5 \
  --entanglement-penalty-weight 10.5 \
  --entanglement-distance 3.0 \
  --entanglement-grace-steps 3 \
  --entanglement-low-speed-penalty-weight 1.90 \
  --path-deviation-penalty-weight 2.6 \
  --path-deviation-tolerance 0.62 \
  --path-deviation-conflict-scale 0.88 \
  --cte-clip-range 4.0 \
  --heading-error-weight 0.95 \
  --heading-relief-factor 0.78 \
  --heading-convergence-reward-weight 0.48 \
  --heading-convergence-threshold-deg 9.0 \
  --heading-correction-reward-weight 0.55 \
  --action-smoothness-weight 3.8 \
  --angular-accel-penalty-weight 1.8 \
  --straight-line-omega-penalty-weight 6.5 \
  --straight-line-omega-conflict-floor 0.02 \
  --straight-line-omega-cte-gate 0.52 \
  --clear-ahead-distance 3.6 \
  --clear-ahead-bearing-deg 34.0 \
  --clear-ahead-cte-weight 1.9 \
  --clear-ahead-heading-weight 0.58 \
  --clear-ahead-omega-weight 3.8 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --forward-speed-change-penalty-weight 0.55 \
  --omega-flip-saturation-threshold 0.22 \
  --pure-cruise-reward-weight 0.16 \
  --pure-idle-penalty-weight 0.10 \
  --pure-turn-penalty-weight 0.42 \
  --pure-spin-penalty-weight 0.60 \
  --head-on-guidance-distance 5.4 \
  --head-on-target-starboard-offset 0.62 \
  --head-on-phase-gate-strength 0.45 \
  --crossing-starboard-turn-reward-weight 4.0 \
  --crossing-forward-reward-weight 3.2 \
  --crossing-slowdown-reward-weight 5.0 \
  --crossing-overspeed-penalty-weight 18.0 \
  --crossing-close-forward-penalty-weight 30.0 \
  --crossing-yield-speed 0.11 \
  --crossing-time-separation-reward-weight 54.0 \
  --crossing-time-separation-penalty-weight 138.0 \
  --crossing-time-gap-target 5.8 \
  --colregs-port-turn-penalty-weight 13.0 \
  --overtaking-starboard-turn-reward-weight 2.4 \
  --overtaking-forward-reward-weight 1.6 \
  --overtaking-corridor-reward-weight 2.4 \
  --overtaking-centerline-penalty-weight 2.5 \
  --overtaking-close-penalty-weight 4.0 \
  --crossing-imitation-weight 0.018 \
  --crossing-imitation-weight-end 0.000 \
  --crossing-imitation-pretrain-epochs 0 \
  --crossing-imitation-clear-speed 0.33 \
  --crossing-imitation-middle-speed 0.20 \
  --crossing-imitation-yield-speed 0.10 \
  --crossing-imitation-clear-omega -0.015 \
  --crossing-imitation-middle-omega -0.040 \
  --crossing-imitation-yield-omega -0.065 \
  --crossing-imitation-eta-gate 0.80 \
  --crossing-imitation-phase-min -0.90 \
  --crossing-imitation-phase-max 0.05 \
  --goal-proximity-reward-weight 4.2 \
  --goal-proximity-relief-distance 4.6 \
  --goal-proximity-heading-relief 0.70 \
  --goal-proximity-smoothness-relief 0.78 \
  --goal-proximity-conflict-relief 0.28 \
  --goal-proximity-speed-relief 0.00 \
  --near-goal-idle-penalty-weight 3.0 \
  --near-goal-finish-weight 2.35 \
  --near-goal-finish-weight-end 1.05 \
  --near-goal-finish-distance 4.2 \
  --near-goal-finish-goal-tolerance 0.8 \
  --near-goal-finish-phase-min -0.25 \
  --near-goal-finish-target-speed 0.24 \
  --near-goal-finish-max-omega 0.040 \
  --near-goal-finish-omega-weight 0.36 \
  --lagging-finish-weight 1.65 \
  --lagging-finish-weight-end 0.88 \
  --lagging-finish-distance 6.4 \
  --lagging-finish-goal-tolerance 0.8 \
  --lagging-finish-phase-min -0.25 \
  --lagging-finish-target-speed 0.24 \
  --lagging-finish-min-speed-scale 0.48 \
  --lagging-finish-max-omega 0.032 \
  --lagging-finish-omega-weight 0.20 \
  --lagging-finish-min-team-completion 0.20 \
  --lagging-finish-max-team-completion 0.999 \
  --lagging-finish-near-team-tolerance 1.2 \
  --lagging-finish-min-team-separation 0.78 \
  --lagging-finish-safe-team-separation 1.20 \
  --lagging-finish-safe-team-separation-power 0.85 \
  --lagging-finish-hold-reached \
  --lagging-finish-hold-weight 1.10 \
  --team-safety-brake-weight 0.060 \
  --team-safety-brake-weight-end 0.032 \
  --team-safety-brake-goal-tolerance 0.8 \
  --team-safety-brake-max-distance 9.0 \
  --team-safety-brake-phase-min -1.0 \
  --team-safety-brake-min-team-completion 0.0 \
  --team-safety-brake-max-team-completion 0.999 \
  --team-safety-brake-near-team-tolerance 0.0 \
  --team-safety-brake-safe-separation 0.78 \
  --team-safety-brake-release-separation 1.12 \
  --team-safety-brake-target-speed 0.10 \
  --team-safety-brake-omega-weight 0.055 \
  --team-safety-brake-target-omega 0.045 \
  --team-safety-brake-turn-mode away \
  --team-safety-brake-require-neighbor \
  --team-safety-brake-local-danger \
  --team-safety-brake-power 1.35 \
  --policy-anchor-weight 1800.0 \
  --policy-anchor-weight-end 2400.0 \
  --policy-anchor-exclude-lagging-finish \
  --policy-anchor-exclude-team-safety-brake \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --attention-scenario-trunk \
  --freeze-actor-base \
  --normalize-observations \
  --freeze-observation-normalizer \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --scenario-spawn-position-std 0.055 \
  --scenario-spawn-heading-std 0.020 \
  --scenario-goal-position-std 0.045 \
  --dr-position-noise-std 0.012 \
  --dr-heading-noise-std 0.004 \
  --dr-velocity-noise-ratio 0.008 \
  --dr-current-speed-max 0.006 \
  --dr-velocity-exec-noise 0.008 \
  --dr-tau-linear-low 0.45 \
  --dr-tau-linear-high 0.80 \
  --dr-tau-angular-low 0.24 \
  --dr-tau-angular-high 0.70 \
  --encounter-type-dropout 0.04 \
  --sim-tau-linear 0.60 \
  --sim-tau-angular 0.35 \
  --num-sampler-workers 2 \
  --base-ros-domain-id "$BASE_ROS_DOMAIN_ID" \
  --separate-actor-critic-grad-clip \
  2>&1 | tee -a "$LOG_FILE"

echo "${RUN_NAME} side-aware conflict risk complete: $OUTPUT" | tee -a "$LOG_FILE"
echo "Candidates: $CKPT_DIR" | tee -a "$LOG_FILE"