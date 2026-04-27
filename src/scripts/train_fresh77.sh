#!/bin/bash
# fresh77: Full-5 stabilize from fresh76 head_on breakthrough
#
# fresh76 postmortem:
#   - true supported head_on/COLREGs priors worked: step_4992 reached
#     head_on prog=0.80 (from 0.36), rand_enc coll=0.0 / prog=0.90
#   - but because fresh76 was still head_on-only curriculum, crossing regressed to c=1.0
#     for every fresh76 checkpoint
#
# working hypothesis:
#   The fresh76 priors are directionally correct for head_on, and the remaining problem is
#   catastrophic forgetting of crossing under single-scenario training. Therefore the next
#   discriminating step is NOT more head_on-only training, but low-LR full-5 stabilization
#   from the strongest head_on checkpoint that already restored rand_enc.
#
# chosen base:
#   fresh76_step_4992
#   - head_on: succ=0.0 coll=0.0 prog=0.80
#   - crossing: succ=0.0 coll=1.0 prog=0.46
#   - overtake: succ=0.0 coll=0.0 prog=0.55
#   - rand_enc: succ=0.0 coll=0.0 prog=0.90
#
# strategy:
#   [T1] warmstart from fresh76_step_4992 (best head_on / rand_enc trade-off)
#   [T2] train on all 5 scenarios again to recover crossing without throwing away head_on priors
#   [T3] keep fresh76 head_on priors active, but use ultra-low LR/entropy and short horizon
#   [T4] pick best checkpoint only by offline full-5 quickeval

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh76_headon_focus.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh77_stabilize.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh77_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh77_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh77_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  echo "Expected: fresh76 step_4992 locked as head_on focus checkpoint"
  exit 1
fi

echo "========== fresh77 Full-5 Stabilize (10K from fresh76_step_4992) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 1000 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 300 \
  --checkpoint-eval-scenario solo_navigation \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario three_usv_crossing \
  --checkpoint-eval-scenario three_usv_overtaking \
  --checkpoint-eval-scenario three_usv_random_encounter \
  --num-agents 3 \
  --total-timesteps 10000 \
  --clip-range 0.10 \
  --learning-rate 8.0e-6 \
  --learning-rate-end 2.0e-6 \
  --entropy-coef 0.0005 \
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
  --encounter-type-dropout 0.20 \
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
  --desired-conflict-speed 0.20 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.45 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --squash-actions \
  --min-forward-speed 0.05 \
  --angular-delta-limit 0.40 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
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
  --dr-tau-angular-high 0.6 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 195

echo "fresh77 full-5 stabilize complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo ""
echo "Next steps:"
echo "  bash scripts/fresh77_quickeval.sh"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"