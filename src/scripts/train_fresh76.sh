#!/bin/bash
# fresh76: Head-on COLREGs-prior curriculum
#
# fresh75 postmortem:
#   - top1 = step_6528, full-5 coll=0.0, but head_on still succ=0 / prog=0.36
#   - fresh75 comments claimed COLREGs/head_on-specific shaping, but command line
#     did not actually pass the currently-supported head_on prior flags
#   - result: fresh75 mostly acted as a conservative stabilize run, not a real
#     symmetry-breaking head_on intervention
#
# fresh76 strategy:
#   [T1] base = fresh75_best.pt (current most stable full-5 checkpoint)
#   [T2] keep single-scenario two_usv_head_on so the learning signal stays pure
#   [T3] re-enable real supported head_on priors:
#        - --head-on-guidance-distance
#        - --head-on-target-starboard-offset
#        - --head-on-phase-gate-strength
#        - --colregs-port-turn-penalty-weight
#   [T4] avoid fresh75's excessive crawl/early disengagement:
#        - desired_conflict_speed 0.16 -> 0.20
#        - episode_timeout 58 -> 70
#        - no_progress_timeout 14 -> 20
#        - min_progress_delta 0.45 -> 0.30
#   [T5] still use low LR/entropy, but train a bit longer (12K) so the newly-added
#        priors have time to move policy behavior
#
# success criteria:
#   primary: head_on prog >= 0.50 AND head_on succ >= 0.2
#   and preserve: crossing coll <= 0.6, rand_enc coll <= 0.6, overtaking coll == 0

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh75_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh76_headon_colregs.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh76_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh76_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh76_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  echo "Run fresh75 first, or:"
  echo "  cp /mnt/data/checkpoints/usv_rl/fresh75_checkpoints/fresh75_headon_step_0006528.pt $BASE_INPUT"
  exit 1
fi

echo "========== fresh76 Head-on COLREGs Prior (12K from fresh75_best) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 1200 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 300 \
  --checkpoint-eval-scenario two_usv_head_on \
  --num-agents 3 \
  --total-timesteps 12000 \
  --clip-range 0.10 \
  --learning-rate 1.5e-5 \
  --learning-rate-end 3.0e-6 \
  --entropy-coef 0.0015 \
  --entropy-coef-end 0.0003 \
  --episode-timeout 70.0 \
  --no-progress-timeout 20.0 \
  --min-progress-delta 0.30 \
  --scenario two_usv_head_on \
  --scenario-spawn-position-std 0.8 \
  --scenario-spawn-heading-std 0.32 \
  --scenario-goal-position-std 0.8 \
  --encounter-type-dropout 0.0 \
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
  --base-ros-domain-id 194

echo "fresh76 head-on COLREGs prior curriculum complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo ""
echo "Next steps (must run full-scenario eval):"
echo "  bash scripts/fresh76_quickeval.sh"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"