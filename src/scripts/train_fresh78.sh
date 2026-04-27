#!/bin/bash
# fresh78: Crossing repair from fresh76 head-on focus
#
# fresh77 postmortem:
#   - full-5 ultra-low-LR stabilize did NOT recover crossing: all ckpts still c=1.0
#   - it also failed to preserve fresh76's rand_enc repair
#   - this suggests the crossing fix signal is too weak when mixed back into full-5
#
# working hypothesis:
#   A short crossing-only repair course from the strongest head_on checkpoint
#   (`fresh76_headon_focus.pt`) can restore crossing behavior, just as fresh74 did,
#   while preserving the now-proven head_on prior gains.
#
# base:
#   fresh76_headon_focus.pt (= fresh76_step_4992)
#   - head_on: prog=0.80, coll=0.0
#   - crossing: coll=1.0 (only remaining blocker)
#   - rand_enc: coll=0.0, prog=0.90
#
# success criteria:
#   crossing coll <= 0.6
#   AND head_on prog >= 0.60
#   AND rand_enc coll <= 0.6
#   AND overtaking coll == 0

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh76_headon_focus.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh78_crossing_repair.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh78_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh78_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh78_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  exit 1
fi

echo "========== fresh78 Crossing Repair (8K from fresh76_headon_focus) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 800 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 300 \
  --checkpoint-eval-scenario three_usv_crossing \
  --num-agents 3 \
  --total-timesteps 8000 \
  --clip-range 0.10 \
  --learning-rate 1.2e-5 \
  --learning-rate-end 3.0e-6 \
  --entropy-coef 0.0010 \
  --entropy-coef-end 0.0002 \
  --episode-timeout 70.0 \
  --no-progress-timeout 20.0 \
  --min-progress-delta 0.30 \
  --scenario three_usv_crossing \
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
  --desired-conflict-speed 0.16 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.25 \
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
  --base-ros-domain-id 196

echo "fresh78 crossing repair complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo ""
echo "Next steps:"
echo "  bash scripts/fresh78_quickeval.sh"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"