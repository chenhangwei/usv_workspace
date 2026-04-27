#!/bin/bash
# fresh75: Head-on 单场景课程 (方向 A from fresh74 分析)
#
# 决策依据 (fresh74 评估):
#   - fresh74_step_1728 一次性修复 crossing (coll 1.0→0.0) + random_enc (coll 1.0→0.0, prog 0.39→0.81)
#   - 所有 5 场景中 head_on 是唯一始终未取得任何成功 (succ=0) 的场景
#   - 连续 fresh72/73/74 三代 head_on 均 prog ~0.36, succ=0, sep~4.6 (过度避让, 不再相遇)
#   - 假设: head_on 几何对称, COLREGs 要求双方右转, 单一 reward shaping 难收敛
#
# fresh75 策略 (镜像 fresh74 单场景 + head_on 专用 levers):
#   [T1] 仅 --scenario two_usv_head_on, 信号纯净
#   [T2] base = fresh74_best.pt, 不能丢失 crossing/random_enc/overtaking 突破
#   [T3] 短训 (8000 步) + 极低 LR (2e-5→5e-6), entropy 锁死 (0.002→0.0005)
#   [T4] head_on-specific lever:
#        - colregs_compliance_reward_weight 抬高 (head_on 强制右转)
#        - cpa_starboard_pass_reward_weight 抬高 (经过点必须在右舷)
#        - encounter_pass_reward_weight 抬高 (鼓励"擦肩而过"而非提早绕开)
#        - separation 软上限 (避免无限远绕)
#        - heading_correction_reward_weight 抬高 (重新瞄准 goal)
#        - goal_proximity_conflict_relief 抬高 (临近终点时 conflict penalty 放松)
#   [T5] ckpt/800 步, auto-eval 仅 head_on (省时)
#   [T6] 评估时仍跑全 5 场景 quickeval, 验证 fresh74 突破不退化
#
# 警告/风险:
#   - 极易让 crossing/random_enc 退化 (catastrophic forgetting)
#   - head_on succ 可能仍为 0 (需要 episode 更长或 reward 更彻底重做)
#
# 成功判据:
#   至少 1 个 ckpt 满足:
#     head_on prog >= 0.50 AND head_on succ >= 0.4 (突破) — 主要目标
#     AND crossing coll <= 0.6 (保留 fresh74 突破)
#     AND random_encounter coll <= 0.6 (保留 fresh74 突破)
#     AND overtaking coll == 0 (不退化)
#   退而求其次: head_on prog >= 0.50 + 不退化 = 部分胜利
#
# 用法:
#   bash scripts/train_fresh75.sh

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh74_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh75_headon.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh75_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh75_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh75_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  echo "Run fresh74 first, or:"
  echo "  cp /mnt/data/checkpoints/usv_rl/fresh74_checkpoints/fresh74_crossing_step_0001728.pt $BASE_INPUT"
  exit 1
fi

echo "========== fresh75 Head-on Curriculum (8K from fresh74_best) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 800 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 275 \
  --checkpoint-eval-scenario two_usv_head_on \
  --num-agents 3 \
  --total-timesteps 8000 \
  --clip-range 0.10 \
  --learning-rate 2.0e-5 \
  --learning-rate-end 5.0e-6 \
  --entropy-coef 0.002 \
  --entropy-coef-end 0.0005 \
  --episode-timeout 58.0 \
  --no-progress-timeout 14.0 \
  --min-progress-delta 0.45 \
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
  --base-ros-domain-id 193

echo "fresh75 head-on curriculum complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo ""
echo "Next steps (CRITICAL: must run full-scenario eval to detect regression):"
echo "  bash scripts/fresh75_quickeval.sh   # full 5-scenario eval"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"
