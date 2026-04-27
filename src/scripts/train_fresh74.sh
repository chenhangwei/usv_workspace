#!/bin/bash
# fresh74: Crossing 单场景课程 (方向 A from fresh73 分析)
#
# 决策依据 (fresh73 评估):
#   - fresh73_step_9149 在 random_encounter 拿到 succ=1.0, overtaking sep 0.82→2.19
#   - 但 crossing 在所有 12 个 ckpt 一致 coll=1.00, 与 fresh72 完全相同
#   - 说明:
#     a) 多场景 uniform 采样下, crossing 的"互相绕死"局部最优特别强
#     b) 全场景 reward 平均会让 crossing 训练信号被淹没
#     c) entanglement_low_speed + conflict_overspeed 对 crossing 几何特性无效
#       (crossing 是 90° 切线相遇, 与 head_on/random 的策略不同)
#
# fresh74 策略 (单变量 + 单场景):
#   [T1] 仅 --scenario three_usv_crossing, 不混其他场景 → 信号纯净
#   [T2] base = fresh73_best.pt, 不能丢失 random_enc/overtaking 突破
#   [T3] 短训 (8000 步) + 极低 LR (2e-5→5e-6), entropy 锁死 (0.002→0.0005)
#       目的: "微调 crossing 行为, 不动其他场景策略"
#   [T4] 加强 crossing-specific lever:
#        - conflict_distance 7.0 → 9.0 (crossing 90° 切线需更早预判)
#        - speed_distance_coupling_threshold 4.5 → 5.5 (更早减速)
#        - avoidance_turn_reward_weight 2.0 → 4.0 (奖励主动转向)
#        - desired_conflict_speed 0.22 → 0.16 (鼓励更慢通过冲突区)
#   [T5] ckpt/800 步, auto-eval 仅 crossing (省时间)
#   [T6] 评估时仍跑全 5 场景 (用 fresh73_quickeval 模式), 验证不退化
#
# 警告/风险:
#   - 极易把 random_encounter 的 succ=1.0 训没 (单场景训练 = 灾难性遗忘)
#   - 必须每个 ckpt 跑全场景 quickeval 监控
#   - 若全场景退化但 crossing 改善, 仍是失败 (违背 ckpt 选择策略)
#
# 成功判据:
#   至少 1 个 ckpt 满足:
#     crossing coll <= 0.6 (从 1.0 降下来)
#     AND random_encounter succ >= 0.6 (保留 fresh73 突破)
#     AND overtaking coll == 0
#
# 用法:
#   bash scripts/train_fresh74.sh

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh73_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh74_crossing.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh74_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh74_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh74_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  echo "Run fresh73 first, or:"
  echo "  cp /mnt/data/checkpoints/usv_rl/fresh73_checkpoints/fresh73_stabilize_step_0009149.pt $BASE_INPUT"
  exit 1
fi

echo "========== fresh74 Crossing-only Curriculum (8K from fresh73_best) =========="
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
  --checkpoint-eval-scenario three_usv_crossing \
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
  --heading-correction-reward-weight 0.0 \
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
  --base-ros-domain-id 192

echo "fresh74 crossing curriculum complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo ""
echo "Next steps (CRITICAL: must run full-scenario eval to detect regression):"
echo "  bash scripts/fresh74_quickeval.sh   # full 5-scenario eval"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"
