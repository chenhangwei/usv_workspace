#!/bin/bash
# fresh73: Stabilize fresh72 step_21241 breakthrough (from 评估 2026-04-23)
#
# 决策依据 (fresh72 eval 结果):
#   - fresh72 在 step_21241 出现 random_encounter 突破 (succ 0→1.0, coll 1.0→0.0, prog 0.42→0.94),
#     但继续训到 step_30000 该成果被抹掉, 回退到 fresh70_stageB 水平。
#   - 新 lever (entanglement_low_speed=2.5, conflict_overspeed=4.0) 方向正确,
#     但 30K 全场景 uniform 采样 + 6e-5→2e-5 LR 让策略在后期过拟合到某个场景,
#     把 random_encounter 的解遗忘。
#   - crossing 场景 fresh72 全部 11 个 ckpt coll=1.0, 说明该场景与其他场景 reward 冲突,
#     不是 "多训几步" 能解决的 —— 留给 fresh74 单场景课程。
#
# fresh73 策略 (相对 fresh72 的最小改动):
#   [T1] 基线换成 fresh72_best.pt (= step_21241), 不再从 fresh70_stageB 重训
#   [T2] 总 steps 减半到 15K, 避免长尾过拟合
#   [T3] LR 再降一档 (3e-5 → 1e-5), 抑制"把好解练没"
#   [T4] entropy 降到 (0.003, 0.001), 减少探索噪声, 锁定当前策略
#   [T5] ckpt 间隔 1500, 评估窗口更密 (便于挑最佳)
#   [T6] 其他 lever 全部冻结, 与 fresh72 对齐 (单变量原则)
#
# 用法:
#   bash scripts/train_fresh73.sh
#
# 预期产物:
#   /mnt/data/checkpoints/usv_rl/fresh73_stabilize.pt        (最后权重)
#   /mnt/data/checkpoints/usv_rl/fresh73_checkpoints/        (~10 个中间 ckpt)
#   /mnt/data/checkpoints/usv_rl/fresh73_eval/               (每个 ckpt 的 quickeval)
#   /mnt/data/checkpoints/usv_rl/fresh73_ranking.json
#
# 成功判据 (fresh73):
#   至少 1 个 ckpt 达到: random_encounter succ >=0.6 AND crossing coll <= 0.8
#   (不要求 crossing 变好, 但不能因为长训让其他场景退化)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh72_best.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh73_stabilize.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh73_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh73_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh73_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  echo "Run: cp /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0021241.pt $BASE_INPUT"
  exit 1
fi

echo "========== fresh73 Stabilize (15K from fresh72_best @ step_21241) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 1500 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 275 \
  --checkpoint-eval-scenario solo_navigation \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario three_usv_crossing \
  --checkpoint-eval-scenario three_usv_overtaking \
  --checkpoint-eval-scenario three_usv_random_encounter \
  --num-agents 3 \
  --total-timesteps 15000 \
  --clip-range 0.10 \
  --learning-rate 3.0e-5 \
  --learning-rate-end 1.0e-5 \
  --entropy-coef 0.003 \
  --entropy-coef-end 0.001 \
  --episode-timeout 58.0 \
  --no-progress-timeout 14.0 \
  --min-progress-delta 0.45 \
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
  --conflict-distance 7.0 \
  --anticipation-distance 7.0 \
  --proximity-gradient-penalty-weight 4.0 \
  --proximity-gradient-distance 5.5 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 4.5 \
  --conflict-overspeed-penalty-weight 4.0 \
  --avoidance-turn-reward-weight 2.0 \
  --desired-conflict-speed 0.22 \
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
  --base-ros-domain-id 191

echo "fresh73 stabilize complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo "Per-ckpt eval JSONs: $EVAL_DIR"
echo "Ranking: $RANKING_JSON"
echo ""
echo "Next step:"
echo "  python3 scripts/fresh72_score_candidates.py --eval-json-dir $EVAL_DIR --top-k 5 --format table"
