#!/bin/bash
# fresh55: 三阶段课程学习 (3-Phase Curriculum Learning)
#
# 从 fresh54 最佳检查点 (300K, score=0.761, Safety=1.000) 热启动
#
# SITL 实测问题诊断 (session 105913):
#   1. vx 饱和: 81%+ tick vx=0.359 (99.7% 极限), 近距离仍全速 → 碰撞
#   2. omega 极端翻转: ±0.398 饱和, 34-38% tick 角速度在极端值
#   3. 轨迹合并锁: usv_01 与 usv_03 锁定 241s 无法分离
#   4. 最小距离 0.19m, 碰撞 tick 2046, 航点完成率仅 11% (2/18)
#
# 改进方案 (B4 + B5 + B6):
#
#   B4 — 新的奖励项 (已实现于 config.py + multi_agent_env.py):
#     - proximity_gradient_penalty: 1/d² 排斥场, 近距离非线性增长
#     - speed_distance_coupling_penalty: 近邻时高速惩罚
#     - heading_convergence_reward: 航向对准正奖励
#
#   B5 — 动作空间优化:
#     - 增强 action smoothness 权重 (3.8 → 5.0), 惩罚剧烈动作变化
#     - 增强 saturated_omega_flip 权重 (4.0 → 6.5), 降低阈值 (0.45 → 0.35)
#     - 增强 forward_speed_change 权重 (3.0 → 5.0), 抑制线速度震荡
#     - 增强 straight_line_omega 权重 (4.5 → 6.0), 直行时禁止大角速度
#
#   C7 — 注意力邻居聚合 (--neighbor-attention):
#     - 替换固定 4-neighbor zero-padding 为 attention-based aggregation
#     - ego state 作为 query, 邻居特征作为 key/value
#     - 网络自动学习关注最危险的邻居 (最近? TCPA? 正前方?)
#     - 从 fresh54 flat MLP 热启动: ego/encounter 权重复制,
#       attention 层 Xavier 初始化, hidden/output 层原样保留
#     - embed_dim=32, num_heads=1 (单查询注意力,足够表达)
#
#   B6 — 三阶段课程学习:
#     Phase 1 (0-100K): 2 USVs, 大间距, 聚焦导航 + 动作平滑
#     Phase 2 (100K-250K): 3 USVs, 中间距, 引入新安全奖励项
#     Phase 3 (250K-400K): 3 USVs, 小间距, 全安全惩罚 + 高遭遇密度

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh55_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh55_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh55_phase2.pt"
PHASE3_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh55.pt"
FRESH54_BEST="/mnt/data/checkpoints/usv_rl/fresh54_checkpoints/fresh54_step_0300542.pt"

mkdir -p "$CKPT_DIR"

# ─────────────────────────────────────────────────────────────
# 公共参数 (所有阶段共享)
# ─────────────────────────────────────────────────────────────
COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 3
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
  --clip-range 0.08
  --max-grad-norm 0.5
  --device auto
  --hidden-size 256 --hidden-size 256
  --max-agents 5
  --squash-actions
  --min-forward-speed 0.08
  --angular-delta-limit 0.40
  --neighbor-attention
  --attention-embed-dim 32
  --attention-num-heads 1
  --normalize-observations
  --per-scenario-advantage-norm
  --scenario-balanced-loss
  --domain-randomization
  --dr-position-noise-std 0.10
  --dr-heading-noise-std 0.02
  --dr-velocity-noise-ratio 0.03
  --dr-current-speed-max 0.04
  --dr-velocity-exec-noise 0.05
  --cruise-speed 0.36
  --max-angular-velocity 0.40
  --heading-omega-deadband 0.05
  --heading-omega-reference 0.75
  --angular-authority-power 1.3
  --angular-accel-limit 1.7
  --angular-decel-limit 2.3
  --conflict-turn-relief 0.50
  --collision-distance 0.75
  --collision-penalty -200.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.12
  --conflict-distance 7.0
  --anticipation-distance 7.0
  --head-on-guidance-distance 7.0
  --head-on-target-starboard-offset 0.90
  --heading-error-weight 1.5
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 24.0
  --checkpoint-interval 2500
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
)

# ─────────────────────────────────────────────────────────────
# Phase 1: 导航基础 + 动作平滑 (100K steps, 2 USVs, 宽松间距)
# 目标: 从 fresh54 checkpoint 开始, 强化动作平滑性,
#        抑制 vx 饱和和 omega bang-bang
# ─────────────────────────────────────────────────────────────
echo "========== Phase 1: Navigation + Smoothness (100K, 2 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE1_OUTPUT" \
  --load-weights-from "$FRESH54_BEST" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 2 \
  --total-timesteps 100000 \
  --learning-rate 3e-5 \
  --learning-rate-end 2e-5 \
  --entropy-coef 0.006 \
  --entropy-coef-end 0.004 \
  --scenario two_usv_head_on \
  --scenario-spawn-position-std 0.5 \
  --scenario-spawn-heading-std 0.20 \
  --scenario-goal-position-std 0.5 \
  --near-miss-distance 2.5 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 7.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 4.0 \
  --progress-weight 3.1 \
  --conflict-risk-weight 4.5 \
  --conflict-brake-weight 2.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 2.8 \
  --conflict-escalation-penalty-weight 3.5 \
  --unsafe-close-speed-penalty-weight 6.5 \
  --conflict-overspeed-penalty-weight 5.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 3.5 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 5.0 \
  --angular-accel-penalty-weight 3.5 \
  --straight-line-omega-penalty-weight 6.0 \
  --saturated-omega-flip-penalty-weight 6.5 \
  --forward-speed-change-penalty-weight 5.0 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.4 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 4.8 \
  --stop-go-penalty-weight 2.6 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  --crossing-starboard-turn-reward-weight 4.8 \
  --crossing-forward-reward-weight 1.6 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.2 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 3.3 \
  --proximity-gradient-penalty-weight 2.0 \
  --proximity-gradient-distance 3.0 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 2.5 \
  --heading-convergence-reward-weight 1.5 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}"

PHASE1_CKPT="$PHASE1_OUTPUT"
echo "Phase 1 complete: $PHASE1_CKPT"

# ─────────────────────────────────────────────────────────────
# Phase 2: 多场景碰撞规避 (150K steps, 3 USVs, 中等间距)
# 目标: 引入所有 3 场景, 增强 B4 安全奖励,
#        3 agent 交互训练碰撞规避
# ─────────────────────────────────────────────────────────────
echo "========== Phase 2: Multi-scenario Avoidance (150K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$PHASE1_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 150000 \
  --learning-rate 2.5e-5 \
  --learning-rate-end 1.5e-5 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.003 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 2.5 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 8.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 5.0 \
  --progress-weight 3.1 \
  --conflict-risk-weight 5.0 \
  --conflict-brake-weight 2.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.0 \
  --conflict-escalation-penalty-weight 4.0 \
  --unsafe-close-speed-penalty-weight 7.5 \
  --conflict-overspeed-penalty-weight 6.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 4.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 5.0 \
  --angular-accel-penalty-weight 3.5 \
  --straight-line-omega-penalty-weight 6.0 \
  --saturated-omega-flip-penalty-weight 6.5 \
  --forward-speed-change-penalty-weight 5.0 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.4 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 4.8 \
  --stop-go-penalty-weight 2.6 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  --crossing-starboard-turn-reward-weight 4.8 \
  --crossing-forward-reward-weight 1.6 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.2 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 3.3 \
  --proximity-gradient-penalty-weight 5.0 \
  --proximity-gradient-distance 3.0 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 2.5 \
  --heading-convergence-reward-weight 1.5 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}"

PHASE2_CKPT="$PHASE2_OUTPUT"
echo "Phase 2 complete: $PHASE2_CKPT"

# ─────────────────────────────────────────────────────────────
# Phase 3: 高密度遭遇 (150K steps, 3 USVs, 紧密间距)
# 目标: 缩小生成间距, 最大化安全惩罚权重,
#        训练极端近距离场景下的避碰能力
# ─────────────────────────────────────────────────────────────
echo "========== Phase 3: Dense Encounters (150K, 3 USVs, close spawn) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE3_OUTPUT" \
  --load-weights-from "$PHASE2_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 150000 \
  --learning-rate 1.5e-5 \
  --learning-rate-end 8e-6 \
  --entropy-coef 0.004 \
  --entropy-coef-end 0.002 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.15 \
  --scenario-spawn-heading-std 0.10 \
  --scenario-goal-position-std 0.15 \
  --near-miss-distance 3.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 9.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 6.0 \
  --progress-weight 3.1 \
  --conflict-risk-weight 5.5 \
  --conflict-brake-weight 3.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.5 \
  --conflict-escalation-penalty-weight 5.0 \
  --unsafe-close-speed-penalty-weight 8.5 \
  --conflict-overspeed-penalty-weight 7.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 5.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 5.5 \
  --angular-accel-penalty-weight 4.0 \
  --straight-line-omega-penalty-weight 6.5 \
  --saturated-omega-flip-penalty-weight 7.0 \
  --forward-speed-change-penalty-weight 5.5 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.4 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 5.5 \
  --stop-go-penalty-weight 3.0 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  --crossing-starboard-turn-reward-weight 4.8 \
  --crossing-forward-reward-weight 1.6 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.2 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 3.3 \
  --proximity-gradient-penalty-weight 7.0 \
  --proximity-gradient-distance 3.5 \
  --speed-distance-coupling-penalty-weight 8.0 \
  --speed-distance-coupling-threshold 3.0 \
  --heading-convergence-reward-weight 2.0 \
  --heading-convergence-threshold-deg 10.0 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 180 \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario three_usv_crossing \
  --checkpoint-eval-scenario three_usv_overtaking \
  --checkpoint-ranking-json /mnt/data/checkpoints/usv_rl/fresh55_ranking.json \
  --checkpoint-eval-json-dir /mnt/data/checkpoints/usv_rl/fresh55_eval \
  "${COMMON_ARGS[@]}"

echo "========== fresh55 training complete =========="
echo "Final model: $PHASE3_OUTPUT"
echo "Checkpoint ranking: /mnt/data/checkpoints/usv_rl/fresh55_ranking.json"
