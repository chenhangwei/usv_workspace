#!/bin/bash
# fresh71: 恢复避让意愿 + 提高巡航速度 (修复 fresh70 “硬冲不避”问题)
#
# fresh70 现状:
#   ✅ omega_flip 填补 (crossing 91→26, head_on 37→16)
#   ✅ solo he/CTE 低很多 (he 0.38, CTE 0.71)
#   ❌ random_encounter collision 87.5%→100% (变差)
#   ❌ progress 普遍卡 0.30-0.42 (巡航太慢)
#   原因: avoidance_turn_reward=2 转弯没收益 + entanglement=1.5
#         强迫“必须脱离” → policy 选择“硬冲过去脱离”
#         且 cruise=0.22 太保守, 60s episode 走不完
#
# fresh71 调整 (vs fresh70):
# [H1] avoidance-turn-reward-weight 2.0 → 5.0 (重启转弯激励)
# [H2] entanglement-penalty-weight 1.5 → 0.5 (弱化强制脱离)
# [H3] separation-recovery-weight 1.0 → 3.0 (脱离奖励 ×3)
# [H4] desired-conflict-speed 0.22 → 0.30 (允许更高巡航)
# [H5] total-timesteps 120000 → 80000 (节约 1h, 能看趋势就够)
# 其它 fresh70 已验证有效的保持不变:
#   saturated_omega_flip=8.0 → 压 omega_flip
#   heading_relief_factor=0.85 → 冲突时 heading 让位
#   action_smoothness=6.0 → 抱动
#   collision_penalty=-400, near_miss=12, proximity_grad=4 → 避撞证明有效
#
# Stage A 复用 fresh68_stageA.pt
#
# fresh69 成果 (保留):
#   - solo: he 0.41 ✅ / CTE 0.74 ✅ / progress 0.396 (差 0.004)
#   - head_on/overtaking: collision 0% ✅ / omega_flip 5.0-36.9
# fresh69 问题:
#   - crossing/random: omega_flip 92-102 (fresh68 是 15-19, 恶化 5×)
#   - 根因: heading_error_weight=5 + avoidance_turn=2 使 3 船近距离时
#     heading 跟踪与避让两个目标冲突，policy 高频抖动
#
# fresh70 修复 (纯 CLI, 不改 env 代码):
# [G1] heading-relief-factor 0.60 → 0.85
#      → conflict_risk=1 时 heading_scale = 0.10 (floor)而不是 0.40
#      → 近距离时 heading 惩罚几乎消失，避让可专注
# [G2] saturated-omega-flip-penalty-weight 0 → 8.0
#      → 直接惩罚 omega 反向 bang-bang (正是 omega_flip=92 的原因)
# [G3] omega-flip-saturation-threshold 0.65 → 0.40
#      → 捕获较轻的翻转（不只是饱和翻转）
# [G4] action-smoothness-weight 4.0 → 6.0
# [G5] separation-recovery-weight 0 → 1.0
#      → 拉开距离有正奖励, 鼓励“脱离”而不是“振荡”
# [G6] entanglement-penalty-weight 0 → 1.5
#      → 反轨道锁定 (防止两船互相圈圈)
# [G7] heading-error-weight 5.0 (不变，保护 solo)
# [G8] near-miss 12 / collision -400 / proximity 4 (不变，fresh69 证明有效)
#
# Stage A 复用 fresh68_stageA.pt
#
# fresh68 Stage B 0/5 根因 (relative to Stage A):
#   - solo he 0.080 → 0.785 (10×恶化), CTE 0.063 → 0.811 (13×恶化)
#   - multi 碰撞率 87.5%, omega_flip 15-19
#   - 根因: avoidance_turn_reward=8 + proximity_gradient=10 压倒了
#     heading 跟踪和 -150 collision_penalty。policy 学会了"剧烈摆头避让"
#     反而撞得更多。
#
# fresh69 修复（相对 fresh68，仅改 Stage B/C 的 reward）：
# [F1] avoidance_turn_reward_weight: 8.0 → 2.0（不再奖励大幅转向）
# [F2] proximity_gradient_penalty_weight: 10.0 → 4.0（缓和接近梯度惩罚）
# [F3] collision_penalty: -150 → -400（撞船痛到不敢转）
# [F4] near_miss_weight: 8.0 → 12.0（提前减速更值钱）
# [F5] heading_error_weight: 3.0 → 5.0 in Stage B（保护 Stage A 学到的航向能力）
# [F6] action_smoothness_weight: 2.0 → 4.0（直接抑制 omega_flip）
# [F7] entropy_coef: 0.015 → 0.008（降低探索, 防止覆盖 Stage A 策略）
# [F8] learning_rate: 2e-4 → 1e-4（更小步长，减少灾难性遗忘）
# Stage A 完全不变（已经收敛, he/CTE 优秀）
# 直接复用 fresh68_stageA.pt 作为 Stage B 的起点
#
# fresh68 stageA 备注 (来自 fresh68 日志)
#
# fresh67 SITL gate 根因（Stage A 0/5, Stage B 0/5, Stage A 比 Stage B 还差）:
#   - solo progress 只有 0.016, he 1.58rad, omega_sat 72.5%
#   - 诊断: heading_correction_reward=6 + heading_error_weight=8 创造了"原地对准"
#     的局部最优。time_penalty=0.035 惩罚 60s × 0.035 = 2.1 远低于 heading 可拿到的
#     累积奖励。policy 学会了"小幅转向不前进"的退化策略。
#
# L2 核心修复（相对 fresh67）:
# [L2-A] 杀 heading_correction_reward_weight: 6.0 → 0.0
# [L2-B] 降 heading_error_weight: 8.0 → 3.0（保留方向惩罚，但不压制 progress）
# [L2-C] 抬 time_penalty: 0.035 → 0.08（"不动"的成本翻倍以上）
# [L2-D] 抬 progress_weight: 4.0/3.0 → 8.0/6.0/5.0（让前进主导）
# [L2-E] 抬 stall_penalty: -25 → -60（重罚原地）
# [L2-F] 禁用 min_forward_speed_floor: 0.08 → 0.0（让 policy 自由选择速度）
# [L2-G] 保留 angular_authority_floor=0.35（它是对的；head_on omega_flip=1.4 证明）
# [L2-H] 分阶段 dry-run: 先只跑 Stage A 50K 验证 solo 学好后再续 B/C
#
# 跑法:
#   bash scripts/train_fresh69.sh stageB        # 直接跑 Stage B 120K（复用 fresh68_stageA.pt）
#   bash scripts/train_fresh69.sh stageC        # 续 Stage C 160K

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

MODE="${1:-stageA}"

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh71_checkpoints"
STAGE_A_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh71_stageB.pt"
STAGE_C_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh71_stageC.pt"

mkdir -p "$CKPT_DIR"

COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 4
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
  --max-grad-norm 0.5
  --device auto
  --hidden-size 256 --hidden-size 256
  --max-agents 5
  --squash-actions
  --min-forward-speed 0.05
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
  --heading-omega-reference 1.0
  --angular-authority-power 1.0
  --angular-accel-limit 1.2
  --angular-decel-limit 2.8
  --conflict-turn-relief 0.50
  --angular-authority-floor 0.35
  --min-forward-speed-floor 0.0
  --collision-distance 0.75
  --sim-tau-linear 0.6
  --sim-tau-angular 0.35
  --dr-tau-linear-low 0.4
  --dr-tau-linear-high 1.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 185
)

run_stage_a() {
  echo "========== Stage A: Solo only (50K dry-run) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_A_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 1 \
    --total-timesteps 80000 \
    --clip-range 0.15 \
    --learning-rate 3.0e-4 \
    --learning-rate-end 1.0e-4 \
    --entropy-coef 0.02 \
    --entropy-coef-end 0.012 \
    --episode-timeout 60.0 \
    --no-progress-timeout 12.0 \
    --min-progress-delta 0.50 \
    --max-waypoints-per-episode 2 \
    --waypoint-bonus 12.0 \
    --scenario solo_navigation \
    --scenario-spawn-position-std 0.5 \
    --scenario-spawn-heading-std 0.30 \
    --scenario-goal-position-std 0.5 \
    --cte-clip-range 5.0 \
    --progress-weight 8.0 \
    --goal-bonus 35.0 \
    --time-penalty 0.08 \
    --stall-penalty -60.0 \
    --heading-error-weight 3.0 \
    --heading-relief-factor 0.55 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 2.0 \
    --collision-penalty -150.0 \
    --near-miss-distance 4.0 \
    --near-miss-weight 5.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 6.0 \
    --anticipation-distance 6.0 \
    --proximity-gradient-penalty-weight 6.0 \
    --proximity-gradient-distance 5.0 \
    --speed-distance-coupling-penalty-weight 2.0 \
    --speed-distance-coupling-threshold 4.0 \
    --avoidance-turn-reward-weight 5.0 \
    --desired-conflict-speed 0.22 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.55 \
    --goal-proximity-smoothness-relief 0.70 \
    --goal-proximity-conflict-relief 0.20 \
    "${COMMON_ARGS[@]}"

  echo "Stage A complete: $STAGE_A_OUTPUT"
}

run_stage_b() {
  echo "========== Stage B: Solo + Head-on + Crossing (120K) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_B_OUTPUT" \
    --load-weights-from "$STAGE_A_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 3 \
    --total-timesteps 80000 \
    --clip-range 0.12 \
    --learning-rate 1.0e-4 \
    --learning-rate-end 3.0e-5 \
    --entropy-coef 0.008 \
    --entropy-coef-end 0.005 \
    --episode-timeout 58.0 \
    --no-progress-timeout 14.0 \
    --min-progress-delta 0.45 \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario-spawn-position-std 0.7 \
    --scenario-spawn-heading-std 0.30 \
    --scenario-goal-position-std 0.7 \
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
    --separation-recovery-weight 3.0 \
    --entanglement-penalty-weight 0.5 \
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
    --avoidance-turn-reward-weight 5.0 \
    --desired-conflict-speed 0.30 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.72 \
    --goal-proximity-conflict-relief 0.25 \
    "${COMMON_ARGS[@]}"

  echo "Stage B complete: $STAGE_B_OUTPUT"
}

run_stage_c() {
  echo "========== Stage C: All scenarios (160K) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_C_OUTPUT" \
    --load-weights-from "$STAGE_B_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 3 \
    --total-timesteps 160000 \
    --clip-range 0.10 \
    --learning-rate 1.2e-4 \
    --learning-rate-end 3.0e-5 \
    --entropy-coef 0.012 \
    --entropy-coef-end 0.008 \
    --episode-timeout 55.0 \
    --no-progress-timeout 16.0 \
    --min-progress-delta 0.45 \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario two_usv_random_encounter \
    --scenario three_usv_random_encounter \
    --scenario-spawn-position-std 1.0 \
    --scenario-spawn-heading-std 0.40 \
    --scenario-goal-position-std 1.0 \
    --encounter-type-dropout 0.3 \
    --cte-clip-range 5.0 \
    --progress-weight 5.0 \
    --goal-bonus 45.0 \
    --time-penalty 0.06 \
    --stall-penalty -50.0 \
    --heading-error-weight 3.0 \
    --heading-relief-factor 0.70 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 2.0 \
    --collision-penalty -180.0 \
    --near-miss-distance 5.0 \
    --near-miss-weight 10.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 8.0 \
    --anticipation-distance 8.0 \
    --proximity-gradient-penalty-weight 14.0 \
    --proximity-gradient-distance 6.0 \
    --speed-distance-coupling-penalty-weight 4.0 \
    --speed-distance-coupling-threshold 5.0 \
    --avoidance-turn-reward-weight 12.0 \
    --desired-conflict-speed 0.22 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.75 \
    --goal-proximity-conflict-relief 0.30 \
    "${COMMON_ARGS[@]}"

  echo "========== fresh68 complete =========="
}

case "$MODE" in
  stageA) echo "fresh69 复用 fresh68_stageA.pt，无需重训。直接 stageB。"; exit 0 ;;
  stageB) run_stage_b ;;
  stageC) run_stage_c ;;
  all)    run_stage_b && run_stage_c ;;
  *) echo "Usage: $0 {stageB|stageC|all}"; exit 1 ;;
esac
