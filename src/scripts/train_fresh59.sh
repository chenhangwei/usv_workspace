#!/bin/bash
# fresh59: 解决 fresh58 3-USV 轨道锁定 (orbital lock) 问题
#
# fresh58 SITL 诊断 (2026-04-10, ~/usv_logs/ 3-USV):
#   碰撞: 0 (安全性合格)
#   轨道锁定: USV01-USV03 (85% 时间 <1.5m, 中位 0.97m)
#   USV03: omega=-0.279 (82% 左转), 5.8 圈全旋转
#   纠缠时长: 845s, USV03 encounter_type=head_on 93% (误分类)
#   结果: 2/3 USV 未到达目标 (USV01: 3.32m, USV03: 2.36m)
#
# 根因: 策略学到了通过轨道运动避碰的局部最优
#        近距离惩罚 (proximity_gradient) 是瞬时的, 轨道运动可以维持
#        缺少时间积分型惩罚来打破稳定的轨道锁定
#
# fresh59 修复 (在 fresh58 基础上, minimal change):
#   F1 — 新增 entanglement duration penalty (核心改动):
#         * 跟踪 pair_min < 3m 的持续步数
#         * 超过 grace period (30 步 ≈ 6s) 后, 惩罚 sqrt 增长
#         * 权重 3.0, 最大 penalty = 3.0 * 10.0 = 30.0/step
#         * 打破轨道锁定的稳定均衡
#   F2 — 加宽 heading convergence threshold (12° → 30°):
#         * 轨道运动时 heading error 大, 窄阈值激活概率低
#         * 30° 阈值让 heading convergence 在脱离过程中也能提供正信号
#   F3 — 提升 heading convergence weight (3.0 → 5.0):
#         * 更强的 goal-heading 吸引力, 避让后快速恢复航向
#   F4 — 提升 deadlock penalty (5.5 → 8.0):
#         * 轨道锁定本质是一种 deadlock, 加强惩罚
#
# 保留 fresh58 所有其它参数:
#   tau DR 0.2-0.6, sim_tau_angular=0.35
#   omega 反饱和: flip=15, threshold=0.30
#   近距离减速: scale_distance=5.0, min=0.15
#   分离奖励: separation_recovery=15.0
#   proximity_gradient=10.0 (1/d² 斥力场)
#
# 训练策略:
#   单阶段: 从 fresh58_phase1.pt 热启动 (干净的 2-USV 安全行为)
#   300K steps, 3 USVs, 多场景
#   不重做 Phase 1 (fresh58 Phase 1 已验证安全)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh59_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh59.pt"
LOAD_FROM="/mnt/data/checkpoints/usv_rl/fresh58_phase1.pt"

mkdir -p "$CKPT_DIR"

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
  --collision-penalty -350.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.15
  --conflict-distance 8.0
  --anticipation-distance 8.0
  --head-on-guidance-distance 8.0
  --head-on-target-starboard-offset 0.90
  --heading-error-weight 2.0
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 24.0
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
  --sim-tau-linear 1.5
  --sim-tau-angular 0.35
  --dr-tau-linear-low 1.0
  --dr-tau-linear-high 2.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --speed-scale-distance 5.0
  --speed-scale-min 0.15
)

echo "========== fresh59: Anti-Entanglement Training (300K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$LOAD_FROM" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 300000 \
  --learning-rate 2.5e-5 \
  --learning-rate-end 8e-6 \
  --entropy-coef 0.004 \
  --entropy-coef-end 0.002 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 5.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 14.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 15.0 \
  --entanglement-penalty-weight 3.0 \
  --entanglement-distance 3.0 \
  --entanglement-grace-steps 30 \
  --progress-weight 2.6 \
  --conflict-risk-weight 7.0 \
  --conflict-brake-weight 3.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 4.0 \
  --conflict-escalation-penalty-weight 6.0 \
  --unsafe-close-speed-penalty-weight 12.0 \
  --conflict-overspeed-penalty-weight 12.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 5.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 7.0 \
  --angular-accel-penalty-weight 5.0 \
  --straight-line-omega-penalty-weight 8.0 \
  --saturated-omega-flip-penalty-weight 15.0 \
  --forward-speed-change-penalty-weight 6.0 \
  --omega-flip-saturation-threshold 0.30 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 8.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 8.0 \
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
  --proximity-gradient-penalty-weight 10.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 12.0 \
  --speed-distance-coupling-threshold 5.0 \
  --heading-convergence-reward-weight 5.0 \
  --heading-convergence-threshold-deg 30.0 \
  "${COMMON_ARGS[@]}"

echo "========== fresh59 training complete =========="
echo "Final model: $OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh59_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh59_sitl_gate.sh"
