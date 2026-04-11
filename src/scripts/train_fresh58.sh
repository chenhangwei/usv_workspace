#!/bin/bash
# fresh58: 修复 fresh57 SITL 3-USV 真机碰撞问题
#
# fresh57 SITL 诊断 (2026-04-09, ~/usv_logs/ 3-USV):
#   碰撞: usv01-usv02 (0.425m), usv02-usv03 (0.444m)
#   近距离: usv01-usv03 (0.895m)
#
# 5 个根因:
#   R1 — tau_angular sim-to-real gap: DR=0.6-1.0, 实际=0.3 → ω效果放大 2-3x
#   R2 — RL omega 饱和 22-26%, 碰撞时双边打满 ±0.398
#   R3 — (不修) ORCA 安全层关闭 (不需要启用)
#   R4 — 长时间近距离纠缠不分离 (538s 在 <5m 内)
#   R5 — 遭遇类型分辨力不足 (type1=75%+)
#
# fresh58 修复:
#   F1 — 修正 tau_angular DR 范围: 0.6-1.0 → 0.2-0.6, sim=0.35
#         * 匹配真实 USV tau_omega ≈ 0.3
#   F2 — omega 反饱和加强: flip_penalty 9→15, threshold 0.35→0.30
#         * 同时加大 action_smoothness 抑制频繁变向
#   F3 — 近距离减速:
#         * speed_scale_distance 3.0→5.0, min=0.35→0.15
#         * speed_distance_coupling 加大到 12.0, threshold 3.0→5.0
#         * desired_conflict_speed 0.12→0.15 (不低于 0.15 m/s)
#   F4 — 分离奖励 (重点):
#         * separation_recovery_weight 6.0→15.0 (距离增加 → 正奖励)
#         * proximity_gradient_penalty 6.0→10.0, distance 4.0→6.0
#         * proximity_gradient 使用 1/d² 斥力场，在 6m 内持续推排
#   F5 — 注意力保留: 32d/1h neighbor-attention (兼容 fresh57 热启动)
#         * 邻居感知通过 attention query/key/value 机制保障
#   F6 — tau_linear 也修正: DR 1.5-2.5 → 1.0-2.0 (更接近真实)
#   F7 — 多场景 Phase 2 加入 reflective 场景:
#         * 训练更多遭遇变体，提升 encounter 分辨力
#
# 保留 fresh57 改进:
#   B1 — collision_penalty=-350, near_miss_distance=5.0
#   B2 — force_actor_log_std=-1.0
#   B3 — neighbor-attention (升级到 64d/2h)
#   B4 — per-scenario-advantage-norm + scenario-balanced-loss
#
# 2阶段课程:
#   Phase 1 (0-80K): 2 USVs head-on, 匹配真实动态
#   Phase 2 (80K-300K): 3 USVs 多场景, 加大 separation 奖励

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh58_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh58_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh58.pt"
# 从 fresh57 最终模型热启动 (保留已学安全行为)
FRESH57_FINAL="/mnt/data/checkpoints/usv_rl/fresh57.pt"

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
  # 注意力: 保持 32d/1h (兼容 fresh57 权重热启动)
  # 邻居感知已通过 attention 机制保障
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
  # F3: conflict speed 不低于 0.15 m/s
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
  # F1+F6: 修正 tau DR 匹配真实 (tau_angular 实际 ≈ 0.3)
  --sim-tau-linear 1.5
  --sim-tau-angular 0.35
  --dr-tau-linear-low 1.0
  --dr-tau-linear-high 2.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  # F3: 近距离减速 (distance 5m, 最低 0.15 m/s)
  --speed-scale-distance 5.0
  --speed-scale-min 0.15
)

# ─────────────────────────────────────────────────────────────
# Phase 1: 真实动态匹配 + 分离行为学习 (80K steps, 2 USVs)
# ─────────────────────────────────────────────────────────────
echo "========== Phase 1: Real Dynamics + Separation (80K, 2 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE1_OUTPUT" \
  --load-weights-from "$FRESH57_FINAL" \
  --force-actor-log-std -1.0 \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 2 \
  --total-timesteps 80000 \
  --learning-rate 3e-5 \
  --learning-rate-end 2e-5 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.003 \
  --scenario two_usv_head_on \
  --scenario-spawn-position-std 0.5 \
  --scenario-spawn-heading-std 0.20 \
  --scenario-goal-position-std 0.5 \
  --near-miss-distance 5.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 12.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 12.0 \
  --progress-weight 2.6 \
  --conflict-risk-weight 6.0 \
  --conflict-brake-weight 3.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.5 \
  --conflict-escalation-penalty-weight 5.0 \
  --unsafe-close-speed-penalty-weight 10.0 \
  --conflict-overspeed-penalty-weight 10.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 4.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 6.5 \
  --angular-accel-penalty-weight 4.5 \
  --straight-line-omega-penalty-weight 8.0 \
  --saturated-omega-flip-penalty-weight 15.0 \
  --forward-speed-change-penalty-weight 5.5 \
  --omega-flip-saturation-threshold 0.30 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 8.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 5.0 \
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
  --proximity-gradient-penalty-weight 8.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 10.0 \
  --speed-distance-coupling-threshold 5.0 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 15.0 \
  "${COMMON_ARGS[@]}"

PHASE1_CKPT="$PHASE1_OUTPUT"
echo "Phase 1 complete: $PHASE1_CKPT"

# ─────────────────────────────────────────────────────────────
# Phase 2: 多场景 + 积极分离 (220K steps, 3 USVs)
# ─────────────────────────────────────────────────────────────
echo "========== Phase 2: Multi-scenario + Active Separation (220K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$PHASE1_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 220000 \
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
  --proximity-gradient-penalty-weight 10.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 12.0 \
  --speed-distance-coupling-threshold 5.0 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}"

echo "========== fresh58 training complete =========="
echo "Final model: $PHASE2_OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh58_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh58_sitl_gate.sh"
