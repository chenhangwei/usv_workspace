#!/bin/bash
# fresh57: 修复 fresh56 近距离惩罚区域不足的问题
#
# fresh56 诊断结果:
#   - Phase 1 策略在 ~30K 步后崩溃 (40% → 100% 碰撞)
#   - 根因: near_miss_distance=2.5m 对 tau_linear=2.0 的慢动态太小
#     - 制动距离 ≈ cruise_speed * tau = 0.36 * 2.0 = 0.72m                                                                                                 
#     - 2.5m 惩罚区 - 0.75m 碰撞区 = 仅 1.75m 有效警告距离
#     - 加上 0.72m 制动距离，agent 进入惩罚区时已来不及停
#   - headon30 成功案例使用 6.0m near_miss，在1m处惩罚-23.8/step
#   - fresh56 在1m处仅 -6.6/step，不足以阻止 agent 冲向目标
#
# fresh57 修复:
#   F1 — 增大 near_miss_distance: 2.5m → 5.0m (匹配慢动态制动需求)
#   F2 — 增大 near_miss_weight: 7.0 → 12.0 (在1m处惩罚: ~-27/step)
#   F3 — 增大 collision_penalty: -200 → -350 (更强硬边界)
#   F4 — 缩短 Phase 1: 120K → 80K (基于 fresh56 学习曲线,30K后衰退)
#   F5 — 减少 checkpoint_interval: 2500 → 2000 (更细粒度选择)
#   F6 — 适度降低 progress 奖励权重 (避免目标奖励主导安全惩罚)
#
# 保留 fresh56 改进:
#   B1 — sim_tau_linear=2.0, sim_tau_angular=0.8
#   B4 — force_actor_log_std=-1.0
#   B5 — 增强 heading/spin 奖励 (保留)
#   训练时 speed_scale (distance=3.0, min=0.35)
#
# 2阶段课程:
#   Phase 1 (0-80K): 2 USVs head-on, 适应慢动态 + 更宽惩罚区
#   Phase 2 (80K-280K): 3 USVs 多场景, 无 auto-eval (单独批量评估)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh57_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh57_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh57.pt"
FRESH55_BEST="/mnt/data/checkpoints/usv_rl/fresh55_checkpoints/fresh55_phase2_step_0077737.pt"

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
  # F1: 增大碰撞/近距离参数
  --collision-distance 0.75
  --collision-penalty -350.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.12
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
  # B1: 慢动态
  --sim-tau-linear 2.0
  --sim-tau-angular 0.8
  # Tau 域随机化: 每个 episode 采样不同的 tau 值
  --dr-tau-linear-low 1.5
  --dr-tau-linear-high 2.5
  --dr-tau-angular-low 0.6
  --dr-tau-angular-high 1.0
  # 训练时 speed scaling
  --speed-scale-distance 3.0
  --speed-scale-min 0.35
)

# ─────────────────────────────────────────────────────────────
# Phase 1: 慢动态适应 + 宽惩罚区 (80K steps, 2 USVs)
# ─────────────────────────────────────────────────────────────
echo "========== Phase 1: Slow Dynamics + Wide Safety Zone (80K, 2 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE1_OUTPUT" \
  --load-weights-from "$FRESH55_BEST" \
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
  --separation-recovery-weight 5.0 \
  --progress-weight 2.6 \
  --conflict-risk-weight 6.0 \
  --conflict-brake-weight 3.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.5 \
  --conflict-escalation-penalty-weight 5.0 \
  --unsafe-close-speed-penalty-weight 8.0 \
  --conflict-overspeed-penalty-weight 6.5 \
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
  --straight-line-omega-penalty-weight 7.0 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 5.0 \
  --omega-flip-saturation-threshold 0.35 \
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
  --proximity-gradient-penalty-weight 3.0 \
  --proximity-gradient-distance 4.0 \
  --speed-distance-coupling-penalty-weight 4.0 \
  --speed-distance-coupling-threshold 3.0 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 15.0 \
  "${COMMON_ARGS[@]}"

PHASE1_CKPT="$PHASE1_OUTPUT"
echo "Phase 1 complete: $PHASE1_CKPT"

# ─────────────────────────────────────────────────────────────
# Phase 2: 多场景碰撞规避 (200K steps, 3 USVs)
# 注意: 不使用 --auto-evaluate-checkpoints (避免 OOM)
# 训练完成后单独运行 fresh57_batch_eval.sh
# ─────────────────────────────────────────────────────────────
echo "========== Phase 2: Multi-scenario Avoidance (200K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$PHASE1_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 200000 \
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
  --separation-recovery-weight 6.0 \
  --progress-weight 2.6 \
  --conflict-risk-weight 7.0 \
  --conflict-brake-weight 3.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.5 \
  --conflict-escalation-penalty-weight 5.5 \
  --unsafe-close-speed-penalty-weight 9.0 \
  --conflict-overspeed-penalty-weight 7.5 \
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
  --straight-line-omega-penalty-weight 7.0 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 5.5 \
  --omega-flip-saturation-threshold 0.35 \
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
  --proximity-gradient-penalty-weight 6.0 \
  --proximity-gradient-distance 4.0 \
  --speed-distance-coupling-penalty-weight 7.0 \
  --speed-distance-coupling-threshold 3.0 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}"

echo "========== fresh57 training complete =========="
echo "Final model: $PHASE2_OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh57_batch_eval.sh"
echo "  2. Check ranking: cat /mnt/data/checkpoints/usv_rl/fresh57_ranking.json"
