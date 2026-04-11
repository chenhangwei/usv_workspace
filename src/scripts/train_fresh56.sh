#!/bin/bash
# fresh56: Sim-Real Gap 修复训练 (2-Phase Curriculum)
#
# 从 fresh55_phase2 最佳检查点热启动 (step 77737, 0% collision, 93.4% progress)
#
# SITL 诊断 (fresh55 部署后):
#   P1: 速度cmd-actual不匹配: RL发vx=0.08, 平台仍0.3-0.5m/s, settle_time 4.4s
#   P2: omega bang-bang: 31%饱和, 0.216 rad/s 平均跟踪误差
#   P3: 5次碰撞 (min 0.332m), 38s total
#   P4: 轨迹环路 (57/19/74 per USV), 14-16% 朝向错误
#   P5: A2 speed scaling 几乎无效 (3-5% triggered)
#   P6: 观测归一化已启用但训练动态不匹配
#
# 方案B 改进:
#   B1 — 增大仿真动力学延迟 (sim_tau_linear: 0.45→2.0, sim_tau_angular: 0.25→0.8)
#        让RL在训练时体验真实平台的惯性延迟
#   B4 — 强制 actor_log_std=-1.0 (σ=0.37) 降低探索噪声
#        fresh55 的 log_std 从未收敛 (0.97-1.01), tanh饱和导致bang-bang
#   B5 — 增强航向对准奖励和反旋转惩罚
#        heading_convergence: 1.5→3.0, pure_spin: 5.0→8.0
#        saturated_omega_flip: 6.5→9.0
#
# 2阶段课程 (跳过Phase3, fresh55 Phase3产生退化):
#   Phase 1 (0-120K): 2 USVs, 适应新慢动态, 宽松间距
#   Phase 2 (120K-350K): 3 USVs, 多场景, auto-eval

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh56_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh56_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh56.pt"
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
  --heading-error-weight 2.0
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 24.0
  --checkpoint-interval 2500
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
  # B1: 增大仿真动力学延迟, 匹配真实平台惯性
  --sim-tau-linear 2.0
  --sim-tau-angular 0.8
  # 训练时也启用 speed scaling, 与 SITL 部署保持一致
  --speed-scale-distance 3.0
  --speed-scale-min 0.35
)

# ─────────────────────────────────────────────────────────────
# Phase 1: 适应慢动态 + 航向对准 (120K steps, 2 USVs)
# 目标: 从 fresh55_phase2 最佳检查点热启动,
#        适应 tau_linear=2.0 的慢响应动态,
#        强制 log_std=-1.0 消除 bang-bang 行为
# ─────────────────────────────────────────────────────────────
echo "========== Phase 1: Slow Dynamics Adaptation (120K, 2 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE1_OUTPUT" \
  --load-weights-from "$FRESH55_BEST" \
  --force-actor-log-std -1.0 \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 2 \
  --total-timesteps 120000 \
  --learning-rate 3e-5 \
  --learning-rate-end 2e-5 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.003 \
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
  --proximity-gradient-penalty-weight 2.0 \
  --proximity-gradient-distance 3.0 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 2.5 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 15.0 \
  "${COMMON_ARGS[@]}"

PHASE1_CKPT="$PHASE1_OUTPUT"
echo "Phase 1 complete: $PHASE1_CKPT"

# ─────────────────────────────────────────────────────────────
# Phase 2: 多场景碰撞规避 (230K steps, 3 USVs, auto-eval)
# 目标: 3场景训练, 增强安全奖励,
#        auto-eval 自动选择最优检查点
# ─────────────────────────────────────────────────────────────
echo "========== Phase 2: Multi-scenario Avoidance (230K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$PHASE1_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 230000 \
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
  --near-miss-distance 2.5 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 8.5 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 5.5 \
  --progress-weight 3.1 \
  --conflict-risk-weight 5.5 \
  --conflict-brake-weight 2.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.0 \
  --conflict-escalation-penalty-weight 4.5 \
  --unsafe-close-speed-penalty-weight 8.0 \
  --conflict-overspeed-penalty-weight 6.5 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 4.5 \
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
  --proximity-gradient-penalty-weight 5.0 \
  --proximity-gradient-distance 3.0 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 2.5 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 12.0 \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 180 \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario three_usv_crossing \
  --checkpoint-eval-scenario three_usv_overtaking \
  --checkpoint-ranking-json /mnt/data/checkpoints/usv_rl/fresh56_ranking.json \
  --checkpoint-eval-json-dir /mnt/data/checkpoints/usv_rl/fresh56_eval \
  "${COMMON_ARGS[@]}"

echo "========== fresh56 training complete =========="
echo "Final model: $PHASE2_OUTPUT"
echo "Checkpoint ranking: /mnt/data/checkpoints/usv_rl/fresh56_ranking.json"
