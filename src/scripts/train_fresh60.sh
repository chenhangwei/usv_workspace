#!/bin/bash
# fresh60: 向RL策略添加cross_track_error感知 + 路径追踪强化
#
# fresh59 SITL 诊断 (2026-04-11, ~/usv_logs/ 3-USV, fresh59_step_0102051):
#   P1 — 路径偏离: CTE mean 0.42-0.56m, max 3.5m, >1m 占 5-21%
#   P2 — 避让无效: omega饱和但碰撞 (USV01-02 min=0.369m, USV02-03 min=0.241m)
#   P3 — 纠缠未解: USV02-03 49% 时间 <3m, 最长连续 247s
#   P4 — 无邻船拐弯: USV02/03 |ω|>0.1 占 62-69% (neighbor>8m)
#
# 根因分析:
#   结构缺陷 — cross_track_error 不在RL观测向量中 (只在reward中使用)
#   策略无法感知自身偏离路线的程度, 无法主动回归
#   heading_error 是目标方向, 不是路线方向
#   path_deviation_tolerance=0.8m 太宽松, weight=0.95 太弱
#   straight_line_omega_conflict_floor=0.25 抑制了无冲突时的直线维持
#
# fresh60 修复:
#   F1 — 观测向量添加 cross_track_error (ego_dim 10→11, obs_dim 37→38)
#         agent 可直接感知偏离路线程度, Q投影自动纳入CTE
#         checkpoint 权重迁移: Q_proj 零填充, MLP0 零填充
#   F2 — 大幅收紧路径偏离惩罚:
#         tolerance: 0.8 → 0.15m (几乎零容忍)
#         weight: 0.95 → 8.0 (8倍强化)
#         conflict_scale: 保持 0.9 (冲突时允许偏离)
#   F3 — 无邻船直线行驶强化:
#         straight_line_omega_conflict_floor: 0.25 → 0.05
#         pure_cruise_reward_weight: 2.3 → 5.0
#         pure_spin_penalty_weight: 8.0 → 14.0
#         straight_line_omega_penalty_weight: 8.0 → 14.0
#   F4 — 纠缠惩罚加码:
#         entanglement_penalty_weight: 3.0 → 8.0
#         entanglement_distance: 3.0 → 4.0
#         entanglement_grace_steps: 30 → 15
#   F5 — 安全裕度微调:
#         proximity_gradient: 10 → 14
#         near_miss_weight: 14 → 18
#         heading_convergence_weight: 5.0 → 8.0
#         heading_convergence_threshold: 30 → 45°
#
# 训练策略:
#   从 fresh58_phase1.pt 热启动 (干净的 2-USV 安全基线)
#   obs_dim 迁移: 37→38 (注意力权重自动零填充)
#   400K steps, 3 USVs, 多场景
#   相比 fresh59 增加 100K 步, 因为策略需要学习使用新的CTE观测

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh60_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh60.pt"
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
  --heading-error-weight 2.5
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

echo "========== fresh60: CTE Observation + Path Tracking (400K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$LOAD_FROM" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 400000 \
  --learning-rate 3.0e-5 \
  --learning-rate-end 8e-6 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.002 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 5.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 18.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 15.0 \
  --entanglement-penalty-weight 8.0 \
  --entanglement-distance 4.0 \
  --entanglement-grace-steps 15 \
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
  --straight-line-omega-penalty-weight 14.0 \
  --saturated-omega-flip-penalty-weight 15.0 \
  --forward-speed-change-penalty-weight 6.0 \
  --omega-flip-saturation-threshold 0.30 \
  --straight-line-omega-conflict-floor 0.05 \
  --pure-cruise-reward-weight 5.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 14.0 \
  --path-deviation-penalty-weight 8.0 \
  --path-deviation-tolerance 0.15 \
  --path-deviation-conflict-scale 0.9 \
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
  --proximity-gradient-penalty-weight 14.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 12.0 \
  --speed-distance-coupling-threshold 5.0 \
  --heading-convergence-reward-weight 8.0 \
  --heading-convergence-threshold-deg 45.0 \
  "${COMMON_ARGS[@]}"

echo "========== fresh60 training complete =========="
echo "Final model: $OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh60_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh60_sitl_gate.sh"
