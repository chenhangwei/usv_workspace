#!/bin/bash
# fresh54: 从 fresh52 最佳检查点 (5K, score=0.689) 热启动
#
# 核心改进: 场景条件化 + 场景均衡损失 (Scenario Conditioning + Balanced Loss)
#
# 根因深层分析 (fresh50-53 均遗忘):
#   1. 策略场景盲 (scenario-blind): 34维观测中无场景类型信息,
#      256x256 MLP 必须同时从neighbor几何隐式分类encounter类型
#      并选择正确行为 — 对小网络极其困难。
#   2. 样本数不均: OT episode ~175步 × 3 agents = 525 agent-steps,
#      CR ~67 × 3 = 201, OT 产生 ~2.6× 更多样本 — 梯度被 OT 主导。
#   3. 梯度干扰: 共享权重被矛盾场景梯度更新,
#      OT 要"直行巡航", CR 需"大角度右转" — 直接冲突。
#
# fresh54 双管齐下方案:
#   A) 场景 one-hot 条件化 (encounter_type):
#      观测尾部追加 3 维 one-hot [HO, CR, OT], Actor 34→37, Critic 209→227。
#      网络可学习 per-scenario 条件行为,大幅降低场景间干扰。
#      旧权重零填充迁移,初始行为等价于旧检查点。
#
#   B) 场景均衡损失 (--scenario-balanced-loss):
#      PPO loss 中每个样本按 w_i = N/(K*n_k) 加权,
#      确保各场景对梯度贡献均等,不论 episode 长度差异。
#
#   C) 保留 per-scenario 优势归一化 (fresh53 引入):
#      与均衡损失互补 — 归一化消除优势尺度差异,均衡损失消除样本数差异。
#
# 检查点权重迁移:
#   旧 34 维 → 新 37 维, Actor 首层零填充 [256,34]->[256,37],
#   Critic 首层零填充 [256,209]->[256,227] (6 个 obs 块各扩 3 维 + 5 fleet stats)。
#   Normalizer: mean 补 0, var 补 1。初始行为完全保持。

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh54.pt \
  --load-weights-from /mnt/data/checkpoints/usv_rl/fresh52_checkpoints/fresh52_step_0005195.pt \
  --num-agents 3 \
  --total-timesteps 400000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 4e-5 \
  --learning-rate-end 1.5e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.08 \
  --entropy-coef 0.008 \
  --entropy-coef-end 0.003 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --squash-actions \
  --min-forward-speed 0.08 \
  --angular-delta-limit 0.40 \
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 0.75 \
  --angular-authority-power 1.3 \
  --angular-accel-limit 1.7 \
  --angular-decel-limit 2.3 \
  --conflict-turn-relief 0.50 \
  --near-miss-distance 2.5 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 7.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -200.0 \
  --team-reward-weight 0.10 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --team-completion-bonus 30.0 \
  --separation-recovery-weight 4.0 \
  --progress-weight 3.1 \
  --goal-bonus 36.0 \
  --conflict-risk-weight 4.5 \
  --conflict-brake-weight 2.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 2.8 \
  --conflict-escalation-penalty-weight 3.5 \
  --unsafe-close-speed-penalty-weight 6.5 \
  --conflict-overspeed-penalty-weight 5.0 \
  --desired-conflict-speed 0.12 \
  --conflict-distance 7.0 \
  --anticipation-distance 7.0 \
  --head-on-guidance-distance 7.0 \
  --head-on-target-starboard-offset 0.90 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 3.5 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 3.8 \
  --angular-accel-penalty-weight 3.0 \
  --straight-line-omega-penalty-weight 4.5 \
  --saturated-omega-flip-penalty-weight 4.0 \
  --forward-speed-change-penalty-weight 3.0 \
  --omega-flip-saturation-threshold 0.45 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.4 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.95 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
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
  --colregs-port-turn-penalty-weight 3.5 \
  --no-progress-timeout 24.0 \
  --checkpoint-interval 2500 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 150 \
