#!/bin/bash
# fresh53: 从 fresh52 最佳检查点 (5K, score=0.689) 热启动
#
# 核心改进: 分场景优势归一化 (Per-Scenario Advantage Normalization)
#
# 灾难性遗忘根因分析 (fresh50/51/52 均一致):
#   全局优势归一化 (adv - mean) / std 导致 OT 场景奖励信号较强,
#   其优势值幅度 >> HO/CR,在全局归一化后 OT 样本获得更大梯度权重,
#   策略更新系统性偏向 OT 行为 → HO/CR 碰撞回避能力退化。
#
# fresh53 解决方案:
#   --per-scenario-advantage-norm: 在每个场景组内独立归一化优势到 N(0,1),
#   使 HO/CR/OT 三个场景对梯度贡献均等,防止任一场景主导更新。
#   这是多任务 RL 中标准的抗任务干扰 (task interference) 技术。
#
# 其他保持 fresh52 配置:
#   - 分离恢复奖励 (separation_recovery_weight=4.0)
#   - 冲突超速惩罚 (conflict_overspeed_penalty_weight=5.0)
#   - 低学习率 + 低 clip (抗遗忘)
#   训练 400K 步 (因遗忘预期被缓解,可以更长训练探索性能上限)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh53.pt \
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
