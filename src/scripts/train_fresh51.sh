#!/bin/bash
# fresh51: 从 fresh50 最佳安全检查点 (100K) 热启动，核心目标：防止灾难性遗忘
#
# fresh50 数据分析:
#   100K:  最佳综合安全模型 (12ep: HO_coll=0.25, CR_coll=0.50, OT_coll=0.25, score=0.532)
#   300K+: 策略开始专精 OT，HO/CR 安全性逐步崩溃
#   600K:  HO=100%碰撞, CR=100%碰撞, OT=100%成功 → 灾难性遗忘
#
# 核心问题诊断:
#   - OT 奖励信号比 HO/CR 更容易优化，导致梯度被 OT 主导
#   - 等概率轮换三场景，但 OT 的 reward scale 实际上更大
#   - clip_range=0.12 + lr=6e-5 允许的策略步幅太大，迅速覆盖 HO/CR 记忆
#
# fresh51 核心策略:
#   1. 热启动 fresh50_step_0100129: 最佳安全点 (HO/CR/OT 均有基本能力)
#   2. 大幅降低 LR (4e-5→1.5e-5) 和 clip_range (0.12→0.08)，约束策略步幅
#   3. 显著增强 HO/CR 碰撞惩罚和正向引导权重，让 HO/CR 信号压过 OT
#   4. 提高全局碰撞惩罚 (-150→-200)，任何碰撞都极重惩罚
#   5. 适度降低 OT 特有奖励权重，减少 OT 梯度主导
#   6. 缩短训练到 400K 步 (fresh50 在 300K 后开始退化)
#   7. 检查点间隔缩至 2500 步，方便及早发现退化

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh51.pt \
  --load-weights-from /mnt/data/checkpoints/usv_rl/fresh50_checkpoints/fresh50_step_0100129.pt \
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
  --progress-weight 3.1 \
  --goal-bonus 36.0 \
  --conflict-risk-weight 4.5 \
  --conflict-brake-weight 2.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 2.8 \
  --conflict-escalation-penalty-weight 3.5 \
  --unsafe-close-speed-penalty-weight 6.5 \
  --desired-conflict-speed 0.12 \
  --conflict-distance 7.0 \
  --anticipation-distance 7.0 \
  --head-on-guidance-distance 7.0 \
  --head-on-target-starboard-offset 0.90 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 2.2 \
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
  --pure-idle-penalty-weight 3.0 \
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
  --log-interval-updates 1
