#!/bin/bash
# fresh46: 从头训练 —— 修复 bang-bang 角速度震荡
#
# 核心修复:
#   1. raw logits clamp [-3,3] 防止 tanh 饱和（代码层已实现）
#   2. straight-line-omega-penalty-weight=2.0 → 航向对齐时惩罚大 |ω|
#   3. angular-accel-penalty-weight=1.5 → 惩罚角速度突变（Δω）
#   4. 从零开始训练，不继承已饱和的权重
#
# 目标: 900K 步，USV 能走直线 + 保持 COLREGS 避碰
#

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh46.pt \
  --num-agents 3 \
  --total-timesteps 900000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.01 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --squash-actions \
  --min-forward-speed 0.12 \
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
  --near-miss-distance 2.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 5.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -150.0 \
  --team-reward-weight 0.10 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 3.0 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 3.0 \
  --conflict-distance 6.0 \
  --anticipation-distance 5.0 \
  --head-on-guidance-distance 5.0 \
  --head-on-target-starboard-offset 0.50 \
  --head-on-corridor-reward-weight 1.4 \
  --head-on-centerline-penalty-weight 1.6 \
  --head-on-close-penalty-weight 1.2 \
  --head-on-no-turn-penalty-weight 1.0 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 2.0 \
  --angular-accel-penalty-weight 1.5 \
  --straight-line-omega-penalty-weight 2.0 \
  --pure-cruise-reward-weight 2.0 \
  --pure-idle-penalty-weight 3.0 \
  --pure-turn-penalty-weight 1.0 \
  --pure-spin-penalty-weight 3.0 \
  --path-deviation-penalty-weight 1.0 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.0 \
  --stop-go-penalty-weight 1.5 \
  --goal-proximity-reward-weight 1.5 \
  --crossing-starboard-turn-reward-weight 2.0 \
  --crossing-forward-reward-weight 1.2 \
  --overtaking-starboard-turn-reward-weight 1.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.0 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 2.5 \
  --colregs-port-turn-penalty-weight 1.5 \
  --no-progress-timeout 20.0 \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
