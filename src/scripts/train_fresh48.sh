#!/bin/bash
# fresh48: 基于 active-agent 修复后的环境，重新平衡“先避碰成形，再恢复推进”
#
# 核心方向:
#   1. head-on 引入 phase gate，先建立右舷通道，再释放前进奖励
#   2. crossing / overtaking 降低冲突期前冲，强化右转与让路走廊 shaping
#   3. 回收 fresh47 过重的全局压制，改用更可靠的 active-agent team completion / proximity 信号
#   4. 角速度按航向误差与邻船冲突动态收敛：误差大时先快建立转向，误差小时自动减小并缓慢收直

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh48.pt \
  --num-agents 3 \
  --total-timesteps 900000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 2e-4 \
  --learning-rate-end 8e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.012 \
  --entropy-coef-end 0.004 \
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
  --near-miss-distance 2.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 5.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -150.0 \
  --team-reward-weight 0.10 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.25 \
  --coordination-reward-weight 0.30 \
  --team-completion-bonus 22.0 \
  --progress-weight 3.1 \
  --goal-bonus 32.0 \
  --conflict-risk-weight 3.4 \
  --conflict-brake-weight 1.5 \
  --conflict-progress-scale 0.55 \
  --conflict-resolution-reward-weight 2.0 \
  --conflict-escalation-penalty-weight 2.6 \
  --unsafe-close-speed-penalty-weight 4.6 \
  --desired-conflict-speed 0.12 \
  --conflict-distance 6.5 \
  --anticipation-distance 6.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.90 \
  --head-on-corridor-reward-weight 2.4 \
  --head-on-centerline-penalty-weight 2.3 \
  --head-on-turn-reward-weight 1.2 \
  --head-on-forward-reward-weight 1.3 \
  --head-on-speed-drop-penalty-weight 1.6 \
  --head-on-close-penalty-weight 2.2 \
  --head-on-no-turn-penalty-weight 2.0 \
  --head-on-phase-gate-strength 0.65 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 3.8 \
  --angular-accel-penalty-weight 3.0 \
  --straight-line-omega-penalty-weight 4.5 \
  --saturated-omega-flip-penalty-weight 4.0 \
  --pure-cruise-reward-weight 2.2 \
  --pure-idle-penalty-weight 3.0 \
  --pure-turn-penalty-weight 2.4 \
  --pure-spin-penalty-weight 5.0 \
  --path-deviation-penalty-weight 0.95 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.0 \
  --stop-go-penalty-weight 2.6 \
  --goal-proximity-reward-weight 2.0 \
  --crossing-starboard-turn-reward-weight 3.4 \
  --crossing-forward-reward-weight 0.8 \
  --overtaking-starboard-turn-reward-weight 2.4 \
  --overtaking-forward-reward-weight 1.1 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.4 \
  --overtaking-close-penalty-weight 3.3 \
  --colregs-port-turn-penalty-weight 2.8 \
  --no-progress-timeout 24.0 \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 130 \
  --log-interval-updates 1