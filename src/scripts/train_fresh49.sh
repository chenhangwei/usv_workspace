#!/bin/bash
# fresh49: 从 fresh48 的安全 basin 热启动，专门补“安全完成”而不是继续在两类坏解之间摆动
#
# 核心方向:
#   1. 从 fresh48_step_0055262 零碰撞 checkpoint 热启动，优先保住安全性
#   2. 降低 PPO 更新步长与探索强度，避免重新漂回 aggressive head-on / crossing
#   3. 保持动态角速度投影与主要安全约束不变，只加强完成性相关 dense / terminal shaping
#   4. 重点补 crossing 的推进恢复，并轻微放松 head-on 右舷偏移，减少过宽绕行导致的全超时

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh49.pt \
  --load-weights-from /mnt/data/checkpoints/usv_rl/fresh48_checkpoints/fresh48_step_0055262.pt \
  --num-agents 3 \
  --total-timesteps 600000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 1e-4 \
  --learning-rate-end 4e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
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
  --near-miss-distance 2.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 5.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -150.0 \
  --team-reward-weight 0.10 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --team-completion-bonus 30.0 \
  --progress-weight 3.1 \
  --goal-bonus 36.0 \
  --conflict-risk-weight 3.4 \
  --conflict-brake-weight 1.5 \
  --conflict-progress-scale 0.55 \
  --conflict-resolution-reward-weight 2.2 \
  --conflict-escalation-penalty-weight 2.6 \
  --unsafe-close-speed-penalty-weight 4.6 \
  --desired-conflict-speed 0.12 \
  --conflict-distance 6.5 \
  --anticipation-distance 6.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.85 \
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
  --crossing-starboard-turn-reward-weight 3.4 \
  --crossing-forward-reward-weight 1.0 \
  --overtaking-starboard-turn-reward-weight 2.4 \
  --overtaking-forward-reward-weight 1.15 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.4 \
  --overtaking-close-penalty-weight 3.3 \
  --colregs-port-turn-penalty-weight 2.8 \
  --no-progress-timeout 24.0 \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 140 \
  --log-interval-updates 1