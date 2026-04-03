#!/bin/bash
# fresh47: 在 fresh46 基础上强化“安全优先 + 抑制饱和/摆振 + 提升超车完成率”
#
# 调参目标:
#   1) 降低 head-on / crossing 的碰撞率
#   2) 降低高比例 |omega| 饱和与低频 bang-bang
#   3) 缓解 overtaking 全超时问题，提升任务完成推进
#
# 关键改动（相对 fresh46）:
#   - 更强平滑/角加速度/直线大角速度惩罚
#   - 更强 COLREGS 右转偏好与左转惩罚
#   - 更长冲突预判距离，提前规避
#   - 适度提高前进与目标推进奖励

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh47.pt \
  --num-agents 3 \
  --total-timesteps 900000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --entropy-coef 0.015 \
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
  --progress-weight 3.2 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 3.2 \
  --conflict-brake-weight 1.3 \
  --conflict-progress-scale 0.65 \
  --conflict-resolution-reward-weight 1.8 \
  --conflict-escalation-penalty-weight 2.4 \
  --unsafe-close-speed-penalty-weight 3.4 \
  --conflict-distance 7.0 \
  --anticipation-distance 6.0 \
  --head-on-guidance-distance 6.0 \
  --head-on-target-starboard-offset 0.70 \
  --head-on-corridor-reward-weight 2.0 \
  --head-on-centerline-penalty-weight 2.0 \
  --head-on-close-penalty-weight 1.8 \
  --head-on-no-turn-penalty-weight 1.6 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 3.0 \
  --angular-accel-penalty-weight 2.2 \
  --straight-line-omega-penalty-weight 3.2 \
  --pure-cruise-reward-weight 2.4 \
  --pure-idle-penalty-weight 3.2 \
  --pure-turn-penalty-weight 1.8 \
  --pure-spin-penalty-weight 4.0 \
  --path-deviation-penalty-weight 1.1 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.5 \
  --stop-go-penalty-weight 2.0 \
  --goal-proximity-reward-weight 1.8 \
  --crossing-starboard-turn-reward-weight 2.8 \
  --crossing-forward-reward-weight 1.4 \
  --overtaking-starboard-turn-reward-weight 2.2 \
  --overtaking-forward-reward-weight 1.6 \
  --overtaking-corridor-reward-weight 2.4 \
  --overtaking-centerline-penalty-weight 2.2 \
  --overtaking-close-penalty-weight 3.0 \
  --colregs-port-turn-penalty-weight 2.4 \
  --no-progress-timeout 20.0 \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 120 \
  --log-interval-updates 1
