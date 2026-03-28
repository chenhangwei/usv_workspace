#!/bin/bash
# unified45: 全局统距 + 提前5m柔和避让 (Smooth Early Avoidance)
#
# 用户核心需求: "提前5m就需要慢慢避让，而不是猛然擦肩"
#
# 之前 headon42 为什么会"猛然擦肩"？
#   head-on-guidance-distance=2.5 → 船到了2.5m才开始转向，太晚了
#   head-on-target-starboard-offset=0.15 → 偏转量极小，像是在"闪避"
#   head-on-near-miss-distance=0.82 → 贴脸0.82m才扣分，鼓励了极限操作
#
# 本次修正策略（柔和渐进式避让）：
#   near_miss_distance: 1.5 (全局统一，无双标)
#   head_on_near_miss_distance: 0.0 (禁用，走全局)
#   anticipation_distance: 5.0 (从5m就开始"意识到"有冲突)
#   head_on_guidance_distance: 5.0 (从5m就开始引导转向)
#   head_on_target_starboard_offset: 0.50 (温和的横向偏移目标)
#   conflict_distance: 6.0 (保持不变，冲突探测依然在6m)
#
# 效果预期:
#   船在距离对方5m时就开始平缓地向右偏转,走一个优雅的弧线错开,
#   而不是等到最后1-2m才猛打方向盘。最终交会间距约1.5m左右。

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/unified45.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon41_checkpoints/headon41_step_0665280.pt \
  --num-agents 3 \
  --total-timesteps 900000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 3e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.03 \
  --max-grad-norm 0.3 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario two_usv_head_on --scenario three_usv_crossing --scenario three_usv_overtaking \
  \
  --squash-actions \
  --min-forward-speed 0.12 \
  --normalize-observations \
  \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  \
  --near-miss-distance 1.5 \
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
  --action-smoothness-weight 1.5 \
  --angular-accel-penalty-weight 1.0 \
  --pure-cruise-reward-weight 2.0 \
  --pure-idle-penalty-weight 3.0 \
  --pure-turn-penalty-weight 0.5 \
  --pure-spin-penalty-weight 3.0 \
  --path-deviation-penalty-weight 1.0 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.0 \
  --stop-go-penalty-weight 1.5 \
  --goal-proximity-reward-weight 1.5 \
  \
  --crossing-starboard-turn-reward-weight 2.0 \
  --crossing-forward-reward-weight 1.2 \
  --overtaking-starboard-turn-reward-weight 1.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.0 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 2.5 \
  --colregs-port-turn-penalty-weight 1.5 \
  \
  --no-progress-timeout 20.0 \
  \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
