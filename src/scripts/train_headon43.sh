#!/bin/bash
# headon43: 温水煮青蛙策略 (Moderate head_on optimization)
#
# 回退原因: headon42 (0.82m) 虽然成功把 head_on 压到了 1.06m，但导致了模型权重的泛化灾难
# (使得模型认为 crossing 也可以贴到 1m 内再躲，导致 crossing 100%相撞)。
# 
# 本次优化方向寻找一个中庸/稳定的平滑界限：
# - 恢复到一个健康的 Checkpoint (h41-665280)
# - --head-on-near-miss-distance 放宽到 1.40 
# - --head-on-target-starboard-offset 设为 0.35 
# - --head-on-guidance-distance 设为 3.5

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon43.pt

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
  --near-miss-distance 2.0 \
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
  --anticipation-distance 3.0 \
  --head-on-near-miss-distance 1.40 \
  --head-on-guidance-distance 3.5 \
  --head-on-target-starboard-offset 0.35 \
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
