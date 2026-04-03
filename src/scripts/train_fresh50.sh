#!/bin/bash
# fresh50: 从 fresh49 的最佳 overtaking 检查点热启动，修复 head_on / crossing 安全回退
#
# 数据分析总结:
#   fresh48_step_55262: 三场景零碰撞 (HO=0.00, CR=0.00, OT=0.00), delta~4.2, completion=0
#   fresh49 演进:
#     step_5228:  coll=0.17 (CR=0.50)，安全但无完成
#     step_100K:  HO碰撞出现(0.50)，开始漂移
#     step_300K:  HO完全崩溃(1.00)，但OT首次完成(0.167)
#     step_500K:  HO恢复(0.00)，CR崩溃(1.00)，OT最佳完成(0.333, delta=5.45)
#     step_600K:  混合状态(HO=0.50, CR=0.50, OT=0.00)，OT delta=7.69 但丢失完成
#
# fresh50 核心策略:
#   1. 热启动 fresh49_step_0500447: 最佳 OT 完成点，HO 零碰撞
#   2. 恢复 head_on_target_starboard_offset 到 0.90（fresh48 安全值），fresh49 降到 0.85 导致 HO 不稳定
#   3. 适度提高 head_on close/no_turn penalty，强化 HO 安全约束
#   4. 增强 crossing forward reward 和 starboard turn reward，修复 CR 崩溃
#   5. 降低 LR 和 clip 范围，减少优化器振荡，保持已学策略稳定性
#   6. 保留 OT reward 设置不变，保住已有完成增益
#   7. 增加 goal_proximity_relief 相关参数，让接近目标时冲突约束适度放松

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh50.pt \
  --load-weights-from /mnt/data/checkpoints/usv_rl/fresh49_checkpoints/fresh49_step_0500447.pt \
  --num-agents 3 \
  --total-timesteps 600000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 6e-5 \
  --learning-rate-end 2e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.12 \
  --entropy-coef 0.006 \
  --entropy-coef-end 0.002 \
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
  --conflict-risk-weight 3.6 \
  --conflict-brake-weight 1.6 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 2.2 \
  --conflict-escalation-penalty-weight 2.8 \
  --unsafe-close-speed-penalty-weight 5.0 \
  --desired-conflict-speed 0.12 \
  --conflict-distance 6.5 \
  --anticipation-distance 6.5 \
  --head-on-guidance-distance 6.5 \
  --head-on-target-starboard-offset 0.90 \
  --head-on-corridor-reward-weight 2.4 \
  --head-on-centerline-penalty-weight 2.5 \
  --head-on-turn-reward-weight 1.3 \
  --head-on-forward-reward-weight 1.3 \
  --head-on-speed-drop-penalty-weight 1.8 \
  --head-on-close-penalty-weight 2.6 \
  --head-on-no-turn-penalty-weight 2.3 \
  --head-on-phase-gate-strength 0.65 \
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
  --crossing-starboard-turn-reward-weight 3.8 \
  --crossing-forward-reward-weight 1.2 \
  --overtaking-starboard-turn-reward-weight 2.4 \
  --overtaking-forward-reward-weight 1.15 \
  --overtaking-corridor-reward-weight 2.8 \
  --overtaking-centerline-penalty-weight 2.4 \
  --overtaking-close-penalty-weight 3.3 \
  --colregs-port-turn-penalty-weight 3.0 \
  --no-progress-timeout 24.0 \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 150 \
  --log-interval-updates 1
