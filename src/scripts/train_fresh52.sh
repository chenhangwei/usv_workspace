#!/bin/bash
# fresh52: 从 fresh51 最佳检查点 (200K) 热启动，核心修复：分离恢复奖励 + 冲突超速惩罚
#
# fresh51 数据分析:
#   200K:  最佳综合模型 (score=0.604, safety=0.775, HO_coll=50%, CR_coll=0%, OT_coll=0%)
#   400K:  灾难性遗忘再次出现 (score=0.387, safety=0.425, CR_coll=100%)
#
# SITL 诊断发现的两个根本问题:
#   1. 无分离恢复奖励: team_reward_weight 只惩罚靠近，从不奖励分离
#      → USV 进入近距离后无脱身动机，形成 "钳制" 死锁 (90s orbital lock)
#   2. 速度控制信号矛盾: conflict_brake_weight 只罚减速不足，不罚超速
#      + head_on_speed_drop_penalty + pure_idle_penalty 均惩罚减速
#      → 策略总是输出最大速度 0.359，冲突中无法主动减速
#
# fresh52 核心修复:
#   1. 新增 --separation-recovery-weight 4.0: 在 near-miss 带内，pair_min 增大时给正奖励
#      鼓励主动脱离接触，打破钳制死锁
#   2. 新增 --conflict-overspeed-penalty-weight 5.0: conflict_level > 0.25 时，
#      超过 desired_conflict_speed 的速度被惩罚，修复只罚减速不罚超速的不对称性
#   3. 保持 fresh51 的抗遗忘措施 (低LR、低clip、高碰撞惩罚)
#   4. 缩短训练到 300K 步 (fresh50/51 均在 200-300K 后退化)
#   5. 适度降低 pure_idle_penalty (3.0→1.5) 和 head_on_speed_drop_penalty (2.2→1.0)
#      减轻对减速的惩罚，配合 conflict_overspeed_penalty 形成双向约束

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 -m usv_rl.train_mappo_policy \
  --output /mnt/data/checkpoints/usv_rl/fresh52.pt \
  --load-weights-from /mnt/data/checkpoints/usv_rl/fresh51_checkpoints/fresh51_step_0200421.pt \
  --num-agents 3 \
  --total-timesteps 300000 \
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
  --log-interval-updates 1
