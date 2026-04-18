#!/bin/bash
# fresh61: 有符号CTE + 反蛇行 + 反纠缠强化
#
# fresh60 SITL 诊断 (2026-04-12, ~/usv_logs/ 3-USV, fresh60_step_0190051):
#   P1 — 蛇行: omega饱和(|ω|>0.35)占28%, bang-bang翻转, RL Δω方向正确率仅33.9%
#   P2 — CTE无效: abs()丢失方向信息, normalizer std=4.43导致信号极弱(1m=0.226)
#   P3 — 避让犹豫: USV01-02 5m内占73.7%, 3m内占55.4%, omega方向对立
#   P4 — 纠缠拖曳: USV01-02最长247s轨道锁定, 速度从0.36降到0.18 m/s
#
# 根因分析:
#   P1 — angular_accel_limit过高(1.7), authority_power过低(1.3), omega可快速饱和
#        straight_line_omega/flip惩罚虽已提高但仍不足以抑制对齐方向的大omega
#   P2 — _compute_route_cross_track_error用abs()丢失偏离方向
#        normalizer std=4.43使CTE信号被稀释3-4倍
#        策略知道偏航但不知道偏向哪侧->纠正方向随机
#   P3 — conflict_escalation(6)/resolution(4)奖励不对称, proximity_gradient(14)斥力不够
#   P4 — entanglement惩罚sqrt增长慢, grace_steps=15仍留3s缓冲, 低速无额外惩罚
#        训练episode_timeout=45s策略从未见过超长纠缠
#
# fresh61 修复:
#   F1 — 有符号CTE: 去abs(), clip[-3,3] → normalizer std ~1.2 (信号增强3.6x)
#         策略可学习 CTE>0→右转, CTE<0→左转
#         奖励中path_deviation仍用abs(CTE)确保两侧对称惩罚
#   F2 — 蛇行抑制:
#         angular_accel_limit: 1.7 → 1.2 (omega上升慢3步→5步到饱和)
#         angular_decel_limit: 2.3 → 2.8 (omega衰减更快)
#         angular_authority_power: 1.3 → 1.8 (小heading_error更强压制omega)
#         heading_omega_reference: 0.75 → 0.65 (authority更早到达上限)
#         straight_line_omega: 14→20, saturated_omega_flip: 15→25
#         omega_flip_threshold: 0.30→0.20, angular_accel_penalty: 5→10
#   F3 — 避让决断:
#         separation_recovery: 15→25 (更强分离激励)
#         proximity_gradient: 14→22, distance 6→7 (更强斥力场)
#         conflict_escalation: 6→10, resolution: 4→7 (更强风险梯度)
#   F4 — 反纠缠:
#         entanglement_penalty: 8→15 (接近翻倍)
#         grace_steps: 15→8 (1.6s即触发)
#         distance: 4→5 (更大检测半径)
#         惩罚增长: sqrt→linear (5步即满罚, 比sqrt快3-4x)
#         衰减: -5→-2 (分离后缓慢遗忘)
#         新: 纠缠低速惩罚 (weight=8) 对抗轨道锁定时的速度骤降
#         episode_timeout: 45→55s, no_progress: 24→30s (暴露长纠缠)
#   F5 — 路径追踪微调:
#         path_deviation: 8→10, tolerance 0.15→0.20 (signal更强不需零容忍)
#         heading_convergence: 8→10
#
# 训练策略:
#   从 fresh60.pt 热启动 (已有38维obs, CTE符号变化靠normalizer自适应)
#   500K steps, LR略低(2e-5→5e-6), entropy略高(0.008→0.003)
#   相比fresh60多100K步适应signed CTE + 更强smoothness

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh61_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh61.pt"
LOAD_FROM="/mnt/data/checkpoints/usv_rl/fresh60.pt"

mkdir -p "$CKPT_DIR"

COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 3
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
  --clip-range 0.08
  --max-grad-norm 0.5
  --device auto
  --hidden-size 256 --hidden-size 256
  --max-agents 5
  --squash-actions
  --min-forward-speed 0.08
  --angular-delta-limit 0.40
  --neighbor-attention
  --attention-embed-dim 32
  --attention-num-heads 1
  --normalize-observations
  --per-scenario-advantage-norm
  --scenario-balanced-loss
  --domain-randomization
  --dr-position-noise-std 0.10
  --dr-heading-noise-std 0.02
  --dr-velocity-noise-ratio 0.03
  --dr-current-speed-max 0.04
  --dr-velocity-exec-noise 0.05
  --cruise-speed 0.36
  --max-angular-velocity 0.40
  --heading-omega-deadband 0.05
  --heading-omega-reference 0.65
  --angular-authority-power 1.8
  --angular-accel-limit 1.2
  --angular-decel-limit 2.8
  --conflict-turn-relief 0.50
  --collision-distance 0.75
  --collision-penalty -350.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.15
  --conflict-distance 8.0
  --anticipation-distance 8.0
  --head-on-guidance-distance 8.0
  --head-on-target-starboard-offset 0.90
  --heading-error-weight 2.5
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 30.0
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
  --sim-tau-linear 1.5
  --sim-tau-angular 0.35
  --dr-tau-linear-low 1.0
  --dr-tau-linear-high 2.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --speed-scale-distance 5.0
  --speed-scale-min 0.15
)

echo "========== fresh61: Signed CTE + Anti-Snake + Anti-Entanglement (500K, 3 USVs) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$LOAD_FROM" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 500000 \
  --learning-rate 2.0e-5 \
  --learning-rate-end 5e-6 \
  --entropy-coef 0.008 \
  --entropy-coef-end 0.003 \
  --episode-timeout 55.0 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 5.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 18.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 25.0 \
  --entanglement-penalty-weight 15.0 \
  --entanglement-distance 5.0 \
  --entanglement-grace-steps 8 \
  --entanglement-low-speed-penalty-weight 8.0 \
  --progress-weight 2.6 \
  --conflict-risk-weight 7.0 \
  --conflict-brake-weight 3.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 7.0 \
  --conflict-escalation-penalty-weight 10.0 \
  --unsafe-close-speed-penalty-weight 12.0 \
  --conflict-overspeed-penalty-weight 12.0 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 5.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 7.0 \
  --angular-accel-penalty-weight 10.0 \
  --straight-line-omega-penalty-weight 20.0 \
  --saturated-omega-flip-penalty-weight 25.0 \
  --forward-speed-change-penalty-weight 8.0 \
  --omega-flip-saturation-threshold 0.20 \
  --straight-line-omega-conflict-floor 0.05 \
  --pure-cruise-reward-weight 5.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 14.0 \
  --path-deviation-penalty-weight 10.0 \
  --path-deviation-tolerance 0.20 \
  --path-deviation-conflict-scale 0.9 \
  --deadlock-penalty-weight 8.0 \
  --stop-go-penalty-weight 3.0 \
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
  --proximity-gradient-penalty-weight 22.0 \
  --proximity-gradient-distance 7.0 \
  --speed-distance-coupling-penalty-weight 12.0 \
  --speed-distance-coupling-threshold 5.0 \
  --heading-convergence-reward-weight 10.0 \
  --heading-convergence-threshold-deg 45.0 \
  "${COMMON_ARGS[@]}"

echo "========== fresh61 training complete =========="
echo "Final model: $OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh61_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh61_sitl_gate.sh"
