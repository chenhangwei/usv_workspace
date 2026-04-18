#!/bin/bash
# fresh62: CTE门控 + 物理避让奖励 + CTE观测扩展
#
# fresh61 SITL 诊断 (2026-04-13, 3-USV, fresh61_step_0458411):
#   P1 — 无邻船时偏离航线: 安全区(>5m) RL ω方向56%与MPC相反
#         HE [10°-45°]: raw_ω=+0.095 vs RL_ω=-0.003
#         根因: straight_line_omega(20)用cos(HE)门控, 即使CTE大也惩罚转向修正
#   P2 — 大幅偏离(CTE达3.59m): 避让后path_deviation_penalty(-0.55×excess)回归太弱
#         CTE观测clip ±3m限制梯度
#   P3 — 避让方向~50%错误/缠绕: COLREGs右转先验与物理避让冲突
#         邻船在右前方时57%仍右转(转向邻船)
#
# fresh62 修复:
#   F1 — straight_line_omega CTE门控:
#         新参数 --straight-line-omega-cte-gate 0.5
#         |CTE|>0.5m时omega惩罚开始衰减, |CTE|>1.5m(3×0.5)时完全消除
#         → RL可以自由转向修正航线, 仅在航线上时抑制蛇行
#   F2 — 物理避让转向奖励:
#         新参数 --avoidance-turn-reward-weight 6.0
#         在near-miss区域内, 奖励"转离邻船方向"(bearing×ω同号)
#         → 不依赖COLREGs场景标签, 直接从几何关系学避让方向
#   F3 — CTE观测clip扩大:
#         --cte-clip-range 5.0 (从3.0)
#         → 大偏离时仍有梯度信号, normalizer自适应不影响
#   F4 — 降低crossing右转奖励, 升级path_deviation:
#         crossing_starboard_turn: 4.8→2.5 (减少右转偏执)
#         path_deviation_penalty_weight: 10→15 (更强回归)
#         straight_line_omega: 20→12 (给CTE门控后仍足够但不过度)
#   F5 — 降低colregs_port_turn_penalty:
#         3.5→1.5, 允许左转避让(当邻船在右侧时)
#
# 训练策略:
#   从 fresh61.pt 热启动 (obs_dim=38不变, 新奖励通过训练学习)
#   500K steps, 3 USVs

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh62_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh62.pt"
LOAD_FROM="/mnt/data/checkpoints/usv_rl/fresh61.pt"

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
  --colregs-port-turn-penalty-weight 1.5
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

echo "========== fresh62: CTE-gated omega + avoidance turn reward (500K, 3 USVs) =========="
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
  --straight-line-omega-penalty-weight 12.0 \
  --saturated-omega-flip-penalty-weight 25.0 \
  --forward-speed-change-penalty-weight 8.0 \
  --omega-flip-saturation-threshold 0.20 \
  --straight-line-omega-conflict-floor 0.05 \
  --straight-line-omega-cte-gate 0.5 \
  --avoidance-turn-reward-weight 6.0 \
  --cte-clip-range 5.0 \
  --pure-cruise-reward-weight 5.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 14.0 \
  --path-deviation-penalty-weight 15.0 \
  --path-deviation-tolerance 0.20 \
  --path-deviation-conflict-scale 0.9 \
  --deadlock-penalty-weight 8.0 \
  --stop-go-penalty-weight 3.0 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  --crossing-starboard-turn-reward-weight 2.5 \
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

echo "========== fresh62 training complete =========="
echo "Final model: $OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh62_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh62_sitl_gate.sh"
