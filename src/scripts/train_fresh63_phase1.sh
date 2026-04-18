#!/bin/bash
# fresh63 Phase 1: 纯路径跟踪 (solo_navigation)
#
# 课程学习第一阶段 — 单USV无邻船，只学直线航行 + 路径跟踪
# 目标: CTE < 0.5m, HE < 15°, 平稳巡航
#
# 奖励策略:
#   ✓ progress_weight         — 前向推进
#   ✓ heading_error_weight    — 航向对准
#   ✓ path_deviation          — CTE偏离惩罚
#   ✓ straight_line_omega     — 抑制蛇行
#   ✓ pure_cruise/idle/turn   — 速度+转弯平滑性
#   ✓ action_smoothness       — 动作平滑
#   ✓ heading_convergence     — 航向收敛奖励
#   ✗ 所有避让/冲突/COLREGs奖励 → 权重=0
#
# 从零开始训练, 150K steps, 3 agents (solo场景只激活1个,其余spectator)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh63_phase1_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh63_phase1.pt"

mkdir -p "$CKPT_DIR"

COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 4
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
  --clip-range 0.15
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
  --sim-tau-linear 1.5
  --sim-tau-angular 0.35
  --dr-tau-linear-low 1.0
  --dr-tau-linear-high 2.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
)

echo "========== fresh63 Phase 1: Solo Navigation — Pure Path Tracking (150K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 150000 \
  --learning-rate 3.0e-4 \
  --learning-rate-end 5.0e-5 \
  --entropy-coef 0.02 \
  --entropy-coef-end 0.008 \
  --episode-timeout 45.0 \
  --no-progress-timeout 15.0 \
  --scenario solo_navigation \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.20 \
  --scenario-goal-position-std 0.3 \
  --cte-clip-range 5.0 \
  --progress-weight 4.0 \
  --heading-error-weight 1.5 \
  --path-deviation-penalty-weight 8.0 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-conflict-scale 0.0 \
  --straight-line-omega-penalty-weight 8.0 \
  --straight-line-omega-cte-gate 0.5 \
  --straight-line-omega-conflict-floor 1.0 \
  --action-smoothness-weight 5.0 \
  --angular-accel-penalty-weight 6.0 \
  --pure-cruise-reward-weight 4.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.0 \
  --pure-spin-penalty-weight 8.0 \
  --heading-convergence-reward-weight 6.0 \
  --heading-convergence-threshold-deg 15.0 \
  --forward-speed-change-penalty-weight 3.0 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 25.0 \
  --time-penalty 0.03 \
  --stall-penalty -15.0 \
  --collision-penalty 0.0 \
  --near-miss-weight 0.0 \
  --near-miss-distance 0.0 \
  --conflict-risk-weight 0.0 \
  --conflict-brake-weight 0.0 \
  --conflict-progress-scale 1.0 \
  --conflict-resolution-reward-weight 0.0 \
  --conflict-escalation-penalty-weight 0.0 \
  --unsafe-close-speed-penalty-weight 0.0 \
  --conflict-overspeed-penalty-weight 0.0 \
  --proximity-gradient-penalty-weight 0.0 \
  --speed-distance-coupling-penalty-weight 0.0 \
  --separation-recovery-weight 0.0 \
  --entanglement-penalty-weight 0.0 \
  --entanglement-low-speed-penalty-weight 0.0 \
  --head-on-corridor-reward-weight 0.0 \
  --head-on-centerline-penalty-weight 0.0 \
  --head-on-turn-reward-weight 0.0 \
  --head-on-forward-reward-weight 0.0 \
  --head-on-speed-drop-penalty-weight 0.0 \
  --head-on-close-penalty-weight 0.0 \
  --head-on-no-turn-penalty-weight 0.0 \
  --head-on-phase-gate-strength 0.0 \
  --crossing-starboard-turn-reward-weight 0.0 \
  --crossing-forward-reward-weight 0.0 \
  --overtaking-starboard-turn-reward-weight 0.0 \
  --overtaking-forward-reward-weight 0.0 \
  --overtaking-corridor-reward-weight 0.0 \
  --overtaking-centerline-penalty-weight 0.0 \
  --overtaking-close-penalty-weight 0.0 \
  --colregs-port-turn-penalty-weight 0.0 \
  --avoidance-turn-reward-weight 0.0 \
  --team-reward-weight 0.0 \
  --team-progress-weight 0.0 \
  --team-goal-proximity-weight 0.0 \
  --coordination-reward-weight 0.0 \
  --team-completion-bonus 0.0 \
  --deadlock-penalty-weight 0.0 \
  --stop-go-penalty-weight 2.0 \
  --goal-proximity-reward-weight 2.0 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.0 \
  --desired-conflict-speed 0.15 \
  "${COMMON_ARGS[@]}"

echo "========== Phase 1 complete =========="
echo "Model: $OUTPUT"
echo ""
echo "Next: bash scripts/train_fresh63_phase2.sh"
