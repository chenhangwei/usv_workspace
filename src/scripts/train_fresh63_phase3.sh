#!/bin/bash
# fresh63 Phase 3: 全场景 — 路径跟踪 + 全避让
#
# 课程学习第三阶段 — solo + head_on + crossing + overtaking
# 从 Phase 2 权重转移 (--load-weights-from)
#
# 新增场景:
#   ✓ three_usv_crossing    — 交叉避让
#   ✓ three_usv_overtaking  — 追越避让
# 避让策略 (纯物理, 无COLREGs先验):
#   ✓ avoidance_turn + proximity_gradient + separation_recovery
#   ✗ head_on/crossing/overtaking/colregs → 全部=0
# 保留solo_navigation 防遗忘
#
# 200K steps, 从 phase2.pt 权重转移

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh63_phase3_checkpoints"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh63_phase3.pt"
LOAD_FROM="/mnt/data/checkpoints/usv_rl/fresh63_phase2.pt"

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
  --sim-tau-linear 1.5
  --sim-tau-angular 0.35
  --dr-tau-linear-low 1.0
  --dr-tau-linear-high 2.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --speed-scale-distance 5.0
  --speed-scale-min 0.15
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
)

echo "========== fresh63 Phase 3: Full Scenarios — All Avoidance (200K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$LOAD_FROM" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 200000 \
  --learning-rate 5.0e-5 \
  --learning-rate-end 1.0e-5 \
  --entropy-coef 0.008 \
  --entropy-coef-end 0.003 \
  --episode-timeout 55.0 \
  --no-progress-timeout 25.0 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario two_usv_random_encounter \
  --scenario three_usv_random_encounter \
  --scenario-spawn-position-std 1.0 \
  --scenario-spawn-heading-std 0.40 \
  --scenario-goal-position-std 1.0 \
  --encounter-type-dropout 0.5 \
  --cte-clip-range 5.0 \
  --progress-weight 3.0 \
  --heading-error-weight 1.8 \
  --heading-relief-factor 0.75 \
  --path-deviation-penalty-weight 12.0 \
  --path-deviation-tolerance 0.20 \
  --path-deviation-conflict-scale 3.0 \
  --straight-line-omega-penalty-weight 12.0 \
  --straight-line-omega-cte-gate 0.5 \
  --straight-line-omega-conflict-floor 0.02 \
  --action-smoothness-weight 7.0 \
  --angular-accel-penalty-weight 10.0 \
  --pure-cruise-reward-weight 5.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 12.0 \
  --heading-convergence-reward-weight 4.0 \
  --heading-convergence-threshold-deg 45.0 \
  --forward-speed-change-penalty-weight 8.0 \
  --saturated-omega-flip-penalty-weight 20.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 36.0 \
  --time-penalty 0.05 \
  --stall-penalty -20.0 \
  --collision-penalty -350.0 \
  --near-miss-distance 5.0 \
  --near-miss-weight 18.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 8.0 \
  --anticipation-distance 8.0 \
  --conflict-risk-weight 7.0 \
  --conflict-brake-weight 1.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 7.0 \
  --conflict-escalation-penalty-weight 10.0 \
  --unsafe-close-speed-penalty-weight 5.0 \
  --conflict-overspeed-penalty-weight 5.0 \
  --desired-conflict-speed 0.22 \
  --avoidance-turn-reward-weight 18.0 \
  --proximity-gradient-penalty-weight 22.0 \
  --proximity-gradient-distance 7.0 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 5.0 \
  --separation-recovery-weight 25.0 \
  --entanglement-penalty-weight 15.0 \
  --entanglement-distance 5.0 \
  --entanglement-grace-steps 8 \
  --entanglement-low-speed-penalty-weight 8.0 \
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
  --team-reward-weight 0.10 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --team-completion-bonus 30.0 \
  --deadlock-penalty-weight 8.0 \
  --stop-go-penalty-weight 3.0 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  "${COMMON_ARGS[@]}"

echo "========== Phase 3 complete =========="
echo "Model: $OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh63_batch_eval.sh"
echo "  2. SITL gate: bash scripts/fresh63_sitl_gate.sh"
