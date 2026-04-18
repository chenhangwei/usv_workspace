#!/bin/bash
# fresh64: 基于 SITL 诊断结果的全面优化重训练
#
# fresh63 SITL 诊断发现的4个核心问题:
#   1. 不跟踪路径 — CTE低但航向误差70-100°均值
#   2. 航点转换时ω振荡 — angular_authority_power=1.8在<20°时压制ω
#   3. 无有效单向旋转 — ω-heading相关性仅-0.198~-0.628
#   4. 无邻船时原地旋转 — 罚项权重过大抑制动作,heading信号不足
#
# fresh64 修复方案:
#   [P0-1] ✓ sin/cos(heading_error) 替代标量 (代码已改, ego_dim 11→12)
#   [P0-2] ✓ heading_correction_reward — 转向方向奖励 (代码已改)
#   [P0-3] angular_authority_power 1.8→1.0, heading_omega_reference 0.65→1.0
#   [P1-4] Phase 1 多航点训练 (max_waypoints=3, 学习航点转换)
#   [P1-5] heading_error_weight 1.8→10.0 (对齐信号强度)
#   [P1-6] 降低惩罚项: smoothness 7→3, accel 10→4, omega 12→6, fwd_speed 8→3
#   [P2-7] sim_tau_linear 1.5→0.6 (更接近真机动态)
#   [P2-8] heading_convergence_threshold_deg 45→90 (更宽的收敛区间)
#
# 三阶段课程:
#   Phase 1 (0-150K): 纯路径跟踪 — solo + 多航点, 学习航向控制
#   Phase 2 (150K-300K): 路径 + 避让 — solo + head_on, 单航点
#   Phase 3 (300K-500K): 全场景 — 4+场景完整训练

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh64_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh64_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh64_phase2.pt"
PHASE3_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh64_phase3.pt"

mkdir -p "$CKPT_DIR"

# ─────────────────────────────────────────────────────────────
# 公共参数 (所有阶段共享)
# ─────────────────────────────────────────────────────────────
COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 4
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
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
  --heading-omega-reference 1.0
  --angular-authority-power 1.0
  --angular-accel-limit 1.2
  --angular-decel-limit 2.8
  --conflict-turn-relief 0.50
  --collision-distance 0.75
  --sim-tau-linear 0.6
  --sim-tau-angular 0.35
  --dr-tau-linear-low 0.4
  --dr-tau-linear-high 1.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
)

# ─────────────────────────────────────────────────────────────
# Phase 1: 纯路径跟踪 — solo + 多航点 (150K)
# 从零开始训练, 3 waypoints/episode 学习航点转换
# ─────────────────────────────────────────────────────────────
echo "========== Phase 1: Solo Navigation + Multi-Waypoint (150K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE1_OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 150000 \
  --clip-range 0.15 \
  --learning-rate 3.0e-4 \
  --learning-rate-end 5.0e-5 \
  --entropy-coef 0.02 \
  --entropy-coef-end 0.008 \
  --episode-timeout 120.0 \
  --no-progress-timeout 20.0 \
  --max-waypoints-per-episode 3 \
  --waypoint-bonus 12.0 \
  --scenario solo_navigation \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.20 \
  --scenario-goal-position-std 0.3 \
  --cte-clip-range 5.0 \
  --progress-weight 4.0 \
  --heading-error-weight 10.0 \
  --heading-relief-factor 0.55 \
  --heading-correction-reward-weight 8.0 \
  --path-deviation-penalty-weight 8.0 \
  --path-deviation-tolerance 0.3 \
  --path-deviation-conflict-scale 0.0 \
  --straight-line-omega-penalty-weight 6.0 \
  --straight-line-omega-cte-gate 0.5 \
  --straight-line-omega-conflict-floor 1.0 \
  --action-smoothness-weight 3.0 \
  --angular-accel-penalty-weight 4.0 \
  --forward-speed-change-penalty-weight 3.0 \
  --pure-cruise-reward-weight 4.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.0 \
  --pure-spin-penalty-weight 8.0 \
  --heading-convergence-reward-weight 6.0 \
  --heading-convergence-threshold-deg 90.0 \
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
  --avoidance-turn-reward-weight 0.0 \
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

echo "Phase 1 complete: $PHASE1_OUTPUT"

# ─────────────────────────────────────────────────────────────
# Phase 2: 路径 + 物理避让 — solo + head_on (150K)
# 从 Phase 1 权重转移, 引入碰撞避让
# ─────────────────────────────────────────────────────────────
echo "========== Phase 2: Path + Avoidance (150K, solo + head_on) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$PHASE1_OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 150000 \
  --clip-range 0.10 \
  --learning-rate 1.0e-4 \
  --learning-rate-end 2.0e-5 \
  --entropy-coef 0.012 \
  --entropy-coef-end 0.005 \
  --episode-timeout 55.0 \
  --no-progress-timeout 20.0 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario-spawn-position-std 0.5 \
  --scenario-spawn-heading-std 0.25 \
  --scenario-goal-position-std 0.5 \
  --encounter-type-dropout 0.3 \
  --cte-clip-range 5.0 \
  --speed-scale-distance 5.0 \
  --speed-scale-min 0.15 \
  --progress-weight 3.0 \
  --heading-error-weight 10.0 \
  --heading-relief-factor 0.65 \
  --heading-correction-reward-weight 8.0 \
  --path-deviation-penalty-weight 10.0 \
  --path-deviation-tolerance 0.25 \
  --path-deviation-conflict-scale 2.0 \
  --straight-line-omega-penalty-weight 6.0 \
  --straight-line-omega-cte-gate 0.5 \
  --straight-line-omega-conflict-floor 0.05 \
  --action-smoothness-weight 3.0 \
  --angular-accel-penalty-weight 4.0 \
  --forward-speed-change-penalty-weight 3.0 \
  --pure-cruise-reward-weight 4.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 2.5 \
  --pure-spin-penalty-weight 10.0 \
  --heading-convergence-reward-weight 5.0 \
  --heading-convergence-threshold-deg 90.0 \
  --saturated-omega-flip-penalty-weight 12.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 30.0 \
  --time-penalty 0.04 \
  --stall-penalty -18.0 \
  --collision-penalty -300.0 \
  --near-miss-distance 5.0 \
  --near-miss-weight 14.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 8.0 \
  --anticipation-distance 8.0 \
  --conflict-risk-weight 5.0 \
  --conflict-brake-weight 2.0 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 5.0 \
  --conflict-escalation-penalty-weight 7.0 \
  --unsafe-close-speed-penalty-weight 5.0 \
  --conflict-overspeed-penalty-weight 4.0 \
  --desired-conflict-speed 0.20 \
  --avoidance-turn-reward-weight 12.0 \
  --proximity-gradient-penalty-weight 15.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 5.0 \
  --speed-distance-coupling-threshold 4.0 \
  --separation-recovery-weight 15.0 \
  --entanglement-penalty-weight 10.0 \
  --entanglement-distance 4.0 \
  --entanglement-grace-steps 10 \
  --entanglement-low-speed-penalty-weight 5.0 \
  --head-on-guidance-distance 8.0 \
  --head-on-target-starboard-offset 0.90 \
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
  --team-progress-weight 1.20 \
  --team-goal-proximity-weight 0.30 \
  --coordination-reward-weight 0.25 \
  --team-completion-bonus 25.0 \
  --deadlock-penalty-weight 5.0 \
  --stop-go-penalty-weight 2.5 \
  --goal-proximity-reward-weight 2.0 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.30 \
  "${COMMON_ARGS[@]}"

echo "Phase 2 complete: $PHASE2_OUTPUT"

# ─────────────────────────────────────────────────────────────
# Phase 3: 全场景 — 4+场景完整训练 (200K)
# 从 Phase 2 权重转移, 全避让策略
# ─────────────────────────────────────────────────────────────
echo "========== Phase 3: Full Scenarios — All Avoidance (200K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE3_OUTPUT" \
  --load-weights-from "$PHASE2_OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 200000 \
  --clip-range 0.08 \
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
  --speed-scale-distance 5.0 \
  --speed-scale-min 0.15 \
  --progress-weight 3.0 \
  --heading-error-weight 10.0 \
  --heading-relief-factor 0.75 \
  --heading-correction-reward-weight 8.0 \
  --path-deviation-penalty-weight 12.0 \
  --path-deviation-tolerance 0.20 \
  --path-deviation-conflict-scale 3.0 \
  --straight-line-omega-penalty-weight 6.0 \
  --straight-line-omega-cte-gate 0.5 \
  --straight-line-omega-conflict-floor 0.02 \
  --action-smoothness-weight 3.0 \
  --angular-accel-penalty-weight 4.0 \
  --forward-speed-change-penalty-weight 3.0 \
  --pure-cruise-reward-weight 5.0 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 12.0 \
  --heading-convergence-reward-weight 4.0 \
  --heading-convergence-threshold-deg 90.0 \
  --saturated-omega-flip-penalty-weight 15.0 \
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

echo "========== fresh64 三阶段训练完成 =========="
echo "Phase 1: $PHASE1_OUTPUT"
echo "Phase 2: $PHASE2_OUTPUT"
echo "Phase 3: $PHASE3_OUTPUT"
echo ""
echo "Next steps:"
echo "  1. Run batch eval: bash scripts/fresh64_batch_eval.sh"
echo "  2. SITL gate test"
