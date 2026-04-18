#!/bin/bash
# fresh65: 在 fresh64 基础上专门打击 slow-roll / zero-success 策略
#
# 目标:
#   1. 保留 fresh64 的三阶段课程结构与 heading/CTE 修复
#   2. 配合环境中的新 reward 修复, 在脚本层进一步压制停滞/慢挪
#   3. 不覆盖 fresh64 产物, 作为下一轮独立实验
#
# 关键增量:
#   [F65-1] 更早 no-progress 截断
#   [F65-2] 更高 min-progress-delta, 减少微小位移刷新计时器
#   [F65-3] 更强 pure_idle_penalty / stall_penalty
#   [F65-4] 略提升 goal_bonus, 强化“真正到达”而非“拖到结束”

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh65_checkpoints"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh65_phase1.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh65_phase2.pt"
PHASE3_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh65_phase3.pt"

mkdir -p "$CKPT_DIR"

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
  --base-ros-domain-id 160
)

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
  --no-progress-timeout 12.0 \
  --min-progress-delta 0.45 \
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
  --pure-idle-penalty-weight 2.4 \
  --pure-turn-penalty-weight 2.0 \
  --pure-spin-penalty-weight 8.0 \
  --heading-convergence-reward-weight 6.0 \
  --heading-convergence-threshold-deg 90.0 \
  --saturated-omega-flip-penalty-weight 10.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 32.0 \
  --time-penalty 0.035 \
  --stall-penalty -28.0 \
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
  --no-progress-timeout 14.0 \
  --min-progress-delta 0.45 \
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
  --pure-idle-penalty-weight 2.2 \
  --pure-turn-penalty-weight 2.5 \
  --pure-spin-penalty-weight 10.0 \
  --heading-convergence-reward-weight 5.0 \
  --heading-convergence-threshold-deg 90.0 \
  --saturated-omega-flip-penalty-weight 12.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 36.0 \
  --time-penalty 0.045 \
  --stall-penalty -30.0 \
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
  --no-progress-timeout 16.0 \
  --min-progress-delta 0.50 \
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
  --pure-idle-penalty-weight 2.0 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 12.0 \
  --heading-convergence-reward-weight 4.0 \
  --heading-convergence-threshold-deg 90.0 \
  --saturated-omega-flip-penalty-weight 15.0 \
  --omega-flip-saturation-threshold 0.20 \
  --goal-bonus 42.0 \
  --time-penalty 0.055 \
  --stall-penalty -34.0 \
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

echo "========== fresh65 三阶段训练脚本已准备 =========="
echo "Phase 1: $PHASE1_OUTPUT"
echo "Phase 2: $PHASE2_OUTPUT"
echo "Phase 3: $PHASE3_OUTPUT"
echo ""
echo "To launch manually:"
echo "  nohup bash scripts/train_fresh65.sh > /tmp/fresh65_train.log 2>&1 &"