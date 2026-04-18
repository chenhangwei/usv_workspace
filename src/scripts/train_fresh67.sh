#!/bin/bash
# fresh67: L1 重构 —— action projection 解死锁 + 奖励精简 + soft curriculum
#
# 核心改动（相对 fresh66）:
# [L1-1] action_projection.py 加入 authority_floor=0.35 + min_forward_speed_floor=0.08
#        → 彻底消除近目标 hover 和 omega-flip（无需 near_goal_idle_penalty / saturated_omega_flip / pure_spin_penalty）
# [L1-2] 奖励精简 ~80 项 → 12 核心项:
#        导航: progress, goal_bonus, time_penalty
#        安全: collision, near_miss (quadratic), proximity_gradient, speed_distance_coupling
#        方向: heading_error, heading_correction, avoidance_turn_reward
#        平滑: action_smoothness
#        终止: stall_penalty
# [L1-3] Soft curriculum: 两阶段（取消 fresh66 的三阶段跳跃）
#        Stage A (120K): solo + head_on 混合，低避碰压力，重点学到达+基本避让
#        Stage B (200K): 全场景 + 强避碰，学鲁棒避碰
#
# 删除的 fresh66 旧项（由 L1-1 吸收或 default=0）:
#   pure_* (4), head_on_specific_* (8), crossing_specific_* (2), overtaking_specific_* (5),
#   colregs_port_turn, path_deviation_* (3), entanglement_* (3), separation_recovery,
#   team_* (5), conflict_overspeed, saturated_omega_flip, straight_line_omega_*,
#   angular_accel_penalty, forward_speed_change, stop_go_penalty, near_goal_idle_penalty,
#   heading_convergence_reward, deadlock_penalty
#
# 脚本体积: fresh66 ~340 行 → fresh67 ~140 行 (-59%)

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh67_checkpoints"
STAGE_A_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh67_stageA.pt"
STAGE_B_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh67_stageB.pt"

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
  # [L1-1] 核心修复
  --angular-authority-floor 0.35
  --min-forward-speed-floor 0.08
  --collision-distance 0.75
  --sim-tau-linear 0.6
  --sim-tau-angular 0.35
  --dr-tau-linear-low 0.4
  --dr-tau-linear-high 1.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 180
)

# ═══════════════════════════════════════════════════════════
# Stage A: Solo + Head-on 混合 (120K)
#   - 目标: 学会导航到目标 + 基础 head-on 避让
#   - 低避碰权重 (near_miss=8, proximity_gradient=10)
#   - entropy 较高 (0.02 → 0.01) 鼓励探索
# ═══════════════════════════════════════════════════════════
echo "========== Stage A: Solo + Head-on (120K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$STAGE_A_OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 120000 \
  --clip-range 0.15 \
  --learning-rate 3.0e-4 \
  --learning-rate-end 8.0e-5 \
  --entropy-coef 0.02 \
  --entropy-coef-end 0.010 \
  --episode-timeout 60.0 \
  --no-progress-timeout 14.0 \
  --min-progress-delta 0.40 \
  --max-waypoints-per-episode 2 \
  --waypoint-bonus 10.0 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario-spawn-position-std 0.5 \
  --scenario-spawn-heading-std 0.25 \
  --scenario-goal-position-std 0.5 \
  --cte-clip-range 5.0 \
  --progress-weight 4.0 \
  --goal-bonus 30.0 \
  --time-penalty 0.035 \
  --stall-penalty -25.0 \
  --heading-error-weight 8.0 \
  --heading-relief-factor 0.55 \
  --heading-correction-reward-weight 6.0 \
  --action-smoothness-weight 2.5 \
  --collision-penalty -300.0 \
  --near-miss-distance 4.0 \
  --near-miss-weight 8.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 6.0 \
  --anticipation-distance 6.0 \
  --proximity-gradient-penalty-weight 10.0 \
  --proximity-gradient-distance 5.0 \
  --speed-distance-coupling-penalty-weight 4.0 \
  --speed-distance-coupling-threshold 4.0 \
  --avoidance-turn-reward-weight 8.0 \
  --desired-conflict-speed 0.20 \
  --goal-proximity-reward-weight 1.2 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.55 \
  --goal-proximity-smoothness-relief 0.70 \
  --goal-proximity-conflict-relief 0.20 \
  "${COMMON_ARGS[@]}"

echo "Stage A complete: $STAGE_A_OUTPUT"

# ═══════════════════════════════════════════════════════════
# Stage B: 全场景 + 强避碰 (200K)
#   - 目标: 在 crossing / overtaking / random encounter 下鲁棒避碰
#   - 加大 near_miss / proximity_gradient / avoidance_turn
#   - entropy 较低 (0.010 → 0.004) 收敛到稳定策略
# ═══════════════════════════════════════════════════════════
echo "========== Stage B: All scenarios (200K) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$STAGE_B_OUTPUT" \
  --load-weights-from "$STAGE_A_OUTPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 200000 \
  --clip-range 0.10 \
  --learning-rate 1.5e-4 \
  --learning-rate-end 2.0e-5 \
  --entropy-coef 0.010 \
  --entropy-coef-end 0.004 \
  --episode-timeout 55.0 \
  --no-progress-timeout 16.0 \
  --min-progress-delta 0.45 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario two_usv_random_encounter \
  --scenario three_usv_random_encounter \
  --scenario-spawn-position-std 1.0 \
  --scenario-spawn-heading-std 0.40 \
  --scenario-goal-position-std 1.0 \
  --encounter-type-dropout 0.4 \
  --cte-clip-range 5.0 \
  --progress-weight 3.0 \
  --goal-bonus 40.0 \
  --time-penalty 0.045 \
  --stall-penalty -30.0 \
  --heading-error-weight 8.0 \
  --heading-relief-factor 0.70 \
  --heading-correction-reward-weight 6.0 \
  --action-smoothness-weight 2.5 \
  --collision-penalty -350.0 \
  --near-miss-distance 5.0 \
  --near-miss-weight 16.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 8.0 \
  --anticipation-distance 8.0 \
  --proximity-gradient-penalty-weight 20.0 \
  --proximity-gradient-distance 6.0 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 5.0 \
  --avoidance-turn-reward-weight 16.0 \
  --desired-conflict-speed 0.22 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.30 \
  "${COMMON_ARGS[@]}"

echo "Stage B complete: $STAGE_B_OUTPUT"

echo ""
echo "========== fresh67 两阶段训练完成 =========="
echo "Stage A: $STAGE_A_OUTPUT"
echo "Stage B: $STAGE_B_OUTPUT"
echo ""
echo "评估: bash scripts/fresh64_sitl_gate.sh $STAGE_B_OUTPUT"
echo "查询: tail -f /tmp/fresh67_train.log"
