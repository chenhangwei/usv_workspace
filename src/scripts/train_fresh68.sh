#!/bin/bash
# fresh68: L2 根本修订 —— 杀死"原地刷 heading 奖励" bug
#
# fresh67 SITL gate 根因（Stage A 0/5, Stage B 0/5, Stage A 比 Stage B 还差）:
#   - solo progress 只有 0.016, he 1.58rad, omega_sat 72.5%
#   - 诊断: heading_correction_reward=6 + heading_error_weight=8 创造了"原地对准"
#     的局部最优。time_penalty=0.035 惩罚 60s × 0.035 = 2.1 远低于 heading 可拿到的
#     累积奖励。policy 学会了"小幅转向不前进"的退化策略。
#
# L2 核心修复（相对 fresh67）:
# [L2-A] 杀 heading_correction_reward_weight: 6.0 → 0.0
# [L2-B] 降 heading_error_weight: 8.0 → 3.0（保留方向惩罚，但不压制 progress）
# [L2-C] 抬 time_penalty: 0.035 → 0.08（"不动"的成本翻倍以上）
# [L2-D] 抬 progress_weight: 4.0/3.0 → 8.0/6.0/5.0（让前进主导）
# [L2-E] 抬 stall_penalty: -25 → -60（重罚原地）
# [L2-F] 禁用 min_forward_speed_floor: 0.08 → 0.0（让 policy 自由选择速度）
# [L2-G] 保留 angular_authority_floor=0.35（它是对的；head_on omega_flip=1.4 证明）
# [L2-H] 分阶段 dry-run: 先只跑 Stage A 50K 验证 solo 学好后再续 B/C
#
# 跑法:
#   bash scripts/train_fresh68.sh stageA        # 只跑 Stage A 50K（~40 min）
#   bash scripts/train_fresh68.sh stageB        # 续 Stage B 120K
#   bash scripts/train_fresh68.sh stageC        # 续 Stage C 160K
#   bash scripts/train_fresh68.sh all           # 全跑（不推荐首次）

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

MODE="${1:-stageA}"

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh68_checkpoints"
STAGE_A_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh68_stageB.pt"
STAGE_C_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh68_stageC.pt"

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
  --min-forward-speed 0.05
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
  --angular-authority-floor 0.35
  --min-forward-speed-floor 0.0
  --collision-distance 0.75
  --sim-tau-linear 0.6
  --sim-tau-angular 0.35
  --dr-tau-linear-low 0.4
  --dr-tau-linear-high 1.0
  --dr-tau-angular-low 0.2
  --dr-tau-angular-high 0.6
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 185
)

run_stage_a() {
  echo "========== Stage A: Solo only (50K dry-run) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_A_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 1 \
    --total-timesteps 80000 \
    --clip-range 0.15 \
    --learning-rate 3.0e-4 \
    --learning-rate-end 1.0e-4 \
    --entropy-coef 0.02 \
    --entropy-coef-end 0.012 \
    --episode-timeout 60.0 \
    --no-progress-timeout 12.0 \
    --min-progress-delta 0.50 \
    --max-waypoints-per-episode 2 \
    --waypoint-bonus 12.0 \
    --scenario solo_navigation \
    --scenario-spawn-position-std 0.5 \
    --scenario-spawn-heading-std 0.30 \
    --scenario-goal-position-std 0.5 \
    --cte-clip-range 5.0 \
    --progress-weight 8.0 \
    --goal-bonus 35.0 \
    --time-penalty 0.08 \
    --stall-penalty -60.0 \
    --heading-error-weight 3.0 \
    --heading-relief-factor 0.55 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 2.0 \
    --collision-penalty -150.0 \
    --near-miss-distance 4.0 \
    --near-miss-weight 5.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 6.0 \
    --anticipation-distance 6.0 \
    --proximity-gradient-penalty-weight 6.0 \
    --proximity-gradient-distance 5.0 \
    --speed-distance-coupling-penalty-weight 2.0 \
    --speed-distance-coupling-threshold 4.0 \
    --avoidance-turn-reward-weight 5.0 \
    --desired-conflict-speed 0.22 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.55 \
    --goal-proximity-smoothness-relief 0.70 \
    --goal-proximity-conflict-relief 0.20 \
    "${COMMON_ARGS[@]}"

  echo "Stage A complete: $STAGE_A_OUTPUT"
}

run_stage_b() {
  echo "========== Stage B: Solo + Head-on + Crossing (120K) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_B_OUTPUT" \
    --load-weights-from "$STAGE_A_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 3 \
    --total-timesteps 120000 \
    --clip-range 0.12 \
    --learning-rate 2.0e-4 \
    --learning-rate-end 5.0e-5 \
    --entropy-coef 0.015 \
    --entropy-coef-end 0.010 \
    --episode-timeout 58.0 \
    --no-progress-timeout 14.0 \
    --min-progress-delta 0.45 \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario-spawn-position-std 0.7 \
    --scenario-spawn-heading-std 0.30 \
    --scenario-goal-position-std 0.7 \
    --cte-clip-range 5.0 \
    --progress-weight 6.0 \
    --goal-bonus 40.0 \
    --time-penalty 0.07 \
    --stall-penalty -50.0 \
    --heading-error-weight 3.0 \
    --heading-relief-factor 0.60 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 2.0 \
    --collision-penalty -150.0 \
    --near-miss-distance 4.5 \
    --near-miss-weight 8.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 7.0 \
    --anticipation-distance 7.0 \
    --proximity-gradient-penalty-weight 10.0 \
    --proximity-gradient-distance 5.5 \
    --speed-distance-coupling-penalty-weight 3.0 \
    --speed-distance-coupling-threshold 4.5 \
    --avoidance-turn-reward-weight 8.0 \
    --desired-conflict-speed 0.22 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.72 \
    --goal-proximity-conflict-relief 0.25 \
    "${COMMON_ARGS[@]}"

  echo "Stage B complete: $STAGE_B_OUTPUT"
}

run_stage_c() {
  echo "========== Stage C: All scenarios (160K) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_C_OUTPUT" \
    --load-weights-from "$STAGE_B_OUTPUT" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 3 \
    --total-timesteps 160000 \
    --clip-range 0.10 \
    --learning-rate 1.2e-4 \
    --learning-rate-end 3.0e-5 \
    --entropy-coef 0.012 \
    --entropy-coef-end 0.008 \
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
    --encounter-type-dropout 0.3 \
    --cte-clip-range 5.0 \
    --progress-weight 5.0 \
    --goal-bonus 45.0 \
    --time-penalty 0.06 \
    --stall-penalty -50.0 \
    --heading-error-weight 3.0 \
    --heading-relief-factor 0.70 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 2.0 \
    --collision-penalty -180.0 \
    --near-miss-distance 5.0 \
    --near-miss-weight 10.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 8.0 \
    --anticipation-distance 8.0 \
    --proximity-gradient-penalty-weight 14.0 \
    --proximity-gradient-distance 6.0 \
    --speed-distance-coupling-penalty-weight 4.0 \
    --speed-distance-coupling-threshold 5.0 \
    --avoidance-turn-reward-weight 12.0 \
    --desired-conflict-speed 0.22 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.75 \
    --goal-proximity-conflict-relief 0.30 \
    "${COMMON_ARGS[@]}"

  echo "========== fresh68 complete =========="
}

case "$MODE" in
  stageA) run_stage_a ;;
  stageB) run_stage_b ;;
  stageC) run_stage_c ;;
  all)    run_stage_a && run_stage_b && run_stage_c ;;
  *) echo "Usage: $0 {stageA|stageB|stageC|all}"; exit 1 ;;
esac
