#!/bin/bash
# fresh72: Single-Lever Sprint (方案①)
#
# 决策依据 (fresh68→71 教训):
#   - 一次动 ≥3 个 reward lever 必导致"修一处崩两处"，4 代 0/5。
#   - fresh70_stageB.pt 是当前综合最优基线 (solo/head_on 最佳, crossing collision 62.5%)。
#   - fresh71 用更高 avoidance_turn + separation_recovery 反而把 head_on/random 砸烂。
#   - 真正没试过的两个 lever: entanglement_low_speed + conflict_overspeed
#     (训练代码已支持但 fresh68-71 全部默认 0)
#
# fresh72 假设 (单一变量):
#   - "fresh70 dense 场景的 100% 碰撞 = 智能体在密集冲突中不肯减速 + 互相绕圈"
#   - 解法: entanglement_low_speed 2.5 (惩罚被困住时低速), conflict_overspeed 4.0
#     (惩罚冲突中过速)。两者方向相反、互相制衡，让"减速避让"变成最优。
#
# fresh72 与 fresh70_stageB 的所有差异:
#   [仅 2 处新增]
#   --entanglement-low-speed-penalty-weight  0    → 2.5
#   --conflict-overspeed-penalty-weight      0    → 4.0
#   [其余全部对齐 fresh70_stageB]
#
# 训练形式:
#   - 单段 30K (vs fresh70 Stage B 120K) — 不允许覆盖 fresh70 已学策略
#   - 全 5 场景从头训练 (vs fresh72v1 分阶段) — dense 短板才是优化目标
#   - checkpoint 每 3000 步, auto-evaluate 全 5 场景 — 候选评分挑最佳, 不用最后权重
#
# 用法:
#   bash scripts/train_fresh72.sh

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh70_stageB.pt"
OUTPUT="/mnt/data/checkpoints/usv_rl/fresh72_sprint.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh72_checkpoints"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh72_ranking.json"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh72_eval"

mkdir -p "$CKPT_DIR" "$EVAL_DIR"

if [[ ! -f "$BASE_INPUT" ]]; then
  echo "ERROR: base checkpoint not found: $BASE_INPUT"
  exit 1
fi

echo "========== fresh72 Single-Lever Sprint (30K from fresh70_stageB) =========="
python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --load-weights-from "$BASE_INPUT" \
  --checkpoint-dir "$CKPT_DIR" \
  --checkpoint-ranking-json "$RANKING_JSON" \
  --checkpoint-eval-json-dir "$EVAL_DIR" \
  --auto-evaluate-checkpoints \
  --checkpoint-interval 3000 \
  --checkpoint-eval-episodes 5 \
  --checkpoint-eval-steps 275 \
  --checkpoint-eval-scenario solo_navigation \
  --checkpoint-eval-scenario two_usv_head_on \
  --checkpoint-eval-scenario three_usv_crossing \
  --checkpoint-eval-scenario three_usv_overtaking \
  --checkpoint-eval-scenario three_usv_random_encounter \
  --num-agents 3 \
  --total-timesteps 30000 \
  --clip-range 0.10 \
  --learning-rate 6.0e-5 \
  --learning-rate-end 2.0e-5 \
  --entropy-coef 0.005 \
  --entropy-coef-end 0.002 \
  --episode-timeout 58.0 \
  --no-progress-timeout 14.0 \
  --min-progress-delta 0.45 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter \
  --scenario-spawn-position-std 0.8 \
  --scenario-spawn-heading-std 0.32 \
  --scenario-goal-position-std 0.8 \
  --encounter-type-dropout 0.20 \
  --cte-clip-range 5.0 \
  --progress-weight 6.0 \
  --goal-bonus 40.0 \
  --time-penalty 0.07 \
  --stall-penalty -50.0 \
  --heading-error-weight 5.0 \
  --heading-relief-factor 0.85 \
  --heading-correction-reward-weight 0.0 \
  --action-smoothness-weight 6.0 \
  --saturated-omega-flip-penalty-weight 8.0 \
  --omega-flip-saturation-threshold 0.40 \
  --separation-recovery-weight 1.0 \
  --entanglement-penalty-weight 1.5 \
  --entanglement-low-speed-penalty-weight 2.5 \
  --collision-penalty -400.0 \
  --near-miss-distance 4.5 \
  --near-miss-weight 12.0 \
  --near-miss-exponent 2.0 \
  --conflict-distance 7.0 \
  --anticipation-distance 7.0 \
  --proximity-gradient-penalty-weight 4.0 \
  --proximity-gradient-distance 5.5 \
  --speed-distance-coupling-penalty-weight 3.0 \
  --speed-distance-coupling-threshold 4.5 \
  --conflict-overspeed-penalty-weight 4.0 \
  --avoidance-turn-reward-weight 2.0 \
  --desired-conflict-speed 0.22 \
  --goal-proximity-reward-weight 1.5 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.72 \
  --goal-proximity-conflict-relief 0.25 \
  --rollout-steps 192 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --squash-actions \
  --min-forward-speed 0.05 \
  --angular-delta-limit 0.40 \
  --neighbor-attention \
  --attention-embed-dim 32 \
  --attention-num-heads 1 \
  --normalize-observations \
  --per-scenario-advantage-norm \
  --scenario-balanced-loss \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  --cruise-speed 0.36 \
  --max-angular-velocity 0.40 \
  --heading-omega-deadband 0.05 \
  --heading-omega-reference 1.0 \
  --angular-authority-power 1.0 \
  --angular-accel-limit 1.2 \
  --angular-decel-limit 2.8 \
  --conflict-turn-relief 0.50 \
  --angular-authority-floor 0.35 \
  --min-forward-speed-floor 0.0 \
  --collision-distance 0.75 \
  --sim-tau-linear 0.6 \
  --sim-tau-angular 0.35 \
  --dr-tau-linear-low 0.4 \
  --dr-tau-linear-high 1.0 \
  --dr-tau-angular-low 0.2 \
  --dr-tau-angular-high 0.6 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 190

echo "fresh72 sprint complete: $OUTPUT"
echo "Candidates: $CKPT_DIR"
echo "Per-ckpt eval JSONs: $EVAL_DIR"
echo "Ranking: $RANKING_JSON"
#!/bin/bash
# fresh72: 从 fresh70_stageB 保守续训，修复纯 RL 导航/避让的 reward 拉扯
#
# 历史结论（fresh61 → fresh71）:
#   - fresh61: 进度很高，但 best_safe 不存在；说明“大而全 reward”会把安全做坏。
#   - fresh64: solo 导航学成了，但 multi-agent 仍然低进度、低最小间距。
#   - fresh67: heading_correction + 高 heading_error 造出了“原地对刷 heading”的局部最优。
#   - fresh68 Stage A: 目前最好的 solo seed；Stage B 把 avoidance/proximity 拉太高，灾难性遗忘。
#   - fresh69: 把 solo/head_on 拉回来，但 dense 3 船 omega_flip 爆炸。
#   - fresh70: 最近最平衡的纯 RL 基线；solo/head_on 最好，但 crossing/random 仍然会硬冲。
#   - fresh71: 用更高 avoidance_turn + separation_recovery 修 dense，结果把 head_on 推成过度规避，
#              还把 solo 也拖慢了。
#
# fresh72 策略:
#   [T1] 不再从 fresh68_stageA.pt 重学 Stage B，而是从 fresh70_stageB.pt 小步续训。
#   [T2] dense 场景不用更高 avoidance_turn 去修，改用 conflict_overspeed + entanglement_low_speed。
#   [T3] head_on 发散不用更高 heading 惩罚去拉，改用 team_dispersion + modest head_on guidance。
#   [T4] 训练内开启 checkpoint 自动评估，Stage C 默认从 pipeline 选出的 Stage B 候选继续。
#
# 用法:
#   bash scripts/train_fresh72.sh stageB
#   FRESH72_STAGE_B_INPUT=/path/to/best_stageB.pt bash scripts/train_fresh72.sh stageC
#   bash scripts/train_fresh72.sh all

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

MODE="${1:-stageB}"

STAGE_B_BASE_INPUT="/mnt/data/checkpoints/usv_rl/fresh70_stageB.pt"
STAGE_B_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh72_stageB.pt"
STAGE_B_SELECTED_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh72_stageB_selected.pt"
STAGE_C_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh72_stageC.pt"

STAGE_B_CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh72_stageB_checkpoints"
STAGE_C_CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh72_stageC_checkpoints"
STAGE_B_RANKING="/mnt/data/checkpoints/usv_rl/fresh72_stageB_ranking.json"
STAGE_C_RANKING="/mnt/data/checkpoints/usv_rl/fresh72_stageC_ranking.json"
STAGE_B_EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh72_stageB_eval"
STAGE_C_EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh72_stageC_eval"

mkdir -p \
  "$STAGE_B_CKPT_DIR" \
  "$STAGE_C_CKPT_DIR" \
  "$STAGE_B_EVAL_DIR" \
  "$STAGE_C_EVAL_DIR"

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
  --num-sampler-workers 2
  --base-ros-domain-id 190
)

resolve_stage_c_input() {
  local candidate="${FRESH72_STAGE_B_INPUT:-$STAGE_B_SELECTED_OUTPUT}"
  if [[ -f "$candidate" ]]; then
    echo "$candidate"
  else
    echo "$STAGE_B_OUTPUT"
  fi
}

run_stage_b() {
  if [[ ! -f "$STAGE_B_BASE_INPUT" ]]; then
    echo "ERROR: Stage B base checkpoint not found: $STAGE_B_BASE_INPUT"
    exit 1
  fi

  echo "========== fresh72 Stage B: retune from fresh70_stageB (60K) =========="
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_B_OUTPUT" \
    --load-weights-from "$STAGE_B_BASE_INPUT" \
    --checkpoint-dir "$STAGE_B_CKPT_DIR" \
    --checkpoint-ranking-json "$STAGE_B_RANKING" \
    --checkpoint-eval-json-dir "$STAGE_B_EVAL_DIR" \
    --auto-evaluate-checkpoints \
    --checkpoint-interval 6000 \
    --checkpoint-eval-episodes 6 \
    --checkpoint-eval-steps 275 \
    --checkpoint-eval-scenario solo_navigation \
    --checkpoint-eval-scenario two_usv_head_on \
    --checkpoint-eval-scenario three_usv_crossing \
    --num-agents 3 \
    --total-timesteps 60000 \
    --clip-range 0.10 \
    --learning-rate 6.0e-5 \
    --learning-rate-end 2.0e-5 \
    --entropy-coef 0.004 \
    --entropy-coef-end 0.0015 \
    --episode-timeout 58.0 \
    --no-progress-timeout 14.0 \
    --min-progress-delta 0.45 \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario-spawn-position-std 0.75 \
    --scenario-spawn-heading-std 0.30 \
    --scenario-goal-position-std 0.75 \
    --cte-clip-range 5.0 \
    --progress-weight 6.0 \
    --goal-bonus 40.0 \
    --time-penalty 0.07 \
    --stall-penalty -50.0 \
    --heading-error-weight 5.0 \
    --heading-relief-factor 0.85 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 6.0 \
    --saturated-omega-flip-penalty-weight 8.0 \
    --omega-flip-saturation-threshold 0.40 \
    --separation-recovery-weight 1.5 \
    --team-dispersion-penalty-weight 0.35 \
    --team-dispersion-margin 0.80 \
    --entanglement-penalty-weight 1.0 \
    --entanglement-distance 4.0 \
    --entanglement-grace-steps 12 \
    --entanglement-low-speed-penalty-weight 2.5 \
    --collision-penalty -400.0 \
    --near-miss-distance 4.5 \
    --near-miss-weight 12.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 7.0 \
    --anticipation-distance 7.0 \
    --head-on-guidance-distance 8.0 \
    --head-on-phase-gate-strength 0.35 \
    --proximity-gradient-penalty-weight 5.0 \
    --proximity-gradient-distance 5.5 \
    --speed-distance-coupling-penalty-weight 4.0 \
    --speed-distance-coupling-threshold 4.5 \
    --unsafe-close-speed-penalty-weight 2.0 \
    --conflict-overspeed-penalty-weight 6.0 \
    --avoidance-turn-reward-weight 3.0 \
    --desired-conflict-speed 0.26 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.72 \
    --goal-proximity-conflict-relief 0.25 \
    "${COMMON_ARGS[@]}"

  echo "Stage B complete: $STAGE_B_OUTPUT"
}

run_stage_c() {
  local stage_b_input
  stage_b_input="$(resolve_stage_c_input)"

  if [[ ! -f "$stage_b_input" ]]; then
    echo "ERROR: Stage C input checkpoint not found: $stage_b_input"
    exit 1
  fi

  echo "========== fresh72 Stage C: all-scenario generalization (120K) =========="
  echo "Stage C load source: $stage_b_input"
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_C_OUTPUT" \
    --load-weights-from "$stage_b_input" \
    --checkpoint-dir "$STAGE_C_CKPT_DIR" \
    --checkpoint-ranking-json "$STAGE_C_RANKING" \
    --checkpoint-eval-json-dir "$STAGE_C_EVAL_DIR" \
    --auto-evaluate-checkpoints \
    --checkpoint-interval 10000 \
    --checkpoint-eval-episodes 5 \
    --checkpoint-eval-steps 275 \
    --checkpoint-eval-scenario solo_navigation \
    --checkpoint-eval-scenario two_usv_head_on \
    --checkpoint-eval-scenario three_usv_crossing \
    --checkpoint-eval-scenario three_usv_overtaking \
    --checkpoint-eval-scenario three_usv_random_encounter \
    --num-agents 3 \
    --total-timesteps 120000 \
    --clip-range 0.08 \
    --learning-rate 4.0e-5 \
    --learning-rate-end 1.0e-5 \
    --entropy-coef 0.003 \
    --entropy-coef-end 0.001 \
    --episode-timeout 55.0 \
    --no-progress-timeout 16.0 \
    --min-progress-delta 0.45 \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario two_usv_random_encounter \
    --scenario three_usv_random_encounter \
    --scenario-spawn-position-std 0.95 \
    --scenario-spawn-heading-std 0.35 \
    --scenario-goal-position-std 0.95 \
    --encounter-type-dropout 0.25 \
    --cte-clip-range 5.0 \
    --progress-weight 5.5 \
    --goal-bonus 45.0 \
    --time-penalty 0.06 \
    --stall-penalty -50.0 \
    --heading-error-weight 4.5 \
    --heading-relief-factor 0.80 \
    --heading-correction-reward-weight 0.0 \
    --action-smoothness-weight 6.0 \
    --saturated-omega-flip-penalty-weight 8.0 \
    --omega-flip-saturation-threshold 0.40 \
    --separation-recovery-weight 1.5 \
    --team-dispersion-penalty-weight 0.30 \
    --team-dispersion-margin 1.00 \
    --entanglement-penalty-weight 1.25 \
    --entanglement-distance 4.0 \
    --entanglement-grace-steps 10 \
    --entanglement-low-speed-penalty-weight 3.5 \
    --collision-penalty -420.0 \
    --near-miss-distance 4.8 \
    --near-miss-weight 12.0 \
    --near-miss-exponent 2.0 \
    --conflict-distance 7.5 \
    --anticipation-distance 7.5 \
    --head-on-guidance-distance 8.0 \
    --head-on-phase-gate-strength 0.35 \
    --proximity-gradient-penalty-weight 5.5 \
    --proximity-gradient-distance 5.8 \
    --speed-distance-coupling-penalty-weight 4.5 \
    --speed-distance-coupling-threshold 4.8 \
    --unsafe-close-speed-penalty-weight 2.5 \
    --conflict-overspeed-penalty-weight 7.0 \
    --avoidance-turn-reward-weight 3.2 \
    --desired-conflict-speed 0.25 \
    --goal-proximity-reward-weight 1.5 \
    --goal-proximity-relief-distance 2.5 \
    --goal-proximity-heading-relief 0.60 \
    --goal-proximity-smoothness-relief 0.75 \
    --goal-proximity-conflict-relief 0.30 \
    "${COMMON_ARGS[@]}"

  echo "Stage C complete: $STAGE_C_OUTPUT"
}

case "$MODE" in
  stageB) run_stage_b ;;
  stageC) run_stage_c ;;
  all)    run_stage_b && run_stage_c ;;
  *) echo "Usage: $0 {stageB|stageC|all}"; exit 1 ;;
esac
