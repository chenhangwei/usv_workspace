#!/bin/bash
# fresh56 Phase 1 中期验证 + Phase 2 安全重启
#
# 按建议执行:
#   1. 评估 Phase 1 关键检查点 (60K 和 120K)
#   2. 如果合格 → 重启 Phase 2 (从最新检查点恢复, 禁用自动eval避免OOM)
#   3. Phase 2 训练完成后单独做 batch eval

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=115
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh56_checkpoints"
EVAL_DIR="/tmp/fresh56_phase1_eval"
mkdir -p "$EVAL_DIR"

# ─────────────────────────────────────────────────────────────
# Step 1: Phase 1 关键检查点评估
# ─────────────────────────────────────────────────────────────
echo "═══════════════════════════════════════════════════"
echo "  Step 1: Phase 1 Checkpoint Quality Check"
echo "═══════════════════════════════════════════════════"

for STEP in 60288 120000; do
  CKPT="$CKPT_DIR/fresh56_phase1_step_$(printf '%07d' $STEP).pt"
  OUT="$EVAL_DIR/fresh56_phase1_${STEP}_eval.json"
  
  if [[ ! -f "$CKPT" ]]; then
    echo "SKIP: $CKPT not found"
    continue
  fi
  
  echo ""
  echo "──── Evaluating Phase1 @ step ${STEP} ────"
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$CKPT" \
    --episodes 10 \
    --steps-per-episode 180 \
    --scenario two_usv_head_on \
    --output-json "$OUT"
  
  echo "Result saved: $OUT"
  python3 -c "
import json, sys
with open('$OUT') as f:
    d = json.load(f)
coll = d.get('collision_rate', -1)
timeout = d.get('timeout_rate', -1)
progress = d.get('mean_team_goal_distance_delta', 0)
sep_min = d.get('worst_episode_min_separation', d.get('mean_episode_min_separation', -1))
print(f'  collision_rate: {coll:.3f}')
print(f'  timeout_rate:   {timeout:.3f}')
print(f'  progress_delta: {progress:.3f}')
print(f'  worst_min_sep:  {sep_min:.3f}')
if coll > 0.2:
    print('  ⚠️  HIGH COLLISION RATE - Phase 1 may have issues')
elif progress < 0:
    print('  ⚠️  NEGATIVE PROGRESS - policy may be over-conservative')
else:
    print('  ✅  Phase 1 @ ${STEP} looks healthy')
"
done

echo ""
echo "═══════════════════════════════════════════════════"
echo "  Step 1 Complete. Review results above."
echo "  If healthy → proceed to Step 2 (Phase 2 restart)"
echo "═══════════════════════════════════════════════════"

# 等用户确认
read -p "Continue to restart Phase 2? [y/N] " CONFIRM
if [[ "$CONFIRM" != "y" && "$CONFIRM" != "Y" ]]; then
  echo "Aborted. Fix issues before restarting Phase 2."
  exit 0
fi

# ─────────────────────────────────────────────────────────────
# Step 2: 重启 Phase 2 (禁用auto-eval避免OOM)
# 从最新的 Phase 2 检查点恢复, 如果没有则从 Phase 1 输出启动
# ─────────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Step 2: Restart Phase 2 Training"
echo "═══════════════════════════════════════════════════"

# 找最新的 Phase 2 检查点
LATEST_P2=$(ls -t "$CKPT_DIR"/fresh56_step_*.pt 2>/dev/null | head -1)
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh56_phase1.pt"

if [[ -n "$LATEST_P2" ]]; then
  RESUME_FROM="$LATEST_P2"
  echo "Resuming Phase 2 from: $RESUME_FROM"
else
  RESUME_FROM="$PHASE1_OUTPUT"
  echo "Starting Phase 2 from Phase 1 output: $RESUME_FROM"
fi

PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh56.pt"

# 公共参数 (与train_fresh56.sh一致)
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
  --heading-omega-reference 0.75
  --angular-authority-power 1.3
  --angular-accel-limit 1.7
  --angular-decel-limit 2.3
  --conflict-turn-relief 0.50
  --collision-distance 0.75
  --collision-penalty -200.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.12
  --conflict-distance 7.0
  --anticipation-distance 7.0
  --head-on-guidance-distance 7.0
  --head-on-target-starboard-offset 0.90
  --heading-error-weight 2.0
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 24.0
  --checkpoint-interval 2500
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
  --sim-tau-linear 2.0
  --sim-tau-angular 0.8
  --speed-scale-distance 3.0
  --speed-scale-min 0.35
)

# Phase 2: 不启用 --auto-evaluate-checkpoints, 训练完成后单独做batch eval
nohup python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --load-weights-from "$RESUME_FROM" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 230000 \
  --learning-rate 2.5e-5 \
  --learning-rate-end 8e-6 \
  --entropy-coef 0.004 \
  --entropy-coef-end 0.002 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 2.5 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 8.5 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 5.5 \
  --progress-weight 3.1 \
  --conflict-risk-weight 5.5 \
  --conflict-brake-weight 2.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.0 \
  --conflict-escalation-penalty-weight 4.5 \
  --unsafe-close-speed-penalty-weight 8.0 \
  --conflict-overspeed-penalty-weight 6.5 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 4.5 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 5.5 \
  --angular-accel-penalty-weight 4.0 \
  --straight-line-omega-penalty-weight 7.0 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 5.5 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 8.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 5.5 \
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
  --proximity-gradient-penalty-weight 5.0 \
  --proximity-gradient-distance 3.0 \
  --speed-distance-coupling-penalty-weight 6.0 \
  --speed-distance-coupling-threshold 2.5 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}" \
  > /tmp/fresh56_phase2_train.log 2>&1 &

P2_PID=$!
echo "Phase 2 training started: PID=$P2_PID"
echo "Log: /tmp/fresh56_phase2_train.log"
echo ""
echo "Monitor with: tail -f /tmp/fresh56_phase2_train.log | grep MAPPO"
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Phase 2 running. After completion, run:"
echo "  bash scripts/fresh56_batch_eval.sh"
echo "═══════════════════════════════════════════════════"
