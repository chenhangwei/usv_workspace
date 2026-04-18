#!/bin/bash
# fresh68 v2 续训脚本 (关机/中断恢复用)
# 用法: bash src/scripts/fresh68v2_resume.sh
#
# 自动行为:
#   1. 找到最新 stageB checkpoint (若没有则从 stageA.pt 全量重跑 120K)
#   2. 计算剩余 timesteps
#   3. 以最新 ckpt 续训 Stage B
#   4. 续训完成后跑最终 SITL gate

set -eo pipefail

REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh68_checkpoints"
STAGE_A_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageB.pt"
LOG_FILE="/tmp/fresh68v2_resume.log"
STAGE_B_TOTAL=120000

exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---------- 1. 找最新 stageB ckpt ----------
LATEST_B=$(ls -1 "$CKPT_DIR"/fresh68_stageB_step_*.pt 2>/dev/null | sort -V | tail -1 || true)

if [ -z "$LATEST_B" ]; then
  log "未找到 Stage B checkpoint，尝试 Stage A.pt 全量重跑 Stage B"
  if [ ! -f "$STAGE_A_OUT" ]; then
    log "ERROR: Stage A 输出 $STAGE_A_OUT 也不存在，无法续训"
    exit 1
  fi
  LOAD_FROM="$STAGE_A_OUT"
  DONE_STEPS=0
else
  LOAD_FROM="$LATEST_B"
  # 从文件名提取已训步数: fresh68_stageB_step_0056320.pt -> 56320
  DONE_STEPS=$(basename "$LATEST_B" | sed -E 's/.*step_0*([0-9]+)\.pt/\1/')
  log "找到最新 Stage B ckpt: $LATEST_B (已训 $DONE_STEPS steps)"
fi

REMAIN=$(( STAGE_B_TOTAL - DONE_STEPS ))
if [ $REMAIN -le 2000 ]; then
  log "剩余步数 $REMAIN ≤ 2000，直接复制为最终输出并跑 gate"
  cp "$LOAD_FROM" "$STAGE_B_OUT"
else
  log "剩余 $REMAIN steps，开始续训 Stage B"

  # ---------- 2. 续训 Stage B (复用 train_fresh68.sh 中的 stageB 参数) ----------
  # 直接调 train_mappo_policy，仅替换 load-weights-from 和 total-timesteps
  python3 -m usv_rl.train_mappo_policy \
    --output "$STAGE_B_OUT" \
    --load-weights-from "$LOAD_FROM" \
    --checkpoint-dir "$CKPT_DIR" \
    --num-agents 3 \
    --total-timesteps "$REMAIN" \
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
    --neighbor-attention --attention-embed-dim 32 --attention-num-heads 1 \
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
    --dr-tau-linear-low 0.4 --dr-tau-linear-high 1.0 \
    --dr-tau-angular-low 0.2 --dr-tau-angular-high 0.6 \
    --checkpoint-interval 2000 \
    --num-sampler-workers 2 \
    --base-ros-domain-id 185

  log "Stage B 续训完成 -> $STAGE_B_OUT"
fi

# ---------- 3. 最终 SITL gate ----------
log "==== 最终 SITL gate: fresh68_stageB.pt ===="
rm -rf /tmp/sitl_gate_fresh68_stageB
bash src/scripts/fresh64_sitl_gate.sh "$STAGE_B_OUT" > /tmp/fresh68v2_stageB_gate.log 2>&1 || true
log "最终 gate 完成"
tail -20 /tmp/fresh68v2_stageB_gate.log
