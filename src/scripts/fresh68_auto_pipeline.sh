#!/bin/bash
# fresh68 auto-pipeline: Stage A 训练完 → SITL gate → 自动判断 → Stage B / 修复重试
#
# 决策逻辑 (solo 场景为准):
#   PASS:  progress ≥ 0.40  AND  heading_error ≤ 0.50  →  启动 Stage B
#   FAIL-HOVER: progress < 0.40  AND  he > 0.50        →  policy 仍原地转
#       * progress_weight × 1.5
#       * heading_error_weight × 0.5
#       * stall_penalty × 1.3
#       * 重跑 Stage A (50K)
#   FAIL-SLOW: progress < 0.40  AND  he ≤ 0.50         →  前进但到不了目标
#       * time_penalty × 1.3
#       * goal_bonus × 1.3
#       * 重跑 Stage A (50K)
#   FAIL-ZIGZAG: progress ≥ 0.40  AND  he > 0.50       →  到达但摇摆
#       * heading_error_weight × 1.3
#       * 小改重跑 Stage A (50K)
#
# 最多 1 次自动重试。重试仍失败则停止并写诊断。

set -eo pipefail

REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"

STAGE_A_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageB.pt"
LOG_FILE="/tmp/fresh68_auto_pipeline.log"
GATE_DIR_A="/tmp/sitl_gate_fresh68_stageA"

exec > >(tee -a "$LOG_FILE") 2>&1

log() { echo "[$(date +%H:%M:%S)] $*"; }

wait_for_training() {
  local tag="$1"
  local logf="$2"
  log "等待 $tag 训练完成 (checkpoint: $STAGE_A_OUT)..."
  while true; do
    # 如果 checkpoint 存在且最近 60s 内无新 MAPPO update 日志，则认为完成
    if [[ -f "$STAGE_A_OUT" ]]; then
      # 检查是否还有 train_mappo 进程
      if ! pgrep -f "train_mappo_policy.*fresh68_stageA" > /dev/null; then
        log "✅ $tag 训练进程已退出, checkpoint 就绪"
        return 0
      fi
    fi
    sleep 30
  done
}

run_gate_stageA() {
  log "==== 运行 SITL gate: fresh68_stageA.pt ===="
  rm -rf "$GATE_DIR_A"
  bash src/scripts/fresh64_sitl_gate.sh "$STAGE_A_OUT" > /tmp/fresh68_stageA_gate.log 2>&1 || true
  log "Gate 完成, 日志: /tmp/fresh68_stageA_gate.log"
}

parse_solo_metrics() {
  # 输出: "progress he cte"
  local log=/tmp/fresh68_stageA_gate.log
  local progress he cte
  progress=$(grep -A3 "solo_navigation" "$log" | grep "progress:" | head -1 | grep -oE "[0-9]+\.[0-9]+" | head -1)
  he=$(grep "heading_error:" "$log" | head -1 | grep -oE "[0-9]+\.[0-9]+" | head -1)
  cte=$(grep "cross_track_err:" "$log" | head -1 | grep -oE "[0-9]+\.[0-9]+" | head -1)
  echo "${progress:-0} ${he:-99} ${cte:-99}"
}

decide_action() {
  local progress="$1" he="$2"
  local prog_ok he_ok
  prog_ok=$(awk -v p="$progress" 'BEGIN{print (p>=0.40)?1:0}')
  he_ok=$(awk -v h="$he" 'BEGIN{print (h<=0.50)?1:0}')

  if [[ "$prog_ok" == "1" && "$he_ok" == "1" ]]; then
    echo "PASS"
  elif [[ "$prog_ok" == "0" && "$he_ok" == "0" ]]; then
    echo "FAIL-HOVER"
  elif [[ "$prog_ok" == "0" && "$he_ok" == "1" ]]; then
    echo "FAIL-SLOW"
  else
    echo "FAIL-ZIGZAG"
  fi
}

retune_stageA() {
  local mode="$1"
  local script="$REPO/src/scripts/train_fresh68.sh"
  cp "$script" "${script}.retune_backup.$(date +%s)"
  log "==== 重调 Stage A 参数 ($mode) ===="

  python3 <<PYEOF
import re
path = "$script"
mode = "$mode"
src = open(path).read()

# 提取 run_stage_a() 函数体
m = re.search(r'run_stage_a\(\)\s*\{(.*?)\n\}\s*\n', src, re.DOTALL)
if not m:
    raise SystemExit("run_stage_a() not found")
body = m.group(1)
orig_body = body

def bump(body, flag, new_val):
    # 替换 "--flag <num>" 的数字
    return re.sub(r'(' + re.escape(flag) + r'\s+)[-\d.eE]+', r'\g<1>' + str(new_val), body, count=1)

if mode == "FAIL-HOVER":
    body = bump(body, '--progress-weight', '12.0')
    body = bump(body, '--heading-error-weight', '1.5')
    body = bump(body, '--stall-penalty', '-80.0')
    print("调整: progress_weight 8→12, heading_error_weight 3→1.5, stall_penalty -60→-80")
elif mode == "FAIL-SLOW":
    body = bump(body, '--time-penalty', '0.10')
    body = bump(body, '--goal-bonus', '45.0')
    print("调整: time_penalty 0.08→0.10, goal_bonus 35→45")
elif mode == "FAIL-ZIGZAG":
    body = bump(body, '--heading-error-weight', '4.0')
    print("调整: heading_error_weight 3→4")

src = src.replace(orig_body, body)
open(path, 'w').write(src)
PYEOF
}

run_stageA() {
  log "==== 启动 Stage A ===="
  rm -f "$STAGE_A_OUT"
  bash src/scripts/train_fresh68.sh stageA > /tmp/fresh68_stageA.log 2>&1
  log "Stage A 训练完成"
}

run_stageB() {
  log "==== 启动 Stage B (120K) ===="
  bash src/scripts/train_fresh68.sh stageB > /tmp/fresh68_stageB.log 2>&1
  log "Stage B 训练完成: $STAGE_B_OUT"
}

# ═══════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════

RETRY=0
MAX_RETRY=1

while true; do
  wait_for_training "Stage A (try=$RETRY)"
  run_gate_stageA

  read -r PROG HE CTE <<<"$(parse_solo_metrics)"
  ACTION=$(decide_action "$PROG" "$HE")
  log ">>> Stage A 指标: progress=$PROG  heading_error=$HE  CTE=$CTE  →  $ACTION"

  if [[ "$ACTION" == "PASS" ]]; then
    log "🎉 Stage A 通过, 进入 Stage B"
    run_stageB

    # Stage B 完成后跑最终 gate
    log "==== 最终评估: fresh68_stageB.pt ===="
    rm -rf /tmp/sitl_gate_fresh68_stageB
    bash src/scripts/fresh64_sitl_gate.sh "$STAGE_B_OUT" > /tmp/fresh68_stageB_gate.log 2>&1 || true
    log "最终 gate 完成, 日志: /tmp/fresh68_stageB_gate.log"
    tail -8 /tmp/fresh68_stageB_gate.log
    exit 0
  fi

  if (( RETRY >= MAX_RETRY )); then
    log "❌ 已达最大重试次数 ($MAX_RETRY), Stage A 仍失败 ($ACTION). 停止."
    log "人工介入: 查看 /tmp/fresh68_stageA_gate.log 与 $GATE_DIR_A/*.json"
    exit 1
  fi

  retune_stageA "$ACTION"
  RETRY=$((RETRY + 1))
  log "==== 重试 Stage A (round=$RETRY) ===="
  run_stageA
done
