#!/bin/bash
# fresh70 pipeline: Stage B 重训 (修复 omega_flip 暴涨) + 最终 SITL gate
set -eo pipefail

REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"

STAGE_A_IN="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUT="/mnt/data/checkpoints/usv_rl/fresh70_stageB.pt"
LOG_FILE="/tmp/fresh70_pipeline.log"

exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date +%H:%M:%S)] $*"; }

if [ ! -f "$STAGE_A_IN" ]; then
  log "ERROR: $STAGE_A_IN 不存在"
  exit 1
fi

log "==== 启动 fresh70 Stage B 120K (load fresh68_stageA.pt) ===="
rm -f "$STAGE_B_OUT"
bash src/scripts/train_fresh70.sh stageB > /tmp/fresh70_stageB.log 2>&1
log "Stage B 训练完成 -> $STAGE_B_OUT"

log "==== 最终 SITL gate: fresh70_stageB.pt ===="
rm -rf /tmp/sitl_gate_fresh70_stageB
bash src/scripts/fresh64_sitl_gate.sh "$STAGE_B_OUT" > /tmp/fresh70_stageB_gate.log 2>&1 || true
log "最终 gate 完成. 关键摘要:"
grep -E "Gate Results|SCENARIO|progress:|heading_error:|cross_track_err:|collision_rate:|worst_min_sep:|omega_flip:" /tmp/fresh70_stageB_gate.log | tail -60
