#!/bin/bash
# fresh68 simple pipeline (方案 A):
#   1. Stage A 80K (恢复 Round 0 参数, 仅延长训练量)
#   2. SITL gate (只看, 不重试; 无论结果都进 Stage B 让数据说话)
#   3. Stage B 120K
#   4. 最终 SITL gate
# 不自动调参，避免破坏 Round 0 已验证的超低 he/CTE。

set -eo pipefail

REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"

STAGE_A_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageA.pt"
STAGE_B_OUT="/mnt/data/checkpoints/usv_rl/fresh68_stageB.pt"
LOG_FILE="/tmp/fresh68v2_pipeline.log"

exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date +%H:%M:%S)] $*"; }

log "==== 启动 Stage A 80K (Round 0 参数) ===="
rm -f "$STAGE_A_OUT"
bash src/scripts/train_fresh68.sh stageA > /tmp/fresh68v2_stageA.log 2>&1
log "Stage A 训练完成"

log "==== Stage A SITL gate ===="
rm -rf /tmp/sitl_gate_fresh68_stageA
bash src/scripts/fresh64_sitl_gate.sh "$STAGE_A_OUT" > /tmp/fresh68v2_stageA_gate.log 2>&1 || true
log "Stage A gate 完成. 关键指标:"
grep -E "solo_navigation|progress:|heading_error:|cross_track_err:" /tmp/fresh68v2_stageA_gate.log | head -10

log "==== 启动 Stage B 120K (load Stage A) ===="
bash src/scripts/train_fresh68.sh stageB > /tmp/fresh68v2_stageB.log 2>&1
log "Stage B 训练完成"

log "==== 最终 SITL gate: fresh68_stageB.pt ===="
rm -rf /tmp/sitl_gate_fresh68_stageB
bash src/scripts/fresh64_sitl_gate.sh "$STAGE_B_OUT" > /tmp/fresh68v2_stageB_gate.log 2>&1 || true
log "最终 gate 完成"
tail -15 /tmp/fresh68v2_stageB_gate.log
