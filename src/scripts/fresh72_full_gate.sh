#!/bin/bash
# fresh72 full SITL gate on top-3 + baseline
set -o pipefail
cd "$(dirname "$0")/.."

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0021241.pt
  /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0009369.pt
  /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0012446.pt
  /mnt/data/checkpoints/usv_rl/fresh70_stageB.pt
)

LOG=/tmp/fresh72_full_gate.log
: > "$LOG"

i=0
for m in "${CANDIDATES[@]}"; do
  name=$(basename "$m" .pt)
  echo "==================================================" | tee -a "$LOG"
  echo "[$(date +%T)] Gate $((i+1))/${#CANDIDATES[@]}: $name" | tee -a "$LOG"
  echo "==================================================" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  bash scripts/fresh64_sitl_gate.sh "$m" 2>&1 | tee -a "$LOG" | grep -E "PASS|FAIL|^\[|collision|progress|sep|omega|heading|cross_track|场景|总结|Gate" | tail -60
  echo "[$(date +%T)] done $name" | tee -a "$LOG"
  i=$((i+1))
done
echo "[$(date +%T)] ==== ALL DONE ====" | tee -a "$LOG"
