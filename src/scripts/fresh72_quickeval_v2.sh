#!/bin/bash
# fresh72 quick_eval v2 — per-candidate unique ROS domain + IPC cleanup
# Avoids the dirty FastDDS shared-memory crash seen in v1.
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh72_eval
mkdir -p "$EVAL_DIR"

CKPT_DIR=/mnt/data/checkpoints/usv_rl/fresh72_checkpoints
CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh70_stageB.pt
  /mnt/data/checkpoints/usv_rl/fresh72_sprint.pt
)
for f in "$CKPT_DIR"/fresh72_sprint_step_*.pt; do
  CANDIDATES+=("$f")
done

SCENARIOS=(solo_navigation two_usv_head_on three_usv_crossing three_usv_overtaking three_usv_random_encounter)

LOG=/tmp/fresh72_quickeval_v2.log
: > "$LOG"

idx=0
for model in "${CANDIDATES[@]}"; do
  base=$(basename "$model" .pt)
  out="$EVAL_DIR/${base}_quickeval.json"
  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip (cached): $base" | tee -a "$LOG"
    idx=$((idx+1))
    continue
  fi
  domain=$((200 + idx))
  echo "[$(date +%T)] eval $base (domain=$domain)" | tee -a "$LOG"
  # clean IPC
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 180 python3 -u -m usv_rl.evaluate_mappo_policy \
    --model "$model" --episodes 5 --steps-per-episode 275 --device cpu \
    --output-json "$out" \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter \
    > "/tmp/fresh72_quickeval_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh72_quickeval_${base}.log)" | tee -a "$LOG"
  else
    echo "[$(date +%T)]   ok  $base" | tee -a "$LOG"
  fi
  idx=$((idx+1))
done

echo "[$(date +%T)] all done. files:" | tee -a "$LOG"
ls "$EVAL_DIR" | tee -a "$LOG"
