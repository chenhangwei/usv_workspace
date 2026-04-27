#!/bin/bash
# fresh77 quick_eval — full 5-scenario eval for fresh77 stabilize run
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh77_eval
mkdir -p "$EVAL_DIR"

CKPT_DIR=/mnt/data/checkpoints/usv_rl/fresh77_checkpoints
CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh76_best.pt
  /mnt/data/checkpoints/usv_rl/fresh76_headon_focus.pt
  /mnt/data/checkpoints/usv_rl/fresh77_stabilize.pt
)
for f in "$CKPT_DIR"/fresh77_stabilize_step_*.pt; do
  CANDIDATES+=("$f")
done

LOG=/tmp/fresh77_quickeval.log
: > "$LOG"

idx=0
for model in "${CANDIDATES[@]}"; do
  if [[ ! -f "$model" ]]; then
    idx=$((idx+1)); continue
  fi
  base=$(basename "$model" .pt)
  out="$EVAL_DIR/${base}_full5_quickeval.json"
  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip (cached): $base" | tee -a "$LOG"
    idx=$((idx+1))
    continue
  fi
  domain=$((150 + idx))
  if [[ $domain -gt 232 ]]; then
    echo "[$(date +%T)] WARN domain $domain > 232, wrap" | tee -a "$LOG"
    domain=$(( ( (domain-150) % 70 ) + 150 ))
  fi
  echo "[$(date +%T)] eval $base (domain=$domain)" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 180 python3 -u -m usv_rl.evaluate_mappo_policy \
    --model "$model" --episodes 5 --steps-per-episode 300 --device cpu \
    --output-json "$out" \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter \
    > "/tmp/fresh77_quickeval_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh77_quickeval_${base}.log)" | tee -a "$LOG"
  else
    echo "[$(date +%T)]   ok  $base" | tee -a "$LOG"
  fi
  idx=$((idx+1))
done

echo "[$(date +%T)] all done." | tee -a "$LOG"