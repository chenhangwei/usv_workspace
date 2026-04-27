#!/bin/bash
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh84_full5_eval
BASE_DOMAIN=${BASE_DOMAIN:-200}
mkdir -p "$EVAL_DIR"

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /home/chenhangwei/usv_workspace/src/scripts/fresh81_router_config.json
  /mnt/data/checkpoints/usv_rl/fresh83_overtaking_residual.pt
)
POLICIES=(
  mappo
  mappo_router
  mappo
)

for checkpoint in "$@"; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
  POLICIES+=(mappo)
done

LOG=/tmp/fresh84_quickeval.log
: > "$LOG"

for idx in "${!CANDIDATES[@]}"; do
  model="${CANDIDATES[$idx]}"
  policy="${POLICIES[$idx]}"
  if [[ ! -f "$model" ]]; then
    echo "[$(date +%T)] skip missing: $model" | tee -a "$LOG"
    continue
  fi
  base=$(basename "$model")
  base="${base%.*}"
  out="$EVAL_DIR/${base}_full5_quickeval.json"
  domain=$((BASE_DOMAIN + idx))
  if (( domain > 232 )); then
    domain=$((domain % 233))
  fi
  echo "[$(date +%T)] eval $base policy=$policy (domain=$domain)" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 240 python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy "$policy" \
    --model "$model" \
    --episodes 5 \
    --steps-per-episode 300 \
    --device cpu \
    --episode-timeout 70 \
    --no-progress-timeout 20 \
    --output-json "$out" \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter \
    > "/tmp/fresh84_quickeval_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh84_quickeval_${base}.log)" | tee -a "$LOG"
  else
    echo "[$(date +%T)]   ok  $base" | tee -a "$LOG"
  fi
done

echo "[$(date +%T)] all done." | tee -a "$LOG"