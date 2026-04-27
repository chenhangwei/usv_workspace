#!/bin/bash
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh85_random_eval
BASE_DOMAIN=${BASE_DOMAIN:-210}
mkdir -p "$EVAL_DIR"

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /mnt/data/checkpoints/usv_rl/fresh83_overtaking_residual.pt
  /mnt/data/checkpoints/usv_rl/fresh84_overtaking_scenario_residual.pt
)
POLICIES=(
  mappo
  mappo
  mappo
)

for checkpoint in "$@"; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
  POLICIES+=(mappo)
done

LOG=/tmp/fresh85_random_eval.log
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
  out="$EVAL_DIR/${base}_random_eval.json"
  domain=$((BASE_DOMAIN + idx))
  if (( domain > 232 )); then
    domain=$((domain % 233))
  fi
  echo "[$(date +%T)] eval $base policy=$policy random_encounter (domain=$domain)" | tee -a "$LOG"
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
    --scenario three_usv_random_encounter \
    > "/tmp/fresh85_random_eval_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh85_random_eval_${base}.log)" | tee -a "$LOG"
  else
    python3 - <<PY | tee -a "$LOG"
import json
with open('$out') as handle:
    payload = json.load(handle)
summary = payload['scenario_summaries']['three_usv_random_encounter']
print(
    '[$(date +%T)]   ok '
    f"coll={summary['collision_rate']:.3f} "
    f"succ={summary['success_rate']:.3f} "
    f"timeout={summary['timeout_rate']:.3f} "
    f"progress={summary['mean_team_goal_progress_ratio']:.3f} "
    f"sep={summary['worst_episode_min_separation']:.3f} "
    f"omega={summary['mean_omega_flip_count']:.1f}"
)
PY
  fi
done

echo "[$(date +%T)] all done." | tee -a "$LOG"