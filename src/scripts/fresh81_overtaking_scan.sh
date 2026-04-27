#!/bin/bash
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh81_overtaking_scan
RANKING_JSON="$EVAL_DIR/ranking.json"
EPISODES=${EPISODES:-5}
STEPS=${STEPS:-300}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-70}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-20}

mkdir -p "$EVAL_DIR"

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /mnt/data/checkpoints/usv_rl/fresh74_best.pt
  /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0021241.pt
  /mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0012446.pt
  /mnt/data/checkpoints/usv_rl/fresh73_checkpoints/fresh73_stabilize_step_0004775.pt
  /mnt/data/checkpoints/usv_rl/fresh73_checkpoints/fresh73_stabilize_step_0009149.pt
)

LOG=/tmp/fresh81_overtaking_scan.log
: > "$LOG"

idx=0
for model in "${CANDIDATES[@]}"; do
  if [[ ! -f "$model" ]]; then
    echo "[$(date +%T)] skip missing: $model" | tee -a "$LOG"
    idx=$((idx + 1))
    continue
  fi

  base=$(basename "$model" .pt)
  out="$EVAL_DIR/${base}_overtaking_eval.json"
  domain=$((150 + idx))

  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip cached: $base" | tee -a "$LOG"
    idx=$((idx + 1))
    continue
  fi

  echo "[$(date +%T)] eval $base (domain=$domain)" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 180 python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$model" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$out" \
    --scenario three_usv_overtaking \
    > "/tmp/fresh81_overtaking_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh81_overtaking_${base}.log)" | tee -a "$LOG"
  else
    python3 - <<PY | tee -a "$LOG"
import json
with open('$out') as f:
    d = json.load(f)
summary = d['scenario_summaries']['three_usv_overtaking']
print(
    f"[$(date +%T)]   ok  {summary['collision_rate']:.3f} coll "
    f"{summary['success_rate']:.3f} succ "
    f"{summary['mean_team_goal_progress_ratio']:.3f} prog "
    f"{summary['worst_episode_min_separation']:.3f} sep"
)
PY
  fi

  idx=$((idx + 1))
done

python3 - <<'PY'
import glob
import json
import os

eval_dir = '/mnt/data/checkpoints/usv_rl/fresh81_overtaking_scan'
ranking_path = os.path.join(eval_dir, 'ranking.json')
results = []

for path in sorted(glob.glob(os.path.join(eval_dir, '*_overtaking_eval.json'))):
    with open(path) as f:
        payload = json.load(f)
    summary = payload['scenario_summaries']['three_usv_overtaking']
    results.append({
    'model': payload.get('model', path),
        'source_json': path,
        'collision_rate': summary['collision_rate'],
        'success_rate': summary['success_rate'],
        'timeout_rate': summary['timeout_rate'],
        'mean_team_goal_progress_ratio': summary['mean_team_goal_progress_ratio'],
        'worst_episode_min_separation': summary['worst_episode_min_separation'],
        'mean_omega_flip_count': summary['mean_omega_flip_count'],
        'mean_entanglement_ratio': summary['mean_entanglement_ratio'],
    })

results.sort(
    key=lambda item: (
        item['collision_rate'],
        -item['success_rate'],
        item['timeout_rate'],
        -item['mean_team_goal_progress_ratio'],
        -item['worst_episode_min_separation'],
        item['mean_omega_flip_count'],
    )
)

ranking = {
    'scenario': 'three_usv_overtaking',
    'episodes': int(os.environ.get('EPISODES', '5')),
    'steps_per_episode': int(os.environ.get('STEPS', '300')),
    'episode_timeout': int(os.environ.get('EPISODE_TIMEOUT', '70')),
    'no_progress_timeout': int(os.environ.get('NO_PROGRESS_TIMEOUT', '20')),
    'candidates': results,
    'best_model': results[0]['model'] if results else None,
}

with open(ranking_path, 'w') as f:
    json.dump(ranking, f, indent=2)

print(f'Ranking saved to {ranking_path}')
if results:
    best = results[0]
    print(
        'Best:',
        best['model'],
        'coll', best['collision_rate'],
        'succ', best['success_rate'],
        'prog', best['mean_team_goal_progress_ratio'],
        'sep', best['worst_episode_min_separation'],
    )
PY