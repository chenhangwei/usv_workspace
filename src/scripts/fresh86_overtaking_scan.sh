#!/bin/bash
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh86_overtaking_scan
RANKING_JSON="$EVAL_DIR/ranking.json"
EPISODES=${EPISODES:-5}
STEPS=${STEPS:-300}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-70}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-20}

mkdir -p "$EVAL_DIR"

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /home/chenhangwei/usv_workspace/src/scripts/fresh81_router_config.json
)
POLICIES=(
  mappo
  mappo_router
)

for checkpoint in /mnt/data/checkpoints/usv_rl/fresh86_checkpoints/fresh86_overtaking_scenario_trunk_step_*.pt; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
  POLICIES+=(mappo)
done

CANDIDATES+=(/mnt/data/checkpoints/usv_rl/fresh86_overtaking_scenario_trunk.pt)
POLICIES+=(mappo)

LOG=/tmp/fresh86_overtaking_scan.log
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
  out="$EVAL_DIR/${base}_overtaking_eval.json"
  domain=$((170 + idx))

  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip cached: $base" | tee -a "$LOG"
    continue
  fi

  echo "[$(date +%T)] eval $base policy=$policy (domain=$domain)" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 240 python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy "$policy" \
    --model "$model" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$out" \
    --scenario three_usv_overtaking \
    > "/tmp/${base}_fresh86_overtaking_eval.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/${base}_fresh86_overtaking_eval.log)" | tee -a "$LOG"
  else
    python3 - <<PY | tee -a "$LOG"
import json
with open('$out') as handle:
    payload = json.load(handle)
summary = payload['scenario_summaries']['three_usv_overtaking']
print(
    '[$(date +%T)]   ok '
    f"coll={summary['collision_rate']:.3f} "
    f"succ={summary['success_rate']:.3f} "
    f"timeout={summary['timeout_rate']:.3f} "
    f"progress={summary['mean_team_goal_progress_ratio']:.3f} "
    f"sep={summary['worst_episode_min_separation']:.3f}"
)
PY
  fi
done

python3 - <<'PY'
import glob
import json
import os

eval_dir = '/mnt/data/checkpoints/usv_rl/fresh86_overtaking_scan'
ranking_path = os.path.join(eval_dir, 'ranking.json')
results = []

for path in sorted(glob.glob(os.path.join(eval_dir, '*_overtaking_eval.json'))):
    with open(path) as handle:
        payload = json.load(handle)
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

with open(ranking_path, 'w', encoding='utf-8') as handle:
    json.dump({
        'scenario': 'three_usv_overtaking',
        'episodes': int(os.environ.get('EPISODES', '5')),
        'steps_per_episode': int(os.environ.get('STEPS', '300')),
        'episode_timeout': int(os.environ.get('EPISODE_TIMEOUT', '70')),
        'no_progress_timeout': int(os.environ.get('NO_PROGRESS_TIMEOUT', '20')),
        'candidates': results,
        'best_model': results[0]['model'] if results else None,
    }, handle, indent=2)

print(f'Ranking saved to {ranking_path}')
if results:
    best = results[0]
    print(
        'Best:', best['model'],
        'coll', best['collision_rate'],
        'succ', best['success_rate'],
        'timeout', best['timeout_rate'],
        'progress', best['mean_team_goal_progress_ratio'],
        'sep', best['worst_episode_min_separation'],
    )
PY