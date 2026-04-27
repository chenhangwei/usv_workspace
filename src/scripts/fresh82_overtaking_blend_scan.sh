#!/bin/bash
set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

BASE_MODEL=/mnt/data/checkpoints/usv_rl/fresh79_best.pt
OVERTAKING_EXPERT=/mnt/data/checkpoints/usv_rl/fresh72_checkpoints/fresh72_sprint_step_0012446.pt
CONFIG_DIR=/mnt/data/checkpoints/usv_rl/fresh82_blend_configs
EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh82_overtaking_scan
LOG=/tmp/fresh82_overtaking_blend_scan.log

mkdir -p "$CONFIG_DIR" "$EVAL_DIR"
: > "$LOG"

WEIGHTS=(0.25 0.50 0.75)

for weight in "${WEIGHTS[@]}"; do
  label=${weight/./}
  config="$CONFIG_DIR/fresh82_overtaking_blend_w${label}.json"
  BASE_MODEL="$BASE_MODEL" OVERTAKING_EXPERT="$OVERTAKING_EXPERT" WEIGHT="$weight" CONFIG_PATH="$config" python3 - <<'PY'
import json
import os

weight = float(os.environ['WEIGHT'])
config = {
    'policy_type': 'mappo_router',
    'env_template': os.environ['BASE_MODEL'],
    'default_model': os.environ['BASE_MODEL'],
    'scenario_blends': {
        'three_usv_overtaking': [
            {'model': os.environ['BASE_MODEL'], 'weight': 1.0 - weight},
            {'model': os.environ['OVERTAKING_EXPERT'], 'weight': weight},
        ]
    },
}
with open(os.environ['CONFIG_PATH'], 'w', encoding='utf-8') as handle:
    json.dump(config, handle, indent=2)
PY
done

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /home/chenhangwei/usv_workspace/src/scripts/fresh81_router_config.json
)
POLICIES=(
  mappo
  mappo_router
)
for weight in "${WEIGHTS[@]}"; do
  label=${weight/./}
  CANDIDATES+=("$CONFIG_DIR/fresh82_overtaking_blend_w${label}.json")
  POLICIES+=(mappo_router)
done

for idx in "${!CANDIDATES[@]}"; do
  model="${CANDIDATES[$idx]}"
  policy="${POLICIES[$idx]}"
  base=$(basename "$model")
  base="${base%.*}"
  out="$EVAL_DIR/${base}_overtaking_eval.json"
  domain=$((160 + idx))

  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip cached: $base" | tee -a "$LOG"
    continue
  fi

  echo "[$(date +%T)] eval $base policy=$policy (domain=$domain)" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 180 python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy "$policy" \
    --model "$model" \
    --episodes 5 \
    --steps-per-episode 300 \
    --device cpu \
    --episode-timeout 70 \
    --no-progress-timeout 20 \
    --output-json "$out" \
    --scenario three_usv_overtaking \
    > "/tmp/${base}_overtaking_eval.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc" | tee -a "$LOG"
  else
    python3 - <<PY | tee -a "$LOG"
import json
with open('$out') as f:
    payload = json.load(f)
summary = payload['scenario_summaries']['three_usv_overtaking']
print(
    '[$(date +%T)]   ok '
    f"coll={summary['collision_rate']:.3f} "
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

eval_dir = '/mnt/data/checkpoints/usv_rl/fresh82_overtaking_scan'
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
        item['timeout_rate'],
        -item['mean_team_goal_progress_ratio'],
        -item['worst_episode_min_separation'],
        item['mean_omega_flip_count'],
    )
)

with open(ranking_path, 'w', encoding='utf-8') as handle:
    json.dump({'candidates': results, 'best_model': results[0]['model'] if results else None}, handle, indent=2)

print(f'Ranking saved to {ranking_path}')
if results:
    best = results[0]
    print(
        'Best:', best['model'],
        'coll', best['collision_rate'],
        'timeout', best['timeout_rate'],
        'progress', best['mean_team_goal_progress_ratio'],
        'sep', best['worst_episode_min_separation'],
    )
PY