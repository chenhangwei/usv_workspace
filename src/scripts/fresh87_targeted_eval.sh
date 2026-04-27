#!/bin/bash
# fresh87 targeted evaluation for difficult scenarios.
# Usage:
#   bash src/scripts/fresh87_targeted_eval.sh [model ...]

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh87_targeted_eval
BASE_DOMAIN=${BASE_DOMAIN:-190}
EPISODES=${EPISODES:-15}
STEPS=${STEPS:-300}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-70}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-20}
mkdir -p "$EVAL_DIR"

SCENARIOS=(
  three_usv_crossing
  three_usv_overtaking
  three_usv_random_encounter
)

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /mnt/data/checkpoints/usv_rl/fresh86_checkpoints/fresh86_overtaking_scenario_trunk_step_0001152.pt
  /mnt/data/checkpoints/usv_rl/fresh87_full_scenario_trunk.pt
)
for checkpoint in /mnt/data/checkpoints/usv_rl/fresh87_checkpoints/fresh87_full_scenario_trunk_step_*.pt; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
done
for checkpoint in "$@"; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
done

LOG=/tmp/fresh87_targeted_eval.log
: > "$LOG"
idx=0
for model in "${CANDIDATES[@]}"; do
  [[ -f "$model" ]] || { idx=$((idx + 1)); continue; }
  base=$(basename "$model" .pt)
  for scenario in "${SCENARIOS[@]}"; do
    out="$EVAL_DIR/${base}_${scenario}_eval.json"
    if [[ -f "$out" ]]; then
      echo "[$(date +%T)] skip cached: $base $scenario" | tee -a "$LOG"
      continue
    fi
    domain=$((BASE_DOMAIN + idx))
    if (( domain > 232 )); then
      domain=$((190 + (idx % 35)))
    fi
    echo "[$(date +%T)] eval $base scenario=$scenario domain=$domain episodes=$EPISODES" | tee -a "$LOG"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    ROS_DOMAIN_ID=$domain timeout 360 python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$model" \
      --episodes "$EPISODES" \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --output-json "$out" \
      --scenario "$scenario" \
      > "/tmp/fresh87_targeted_${base}_${scenario}.log" 2>&1
    rc=$?
    if [[ $rc -ne 0 || ! -f "$out" ]]; then
      echo "[$(date +%T)]   FAIL $base $scenario rc=$rc" | tee -a "$LOG"
      rm -f "$out"
    else
      python3 - <<PY | tee -a "$LOG"
import json
with open('$out', encoding='utf-8') as handle:
    payload = json.load(handle)
summary = payload['scenario_summaries'].get('$scenario', payload)
print(
    "[$(date +%T)]   ok "
    f"coll={summary.get('collision_rate', 1.0):.3f} "
    f"succ={summary.get('success_rate', 0.0):.3f} "
    f"timeout={summary.get('timeout_rate', 0.0):.3f} "
    f"progress={summary.get('mean_team_goal_progress_ratio', 0.0):.3f} "
    f"sep={summary.get('worst_episode_min_separation', 0.0):.3f} "
    f"ent={summary.get('mean_entanglement_ratio', 1.0):.3f} "
    f"colreg_v={summary.get('mean_colregs_violation_ratio', 1.0):.3f} "
    f"omega={summary.get('mean_omega_flip_count', 999.0):.1f}"
)
PY
    fi
    idx=$((idx + 1))
  done
done

python3 - <<'PY'
import glob
import json
import os

rows = []
eval_dir = '/mnt/data/checkpoints/usv_rl/fresh87_targeted_eval'
for path in sorted(glob.glob(os.path.join(eval_dir, '*_eval.json'))):
    with open(path, encoding='utf-8') as handle:
        payload = json.load(handle)
    scenario_summaries = payload.get('scenario_summaries', {}) or {}
    if scenario_summaries:
        scenario, summary = next(iter(scenario_summaries.items()))
    else:
        scenario, summary = 'unknown', payload
    rows.append({
        'model': payload.get('model', path),
        'scenario': scenario,
        'collision_rate': summary.get('collision_rate', 1.0),
        'success_rate': summary.get('success_rate', 0.0),
        'timeout_rate': summary.get('timeout_rate', 1.0),
        'progress': summary.get('mean_team_goal_progress_ratio', 0.0),
        'worst_sep': summary.get('worst_episode_min_separation', 0.0),
        'entanglement': summary.get('mean_entanglement_ratio', 1.0),
        'colregs_violation': summary.get('mean_colregs_violation_ratio', 1.0),
        'omega_flip': summary.get('mean_omega_flip_count', 999.0),
        'source_json': path,
    })
rows.sort(key=lambda item: (
    item['model'],
    item['scenario'],
))

by_model = {}
for row in rows:
    bucket = by_model.setdefault(row['model'], [])
    bucket.append(row)
model_scores = []
for model, items in by_model.items():
    count = max(1, len(items))
    scenario_lookup = {item['scenario']: item for item in items}
    crossing = scenario_lookup.get('three_usv_crossing', {})
    overtaking = scenario_lookup.get('three_usv_overtaking', {})
    random_enc = scenario_lookup.get('three_usv_random_encounter', {})
    model_scores.append({
        'model': model,
        'scenarios': sorted(scenario_lookup),
        'mean_collision_rate': sum(item['collision_rate'] for item in items) / count,
        'mean_success_rate': sum(item['success_rate'] for item in items) / count,
        'mean_timeout_rate': sum(item['timeout_rate'] for item in items) / count,
        'mean_progress': sum(item['progress'] for item in items) / count,
        'worst_sep': min(item['worst_sep'] for item in items),
        'mean_entanglement': sum(item['entanglement'] for item in items) / count,
        'mean_colregs_violation': sum(item['colregs_violation'] for item in items) / count,
        'mean_omega_flip': sum(item['omega_flip'] for item in items) / count,
        'crossing_collision': crossing.get('collision_rate', 1.0),
        'random_collision': random_enc.get('collision_rate', 1.0),
        'overtaking_timeout': overtaking.get('timeout_rate', 1.0),
    })
model_scores.sort(key=lambda item: (
    item['mean_collision_rate'],
    item['crossing_collision'],
    item['random_collision'],
    item['mean_timeout_rate'],
    item['overtaking_timeout'],
    item['mean_entanglement'],
    item['mean_colregs_violation'],
    item['mean_omega_flip'],
    -item['mean_success_rate'],
    -item['mean_progress'],
    -item['worst_sep'],
))
ranking_path = os.path.join(eval_dir, 'summary.json')
with open(ranking_path, 'w', encoding='utf-8') as handle:
    json.dump({
        'results': rows,
        'model_scores': model_scores,
        'best_model': model_scores[0]['model'] if model_scores else None,
    }, handle, ensure_ascii=False, indent=2)
print(f'Summary saved to {ranking_path}')
if model_scores:
    print('Best targeted:', model_scores[0]['model'])
PY
