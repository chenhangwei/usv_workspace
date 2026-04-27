#!/bin/bash
# fresh87 full-5 quick evaluation for all checkpoints.
# Note: --episodes 25 gives 5 rollouts per listed scenario.

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

EVAL_DIR=/mnt/data/checkpoints/usv_rl/fresh87_full5_eval
CKPT_DIR=/mnt/data/checkpoints/usv_rl/fresh87_checkpoints
BASE_DOMAIN=${BASE_DOMAIN:-180}
EPISODES=${EPISODES:-25}
STEPS=${STEPS:-300}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-70}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-20}
mkdir -p "$EVAL_DIR"

CANDIDATES=(
  /mnt/data/checkpoints/usv_rl/fresh79_best.pt
  /mnt/data/checkpoints/usv_rl/fresh83_overtaking_residual.pt
  /mnt/data/checkpoints/usv_rl/fresh86_overtaking_scenario_trunk.pt
  /mnt/data/checkpoints/usv_rl/fresh87_full_scenario_trunk.pt
)
for checkpoint in "$CKPT_DIR"/fresh87_full_scenario_trunk_step_*.pt; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
done
for checkpoint in "$@"; do
  [[ -f "$checkpoint" ]] || continue
  CANDIDATES+=("$checkpoint")
done

LOG=/tmp/fresh87_quickeval.log
: > "$LOG"

idx=0
for model in "${CANDIDATES[@]}"; do
  if [[ ! -f "$model" ]]; then
    idx=$((idx + 1))
    continue
  fi
  base=$(basename "$model" .pt)
  out="$EVAL_DIR/${base}_full5_eval.json"
  if [[ -f "$out" ]]; then
    echo "[$(date +%T)] skip cached: $base" | tee -a "$LOG"
    idx=$((idx + 1))
    continue
  fi
  domain=$((BASE_DOMAIN + idx))
  if (( domain > 232 )); then
    domain=$((180 + (idx % 40)))
  fi
  echo "[$(date +%T)] eval $base domain=$domain episodes=$EPISODES" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ROS_DOMAIN_ID=$domain timeout 420 python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$model" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$out" \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter \
    > "/tmp/fresh87_quickeval_${base}.log" 2>&1
  rc=$?
  if [[ $rc -ne 0 || ! -f "$out" ]]; then
    echo "[$(date +%T)]   FAIL $base rc=$rc (see /tmp/fresh87_quickeval_${base}.log)" | tee -a "$LOG"
    rm -f "$out"
  else
    python3 - <<PY | tee -a "$LOG"
import json
with open('$out', encoding='utf-8') as handle:
    payload = json.load(handle)
print(
    "[$(date +%T)]   ok "
    f"coll={payload.get('collision_rate', 1.0):.3f} "
    f"succ={payload.get('success_rate', 0.0):.3f} "
    f"timeout={payload.get('timeout_rate', 0.0):.3f} "
  f"progress={payload.get('mean_team_goal_progress_ratio', 0.0):.3f} "
  f"sep={payload.get('worst_episode_min_separation', 0.0):.3f} "
  f"ent={payload.get('mean_entanglement_ratio', 1.0):.3f} "
  f"omega={payload.get('mean_omega_flip_count', 999.0):.1f}"
)
PY
  fi
  idx=$((idx + 1))
done

python3 - <<'PY'
import glob
import json
import os

eval_dir = '/mnt/data/checkpoints/usv_rl/fresh87_full5_eval'
ranking_path = os.path.join(eval_dir, 'ranking.json')
rows = []
for path in sorted(glob.glob(os.path.join(eval_dir, '*_full5_eval.json'))):
    with open(path, encoding='utf-8') as handle:
        payload = json.load(handle)
    scenarios = payload.get('scenario_summaries', {}) or {}
    crossing = scenarios.get('three_usv_crossing', {})
    head_on = scenarios.get('two_usv_head_on', {})
    overtaking = scenarios.get('three_usv_overtaking', {})
    random_enc = scenarios.get('three_usv_random_encounter', {})
    rows.append({
        'model': payload.get('model', path),
        'source_json': path,
        'collision_rate': payload.get('collision_rate', 1.0),
        'success_rate': payload.get('success_rate', 0.0),
        'timeout_rate': payload.get('timeout_rate', 1.0),
        'progress': payload.get('mean_team_goal_progress_ratio', 0.0),
      'worst_episode_min_separation': payload.get('worst_episode_min_separation', 0.0),
      'mean_entanglement_ratio': payload.get('mean_entanglement_ratio', 1.0),
      'mean_colregs_violation_ratio': payload.get('mean_colregs_violation_ratio', 1.0),
      'head_on_collision': head_on.get('collision_rate', 1.0),
        'crossing_collision': crossing.get('collision_rate', 1.0),
        'overtaking_timeout': overtaking.get('timeout_rate', 1.0),
        'random_collision': random_enc.get('collision_rate', 1.0),
        'mean_omega_flip_count': payload.get('mean_omega_flip_count', 999.0),
    })
rows.sort(key=lambda item: (
    item['collision_rate'],
    item['head_on_collision'],
    item['crossing_collision'],
    item['random_collision'],
    item['timeout_rate'],
    item['overtaking_timeout'],
    item['mean_entanglement_ratio'],
    item['mean_colregs_violation_ratio'],
    item['mean_omega_flip_count'],
    -item['success_rate'],
    -item['progress'],
    -item['worst_episode_min_separation'],
))
with open(ranking_path, 'w', encoding='utf-8') as handle:
    json.dump({'candidates': rows, 'best_model': rows[0]['model'] if rows else None}, handle, ensure_ascii=False, indent=2)
print(f'Ranking saved to {ranking_path}')
if rows:
    print('Best:', rows[0]['model'])
PY
