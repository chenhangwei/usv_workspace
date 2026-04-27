#!/bin/bash
# fresh99 crossing-only triage. Do not run expensive selected eval unless this finds a crossing breakthrough.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash
cd src

set -u

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

OUT=${OUT:-/mnt/data/checkpoints/usv_rl/fresh99_crossing_focus_eval}
LOG=${LOG:-/tmp/fresh99_crossing_focus_eval.log}
EPISODES=${EPISODES:-5}
STEPS=${STEPS:-360}
EVAL_TIMEOUT=${EVAL_TIMEOUT:-330}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-96}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-32}
BASE_DOMAIN=${BASE_DOMAIN:-173}
FRESH99_LATEST_COUNT=${FRESH99_LATEST_COUNT:-18}
FRESH98_LATEST_COUNT=${FRESH98_LATEST_COUNT:-4}
INCLUDE_BASELINE=${INCLUDE_BASELINE:-1}

mkdir -p "$OUT"
: > "$LOG"

CANDIDATES=()
add_candidate() {
  local model="$1"
  [[ -n "$model" && -f "$model" ]] || return 0
  for existing in "${CANDIDATES[@]}"; do
    [[ "$existing" == "$model" ]] && return 0
  done
  CANDIDATES+=("$model")
}

if [[ "$INCLUDE_BASELINE" == "1" ]]; then
  add_candidate /mnt/data/checkpoints/usv_rl/fresh98_checkpoints/fresh98_pairwise_conflict_repair_step_0062784.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh98_pairwise_conflict_repair.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_0040896.pt
fi
add_candidate /mnt/data/checkpoints/usv_rl/fresh99_crossing_role_imitation.pt

mapfile -t latest_fresh99 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh99_checkpoints/fresh99_crossing_role_imitation_step_*.pt 2>/dev/null | head -n "$FRESH99_LATEST_COUNT")
for model in "${latest_fresh99[@]}"; do
  add_candidate "$model"
done

if (( FRESH98_LATEST_COUNT > 0 )); then
  mapfile -t latest_fresh98 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh98_checkpoints/fresh98_pairwise_conflict_repair_step_*.pt 2>/dev/null | head -n "$FRESH98_LATEST_COUNT")
  for model in "${latest_fresh98[@]}"; do
    add_candidate "$model"
  done
fi

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No fresh99 crossing-focus candidates found." | tee -a "$LOG"
  exit 1
fi

idx=0
for model in "${CANDIDATES[@]}"; do
  label=$(basename "$model" .pt)
  json="$OUT/${label}_crossing${EPISODES}_eval.json"
  eval_log="/tmp/fresh99_focus_${label}.log"
  domain=$((BASE_DOMAIN + (idx % 40)))
  idx=$((idx + 1))
  echo "[$(date +%T)] crossing eval $label domain=$domain" | tee -a "$LOG"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  if ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$model" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$json" \
    --scenario three_usv_crossing \
    > "$eval_log" 2>&1; then
    /bin/python3 - "$json" <<'PY' | tee -a "$LOG"
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
data = json.loads(p.read_text(encoding='utf-8'))
s = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
print(
    f"  ok {p.name}: "
    f"coll={s.get('collision_rate', 1.0):.3f} "
    f"succ={s.get('success_rate', 0.0):.3f} "
    f"timeout={s.get('timeout_rate', 0.0):.3f} "
    f"progress={s.get('mean_team_goal_progress_ratio', 0.0):.3f} "
    f"sep={s.get('worst_episode_min_separation', 0.0):.3f} "
    f"ent={s.get('mean_entanglement_ratio', 1.0):.3f} "
    f"colregs={s.get('mean_colregs_violation_ratio', 1.0):.3f} "
    f"omega={s.get('mean_omega_flip_count', 999.0):.1f}"
)
PY
  else
    rc=$?
    echo "  FAIL $label rc=$rc log=$eval_log" | tee -a "$LOG"
    rm -f "$json"
  fi
done

/bin/python3 - "$OUT" "$EPISODES" <<'PY' | tee -a "$LOG"
import glob
import json
import sys
from pathlib import Path
out = Path(sys.argv[1])
episodes = sys.argv[2]
rows = []
for raw in glob.glob(str(out / f'*_crossing{episodes}_eval.json')):
    p = Path(raw)
    data = json.loads(p.read_text(encoding='utf-8'))
    s = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
    rows.append({
        'label': p.name.replace(f'_crossing{episodes}_eval.json', ''),
        'model': data.get('model', ''),
        'collision_rate': float(s.get('collision_rate', 1.0)),
        'success_rate': float(s.get('success_rate', 0.0)),
        'timeout_rate': float(s.get('timeout_rate', 1.0)),
        'progress': float(s.get('mean_team_goal_progress_ratio', 0.0)),
        'worst_sep': float(s.get('worst_episode_min_separation', 0.0)),
        'entanglement': float(s.get('mean_entanglement_ratio', 1.0)),
        'colregs': float(s.get('mean_colregs_violation_ratio', 1.0)),
        'omega': float(s.get('mean_omega_flip_count', 999.0)),
    })
rows.sort(key=lambda r: (
    r['collision_rate'],
    -r['success_rate'],
    r['timeout_rate'],
    -r['progress'],
    -r['worst_sep'],
    r['entanglement'],
    r['colregs'],
    r['omega'],
))
summary = {
    'eval_dir': str(out),
    'episodes': int(episodes),
    'best': rows[0] if rows else None,
    'has_crossing_breakthrough': any(r['collision_rate'] < 1.0 for r in rows),
    'rows': rows,
}
summary_path = out / 'crossing_focus_summary.json'
summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print('=== crossing_focus_summary ===')
for r in rows:
    print(
        f"coll={r['collision_rate']:.3f} succ={r['success_rate']:.3f} "
        f"timeout={r['timeout_rate']:.3f} progress={r['progress']:.3f} "
        f"sep={r['worst_sep']:.3f} ent={r['entanglement']:.3f} "
        f"colregs={r['colregs']:.3f} omega={r['omega']:.1f} {r['label']}"
    )
print(f"breakthrough={summary['has_crossing_breakthrough']}")
print(f"summary={summary_path}")
PY

echo "[$(date +%T)] fresh99 crossing focus eval complete: $OUT/crossing_focus_summary.json" | tee -a "$LOG"
