#!/bin/bash
# fresh108 crossing-only triage for partial-team lagging-finish candidates.

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

OUT=${OUT:-/mnt/data/checkpoints/usv_rl/fresh108_crossing_focus_eval}
LOG=${LOG:-/tmp/fresh108_crossing_focus_eval.log}
EPISODES=${EPISODES:-3}
STEPS=${STEPS:-400}
EVAL_TIMEOUT=${EVAL_TIMEOUT:-450}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-135}
NO_PROGRESS_TIMEOUT=${NO_PROGRESS_TIMEOUT:-95}
BASE_DOMAIN=${BASE_DOMAIN:-166}
FRESH108_LATEST_COUNT=${FRESH108_LATEST_COUNT:-12}
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
  add_candidate /mnt/data/checkpoints/usv_rl/fresh104_checkpoints/fresh104_goal_hold_step_0003006.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh106_checkpoints/fresh106_post_conflict_narrow_finish_step_0001152.pt
fi
add_candidate /mnt/data/checkpoints/usv_rl/fresh108_partial_lagging_finish.pt

mapfile -t latest_fresh108 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh108_checkpoints/fresh108_partial_lagging_finish_step_*.pt 2>/dev/null | head -n "$FRESH108_LATEST_COUNT")
for model in "${latest_fresh108[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No fresh108 crossing-focus candidates found." | tee -a "$LOG"
  exit 1
fi

idx=0
for model in "${CANDIDATES[@]}"; do
  label=$(basename "$model" .pt)
  json="$OUT/${label}_crossing${EPISODES}_eval.json"
  eval_log="/tmp/fresh108_focus_${label}.log"
  domain=$((BASE_DOMAIN + (idx % 56)))
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
    f"goal={s.get('mean_goal_completion_ratio', 0.0):.3f} "
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
import glob, json, sys
from pathlib import Path
out = Path(sys.argv[1]); episodes = sys.argv[2]
rows = []
for raw in glob.glob(str(out / f'*_crossing{episodes}_eval.json')):
    p = Path(raw); data = json.loads(p.read_text(encoding='utf-8'))
    s = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
    rows.append({
        'label': p.name.replace(f'_crossing{episodes}_eval.json', ''),
        'model': data.get('model', ''),
        'collision_rate': float(s.get('collision_rate', 1.0)),
        'success_rate': float(s.get('success_rate', 0.0)),
        'timeout_rate': float(s.get('timeout_rate', 1.0)),
        'progress': float(s.get('mean_team_goal_progress_ratio', 0.0)),
        'goal_completion': float(s.get('mean_goal_completion_ratio', 0.0)),
        'worst_sep': float(s.get('worst_episode_min_separation', 0.0)),
        'entanglement': float(s.get('mean_entanglement_ratio', 1.0)),
        'colregs': float(s.get('mean_colregs_violation_ratio', 1.0)),
        'omega': float(s.get('mean_omega_flip_count', 999.0)),
    })
rows.sort(key=lambda r: (r['collision_rate'], -r['success_rate'], r['timeout_rate'], -r['goal_completion'], -r['progress'], -r['worst_sep']))
summary = {
    'eval_dir': str(out),
    'episodes': int(episodes),
    'best': rows[0] if rows else None,
    'has_crossing_breakthrough': any(r['collision_rate'] < 1.0 for r in rows),
    'has_crossing_success': any(r['collision_rate'] == 0.0 and r['success_rate'] > 0.0 for r in rows),
    'rows': rows,
}
summary_path = out / 'crossing_focus_summary.json'
summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print('=== crossing_focus_summary ===')
for r in rows:
    print(f"coll={r['collision_rate']:.3f} succ={r['success_rate']:.3f} timeout={r['timeout_rate']:.3f} progress={r['progress']:.3f} goal={r['goal_completion']:.3f} sep={r['worst_sep']:.3f} ent={r['entanglement']:.3f} colregs={r['colregs']:.3f} omega={r['omega']:.1f} {r['label']}")
print(f"breakthrough={summary['has_crossing_breakthrough']} success={summary['has_crossing_success']}")
print(f"summary={summary_path}")
PY

echo "[$(date +%T)] fresh108 crossing focus eval complete: $OUT/crossing_focus_summary.json" | tee -a "$LOG"