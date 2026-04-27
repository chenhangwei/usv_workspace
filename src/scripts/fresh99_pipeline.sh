#!/bin/bash
# fresh99 pipeline: train role-imitation repair, run crossing focus eval, then full selected eval only after a crossing breakthrough.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh99_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "[$(date +%T)] ==== fresh99 pipeline start ===="
  echo "[$(date +%T)] Train log: ${LOG_FILE:-/tmp/fresh99_train.log}"
  bash src/scripts/train_fresh99.sh
  echo "[$(date +%T)] ==== training complete; running crossing-focus eval ===="
  bash src/scripts/fresh99_crossing_focus_eval.sh

  set +e
  /bin/python3 - <<'PY'
import json
from pathlib import Path

summary_path = Path('/mnt/data/checkpoints/usv_rl/fresh99_crossing_focus_eval/crossing_focus_summary.json')
summary = json.loads(summary_path.read_text(encoding='utf-8'))
best = summary.get('best') or {}
print(
    'crossing_focus_best:',
    f"coll={best.get('collision_rate', 1.0):.3f}",
    f"succ={best.get('success_rate', 0.0):.3f}",
    f"timeout={best.get('timeout_rate', 1.0):.3f}",
    f"progress={best.get('progress', 0.0):.3f}",
    f"sep={best.get('worst_sep', 0.0):.3f}",
    best.get('model'),
)
if not summary.get('has_crossing_breakthrough'):
    raise SystemExit(42)
PY
  focus_rc=$?
  set -e
  if [[ $focus_rc -eq 42 ]]; then
    echo "[$(date +%T)] ==== crossing still collision=1.0 for all fresh99 focus candidates; skipping expensive selected eval ===="
    exit 0
  elif [[ $focus_rc -ne 0 ]]; then
    echo "[$(date +%T)] ==== crossing-focus summary parse failed rc=$focus_rc; continuing selected eval for safety ===="
  fi

  echo "[$(date +%T)] ==== crossing breakthrough found; starting selected eval ===="
  bash src/scripts/fresh99_selected_eval.sh
  echo "[$(date +%T)] ==== selected eval complete; parsing crossing summaries ===="
  /bin/python3 - <<'PY'
import json
from pathlib import Path

eval_dir = Path('/mnt/data/checkpoints/usv_rl/fresh99_selected_eval')
summary_path = eval_dir / 'selected_summary.json'
if summary_path.exists():
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    print('selected_summary:', summary_path)
    print('best_by_aggregate:', summary.get('best_model'))
else:
    print('selected_summary missing:', summary_path)

rows = []
for path in sorted(eval_dir.glob('*_targeted3_eval.json')):
    payload = json.loads(path.read_text(encoding='utf-8'))
    crossing = (payload.get('scenario_summaries') or {}).get('three_usv_crossing', {})
    if not crossing:
        continue
    rows.append({
        'model': payload.get('model', str(path)),
        'source_json': str(path),
        'targeted_collision_rate': payload.get('collision_rate'),
        'targeted_progress': payload.get('mean_team_goal_progress_ratio'),
        'crossing_collision_rate': crossing.get('collision_rate'),
        'crossing_success_rate': crossing.get('success_rate'),
        'crossing_timeout_rate': crossing.get('timeout_rate'),
        'crossing_progress': crossing.get('mean_team_goal_progress_ratio'),
        'crossing_sep': crossing.get('worst_episode_min_separation'),
        'crossing_ent': crossing.get('mean_entanglement_ratio'),
        'crossing_colreg_violation': crossing.get('mean_colregs_violation_ratio'),
        'crossing_omega_flip': crossing.get('mean_omega_flip_count'),
    })
rows.sort(key=lambda r: (
    float(r.get('crossing_collision_rate') if r.get('crossing_collision_rate') is not None else 1.0),
    -float(r.get('crossing_success_rate') if r.get('crossing_success_rate') is not None else 0.0),
    float(r.get('crossing_timeout_rate') if r.get('crossing_timeout_rate') is not None else 1.0),
    -float(r.get('crossing_progress') if r.get('crossing_progress') is not None else 0.0),
    -float(r.get('crossing_sep') if r.get('crossing_sep') is not None else 0.0),
))
out = eval_dir / 'crossing_ranked_summary.json'
out.write_text(json.dumps({'candidates': rows}, ensure_ascii=False, indent=2), encoding='utf-8')
print('crossing_ranked_summary:', out)
for r in rows[:16]:
    print(
        f"crossing coll={r['crossing_collision_rate']:.3f} "
        f"succ={r['crossing_success_rate']:.3f} "
        f"timeout={r['crossing_timeout_rate']:.3f} "
        f"prog={r['crossing_progress']:.3f} "
        f"sep={r['crossing_sep']:.3f} "
        f"ent={r['crossing_ent']:.3f} "
        f"omega={r['crossing_omega_flip']:.1f} | {r['model']}"
    )
PY
  echo "[$(date +%T)] ==== fresh99 pipeline complete ===="
} 2>&1 | tee "$LOG"
