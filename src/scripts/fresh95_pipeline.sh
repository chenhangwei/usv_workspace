#!/bin/bash
# fresh95 pipeline: train, then selected-evaluate, then print per-scenario crossing summary.
# This keeps the run autonomous while the user is away.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh95_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "[$(date +%T)] ==== fresh95 pipeline start ===="
  echo "[$(date +%T)] Train log: ${LOG_FILE:-/tmp/fresh95_train.log}"
  bash src/scripts/train_fresh95.sh
  echo "[$(date +%T)] ==== training complete; starting selected eval ===="
  bash src/scripts/fresh95_selected_eval.sh
  echo "[$(date +%T)] ==== selected eval complete; parsing crossing summaries ===="
  /bin/python3 - <<'PY'
import json
from pathlib import Path

eval_dir = Path('/mnt/data/checkpoints/usv_rl/fresh95_selected_eval')
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
    })
rows.sort(key=lambda r: (
    float(r.get('crossing_collision_rate') if r.get('crossing_collision_rate') is not None else 1.0),
    -float(r.get('crossing_success_rate') if r.get('crossing_success_rate') is not None else 0.0),
    -float(r.get('crossing_progress') if r.get('crossing_progress') is not None else 0.0),
    -float(r.get('crossing_sep') if r.get('crossing_sep') is not None else 0.0),
))
out = eval_dir / 'crossing_ranked_summary.json'
out.write_text(json.dumps({'candidates': rows}, ensure_ascii=False, indent=2), encoding='utf-8')
print('crossing_ranked_summary:', out)
for r in rows[:12]:
    print(
        f"crossing coll={r['crossing_collision_rate']:.3f} "
        f"succ={r['crossing_success_rate']:.3f} "
        f"timeout={r['crossing_timeout_rate']:.3f} "
        f"prog={r['crossing_progress']:.3f} "
        f"sep={r['crossing_sep']:.3f} | {r['model']}"
    )
PY
  echo "[$(date +%T)] ==== fresh95 pipeline complete ===="
} 2>&1 | tee "$LOG"
