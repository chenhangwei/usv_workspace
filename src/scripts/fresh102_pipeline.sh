#!/bin/bash
# fresh102 pipeline: train post-conflict finish recovery, then run crossing focus.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh102_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "[$(date +%T)] ==== fresh102 pipeline start ===="
  echo "[$(date +%T)] Train log: ${LOG_FILE:-/tmp/fresh102_train.log}"
  bash src/scripts/train_fresh102.sh
  echo "[$(date +%T)] ==== training complete; running crossing-focus eval ===="
  bash src/scripts/fresh102_crossing_focus_eval.sh
  /bin/python3 - <<'PY'
import json
from pathlib import Path
summary_path = Path('/mnt/data/checkpoints/usv_rl/fresh102_crossing_focus_eval/crossing_focus_summary.json')
summary = json.loads(summary_path.read_text(encoding='utf-8'))
best = summary.get('best') or {}
print(
    'crossing_focus_best:',
    f"coll={best.get('collision_rate', 1.0):.3f}",
    f"succ={best.get('success_rate', 0.0):.3f}",
    f"timeout={best.get('timeout_rate', 1.0):.3f}",
    f"progress={best.get('progress', 0.0):.3f}",
    f"goal={best.get('goal_completion', 0.0):.3f}",
    f"sep={best.get('worst_sep', 0.0):.3f}",
    best.get('model'),
)
PY
  echo "[$(date +%T)] ==== fresh102 pipeline complete ===="
} 2>&1 | tee "$LOG"
