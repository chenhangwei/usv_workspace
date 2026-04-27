#!/bin/bash
# fresh102 selected evaluation: full-5 + targeted hard-scenario triage for
# post-conflict finish candidates that pass/approach crossing-focus success.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh102_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh102_checkpoints}"
export LOG="${LOG:-/tmp/fresh102_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-191}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-380}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-120}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-58}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-620}"
export FRESH102_LATEST_COUNT="${FRESH102_LATEST_COUNT:-8}"

CANDIDATES=()
add_candidate() {
  local model="$1"
  if [[ -z "$model" || ! -f "$model" ]]; then
    return 0
  fi
  for existing in "${CANDIDATES[@]}"; do
    [[ "$existing" == "$model" ]] && return 0
  done
  CANDIDATES+=("$model")
}

if [[ $# -gt 0 ]]; then
  for model in "$@"; do
    add_candidate "$model"
  done
else
  SUMMARY="/mnt/data/checkpoints/usv_rl/fresh102_crossing_focus_eval/crossing_focus_summary.json"
  if [[ -f "$SUMMARY" ]]; then
    mapfile -t summary_models < <(/bin/python3 - "$SUMMARY" <<'PY'
import json, sys
from pathlib import Path
summary = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
rows = summary.get('rows') or []
rows.sort(key=lambda r: (
    float(r.get('collision_rate', 1.0)),
    -float(r.get('success_rate', 0.0)),
    float(r.get('timeout_rate', 1.0)),
    -float(r.get('goal_completion', 0.0)),
    -float(r.get('progress', 0.0)),
))
for row in rows:
    model = row.get('model') or ''
    if model and float(row.get('collision_rate', 1.0)) == 0.0:
        print(model)
PY
)
    for model in "${summary_models[@]}"; do
      add_candidate "$model"
    done
  fi

  add_candidate /mnt/data/checkpoints/usv_rl/fresh102_post_conflict_finish.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh101_checkpoints/fresh101_crossing_progress_recover_step_0008640.pt

  mapfile -t latest_fresh102 < <(ls -1t "$CKPT_DIR"/fresh102_post_conflict_finish_step_*.pt 2>/dev/null | head -n "$FRESH102_LATEST_COUNT")
  for model in "${latest_fresh102[@]}"; do
    add_candidate "$model"
  done
fi

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh102 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"
