#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: watch_mappo_checkpoint_online_repeat.sh <checkpoint-dir> <baseline_benchmark.json> [extra repeat args...]" >&2
  exit 2
fi

CHECKPOINT_DIR="$1"
BASELINE_BENCHMARK_PATH="$2"
shift 2 || true

POLL_INTERVAL="${POLL_INTERVAL:-30}"
MAX_COLLISION_RATE="${MAX_COLLISION_RATE:-0.0}"
MIN_PROGRESS="${MIN_PROGRESS:-0.16}"
MIN_WORST_PAIRWISE_MIN_SEPARATION="${MIN_WORST_PAIRWISE_MIN_SEPARATION:-0.0}"
EXTRA_REPEAT_ARGS=("$@")

if [[ ! -d "${CHECKPOINT_DIR}" ]]; then
  echo "error: checkpoint dir not found: ${CHECKPOINT_DIR}" >&2
  exit 2
fi

if [[ ! -f "${BASELINE_BENCHMARK_PATH}" ]]; then
  echo "error: baseline benchmark JSON not found: ${BASELINE_BENCHMARK_PATH}" >&2
  exit 2
fi

echo "watching_checkpoint_dir=${CHECKPOINT_DIR}"
echo "baseline_benchmark_path=${BASELINE_BENCHMARK_PATH}"
echo "poll_interval=${POLL_INTERVAL}s"
echo "max_collision_rate=${MAX_COLLISION_RATE}"
echo "min_progress=${MIN_PROGRESS}"
echo "min_worst_pairwise_min_separation=${MIN_WORST_PAIRWISE_MIN_SEPARATION}"

while true; do
  found_eval_json=0
  while IFS= read -r eval_json; do
    found_eval_json=1
    checkpoint_path="${eval_json%.eval.json}.pt"
    checkpoint_stem="${checkpoint_path%.pt}"
    repeat_summary_json="${checkpoint_stem}.online.repeat.summary.json"
    repeat_skip_json="${checkpoint_stem}.online.repeat.skip.json"

    if [[ ! -f "${checkpoint_path}" ]]; then
      continue
    fi

    if [[ -f "${repeat_summary_json}" || -f "${repeat_skip_json}" ]]; then
      continue
    fi

    decision_json="$(python3 - "${eval_json}" "${MAX_COLLISION_RATE}" "${MIN_PROGRESS}" "${MIN_WORST_PAIRWISE_MIN_SEPARATION}" <<'PY'
import json
import sys
from pathlib import Path

eval_json_path = Path(sys.argv[1])
max_collision_rate = float(sys.argv[2])
min_progress = float(sys.argv[3])
min_worst_pairwise_min_separation = float(sys.argv[4])

data = json.loads(eval_json_path.read_text(encoding='utf-8'))
collision_rate = float(data.get('collision_rate', 1.0))
progress = float(data.get('mean_team_goal_progress_ratio', 0.0))
worst_pairwise = float(data.get('worst_pairwise_min_separation', 0.0))

reasons = []
if collision_rate > max_collision_rate:
    reasons.append(f'collision_rate_exceeds_threshold:{collision_rate}>{max_collision_rate}')
if progress < min_progress:
    reasons.append(f'progress_below_threshold:{progress}<{min_progress}')
if worst_pairwise < min_worst_pairwise_min_separation:
    reasons.append(
        f'worst_pairwise_min_separation_below_threshold:{worst_pairwise}<{min_worst_pairwise_min_separation}'
    )

print(json.dumps({
    'eligible': not reasons,
    'collision_rate': collision_rate,
    'mean_team_goal_progress_ratio': progress,
    'worst_pairwise_min_separation': worst_pairwise,
    'reasons': reasons,
}, ensure_ascii=False))
PY
)"

    eligible="$(python3 - "${decision_json}" <<'PY'
import json
import sys
print('1' if json.loads(sys.argv[1]).get('eligible') else '0')
PY
)"

    if [[ "${eligible}" != "1" ]]; then
      printf '%s\n' "${decision_json}" > "${repeat_skip_json}"
      echo "skip_online_repeat=${checkpoint_path} reason=$(cat "${repeat_skip_json}")"
      continue
    fi

    echo "repeat_online_validate=${checkpoint_path} metrics=${decision_json}"
    /mnt/workspace/usv_workspace/scripts/repeat_validate_mappo_checkpoint_online.sh \
      "${checkpoint_path}" \
      "${BASELINE_BENCHMARK_PATH}" \
      "${EXTRA_REPEAT_ARGS[@]}" || true
  done < <(find "${CHECKPOINT_DIR}" -maxdepth 1 -type f -name '*_step_*.eval.json' | sort)

  if [[ ${found_eval_json} -eq 0 ]]; then
    echo "status=no_eval_json_yet"
  fi

  sleep "${POLL_INTERVAL}"
done