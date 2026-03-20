#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: watch_mappo_checkpoint_eval.sh <checkpoint-dir>" >&2
  exit 2
fi

CHECKPOINT_DIR="$1"
POLL_INTERVAL="${POLL_INTERVAL:-30}"

if [[ ! -d "${CHECKPOINT_DIR}" ]]; then
  echo "error: checkpoint dir not found: ${CHECKPOINT_DIR}" >&2
  exit 2
fi

echo "watching_checkpoint_dir=${CHECKPOINT_DIR}"
echo "poll_interval=${POLL_INTERVAL}s"

while true; do
  found_checkpoint=0
  while IFS= read -r checkpoint_path; do
    found_checkpoint=1
    eval_json="${checkpoint_path%.pt}.eval.json"
    if [[ -f "${eval_json}" ]]; then
      continue
    fi

    echo "evaluating_checkpoint=${checkpoint_path}"
    /mnt/workspace/usv_workspace/scripts/evaluate_mappo_checkpoint.sh "${checkpoint_path}" || true
  done < <(find "${CHECKPOINT_DIR}" -maxdepth 1 -type f -name '*_step_*.pt' | sort)

  if [[ ${found_checkpoint} -eq 0 ]]; then
    echo "status=no_checkpoints_yet"
  fi

  sleep "${POLL_INTERVAL}"
done
