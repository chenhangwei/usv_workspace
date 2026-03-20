#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"
DEFAULT_BUNDLE_ROOT="/mnt/data/migration"
POLL_INTERVAL="${POLL_INTERVAL:-60}"

infer_run_name() {
  local latest_file
  latest_file="$(ls -1t /tmp/current_*_run_name 2>/dev/null | head -n 1 || true)"
  if [[ -n "${latest_file}" && -f "${latest_file}" ]]; then
    cat "${latest_file}"
  fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "usage: watch_training_and_prepare_local_migration.sh [run_name] [bundle_root]"
  echo "env: POLL_INTERVAL(default=60 seconds)"
  exit 0
fi

RUN_NAME="${1:-$(infer_run_name)}"
BUNDLE_ROOT="${2:-${DEFAULT_BUNDLE_ROOT}}"

if [[ -z "${RUN_NAME}" ]]; then
  echo "error: run_name is required or must be inferable from /tmp/current_*_run_name" >&2
  exit 2
fi

OUTPUT_PATH="${DATA_DIR}/${RUN_NAME}.pt"
LOG_PATH="${DATA_DIR}/${RUN_NAME}.log"
PROCESS_PATTERN="train_mappo_residual --output ${OUTPUT_PATH}"

echo "watching_run=${RUN_NAME}"
echo "watching_output=${OUTPUT_PATH}"
echo "bundle_root=${BUNDLE_ROOT}"

while pgrep -af "${PROCESS_PATTERN}" >/dev/null; do
  LAST_PROGRESS="$(grep '\[MAPPO\]\|Saved MAPPO checkpoint scaffold' "${LOG_PATH}" 2>/dev/null | tail -n 1 || true)"
  if [[ -n "${LAST_PROGRESS}" ]]; then
    echo "$(date '+%F %T') ${LAST_PROGRESS}"
  else
    echo "$(date '+%F %T') waiting_for_first_mappo_update"
  fi
  sleep "${POLL_INTERVAL}"
done

echo "$(date '+%F %T') training_process_exited"

for _ in $(seq 1 30); do
  if [[ -f "${OUTPUT_PATH}" ]]; then
    break
  fi
  sleep 10
done

if [[ ! -f "${OUTPUT_PATH}" ]]; then
  echo "error: final output checkpoint not found after training exit: ${OUTPUT_PATH}" >&2
  exit 3
fi

sleep 15
cd "${WORKSPACE_DIR}"
./scripts/prepare_local_migration_bundle.sh "${RUN_NAME}" "${BUNDLE_ROOT}"