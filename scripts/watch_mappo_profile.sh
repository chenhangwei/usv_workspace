#!/usr/bin/env bash

set -euo pipefail

LOG_PATH="${1:?usage: watch_mappo_profile.sh <training-log-path>}"

if [[ ! -f "${LOG_PATH}" ]]; then
  echo "log_missing=${LOG_PATH}"
  exit 1
fi

echo "watching=${LOG_PATH}"
echo "--- latest throughput lines ---"
grep '\[MAPPO\]\[update=' "${LOG_PATH}" | tail -n 20 || true
echo "--- latest checkpoints/warnings ---"
grep 'Saved MAPPO checkpoint scaffold|Warning: MAPPO rollout step failed|Warning: MAPPO episode reset failed|Timed out waiting for multi-agent observations|Traceback|RuntimeError' "${LOG_PATH}" | tail -n 20 || true
echo "--- recent tail ---"
tail -n 40 "${LOG_PATH}"