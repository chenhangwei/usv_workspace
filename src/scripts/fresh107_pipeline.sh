#!/bin/bash
# fresh107 pipeline: train lagging-finish continuation, then run crossing-focused triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh107_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh107 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh107.sh
  echo "========== fresh107 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh107_crossing_focus_eval.sh
  echo "========== fresh107 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"