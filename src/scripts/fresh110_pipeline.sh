#!/bin/bash
# fresh110 pipeline: stronger close-range lagging-finish continuation, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh110_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh110 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh110.sh
  echo "========== fresh110 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh110_crossing_focus_eval.sh
  echo "========== fresh110 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"