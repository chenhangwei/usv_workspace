#!/bin/bash
# fresh108 pipeline: train partial-team lagging-finish continuation, then run crossing-focused triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh108_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh108 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh108.sh
  echo "========== fresh108 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh108_crossing_focus_eval.sh
  echo "========== fresh108 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"