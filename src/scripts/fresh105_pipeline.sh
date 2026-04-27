#!/bin/bash
# fresh105 pipeline: train short wide-finish-band continuation, then run crossing-focused triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh105_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh105 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh105.sh
  echo "========== fresh105 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh105_crossing_focus_eval.sh
  echo "========== fresh105 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"