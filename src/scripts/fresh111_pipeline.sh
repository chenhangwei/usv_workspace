#!/bin/bash
# fresh111 pipeline: continue from fresh109 safe 2/3 and nudge only the last ~5m.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh111_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh111 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh111.sh
  echo "========== fresh111 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh111_crossing_focus_eval.sh
  echo "========== fresh111 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"