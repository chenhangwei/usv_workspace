#!/bin/bash
# fresh114 pipeline: conservative anchored raw-target continuation, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh114_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh114 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh114.sh
  echo "========== fresh114 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh114_crossing_focus_eval.sh
  echo "========== fresh114 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"