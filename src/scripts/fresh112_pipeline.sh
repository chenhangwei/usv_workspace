#!/bin/bash
# fresh112 pipeline: raw-target last-meter continuation from fresh109, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh112_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh112 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh112.sh
  echo "========== fresh112 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh112_crossing_focus_eval.sh
  echo "========== fresh112 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"