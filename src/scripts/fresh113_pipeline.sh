#!/bin/bash
# fresh113 pipeline: anchored raw-target continuation from fresh109, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh113_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh113 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh113.sh
  echo "========== fresh113 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh113_crossing_focus_eval.sh
  echo "========== fresh113 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"