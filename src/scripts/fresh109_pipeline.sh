#!/bin/bash
# fresh109 pipeline: train low-noise near-team lagging-finish continuation, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh109_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh109 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh109.sh
  echo "========== fresh109 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh109_crossing_focus_eval.sh
  echo "========== fresh109 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"