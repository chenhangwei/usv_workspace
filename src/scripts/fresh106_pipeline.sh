#!/bin/bash
# fresh106 pipeline: train narrow post-conflict finish continuation, then run crossing-focused triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh106_pipeline.log}"
export PYTHONUNBUFFERED=1

{
  echo "========== fresh106 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh106.sh
  echo "========== fresh106 train complete; crossing focus =========="
  EPISODES="${EPISODES:-3}" INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}" bash src/scripts/fresh106_crossing_focus_eval.sh
  echo "========== fresh106 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"