#!/bin/bash
# fresh104 train -> crossing-focused triage.

set -eo pipefail

cd "$(dirname "$0")/../.."

LOG="${LOG:-/tmp/fresh104_pipeline.log}"
: > "$LOG"

{
  echo "========== fresh104 pipeline =========="
  echo "Started: $(date -Is)"
} | tee -a "$LOG"

bash src/scripts/train_fresh104.sh 2>&1 | tee -a "$LOG"

EPISODES="${EPISODES:-3}" \
OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh104_crossing_focus_eval}" \
LOG="${EVAL_LOG:-/tmp/fresh104_crossing_focus_eval.log}" \
BASE_DOMAIN="${BASE_DOMAIN:-191}" \
FRESH104_LATEST_COUNT="${FRESH104_LATEST_COUNT:-12}" \
bash src/scripts/fresh104_crossing_focus_eval.sh 2>&1 | tee -a "$LOG"

echo "Completed: $(date -Is)" | tee -a "$LOG"
