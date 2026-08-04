#!/bin/bash
# Watcher for fresh632. No broken offline-gate stub (see fresh631 session
# notes: the old gate script only loaded two checkpoints' state_dicts and
# printed key counts -- it never ran real P1-P4 metrics). Real progress
# comes from --auto-evaluate-checkpoints (ranking JSON updated every
# checkpoint-interval) which fresh632's training script already enables.
# This watcher just waits for completion and prints a final summary of the
# last few checkpoint rankings, then reminds to run a manual SITL batch.
set -eo pipefail

cd "$(dirname "$0")/.."

TRAIN_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh632_scale_decisive_5usv.pt"
TRAIN_LOG="/mnt/data/checkpoints/usv_rl/fresh632_train.log"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh632_checkpoints/fresh632_ranking.json"
AUTO_LOG="${AUTO_LOG:-/mnt/data/checkpoints/usv_rl/fresh632_auto_eval.log}"
LOCK_FILE="/tmp/fresh632_auto_eval.lock"

if [[ -e "$LOCK_FILE" ]]; then
  echo "[$(date '+%F %T')] lock exists: $LOCK_FILE, exit." | tee -a "$AUTO_LOG"
  exit 0
fi
trap 'rm -f "$LOCK_FILE"' EXIT
printf "%s\n" "$$" > "$LOCK_FILE"

echo "[$(date '+%F %T')] fresh632 watcher started." | tee -a "$AUTO_LOG"

auto_running() {
  pgrep -af "python3 -m usv_rl.train_mappo_policy" | grep -q "fresh632_scale_decisive_5usv.pt"
}

while true; do
  if [[ -f "$TRAIN_OUTPUT" ]] && ! auto_running; then
    break
  fi
  if [[ -f "$TRAIN_LOG" ]]; then
    last_line=$(grep '\[MAPPO\]\[update=' "$TRAIN_LOG" 2>/dev/null | tail -1 || true)
    [[ -n "$last_line" ]] && echo "[$(date '+%F %T')] waiting... $last_line" | tee -a "$AUTO_LOG"
  fi
  sleep 60
done

echo "[$(date '+%F %T')] fresh632 training complete." | tee -a "$AUTO_LOG"

if [[ -f "$RANKING_JSON" ]]; then
  echo "[$(date '+%F %T')] latest checkpoint ranking snapshot:" | tee -a "$AUTO_LOG"
  python3 -c "
import json
with open('$RANKING_JSON') as f:
    data = json.load(f)
print(json.dumps(data, indent=2, ensure_ascii=False)[:4000])
" | tee -a "$AUTO_LOG"
else
  echo "[$(date '+%F %T')] no ranking JSON found at $RANKING_JSON" | tee -a "$AUTO_LOG"
fi

NEXT_STEP_MSG="[$(date '+%F %T')] NEXT STEP: run a manual SITL batch (pentagram 4-5USV / Z-mirror / figure-8) against $TRAIN_OUTPUT and compare with the 20260717 13:12-13:58 fresh631 baseline metrics recorded in memories/repo/sitl_20260717_1312_1358_avoidance_smoothness_diagnosis.md."
echo "$NEXT_STEP_MSG" | tee -a "$AUTO_LOG"
