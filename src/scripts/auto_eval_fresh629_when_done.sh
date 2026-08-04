#!/bin/bash
# Auto-run fresh629 offline gate after training completion, then compare with fresh625.
set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

TRAIN_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh629_route_tight_queue_3usv.pt"
TRAIN_LOG="/mnt/data/checkpoints/usv_rl/fresh629_train.log"
OUT_DIR="${OUT_DIR:-/home/chenhangwei/usv_workspace/src/logs/eval_fresh629_gate}"
AUTO_LOG="${AUTO_LOG:-/mnt/data/checkpoints/usv_rl/fresh629_auto_eval.log}"
EVAL_LOG="$OUT_DIR/eval_fresh628_vs_fresh629.log"
SUMMARY_LOG="$OUT_DIR/summary_fresh628_vs_fresh629.log"
LOCK_FILE="/tmp/fresh629_auto_eval.lock"

mkdir -p "$OUT_DIR"

# simple lock to avoid duplicate watchers
if [[ -e "$LOCK_FILE" ]]; then
  echo "[$(date '+%F %T')] lock exists: $LOCK_FILE, exit." | tee -a "$AUTO_LOG"
  exit 0
fi
trap 'rm -f "$LOCK_FILE"' EXIT
printf "%s\n" "$$" > "$LOCK_FILE"

echo "[$(date '+%F %T')] watcher started." | tee -a "$AUTO_LOG"
echo "[$(date '+%F %T')] wait condition: final model exists AND fresh629 train process exits." | tee -a "$AUTO_LOG"

auto_running() {
  pgrep -af "python3 -m usv_rl.train_mappo_policy" | grep -q "fresh629_route_tight_queue_3usv.pt"
}

while true; do
  if [[ -f "$TRAIN_OUTPUT" ]] && ! auto_running; then
    break
  fi
  if [[ -f "$TRAIN_LOG" ]]; then
    last_line=$(grep '\[MAPPO\]\[update=' "$TRAIN_LOG" | tail -1 || true)
    [[ -n "$last_line" ]] && echo "[$(date '+%F %T')] waiting... $last_line" >> "$AUTO_LOG"
  fi
  sleep 60
done

echo "[$(date '+%F %T')] training complete detected, start offline gate." | tee -a "$AUTO_LOG"

MODEL_NAMES="fresh628 fresh629" \
MODELS_fresh628="/mnt/data/checkpoints/usv_rl/fresh628_role_asymmetry_3usv.pt" \
MODELS_fresh629="$TRAIN_OUTPUT" \
OUT="$OUT_DIR" \
bash scripts/eval_fresh625_gate.sh 2>&1 | tee "$EVAL_LOG"

echo "[$(date '+%F %T')] gate run finished, start summarize." | tee -a "$AUTO_LOG"

MODEL_NAMES="fresh628 fresh629" \
OUT="$OUT_DIR" \
python3 scripts/summarize_fresh625_gate.py 2>&1 | tee "$SUMMARY_LOG"

echo "[$(date '+%F %T')] auto eval completed." | tee -a "$AUTO_LOG"
echo "summary: $SUMMARY_LOG" | tee -a "$AUTO_LOG"
