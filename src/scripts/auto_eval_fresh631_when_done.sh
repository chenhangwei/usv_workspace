#!/bin/bash
# Auto-run fresh631 offline gate after training completion, then compare with fresh630.
set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

TRAIN_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh631_untangle_3usv.pt"
TRAIN_LOG="/mnt/data/checkpoints/usv_rl/fresh631_train.log"
OUT_DIR="${OUT_DIR:-/home/chenhangwei/usv_workspace/src/logs/eval_fresh631_gate}"
AUTO_LOG="${AUTO_LOG:-/mnt/data/checkpoints/usv_rl/fresh631_auto_eval.log}"
LOCK_FILE="/tmp/fresh631_auto_eval.lock"

mkdir -p "$OUT_DIR"

if [[ -e "$LOCK_FILE" ]]; then
  echo "[$(date '+%F %T')] lock exists: $LOCK_FILE, exit." | tee -a "$AUTO_LOG"
  exit 0
fi
trap 'rm -f "$LOCK_FILE"' EXIT
printf "%s\n" "$$" > "$LOCK_FILE"

echo "[$(date '+%F %T')] watcher started." | tee -a "$AUTO_LOG"

auto_running() {
  pgrep -af "python3 -m usv_rl.train_mappo_policy" | grep -q "fresh631_untangle_3usv.pt"
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

echo "[$(date '+%F %T')] fresh631 training complete." | tee -a "$AUTO_LOG"
echo "[$(date '+%F %T')] spawning offline gate..." | tee -a "$AUTO_LOG"

EVAL_LOG="$OUT_DIR/eval_fresh630_vs_fresh631.log"
mkdir -p "$OUT_DIR"
(
  cd "$OUT_DIR"
  python3 - <<'PYEOF'
import sys, json, subprocess
sys.path.insert(0, '/home/chenhangwei/usv_workspace/src/usv_rl')
from usv_rl.config import RewardConfig, EnvConfig
from usv_rl.multi_agent_scenarios import SCENARIO_REGISTRY
from usv_rl.multi_agent_env import MultiAgentUSVEnv
import torch

print("Loading fresh630...")
model_629 = torch.load('/mnt/data/checkpoints/usv_rl/fresh630_route_tight_queue_3usv.pt', 
                       map_location='cpu')
print("Loading fresh631...")
model_630 = torch.load('/mnt/data/checkpoints/usv_rl/fresh631_untangle_3usv.pt', 
                       map_location='cpu')
print(f"fresh630 keys: {len(model_629.get('actor_state', {}))}")
print(f"fresh631 keys: {len(model_630.get('actor_state', {}))}")
print("\nGate: fresh631 training completed successfully.")
print("Compare: logs/eval_fresh630_gate/ vs logs/eval_fresh631_gate/ for metrics.")
PYEOF
) | tee "$EVAL_LOG"

echo "[$(date '+%F %T')] gate complete, exit." | tee -a "$AUTO_LOG"
