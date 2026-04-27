#!/bin/bash
# fresh115 pipeline: true-2/3-gated hold-and-finish continuation, final crossing eval.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh115_pipeline.log}"
OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh115_crossing_focus_eval}"
EPISODES="${EPISODES:-1}"
STEPS="${STEPS:-400}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-450}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-135}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-105}"
BASE_DOMAIN="${BASE_DOMAIN:-156}"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT"

{
  echo "========== fresh115 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh115.sh
  echo "========== fresh115 train complete; crossing focus =========="
  cd src
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  json="$OUT/fresh115_true_2of3_hold_finish_crossing${EPISODES}_eval.json"
  ROS_DOMAIN_ID="$BASE_DOMAIN" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model /mnt/data/checkpoints/usv_rl/fresh115_true_2of3_hold_finish.pt \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$json" \
    --scenario three_usv_crossing
  /bin/python3 - "$json" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
data = json.loads(p.read_text(encoding='utf-8'))
s = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
print(
    f"fresh115 crossing: coll={s.get('collision_rate', 1.0):.3f} "
    f"succ={s.get('success_rate', 0.0):.3f} "
    f"timeout={s.get('timeout_rate', 0.0):.3f} "
    f"progress={s.get('mean_team_goal_progress_ratio', 0.0):.3f} "
    f"goal={s.get('mean_goal_completion_ratio', 0.0):.3f} "
    f"sep={s.get('worst_episode_min_separation', 0.0):.3f} "
    f"ent={s.get('mean_entanglement_ratio', 1.0):.3f} "
    f"colregs={s.get('mean_colregs_violation_ratio', 1.0):.3f} "
    f"omega={s.get('mean_omega_flip_count', 999.0):.1f}"
)
PY
  echo "========== fresh115 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"