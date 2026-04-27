#!/bin/bash
# fresh119 pipeline: train quadratic safe-gap lagging finish, then crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh119_pipeline.log}"
OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh119_crossing_focus_eval}"
EPISODES="${EPISODES:-1}"
STEPS="${STEPS:-400}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-450}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-135}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-105}"
BASE_DOMAIN="${BASE_DOMAIN:-214}"
MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh119_quadratic_safe_gap_lagging_finish.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh119_checkpoints}"
FRESH106="${FRESH106:-/mnt/data/checkpoints/usv_rl/fresh106_checkpoints/fresh106_post_conflict_narrow_finish_step_0001152.pt}"
FRESH109="${FRESH109:-/mnt/data/checkpoints/usv_rl/fresh109_near_team_lagging_finish.pt}"
FRESH118_1152="${FRESH118_1152:-/mnt/data/checkpoints/usv_rl/fresh118_checkpoints/fresh118_safe_gap_lagging_finish_step_0001152.pt}"
FRESH118_0864="${FRESH118_0864:-/mnt/data/checkpoints/usv_rl/fresh118_checkpoints/fresh118_safe_gap_lagging_finish_step_0000864.pt}"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT"

{
  echo "========== fresh119 pipeline =========="
  echo "Started: $(date -Iseconds)"
  bash src/scripts/train_fresh119.sh
  echo "========== fresh119 train complete; crossing focus =========="

  cd src
  declare -a candidates=()
  [[ -f "$FRESH106" ]] && candidates+=("$FRESH106")
  [[ -f "$FRESH109" ]] && candidates+=("$FRESH109")
  [[ -f "$FRESH118_1152" ]] && candidates+=("$FRESH118_1152")
  [[ -f "$FRESH118_0864" ]] && candidates+=("$FRESH118_0864")
  [[ -f "$MODEL" ]] && candidates+=("$MODEL")
  if [[ -d "$CKPT_DIR" ]]; then
    while IFS= read -r ckpt; do
      candidates+=("$ckpt")
    done < <(ls -1t "$CKPT_DIR"/fresh119_quadratic_safe_gap_lagging_finish_step_*.pt 2>/dev/null || true)
  fi

  declare -A seen=()
  idx=0
  summary="$OUT/fresh119_crossing_summary.tsv"
  echo -e "label\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tentanglement\tcolregs\tomega\tmodel" > "$summary"
  for model in "${candidates[@]}"; do
    [[ -f "$model" ]] || continue
    real_model="$(readlink -f "$model")"
    if [[ -n "${seen[$real_model]:-}" ]]; then
      continue
    fi
    seen[$real_model]=1
    label="$(basename "$model" .pt)"
    json="$OUT/${label}_crossing${EPISODES}_eval.json"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    domain=$((BASE_DOMAIN + idx))
    idx=$((idx + 1))
    echo "----- crossing eval: $label domain=$domain -----"
    ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$model" \
      --episodes "$EPISODES" \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --output-json "$json" \
      --scenario three_usv_crossing
    /bin/python3 - "$json" "$summary" "$label" "$model" <<'PY'
import json, sys
from pathlib import Path

json_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
label = sys.argv[3]
model = sys.argv[4]
data = json.loads(json_path.read_text(encoding='utf-8'))
s = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
row = {
    'collision': float(s.get('collision_rate', 1.0)),
    'success': float(s.get('success_rate', 0.0)),
    'timeout': float(s.get('timeout_rate', 0.0)),
    'progress': float(s.get('mean_team_goal_progress_ratio', 0.0)),
    'goal': float(s.get('mean_goal_completion_ratio', 0.0)),
    'sep': float(s.get('worst_episode_min_separation', 0.0)),
    'entanglement': float(s.get('mean_entanglement_ratio', 1.0)),
    'colregs': float(s.get('mean_colregs_violation_ratio', 1.0)),
    'omega': float(s.get('mean_omega_flip_count', 999.0)),
}
print(
    f"fresh119 crossing {label}: coll={row['collision']:.3f} "
    f"succ={row['success']:.3f} timeout={row['timeout']:.3f} "
    f"progress={row['progress']:.3f} goal={row['goal']:.3f} "
    f"sep={row['sep']:.3f} ent={row['entanglement']:.3f} "
    f"colregs={row['colregs']:.3f} omega={row['omega']:.1f}"
)
summary_path.open('a', encoding='utf-8').write(
    f"{label}\t{row['collision']:.6f}\t{row['success']:.6f}\t{row['timeout']:.6f}\t"
    f"{row['progress']:.6f}\t{row['goal']:.6f}\t{row['sep']:.6f}\t"
    f"{row['entanglement']:.6f}\t{row['colregs']:.6f}\t{row['omega']:.6f}\t{model}\n"
)
PY
  done
  echo "========== fresh119 crossing summary =========="
  cat "$summary"
  echo "========== fresh119 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"