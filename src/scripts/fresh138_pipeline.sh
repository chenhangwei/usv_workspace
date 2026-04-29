#!/bin/bash
# fresh138 pipeline: low-omega continuation and seeded crossing triage.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh138_pipeline.log}"
OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh138_crossing_focus_eval}"
EPISODES="${EPISODES:-1}"
STEPS="${STEPS:-400}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-520}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-135}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-105}"
BASE_DOMAIN="${BASE_DOMAIN:-194}"
MAX_DOMAIN="${MAX_DOMAIN:-232}"
SEED="${SEED:-1380}"
MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh138_low_omega_finish_from_fresh136_step0864.pt}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh138_checkpoints}"
FRESH128="${FRESH128:-/mnt/data/checkpoints/usv_rl/fresh128_balanced_finish_from_fresh124_step0576.pt}"
FRESH136="${FRESH136:-/mnt/data/checkpoints/usv_rl/fresh136_checkpoints/fresh136_active_finish_from_fresh128_step_0000864.pt}"
FRESH137="${FRESH137:-/mnt/data/checkpoints/usv_rl/fresh137_stabilize_from_fresh136_step0864.pt}"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT"

{
  echo "========== fresh138 pipeline =========="
  echo "Started: $(date -Iseconds)"
  echo "Eval seed: $SEED"
  bash src/scripts/train_fresh138.sh
  echo "========== fresh138 train complete; paired crossing focus =========="

  cd src
  declare -a candidates=()
  [[ -f "$FRESH128" ]] && candidates+=("$FRESH128")
  [[ -f "$FRESH136" ]] && candidates+=("$FRESH136")
  [[ -f "$FRESH137" ]] && candidates+=("$FRESH137")
  [[ -f "$MODEL" ]] && candidates+=("$MODEL")
  if [[ -d "$CKPT_DIR" ]]; then
    while IFS= read -r ckpt; do
      candidates+=("$ckpt")
    done < <(ls -1t "$CKPT_DIR"/fresh138_low_omega_finish_from_fresh136_step0864_step_*.pt 2>/dev/null || true)
  fi

  declare -A seen=()
  idx=0
  domain_span=$((MAX_DOMAIN - BASE_DOMAIN + 1))
  if (( domain_span <= 0 )); then
    echo "Invalid ROS domain range: BASE_DOMAIN=$BASE_DOMAIN MAX_DOMAIN=$MAX_DOMAIN" >&2
    exit 1
  fi
  summary="$OUT/fresh138_crossing_summary.tsv"
  echo -e "label\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tentanglement\tcolregs\tomega\tmodel" > "$summary"
  for model in "${candidates[@]}"; do
    [[ -f "$model" ]] || continue
    real_model="$(readlink -f "$model")"
    if [[ -n "${seen[$real_model]:-}" ]]; then
      continue
    fi
    seen[$real_model]=1
    label="$(basename "$model" .pt)"
    json="$OUT/${label}_crossing${EPISODES}_seed${SEED}_eval.json"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    domain=$((BASE_DOMAIN + (idx % domain_span)))
    idx=$((idx + 1))
    echo "----- crossing eval: $label domain=$domain seed=$SEED -----"
    ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$model" \
      --episodes "$EPISODES" \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --seed "$SEED" \
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
scenario = data.get('scenario_summaries', {}).get('three_usv_crossing') or data
row = {
    'collision': float(scenario.get('collision_rate', 1.0)),
    'success': float(scenario.get('success_rate', 0.0)),
    'timeout': float(scenario.get('timeout_rate', 0.0)),
    'progress': float(scenario.get('mean_team_goal_progress_ratio', 0.0)),
    'goal': float(scenario.get('mean_goal_completion_ratio', 0.0)),
    'sep': float(scenario.get('worst_episode_min_separation', 0.0)),
    'entanglement': float(scenario.get('mean_entanglement_ratio', 1.0)),
    'colregs': float(scenario.get('mean_colregs_violation_ratio', 1.0)),
    'omega': float(scenario.get('mean_omega_flip_count', 999.0)),
}
print(
    f"fresh138 crossing {label}: coll={row['collision']:.3f} "
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
  echo "========== fresh138 crossing summary =========="
  cat "$summary"
  echo "========== fresh138 pipeline complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"