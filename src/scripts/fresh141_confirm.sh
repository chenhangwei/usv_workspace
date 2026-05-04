#!/bin/bash
# fresh141 confirm: multi-scenario validation against fresh140.

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

LOG="${LOG:-/tmp/fresh141_confirm.log}"
OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh141_confirm_eval}"
EPISODES="${EPISODES:-3}"
STEPS="${STEPS:-420}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1900}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108}"
BASE_DOMAIN="${BASE_DOMAIN:-176}"
MAX_DOMAIN="${MAX_DOMAIN:-232}"
SEED="${SEED:-1410}"
INCLUDE_DENSE="${INCLUDE_DENSE:-1}"

FRESH140_FINAL="${FRESH140_FINAL:-/mnt/data/checkpoints/usv_rl/fresh140_guarded_finish_from_fresh139_step0864.pt}"
FRESH141_FINAL="${FRESH141_FINAL:-/mnt/data/checkpoints/usv_rl/fresh141_close_margin_finish_generalize_from_fresh140.pt}"
FRESH141_STEP0480="${FRESH141_STEP0480:-/mnt/data/checkpoints/usv_rl/fresh141_checkpoints/fresh141_close_margin_finish_generalize_from_fresh140_step_0000480.pt}"
FRESH141_STEP0960="${FRESH141_STEP0960:-/mnt/data/checkpoints/usv_rl/fresh141_checkpoints/fresh141_close_margin_finish_generalize_from_fresh140_step_0000960.pt}"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT"

{
  echo "========== fresh141 confirm =========="
  echo "Started: $(date -Iseconds)"
  echo "Eval seed: $SEED episodes=$EPISODES steps=$STEPS include_dense=$INCLUDE_DENSE"

  cd src
  declare -a labels=(
    "fresh141_final"
    "fresh141_step0960"
    "fresh141_step0480"
    "fresh140_final"
  )
  declare -a models=(
    "$FRESH141_FINAL"
    "$FRESH141_STEP0960"
    "$FRESH141_STEP0480"
    "$FRESH140_FINAL"
  )

  declare -a scenarios=(
    "three_usv_crossing"
    "three_usv_random_encounter"
  )
  if [[ "$INCLUDE_DENSE" == "1" ]]; then
    scenarios+=("five_usv_dense_crossing")
  fi

  domain_span=$((MAX_DOMAIN - BASE_DOMAIN + 1))
  if (( domain_span <= 0 )); then
    echo "Invalid ROS domain range: BASE_DOMAIN=$BASE_DOMAIN MAX_DOMAIN=$MAX_DOMAIN" >&2
    exit 1
  fi

  summary="$OUT/fresh141_confirm_summary.tsv"
  echo -e "label\tscenario\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tentanglement\tcolregs\tomega\tmodel" > "$summary"

  idx=0
  declare -A seen=()
  for i in "${!labels[@]}"; do
    label="${labels[$i]}"
    model="${models[$i]}"
    if [[ ! -f "$model" ]]; then
      echo "Skip missing model: $label $model"
      continue
    fi
    real_model="$(readlink -f "$model")"
    if [[ -n "${seen[$real_model]:-}" ]]; then
      echo "Skip duplicate model: $label $real_model"
      continue
    fi
    seen[$real_model]=1

    for scenario in "${scenarios[@]}"; do
      json="$OUT/${label}_${scenario}_episodes${EPISODES}_seed${SEED}_eval.json"
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
      domain=$((BASE_DOMAIN + (idx % domain_span)))
      idx=$((idx + 1))
      echo "----- confirm eval: $label scenario=$scenario domain=$domain seed=$SEED -----"
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
        --scenario "$scenario"

      /bin/python3 - "$json" "$summary" "$label" "$scenario" "$model" <<'PY'
import json
import sys
from pathlib import Path

json_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
label = sys.argv[3]
scenario_name = sys.argv[4]
model = sys.argv[5]
data = json.loads(json_path.read_text(encoding="utf-8"))
scenario = data.get("scenario_summaries", {}).get(scenario_name) or data
row = {
    "collision": float(scenario.get("collision_rate", 1.0)),
    "success": float(scenario.get("success_rate", 0.0)),
    "timeout": float(scenario.get("timeout_rate", 0.0)),
    "progress": float(scenario.get("mean_team_goal_progress_ratio", 0.0)),
    "goal": float(scenario.get("mean_goal_completion_ratio", 0.0)),
    "sep": float(scenario.get("worst_episode_min_separation", 0.0)),
    "cte": float(scenario.get("mean_cross_track_error", 999.0)),
    "entanglement": float(scenario.get("mean_entanglement_ratio", 1.0)),
    "colregs": float(scenario.get("mean_colregs_violation_ratio", 1.0)),
    "omega": float(scenario.get("mean_omega_flip_count", 999.0)),
}
print(
    f"confirm {label} {scenario_name}: coll={row['collision']:.3f} "
    f"succ={row['success']:.3f} timeout={row['timeout']:.3f} "
    f"progress={row['progress']:.3f} goal={row['goal']:.3f} "
    f"sep={row['sep']:.3f} cte={row['cte']:.3f} ent={row['entanglement']:.3f} "
    f"colregs={row['colregs']:.3f} omega={row['omega']:.1f}"
)
summary_path.open("a", encoding="utf-8").write(
    f"{label}\t{scenario_name}\t{row['collision']:.6f}\t{row['success']:.6f}\t"
    f"{row['timeout']:.6f}\t{row['progress']:.6f}\t{row['goal']:.6f}\t"
    f"{row['sep']:.6f}\t{row['cte']:.6f}\t{row['entanglement']:.6f}\t"
    f"{row['colregs']:.6f}\t{row['omega']:.6f}\t{model}\n"
)
PY
    done
  done

  echo "========== fresh141 confirm summary =========="
  cat "$summary"
  echo "========== fresh141 confirm complete =========="
  echo "Finished: $(date -Iseconds)"
} 2>&1 | tee "$LOG"
