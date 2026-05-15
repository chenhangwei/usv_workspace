#!/bin/bash
# Three-episode confirm for a selected fresh157 checkpoint.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh157_eval}"
MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh157_role_weighted_guard_from_fresh153.pt}"
LABEL="${LABEL:-$(basename "$MODEL" .pt)}"
SUMMARY="${SUMMARY:-$OUT/fresh157_confirm_summary.tsv}"
EPISODES="${EPISODES:-3}"
STEPS="${STEPS:-420}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108}"
SEED="${SEED:-1460}"
BASE_DOMAIN="${BASE_DOMAIN:-228}"

mkdir -p "$OUT"
printf 'label\tscenario\tepisodes\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tentanglement\tcolregs\tomega\tsteps\tmodel\n' > "$SUMMARY"

scenarios=(three_usv_crossing three_usv_random_encounter)

idx=0
for scenario in "${scenarios[@]}"; do
  domain=$((BASE_DOMAIN + idx))
  idx=$((idx + 1))
  json="$OUT/${LABEL}_${scenario}_episodes${EPISODES}_seed${SEED}_confirm.json"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  printf -- '----- confirm: %s scenario=%s episodes=%s domain=%s seed=%s -----\n' "$LABEL" "$scenario" "$EPISODES" "$domain" "$SEED"
  ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$MODEL" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$SEED" \
    --output-json "$json" \
    --scenario "$scenario"

  /bin/python3 - "$json" "$SUMMARY" "$LABEL" "$scenario" "$MODEL" <<'PY'
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
    "episodes": int(scenario.get("episodes", data.get("episodes", 0))),
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
    "steps": float(scenario.get("mean_steps", 0.0)),
}
print(
    f"fresh157-confirm {label} {scenario_name}: episodes={row['episodes']} "
    f"coll={row['collision']:.3f} succ={row['success']:.3f} timeout={row['timeout']:.3f} "
    f"progress={row['progress']:.3f} goal={row['goal']:.3f} sep={row['sep']:.3f} "
    f"cte={row['cte']:.3f} ent={row['entanglement']:.3f} colregs={row['colregs']:.3f} "
    f"omega={row['omega']:.1f} steps={row['steps']:.1f}"
)
summary_path.open("a", encoding="utf-8").write(
    f"{label}\t{scenario_name}\t{row['episodes']}\t{row['collision']:.6f}\t"
    f"{row['success']:.6f}\t{row['timeout']:.6f}\t{row['progress']:.6f}\t"
    f"{row['goal']:.6f}\t{row['sep']:.6f}\t{row['cte']:.6f}\t"
    f"{row['entanglement']:.6f}\t{row['colregs']:.6f}\t{row['omega']:.6f}\t"
    f"{row['steps']:.6f}\t{model}\n"
)
PY
done

printf '========== fresh157 confirm summary =========='"\n"
cat "$SUMMARY"