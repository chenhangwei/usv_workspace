#!/bin/bash
# One-episode paired triage for fresh153 checkpoints.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

OUT="${OUT:-/mnt/data/checkpoints/usv_rl/fresh153_eval}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh153_checkpoints}"
FINAL_MODEL="${FINAL_MODEL:-/mnt/data/checkpoints/usv_rl/fresh153_early_headon_brake_from_fresh150.pt}"
SUMMARY="${SUMMARY:-$OUT/fresh153_triage_summary.tsv}"
EPISODES="${EPISODES:-1}"
STEPS="${STEPS:-420}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-900}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108}"
SEED="${SEED:-1460}"
BASE_DOMAIN="${BASE_DOMAIN:-224}"

mkdir -p "$OUT"
printf 'label\tscenario\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tentanglement\tcolregs\tomega\tmodel\n' > "$SUMMARY"

candidates=(
  "fresh153_early_headon_brake_from_fresh150_step_0000288:$CKPT_DIR/fresh153_early_headon_brake_from_fresh150_step_0000288.pt"
  "fresh153_early_headon_brake_from_fresh150_step_0000576:$CKPT_DIR/fresh153_early_headon_brake_from_fresh150_step_0000576.pt"
  "fresh153_early_headon_brake_from_fresh150_step_0000864:$CKPT_DIR/fresh153_early_headon_brake_from_fresh150_step_0000864.pt"
  "fresh153_early_headon_brake_from_fresh150:$FINAL_MODEL"
)
scenarios=(three_usv_crossing three_usv_random_encounter)

idx=0
for item in "${candidates[@]}"; do
  label="${item%%:*}"
  model="${item#*:}"
  if [[ ! -f "$model" ]]; then
    echo "skip missing candidate: $model"
    continue
  fi
  for scenario in "${scenarios[@]}"; do
    domain=$((BASE_DOMAIN + idx))
    idx=$(((idx + 1) % 8))
    json="$OUT/${label}_${scenario}_episodes${EPISODES}_seed${SEED}_eval.json"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    printf -- '----- triage: %s scenario=%s domain=%s seed=%s -----\n' "$label" "$scenario" "$domain" "$SEED"
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

    /bin/python3 - "$json" "$SUMMARY" "$label" "$scenario" "$model" <<'PY'
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
    f"fresh153-triage {label} {scenario_name}: coll={row['collision']:.3f} "
    f"succ={row['success']:.3f} timeout={row['timeout']:.3f} "
    f"progress={row['progress']:.3f} goal={row['goal']:.3f} sep={row['sep']:.3f} "
    f"cte={row['cte']:.3f} ent={row['entanglement']:.3f} "
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

printf '========== fresh153 triage summary =========='"\n"
cat "$SUMMARY"